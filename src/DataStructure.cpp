//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include <set>
#include <numeric>
#include "util.h"
#include "DataStructure.h"
#include "UTM.h"
#include <pcl/common/pca.h>

using namespace std;

DataStructure::DataStructure(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& gmlFile, const std::string& imgFile): imgFile(imgFile) {

    DataIO dataIO = DataIO();//lasFiles, shpFile, imgFile);
    std::vector<int> pointClasses;
    bool cachedSplats = dataIO.readData(lasFiles, shpFile, gmlFile, imgFile, cloud,  texCoords, tangent1Vec, tangent2Vec, wallPointsStartIndex, pointClasses);
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);

    if (!cachedSplats) {

        adaSplats(tree, pointClasses);

        std::string lasDir = ".." + Util::PATH_SEPARATOR + "las" + Util::PATH_SEPARATOR;
        const auto& file = lasFiles[0];

        dataIO.writeCache(lasDir + file, true, cloud, texCoords, tangent1Vec, tangent2Vec, wallPointsStartIndex, pointClasses);
    }

}

void DataStructure::adaSplats(pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree, std::vector<int>& pointClasses) {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start ada" << std::endl;


    int k = 40;
    std::vector<pcl::Indices> pointNeighbourhoods(cloud->points.size());
    std::vector<vector<float>> pointNeighbourhoodsDistance(cloud->points.size());


    // ********** knn and compute avgRadius **********
    float avgRadiusNeighbourhoods = adaKnnAndRadius(k, tree, pointNeighbourhoods, pointNeighbourhoodsDistance);// * 4;
    std::cout << TAG << "avg radius R is: " << avgRadiusNeighbourhoods << std::endl;

    // ********** get neighbourhood with radius, use pca for normal and classification and compute epsilon **********
    float splatGrowEpsilon = adaNeighbourhoodsClassificationAndEpsilon(avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);
    std::cout << TAG << "splat grow epsilon is: " << splatGrowEpsilon << std::endl;

    // ********** get new neighbourhood with new k and new radius (depending on classification) and use pca for normal **********
    adaNewNeighbourhoods(k, tree, avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);

    // ********** compute splats **********
    adaComputeSplats(splatGrowEpsilon, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);

//    adaResampling(avgRadiusNeighbourhoods, tree, pointClasses);


    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished ada in " << duration.count() << "s" << std::endl;
}

float
DataStructure::adaKnnAndRadius(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree,
                               std::vector<pcl::Indices>& pointNeighbourhoods,
                               std::vector<std::vector<float>>& pointNeighbourhoodsDistance) {
    // ********** knn with default k and compute avgRadius **********
    float avgRadiusSumNeighbourhoods = 0;
    auto radiusCount = 0;

    // normal points
    pcl::Indices normalPoints(wallPointsStartIndex);
    std::iota (std::begin(normalPoints), std::end(normalPoints), 0);
    auto normalPointsPtr = make_shared<pcl::Indices>(normalPoints);
    tree->setInputCloud(cloud);//, normalPointsPtr);
    for (auto pointIdx = 0; pointIdx < wallPointsStartIndex; pointIdx++) {
        pcl::Indices neighboursPointIdx(k);
        auto neighboursSquaredDistance = std::vector<float>(k);
        if (tree->nearestKSearch(pointIdx, k, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca

            // desquare it
            auto neighboursDistance = std::vector<float>(k);
            std::transform(neighboursSquaredDistance.begin(), neighboursSquaredDistance.end(), neighboursDistance.begin(), [](float number){ return sqrt(number);});

            auto const count = static_cast<float>(neighboursDistance.size());
            auto maxRadius = *(neighboursDistance.end() - 1);
            avgRadiusSumNeighbourhoods += maxRadius;

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursDistance;

            radiusCount++;
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }
    }
    // wall points
    pcl::Indices wallPoints((*cloud).size() - wallPointsStartIndex);
    std::iota (std::begin(wallPoints), std::end(wallPoints), wallPointsStartIndex);
    auto wallPointsPtr = make_shared<pcl::Indices>(wallPoints);
    tree->setInputCloud(cloud, wallPointsPtr);
    for (auto pointIdx = wallPointsStartIndex; pointIdx < cloud->points.size(); pointIdx++) {
        pcl::Indices neighboursPointIdx(k);
        auto searchIndex = pointIdx - wallPointsStartIndex;
        auto neighboursSquaredDistance = std::vector<float>(k);
        if (tree->nearestKSearch(searchIndex, k, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca

            // desquare it
            auto neighboursDistance = std::vector<float>(k);
            std::transform(neighboursSquaredDistance.begin(), neighboursSquaredDistance.end(), neighboursDistance.begin(), [](float number){ return sqrt(number);});

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursDistance;
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }
    }

    return avgRadiusSumNeighbourhoods / static_cast<float>(radiusCount);;
}

float DataStructure::adaNeighbourhoodsClassificationAndEpsilon(float avgRadiusNeighbourhoods, std::vector<pcl::Indices>& pointNeighbourhoods,
                                                               std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses) {
    float uPtpDistSumNeighbourhoods = 0;
    auto epsilonSumCount = 0;

    for (auto pointIdx = 0; pointIdx < wallPointsStartIndex; pointIdx++) {

        if (pointNeighbourhoods[pointIdx].size() >= 3) { // need at least 3 points for pca

            // define Npi as all points here from knn search but within avg radius?
            // find border index
            int lastNeighbour = findIndex(avgRadiusNeighbourhoods, pointNeighbourhoodsDistance[pointIdx]);
            // do stuff
            auto neighbourhood = pcl::Indices(pointNeighbourhoods[pointIdx].begin(),
                                              pointNeighbourhoods[pointIdx].begin() + lastNeighbour);
            if (neighbourhood.size() >= 3) {
                pcl::IndicesPtr neighboursPtr = make_shared<pcl::Indices>(neighbourhood);
                // pca on the neighbourhood
                pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
                pca.setInputCloud(cloud);
                pca.setIndices(neighboursPtr);
                Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
                Eigen::Vector3f eigenValues = pca.getEigenValues();

                // set normal and tangents for non-wall points
                // else: are already set from wall detection

                    // set normal
                    cloud->points[pointIdx].normal_x = eigenVectors(0, 2);
                    cloud->points[pointIdx].normal_y = eigenVectors(1, 2);
                    cloud->points[pointIdx].normal_z = eigenVectors(2, 2);

                    // also set tangents
                    auto tangent1 = pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0));
                    tangent1Vec[pointIdx] = tangent1;
                    auto tangent2 = pcl::PointXYZ(eigenVectors(0, 1), eigenVectors(1, 1), eigenVectors(2, 1));
                    tangent2Vec[pointIdx] = tangent2;

                    // check if vertical normal is oriented up
                    const auto& point = cloud->points[pointIdx];
                    auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                    auto vertLen = abs(point.normal_y);
                    if (horLen < vertLen) {
                        if (point.normal_y < 0) {
                            (*cloud)[pointIdx].normal_x *= -1;
                            (*cloud)[pointIdx].normal_y *= -1;
                            (*cloud)[pointIdx].normal_z *= -1;

                            // also flip tangents
                            tangent1Vec[pointIdx] = tangent2;
                            tangent2Vec[pointIdx] = tangent1;
                        }
                    }

                    // local descriptors
                    const auto& l1 = eigenValues(0);
                    const auto& l2 = eigenValues(1);
                    const auto& l3 = eigenValues(2);
                    float linearity = (l1 - l2) / l1;
                    float planarity = (l2 - l3) / l1;
                    float sphericity = l3 / l1;
                    std::array bla = {linearity, planarity, sphericity};
                    int mainDings = std::distance(bla.begin(), std::max_element(bla.begin(), bla.end()));

                    pointClasses[pointIdx] = mainDings;



                if (colorClasses) {
                    switch (pointClasses[pointIdx]) {
                        case 0:
                            // linearity is main
                                (*cloud)[pointIdx].r = 255;
                                (*cloud)[pointIdx].g = 0;
                                (*cloud)[pointIdx].b = 0;
                            break;
                        case 1:
                            // planarity is main
                                (*cloud)[pointIdx].r = 0;
                                (*cloud)[pointIdx].g = 255;
                                (*cloud)[pointIdx].b = 0;
                            break;
                        default:
                            // sphericity is main
                                (*cloud)[pointIdx].r = 0;
                                (*cloud)[pointIdx].g = 0;
                                (*cloud)[pointIdx].b = 255;
                            break;
                    }
                }

                // splat grow epsilon
                // compute avg unsigned point to plane distance for splat grow epsilon
                float uPtpDistSum = 0;
                // first neighbour is the point itself
                auto bla2 = std::vector<float>(neighbourhood.size());
                for (auto nPointIdx = 1; nPointIdx < neighbourhood.size(); nPointIdx++) {
                    auto ppd = Util::pointPlaneDistance((*cloud)[neighbourhood[nPointIdx]], (*cloud)[pointIdx]);
                    uPtpDistSum += ppd;

                    bla2[nPointIdx] = ppd;
                }
                float uPtpDistAvg = uPtpDistSum / static_cast<float>(neighbourhood.size() - 1);
                uPtpDistSumNeighbourhoods += uPtpDistAvg;
                epsilonSumCount++;

            } else {
                tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
                tangent2Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);

                (*cloud)[pointIdx].b = 255;
                (*cloud)[pointIdx].r = 255;
                (*cloud)[pointIdx].g = 0;
            }
        } else {
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            tangent2Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            (*cloud)[pointIdx].b = 255;
            (*cloud)[pointIdx].r = 0;
            (*cloud)[pointIdx].g = 0;

        }
    }


    if (colorClasses) {
        for (auto pointIdx = wallPointsStartIndex; pointIdx < (*cloud).size(); pointIdx++) {
            switch (pointClasses[pointIdx]) {
                case 0:
                    // linearity is main
                    (*cloud)[pointIdx].r = 255;
                    (*cloud)[pointIdx].g = 0;
                    (*cloud)[pointIdx].b = 0;
                    break;
                case 1:
                    // planarity is main
                    (*cloud)[pointIdx].r = 0;
                    (*cloud)[pointIdx].g = 255;
                    (*cloud)[pointIdx].b = 0;
                    break;
                default:
                    // sphericity is main
                    (*cloud)[pointIdx].r = 0;
                    (*cloud)[pointIdx].g = 0;
                    (*cloud)[pointIdx].b = 255;
                    break;
            }
        }
    }

    // ********** compute epsilon **********
    return uPtpDistSumNeighbourhoods / static_cast<float>(epsilonSumCount);
}

void DataStructure::adaNewNeighbourhoods(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, float avgRadiusNeighbourhoods,
                                         std::vector<pcl::Indices>& pointNeighbourhoods,
                                         std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses) {

    // get new knn
    int currentK;

    // normal points
    tree->setInputCloud(cloud);
    for (auto pointIdx = 0; pointIdx < wallPointsStartIndex; pointIdx++) {

        // get new k-neighbourhood with adjusted k
        switch (pointClasses[pointIdx]) {
            case 0: // linearity is main
                currentK = k * 2;//0.33;
                break;
            case 1: // planarity is main
                currentK = k * 2;
                break;
            default: // sphericity is main
                currentK = k * 0.25;
                break;
        }
        pcl::Indices neighboursPointIdx(currentK);
        auto neighboursSquaredDistance = std::vector<float>(currentK);
        if (tree->nearestKSearch(pointIdx, currentK, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca

            auto neighboursDistance = std::vector<float>(currentK);
            std::transform(neighboursSquaredDistance.begin(), neighboursSquaredDistance.end(), neighboursDistance.begin(), [](float number){ return sqrt(number);});

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursDistance;
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }



        // get new neighbourhood with adjusted radius
        float currentAvgRadius;

        if (pointNeighbourhoods[pointIdx].size() >= 3) { // need at least 3 points for pca
            switch (pointClasses[pointIdx]) {
                case 0: // linearity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 2;//0.33;
                    break;
                case 1: // planarity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 2;
                    break;
                default: // sphericity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 0.25;
                    break;
            }
            // define Npi as all points here from knn search but within avg radius?
            // find border index
            int lastNeighbour = findIndex(currentAvgRadius, pointNeighbourhoodsDistance[pointIdx]);
            // do stuff
            auto neighbours = pcl::Indices(pointNeighbourhoods[pointIdx].begin(),
                                           pointNeighbourhoods[pointIdx].begin() + lastNeighbour);
            if (neighbours.size() >= 3) {
                // save new neighbourhood
                pointNeighbourhoods[pointIdx] = neighbours;
            } else {
                pointNeighbourhoods[pointIdx] = pcl::Indices();
            }
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
        }
    }


    // wall points
    pcl::Indices wallPoints((*cloud).size() - wallPointsStartIndex);
    std::iota (std::begin(wallPoints), std::end(wallPoints), wallPointsStartIndex);
    auto wallPointsPtr = make_shared<pcl::Indices>(wallPoints);
    tree->setInputCloud(cloud, wallPointsPtr);
    for (auto pointIdx = wallPointsStartIndex; pointIdx < cloud->points.size(); pointIdx++) {
        // get new k-neighbourhood with adjusted k
        switch (pointClasses[pointIdx]) {
            case 0: // linearity is main
                currentK = k * 2;//0.33;
                break;
            case 1: // planarity is main
                currentK = k * 2;
                break;
            default: // sphericity is main
                currentK = k * 0.25;
                break;
        }
        auto searchIndex = pointIdx - wallPointsStartIndex;
        pcl::Indices neighboursPointIdx(currentK);
        auto neighboursSquaredDistance = std::vector<float>(currentK);
        if (tree->nearestKSearch(searchIndex, currentK, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca

            auto neighboursDistance = std::vector<float>(currentK);
            std::transform(neighboursSquaredDistance.begin(), neighboursSquaredDistance.end(), neighboursDistance.begin(), [](float number){ return sqrt(number);});

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursDistance;
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }



        // get new neighbourhood with adjusted radius
        float currentAvgRadius;

        if (pointNeighbourhoods[pointIdx].size() >= 3) { // need at least 3 points for pca
            switch (pointClasses[pointIdx]) {
                case 0: // linearity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 2;//0.33;
                    break;
                case 1: // planarity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 2;
                    break;
                default: // sphericity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 0.25;
                    break;
            }
            // define Npi as all points here from knn search but within avg radius?
            // find border index
            int lastNeighbour = findIndex(currentAvgRadius, pointNeighbourhoodsDistance[pointIdx]);
            // do stuff
            auto neighbours = pcl::Indices(pointNeighbourhoods[pointIdx].begin(),
                                           pointNeighbourhoods[pointIdx].begin() + lastNeighbour);
            if (neighbours.size() >= 3) {
                // save new neighbourhood
                pointNeighbourhoods[pointIdx] = neighbours;
            } else {
                pointNeighbourhoods[pointIdx] = pcl::Indices();
            }
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
        }
    }
}

void
DataStructure::adaComputeSplats(float splatGrowEpsilon, std::vector<pcl::Indices>& pointNeighbourhoods,
                                std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses) {
    float alpha = 0.5;

    // 1 → 0°, 0.7 → 45°, 0 → 90°(pi/2). i guess: -0.7 → 135°, -1 → 180°(pi). (arccos kann nur zwischen 0° und 180° zeigen, richtung nicht beachtet)
    float angleThreshold = 0.26;//0.26;//0.6;//0.86; // ~30°

    std::vector<bool> discardPoint(cloud->points.size());
    fill(discardPoint.begin(), discardPoint.end(), false);

    float currentSplatGrowEpsilon;

    int splatCount = 0;

    // for every point
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

        if (discardPoint[pointIdx]) {
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            tangent2Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            continue;
        }

        // adapt splat parameters to point class
        switch (pointClasses[pointIdx]) {
            case 0: // linearity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 0.33;
                break;
            case 1: // planarity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 2;
                break;
            default: // sphericity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 0.25;
                break;
        }

        auto const& point = cloud->points[pointIdx];
        auto const& neighbourhood = pointNeighbourhoods[pointIdx];

        (*cloud)[pointIdx].curvature = 0; // this is radius

        auto normal = pcl::PointXYZ(point.normal_x, point.normal_y, point.normal_z);

        float epsilonSum = 0;
        int epsilonCount = 0;
        int lastEpsilonNeighbourIdx = 0;

        // grow
        // for every neighbour
        for (auto nIdx = 1; nIdx < neighbourhood.size(); nIdx++) {

            pcl::PointXYZ neighbourNormal;
            neighbourNormal.x = (*cloud)[neighbourhood[nIdx]].normal_x;
            neighbourNormal.y = (*cloud)[neighbourhood[nIdx]].normal_y;
            neighbourNormal.z = (*cloud)[neighbourhood[nIdx]].normal_z;

            // stop growing when angle is too big
            float angle = acos(Util::dotProduct(normal, neighbourNormal));
            if (abs(angle) > angleThreshold) {
                break;
            }

            // stop growing when point plane distance is too big
            auto eps = Util::signedPointPlaneDistance(point, cloud->points[neighbourhood[nIdx]], normal);
            if (abs(eps) > currentSplatGrowEpsilon) {
                break;
            }

            // stop growing when neighbour has different class
            if (pointClasses[pointIdx] != pointClasses[neighbourhood[nIdx]]) {
                break;
            }


            epsilonSum += eps;
            epsilonCount++;
            lastEpsilonNeighbourIdx = nIdx;
        }


        if (epsilonCount == 0 ) {
            // no valid neighbours (all have been discarded or nearest neighbours eps dist is too big)
            if (colorInvalid) {
                if ((*cloud)[pointIdx].g != 255) {
                    (*cloud)[pointIdx].r = 255;
                    (*cloud)[pointIdx].g = 0;
                    (*cloud)[pointIdx].b = 255;
                }
            }
            // used in shader to skip this splat
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            continue;
        }

        // compute avg of the epsilons
        float epsilonAvg = epsilonSum / static_cast<float>(epsilonCount);//(epsilonCount1 + epsilonCount2);

        // move splat point
        (*cloud)[pointIdx].x += epsilonAvg * normal.x;
        (*cloud)[pointIdx].y += epsilonAvg * normal.y;
        (*cloud)[pointIdx].z += epsilonAvg * normal.z;

        // compute splat radii
        // tangent 1
        const auto& lastNeighbourPoint1 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx]];
        auto pointToNeighbourVec1 = Util::vectorSubtract(lastNeighbourPoint1, point);
        auto bla1 = Util::dotProduct(normal, pointToNeighbourVec1);
        auto rightSide1 = pcl::PointXYZ(bla1 * normal.x, bla1 * normal.y, bla1 * normal.z);
        float radius1 = Util::vectorLength(Util::vectorSubtract(pointToNeighbourVec1, rightSide1));
        // tangent 2
        // TODO elliptical splats
//        const auto& lastNeighbourPoint2 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx]];
//        auto pointToNeighbourVec2 = Util::vectorSubtract(lastNeighbourPoint2, point);
//        auto bla2 = Util::dotProduct(normal, pointToNeighbourVec2);
//        auto rightSide2 = pcl::PointXYZ(bla2 * normal.x, bla2 * normal.y, bla2 * normal.z);
        float radius2 = radius1;//Util::vectorLength(Util::vectorSubtract(pointToNeighbourVec2, rightSide2));
        // happens when last valid neighbour is at the same position
        if (radius1  == 0)
            continue;
        // length of axes has to be 1/radius
        tangent1Vec[pointIdx] = pcl::PointXYZ(tangent1Vec[pointIdx].x / radius1, tangent1Vec[pointIdx].y / radius1, tangent1Vec[pointIdx].z / radius1);
        tangent2Vec[pointIdx] = pcl::PointXYZ(tangent2Vec[pointIdx].x / radius2, tangent2Vec[pointIdx].y / radius2, tangent2Vec[pointIdx].z / radius2);


        // discard points
        auto neighbourhoodDistances = pointNeighbourhoodsDistance[pointIdx];
        discardPoint[pointIdx] = true;

        // TODO temp solution for elliptical splats
        float radius = max(radius1, radius2);
        for (auto nIdx = 0; nIdx < neighbourhood.size(); nIdx++) {

            if (discardPoint[neighbourhood[nIdx]])
                continue;

            auto dist = neighbourhoodDistances[nIdx];

            if (dist < alpha * radius) {
                discardPoint[neighbourhood[nIdx]] = true;
                // color debug - discarded points
                if (true) {
                    if (nIdx != 0) {
                        (*cloud)[neighbourhood[nIdx]].r = 255;
                        (*cloud)[neighbourhood[nIdx]].g = 255;
                        cloud->points[neighbourhood[nIdx]].b = 0;
                    }
                }
            } else {
                // dist values are ascending
                break;
            }
        }

        if (colorSplats) {
            // color debug - random color for every point
            int randR = rand() % (255 - 0 + 1) + 0;
            int randG = rand() % (255 - 0 + 1) + 0;
            int randB = rand() % (255 - 0 + 1) + 0;
            (*cloud)[pointIdx].r = randR;
            (*cloud)[pointIdx].g = randG;
            (*cloud)[pointIdx].b = randB;
        }
    }
}


void DataStructure::adaResampling(float avgRadiusNeighbourhoods, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<int>& pointClasses) {

    // determine avg splats density
    float avgSplatsDensity = 0;
    int avgSplatCounter = 0;
    int pointSplatCount;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        const auto& point = (*cloud)[pointIdx];
        pointSplatCount = 0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (tree->radiusSearch(point, avgRadiusNeighbourhoods, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // count points that are splats, but dont count non-surface group
            for (int searchIdx = 0; searchIdx < pointIdxRadiusSearch.size(); searchIdx++) {
                int& nPointIdx = pointIdxRadiusSearch[searchIdx];
                const auto& nPoint = (*cloud)[nPointIdx];
                // continue if not splat
                if (tangent1Vec[nPointIdx].x == 0 && tangent1Vec[nPointIdx].y == 0 && tangent1Vec[nPointIdx].z == 0) { // for elliptical splats
                    continue;
                }

                // continue if non-surface group
                if (pointClasses[nPointIdx] == 3) {
                    continue;
                }

                pointSplatCount++;
            }
            avgSplatsDensity += static_cast<float>(pointSplatCount) / static_cast<float>(pointIdxRadiusSearch.size());
            avgSplatCounter++;

        }
    }
    avgSplatsDensity /= static_cast<float>(avgSplatCounter);


    // resampling
    std::vector<pcl::PointXYZRGBNormal> newPoints;
    // 1 → 0°, 0.7 → 45°, 0 → 90°(pi/2). i guess: -0.7 → 135°, -1 → 180°(pi). (arccos kann nur zwischen 0° und 180° zeigen, richtung nicht beachtet)
    float angleThreshold = 0.6; // ~30°
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        const auto& point = (*cloud)[pointIdx];
        // only for splats
        if (tangent1Vec[pointIdx].x == 0 && tangent1Vec[pointIdx].y == 0 && tangent1Vec[pointIdx].z == 0) { // for elliptical splats
            continue;
        }

        pointSplatCount = 0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (tree->radiusSearch(point, avgRadiusNeighbourhoods, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // count points that are splats, but dont count non-surface group
            for (int searchIdx = 0; searchIdx < pointIdxRadiusSearch.size(); searchIdx++) {
                int& nPointIdx = pointIdxRadiusSearch[searchIdx];
                const auto& nPoint = (*cloud)[nPointIdx];
                // continue if not splat
                if (tangent1Vec[nPointIdx].x == 0 && tangent1Vec[nPointIdx].y == 0 && tangent1Vec[nPointIdx].z == 0) { // for elliptical splats
                    continue;
                }

                // continue if non-surface group
                if (pointClasses[nPointIdx] == 3) {
                    continue;
                }

                pointSplatCount++;
            }
            float splatDensity = static_cast<float>(pointSplatCount) / static_cast<float>(pointIdxRadiusSearch.size());

            if (splatDensity < avgSplatsDensity) {
                // resample
                for (auto nPointIdxIt = pointIdxRadiusSearch.rbegin(); nPointIdxIt != pointIdxRadiusSearch.rend(); nPointIdxIt++) {
                    const auto& nPointIdx = *nPointIdxIt;

                    if (pointClasses[pointIdx] != pointClasses[nPointIdx]) {
                        continue;
                    }
                    // smoothness check
                    auto normal = pcl::PointXYZ((*cloud)[pointIdx].normal_x, (*cloud)[pointIdx].normal_y, (*cloud)[pointIdx].normal_z);
                    pcl::PointXYZ neighbourNormal;
                    neighbourNormal.x = (*cloud)[nPointIdx].normal_x;
                    neighbourNormal.y = (*cloud)[nPointIdx].normal_y;
                    neighbourNormal.z = (*cloud)[nPointIdx].normal_z;
                    float angle = Util::dotProduct(normal, neighbourNormal);
                    if (angle < angleThreshold) {
                        continue;
                    }
                    // found the farthest valid splat

                    const auto& farthestSplat = (*cloud)[nPointIdx];
                    // "interpolate a new point that lies at the center of the segment connecting the splats’ centers"
                    auto newPoint = pcl::PointXYZRGBNormal((point.x + farthestSplat.x) / 2, (point.y + farthestSplat.y) / 2, (point.z + farthestSplat.z) / 2);
                    // debug color
                    newPoint.b = 255;
                    newPoint.g = 0;
                    newPoint.r = 255;
                    newPoint.normal_x = point.normal_x;
                    newPoint.normal_y = point.normal_y;
                    newPoint.normal_z = point.normal_z;
                    pointClasses.push_back(pointClasses[pointIdx]);
                    tangent1Vec.push_back(pcl::PointXYZ(0, 0, 0));
                    tangent2Vec.push_back(pcl::PointXYZ(0, 0, 0));
                    newPoints.push_back(newPoint);
                    break;

                }
            }
        }

    }
    cloud->insert(cloud->end(), newPoints.begin(), newPoints.end());


}

uint32_t DataStructure::getVertexCount() {
    return (uint32_t) cloud->width;
}

/**
 * returns index of first element greater then border
 * returns lastIndex+1 if all elements are smaller
 * @param border
 * @param vector1
 * @return
 */
int DataStructure::findIndex(float border, std::vector<float> vector1) {
    auto begin = vector1.begin();
    auto end = vector1.end();
    auto mid = begin + (end - begin) / 2;
    int index = (end - begin);
    while (begin <= end) {

        // check mid
        mid = begin + (end - begin) / 2;
        if (mid == vector1.end()) {
            // all elements are smaller then border
            return vector1.size();
        }
        if (*mid < border) {
            //search right side
            begin = mid + 1;
        } else {
            // search left side
            // mid element is current border candidate
            index = mid - vector1.begin();
            end = mid - 1;
        }
    }
    return index;
}