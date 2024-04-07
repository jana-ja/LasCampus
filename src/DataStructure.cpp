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
    //TODO tree ist grad doppelt

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
    float avgRadiusNeighbourhoods = adaKnnAndRadius(k, tree, pointNeighbourhoods, pointNeighbourhoodsDistance);
    std::cout << TAG << "avg radius R is: " << avgRadiusNeighbourhoods << std::endl;

    // ********** get neighbourhood with radius, use pca for normal and classification and compute epsilon **********
    float splatGrowEpsilon = adaNeighbourhoodsClassificationAndEpsilon(avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);
    std::cout << TAG << "splat grow epsilon is: " << splatGrowEpsilon << std::endl;

    // ********** get new neighbourhood with new k and new radius (depending on classification) and use pca for normal **********
    adaNewNeighbourhoods(k, tree, avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);

    // ********** horizontal normal orientation (walls) **********
//    float wallThreshold = 1.0;
//    adaNormalOrientation(wallThreshold, tree);

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
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        pcl::Indices neighboursPointIdx(k);
        auto neighboursSquaredDistance = std::vector<float>(k);
        if (tree->nearestKSearch(pointIdx, k, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca

            auto const count = static_cast<float>(neighboursSquaredDistance.size());
            auto avgRadius = std::reduce(neighboursSquaredDistance.begin(), neighboursSquaredDistance.end()) /
                             (count - 1); // count - 1, weil die erste distance immer 0 ist
            avgRadiusSumNeighbourhoods += avgRadius;

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursSquaredDistance;
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }
    }

    return avgRadiusSumNeighbourhoods / static_cast<float>(cloud->points.size());;
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

    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

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

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursSquaredDistance;
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
    float alpha = 0.4;

    // 1 → 0°, 0.7 → 45°, 0 → 90°(pi/2). i guess: -0.7 → 135°, -1 → 180°(pi). (arccos kann nur zwischen 0° und 180° zeigen, richtung nicht beachtet)
    float angleThreshold = 0.86; // ~30°

    std::vector<bool> discardPoint(cloud->points.size());
    fill(discardPoint.begin(), discardPoint.end(), false);

    float currentSplatGrowEpsilon = splatGrowEpsilon;

    // for every point
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
//        if (pointIdx < wallPointsStartIndex + 39100) {
//            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
//            continue;
//        }
//        if (pointIdx > wallPointsStartIndex + 39128) {
//            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
//            continue;
//        }
//
//        // TODO debug
//        (*cloud)[pointIdx].r = 0;
//        (*cloud)[pointIdx].g = 0;
//        (*cloud)[pointIdx].b = 255;
//
//
//        if (pointIdx == wallPointsStartIndex + 39125) {
//            (*cloud)[pointIdx].r = 0;
//            (*cloud)[pointIdx].g = 255;
//            (*cloud)[pointIdx].b = 255;
//            auto r = 3;
//        }
//        if (pointIdx % 1000 != 0){
//            continue;
//        }

        if (discardPoint[pointIdx]) {
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
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
        int epsilonCount1 = 0;
        int epsilonCount2 = 0;
        int lastEpsilonNeighbourIdx1 = 0;
        int lastEpsilonNeighbourIdx2 = 0;

        bool growTangent1 = true;
        bool growTangent2 = true;

        bool concernsTangent1;

        // grow
        // for every neighbour
        for (auto nIdx = 1; nIdx < neighbourhood.size(); nIdx++) {

            if (!growTangent1 && !growTangent2) {
                break;
            }

            // first check if neighbour point concerns tangent1 growth or tangent2 growth
            // project vector onto plane:
            auto neighbourVec = Util::vectorSubtract(cloud->points[neighbourhood[nIdx]], point);
            auto projectedDistance = Util::dotProduct(normal, neighbourVec);
            auto normalProjectedVec = pcl::PointXYZ(projectedDistance * normal.x, projectedDistance * normal.y, projectedDistance * normal.z);
            auto planeProjectedVec = Util::vectorSubtract(neighbourVec, normalProjectedVec);
            // angle
            auto vecLen = Util::vectorLength(planeProjectedVec);
            float cosAngle = Util::dotProduct(tangent1Vec[pointIdx], planeProjectedVec) / vecLen;
            if (abs(cosAngle) > 0.7) { // 45°
                // concerns tangent1 -> less then 45° between tangent1 and projected neighbour vector
                concernsTangent1 = true;
                if (!growTangent1) {
                    continue;
                }
            } else {
                concernsTangent1 = false;
                if (!growTangent2) {
                    continue;
                }
            }

            // TODO ich teste jetzt abbruchbedingungen auch bei discardeten punkten zu checken

            // TODO bei discardeten abbrechen!
            // TODO erst kreis growen und dann mit dem komischen lambda ding ellipse growen!

            // stop growing when angle between point normal and neighbour normal is too big
            // smoothness check
            pcl::PointXYZ neighbourNormal;
            neighbourNormal.x = (*cloud)[neighbourhood[nIdx]].normal_x;
            neighbourNormal.y = (*cloud)[neighbourhood[nIdx]].normal_y;
            neighbourNormal.z = (*cloud)[neighbourhood[nIdx]].normal_z;

            float angle = Util::dotProduct(normal, neighbourNormal);
            if (angle < angleThreshold) {
                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            auto eps = Util::signedPointPlaneDistance(point, cloud->points[neighbourhood[nIdx]], normal);
            if (abs(eps) > currentSplatGrowEpsilon) {
                // stop growing this neighbourhood
                // point nIdx does NOT belong to neighbourhood
//                if (nIdx == 1) {
//                    (*cloud)[pointIdx].r = 0;
//                    (*cloud)[pointIdx].g = 255;
//                    (*cloud)[pointIdx].b = 0;
//                }

                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            // stop growing when neighbour has different class
            if (pointClasses[pointIdx] != pointClasses[neighbourhood[nIdx]]) {
                // TODO neu der punkt kommt aber noch mit rein
                epsilonSum += eps;
                if (concernsTangent1) {
                    epsilonCount1++;
                    lastEpsilonNeighbourIdx1 = nIdx;
                } else {
                    epsilonCount2++;
                    lastEpsilonNeighbourIdx2 = nIdx;
                }
                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            // skip discarded points
            if (discardPoint[neighbourhood[nIdx]]) {
                continue;
            }



            epsilonSum += eps;
            if (concernsTangent1) {
                epsilonCount1++;
                lastEpsilonNeighbourIdx1 = nIdx;
            } else {
                epsilonCount2++;
                lastEpsilonNeighbourIdx2 = nIdx;
            }
        }

        // no valid neighbours in at least one direction // TODO schauen ob sinn macht getrennt zu betrachten
        // TODO PROBLEM: glaube schon dass das sinn macht, aber die ungewollten kanten sind so grade dass die hier invalid werden weil in eine richtung nichts passiert
        if (epsilonCount1 == 0 || epsilonCount2 == 0) {
            // no valid neighbours (all have been discarded or nearest neighbours eps dist is too big)
            if (colorInvalid) {
                if ((*cloud)[pointIdx].g != 255) {
                    (*cloud)[pointIdx].r = 255;
                    (*cloud)[pointIdx].g = 0;
                    (*cloud)[pointIdx].b = 255;
                }
            }
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            continue;
        }
//        else if (epsilonCount2 == 0) {
//            // minor achse
//            if (colorInvalid) {
//                (*cloud)[pointIdx].r = 255;
//                (*cloud)[pointIdx].g = 0;
//                (*cloud)[pointIdx].b = 0;
//            }
//            epsilonCount2++;
//            lastEpsilonNeighbourIdx2 = 1;
//        } else if (epsilonCount1 == 0) {
//            if (colorInvalid) {
//                (*cloud)[pointIdx].r = 255;
//                (*cloud)[pointIdx].g = 0;
//                (*cloud)[pointIdx].b = 0;
//            }
//            epsilonCount1++;
//            lastEpsilonNeighbourIdx1 = 1;
//        }

        // compute avg of the epsilons
        float epsilonAvg = epsilonSum / (epsilonCount1 + epsilonCount2);

        // move splat point
        (*cloud)[pointIdx].x += epsilonAvg * normal.x;
        (*cloud)[pointIdx].y += epsilonAvg * normal.y;
        (*cloud)[pointIdx].z += epsilonAvg * normal.z;

        // compute splat radii
        // tangent 1
        const auto& lastNeighbourPoint1 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx1]];
        auto pointToNeighbourVec1 = Util::vectorSubtract(lastNeighbourPoint1, point);
        auto bla1 = Util::dotProduct(normal, pointToNeighbourVec1);
        auto rightSide1 = pcl::PointXYZ(bla1 * normal.x, bla1 * normal.y, bla1 * normal.z);
        float radius1 = Util::vectorLength(Util::vectorSubtract(pointToNeighbourVec1, rightSide1));
//        float radius1 = Util::vectorLength(pointToNeighbourVec1);
        // tangent 2
        const auto& lastNeighbourPoint2 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx2]];
        auto pointToNeighbourVec2 = Util::vectorSubtract(lastNeighbourPoint2, point);
        auto bla2 = Util::dotProduct(normal, pointToNeighbourVec2);
        auto rightSide2 = pcl::PointXYZ(bla2 * normal.x, bla2 * normal.y, bla2 * normal.z);
        float radius2 = Util::vectorLength(Util::vectorSubtract(pointToNeighbourVec2, rightSide2));
//        float radius2 = Util::vectorLength(pointToNeighbourVec2);
        // length of axes has to be 1/radius

        // TODO test debug
//        auto minRadius = 2.0f;//std::min(radius1, radius2);
//        radius1 = 0.5;
//        radius2 = 1.5;
        tangent1Vec[pointIdx] = pcl::PointXYZ(tangent1Vec[pointIdx].x / radius1, tangent1Vec[pointIdx].y / radius1, tangent1Vec[pointIdx].z / radius1);
        tangent2Vec[pointIdx] = pcl::PointXYZ(tangent2Vec[pointIdx].x / radius2, tangent2Vec[pointIdx].y / radius2, tangent2Vec[pointIdx].z / radius2);
//        (*cloud)[pointIdx].curvature = radius;

auto bla = Util::vectorLength(tangent1Vec[pointIdx]);


        // discard points
        auto neighbourhoodDistances = pointNeighbourhoodsDistance[pointIdx];
//        int randR = rand() % (255 - 0 + 1) + 0;
//        int randG = rand() % (255 - 0 + 1) + 0;
//        int randB = rand() % (255 - 0 + 1) + 0;
        // TODO temp lösung
        float radius = max(radius1, radius2);
        for (auto nIdx = 0; nIdx < neighbourhood.size(); nIdx++) {

            if (discardPoint[neighbourhood[nIdx]])
                continue;

            auto dist = neighbourhoodDistances[nIdx];

            // TODO komplexe winkel berechnung zum discarden
            // https://math.stackexchange.com/questions/76457/check-if-a-point-is-within-an-ellipse
            // rotate tangents and neighbour point to be parallel to xz-plane or something
            // ellipse center is c
            // then i can ignore y component
            // use: if ( (n.x - c.x)² / r1²  +  (n.z - c.z)² / r2²  <= 1 ) -> isInside
            // umformen: if ( r2² * (n.x - c.x)²  +  r1² * (n.z - c.z)² <= r1² * r2² -> isInside

            if (dist < alpha * radius) {
                discardPoint[neighbourhood[nIdx]] = true;
                // color debug - discarded points
                if (colorDiscarded) {
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
//            (*cloud)[pointIdx].r = 0;
//            (*cloud)[pointIdx].g = 255;
//            (*cloud)[pointIdx].b = 255;
        }
    }
}

// TODO die machen dann vor splat generation noch denoising
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
    float angleThreshold = 0.6; // ~30° TODO copied from splat generation
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        const auto& point = (*cloud)[pointIdx];
        // only for splats
        if (tangent1Vec[pointIdx].x == 0 && tangent1Vec[pointIdx].y == 0 && tangent1Vec[pointIdx].z == 0) { // for elliptical splats
            continue;
        }
//        auto bla = tangent1Vec[pointIdx];
//        auto blaNorm = Util::normalize(bla);
//        blaNorm.x = blaNorm.x / avgRadiusNeighbourhoods;
//        blaNorm.y = blaNorm.y / avgRadiusNeighbourhoods;
//        blaNorm.z = blaNorm.z / avgRadiusNeighbourhoods;
//        tangent1Vec[pointIdx] = blaNorm;
//        auto bla2 = tangent2Vec[pointIdx];
//        auto blaNorm2 = Util::normalize(bla2);
//        blaNorm2.x = blaNorm2.x / avgRadiusNeighbourhoods;
//        blaNorm2.y = blaNorm2.y / avgRadiusNeighbourhoods;
//        blaNorm2.z = blaNorm2.z / avgRadiusNeighbourhoods;
//        tangent2Vec[pointIdx] = blaNorm2;
        // TODO vllt oben speichern für splats?
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
//                    if (tangent1Vec[nPointIdx].x == 0 && tangent1Vec[nPointIdx].y == 0 && tangent1Vec[nPointIdx].z == 0) { // for elliptical splats
//                        continue;
//                    }
                    // found the farthest splat

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
                    // TODO braucht noch normale, tangente1 und 2, radii
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

pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr
DataStructure::kdTreePcaNormalEstimation(const uint32_t& startIdx, const uint32_t& endIdx) { // TODO use indices

    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start normal calculation" << std::endl;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    ne.setViewPoint(0.0f, 100000000.0f, 750000.0f);

    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    ne.setSearchMethod(tree);

    ne.setKSearch(16);

    // Compute the features
    ne.compute(*cloud);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal estimation in " << duration.count() << "s" << std::endl;


    // compute radii
    start = std::chrono::high_resolution_clock::now();
    for (auto it = cloud->points.begin(); it != cloud->points.end(); it++) {
        const auto& bla = *it;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        tree->radiusSearch(bla, 1.5, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        auto const count = static_cast<float>(pointRadiusSquaredDistance.size());

        auto diff = std::max((count - 7.0f), 0.0f);

        auto avg = std::reduce(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end() - diff) /
                   (count - diff);
        (*it).curvature = avg;

    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished radius calculation in " << duration.count() << "s" << std::endl;

    return tree;
}


// TODO später in util

//Vertex DataStructure::getUTMForOpenGL(Vertex *vertexOpenGL) {
//    // TODO offset is float, losing precision
//    return Vertex{vertexOpenGL->x + xOffset, vertexOpenGL->y + yOffset, vertexOpenGL->z + zOffset};
//}
//
//Vertex DataStructure::getWGSForOpenGL(Vertex *vertex) {
//    // TODO offset is float, losing precision
//
//    // wert in utm holen, dann:
//
//    // zone number: 60 zones, each 6 degrees of longitude (horizontal stripes), number is consistent in horizontal stripes
//    // zone letter: 20 zones, each 8 degrees of latitude (vertical stripes), letter is consistent in vertical stripes
//    // x wert zwischen 100.000 und 899.999 meter in zone
//    // y wert ist entfernugn vom äquator (zumindest auf nordhalbkugel)
//    return Vertex();
//}

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