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

DataStructure::DataStructure(const std::vector<std::string>& lasFiles, const std::string& shpFile) {

    // read shape file
    // TODO hard coded coordinates from current test las file
    double maxX = 7.415424;
    double maxY = 51.494428;
    double minX = 7.401340;
    double minY = 51.485245;

    std::string shpDir = ".." + PATH_SEPARATOR + "shp" + PATH_SEPARATOR;
    ShpDataIO shpIo = ShpDataIO(maxX, maxY, minX, minY);
    float xOffset2 = 389500;
    float zOffset2 = 5705500; // TODO get from las file
    shpIo.readShp(shpDir + shpFile, &buildings, xOffset2, zOffset2);


    // read las file
    std::string lasDir = ".." + PATH_SEPARATOR + "las" + PATH_SEPARATOR;
    LasDataIO lasIo = LasDataIO();

    uint32_t startIdx = 0;
    uint32_t endIdx;
    uint32_t pointCount = 0;//; // TODO remove
    std::cout << TAG << "begin loading data" << std::endl;
    for (const auto& file: lasFiles) {

        // get points
        lasIo.readLas(lasDir + file, cloud, &pointCount, xOffset, yOffset, zOffset);
//        lasIo.random(cloud);

        endIdx = pointCount;

        // get normals
        std::string normalFile = file;
        normalFile.replace(normalFile.end() - 3, normalFile.end(), "features");
        adaSplats();
//        if (!lasIo.readFeaturesFromCache(lasDir + normalFile, cloud, startIdx, endIdx)) {
//            auto treePtr = kdTreePcaNormalEstimation(startIdx, endIdx);
//            normalOrientation(startIdx, endIdx, treePtr);
//            lasIo.writeFeaturesToCache(lasDir + normalFile, cloud, startIdx, endIdx);
//        }

        startIdx += pointCount;
    }
    std::cout << TAG << "loading data successful" << std::endl;
}


void DataStructure::adaSplats() {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start ada" << std::endl;

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);

    int k = 40;
    std::vector<pcl::Indices> pointNeighbourhoods(cloud->points.size());
    std::vector<vector<float>> pointNeighbourhoodsDistance(cloud->points.size());

    // ********** knn and compute avgRadius **********
    float avgRadiusNeighbourhoods = adaKnnAndAvgRadius(k, tree, pointNeighbourhoods, pointNeighbourhoodsDistance);
//    avgRadiusNeighbourhoods *= 3;

    // ********** get neighbourhood with radius and pca normal **********
    float splatGrowEpsilon = adaNeigbourhoodsAndNormals(avgRadiusNeighbourhoods, pointNeighbourhoods,
                                                        pointNeighbourhoodsDistance);

    // ********** normal orientation **********
    float wallThreshold = 1.0;
    adaNormalOrientation(wallThreshold, tree);

    // ********** compute splats **********
    float alpha = 0.2;
    adaComputeSplats(alpha, splatGrowEpsilon, pointNeighbourhoods, pointNeighbourhoodsDistance);


    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished ada in " << duration.count() << "s" << std::endl;

    // TODO im datensatz de intensity der punkte ansehen ob ich darüber was filtern kann?= zB bäume raus

    for (int pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        const auto& point = (*cloud)[pointIdx];
        if(point.curvature != 0 && (*cloud)[pointIdx].r == 255){
            int k = 3;
        } // 7, 18, 20
    }

}

float
DataStructure::adaKnnAndAvgRadius(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree,
                                  std::vector<pcl::Indices>& pointNeighbourhoods,
                                  std::vector<std::vector<float>>& pointNeighbourhoodsDistance) {

    // ********** knn and compute avgRadius **********
    float avgRadiusSumNeighbourhoods = 0;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        pcl::Indices neighboursPointIdx(k);
        auto neighboursSquaredDistance = std::vector<float>(k);
        if (tree->nearestKSearch(pointIdx, k, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca
            // i need not squared distance
//            for (auto& item: neighboursSquaredDistance){
//                item = sqrt(item);
//            }

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
    return avgRadiusSumNeighbourhoods / static_cast<float>(cloud->points.size());
}

float
DataStructure::adaNeigbourhoodsAndNormals(float avgRadiusNeighbourhoods, std::vector<pcl::Indices>& pointNeighbourhoods,
                                          std::vector<std::vector<float>>& pointNeighbourhoodsDistance) {

    float uPtpDistSumNeighbourhoods = 0;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

        if (pointNeighbourhoods[pointIdx].size() >= 3) { // need at least 3 points for pca

            // define Npi as all points here from knn search but within avg radius?
            // find border index
            int lastNeighbour = findIndex(avgRadiusNeighbourhoods, pointNeighbourhoodsDistance[pointIdx]);
            // do stuff
            auto neighbours = pcl::Indices(pointNeighbourhoods[pointIdx].begin(),
                                           pointNeighbourhoods[pointIdx].begin() + lastNeighbour);
            if (neighbours.size() >= 3) {
                pcl::IndicesPtr neighboursPtr = make_shared<pcl::Indices>(
                        neighbours); //pcl::Indices(neighboursPointIdx.begin(), neighboursPointIdx.begin() + lastNeighbour));// = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                // pca on the neighbours
                pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
                pca.setInputCloud(cloud);
                pca.setIndices(neighboursPtr);
                Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
                Eigen::Vector3f eigenValues = pca.getEigenValues();
                cloud->points[pointIdx].normal_x = eigenVectors(0, 2);
                cloud->points[pointIdx].normal_y = eigenVectors(1, 2);
                cloud->points[pointIdx].normal_z = eigenVectors(2, 2);

                pointNeighbourhoods[pointIdx] = neighbours;

                // compute avg unsigned point to plane distance for splat grow epsilon
                float uPtpDistSum = 0;
                // first neighbour is the point itself
                auto bla2 = std::vector<float>(neighbours.size());
                for (auto nPointIdx = 1; nPointIdx < neighbours.size(); nPointIdx++) {
                    auto ppd = pointPlaneDistance((*cloud)[neighbours[nPointIdx]], (*cloud)[pointIdx]);
                    uPtpDistSum += ppd;

                    bla2[nPointIdx] = ppd;
                }
                float uPtpDistAvg = uPtpDistSum / static_cast<float>(neighbours.size() - 1);
                uPtpDistSumNeighbourhoods += uPtpDistAvg;

                const auto& point = cloud->points[pointIdx];
                auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                auto vertLen = abs(point.normal_y);
                if (horLen < vertLen) {
                    // check if vertical normal is oriented up
                    if (point.normal_y < 0) {
                        (*cloud)[pointIdx].normal_x *= -1;
                        (*cloud)[pointIdx].normal_y *= -1;
                        (*cloud)[pointIdx].normal_z *= -1;
                    }
                }
            } else {
                pointNeighbourhoods[pointIdx] = pcl::Indices();
//                // color debug - less then 3 in neighbourhood
//                (*cloud)[pointIdx].r = 255;
//                (*cloud)[pointIdx].g = 255;
//                (*cloud)[pointIdx].b = 255;
            }
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
        }
    }
    // ********** compute epsilon **********
    return uPtpDistSumNeighbourhoods / static_cast<float>(pointNeighbourhoods.size());
}

void DataStructure::adaNormalOrientation(float wallThreshold, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree) {

    for (auto building: buildings) {
        // get point index of next part/ring if there are more than one, skip "walls" which connect different parts
        auto partIdx = building.parts.begin();
        uint32_t nextPartIndex = *partIdx;
        partIdx++;
        if (partIdx != building.parts.end()) {
            nextPartIndex = *partIdx;
        }
        for (auto pointIdx = 0; pointIdx < building.points.size() - 1; pointIdx++) {

            // if reached end of part/ring -> skip this "wall"
            if (pointIdx + 1 == nextPartIndex) {
                partIdx++;
                if (partIdx != building.parts.end()) {
                    nextPartIndex = *partIdx;
                }
                continue;
            }

            float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
            float ground = -38; // minY // TODO boden ist wegen opengl offset grad bei -38
            pcl::PointXYZRGBNormal wallPoint1, wallPoint2;
            wallPoint1.x = static_cast<float>(building.points[pointIdx].x);
            wallPoint1.y = ground + wallHeight;
            wallPoint1.z = static_cast<float>(building.points[pointIdx].z);
            wallPoint2.x = static_cast<float>(building.points[pointIdx + 1].x);
            wallPoint2.y = ground + wallHeight;
            wallPoint2.z = static_cast<float>(building.points[pointIdx + 1].z);

            // detect (and color) alle points on this wall
            pcl::PointXYZRGBNormal mid;
            mid.x = (wallPoint1.x + wallPoint2.x) / 2;
            mid.y = (ground + wallHeight) / 2;
            mid.z = (wallPoint1.z + wallPoint2.z) / 2;


            Plane wallPlane = {wallPoint1, wallPoint2, mid};

            auto r = static_cast<float>(sqrt(
                    pow(wallPoint2.x - mid.x, 2) + pow(wallHeight - mid.y, 2) + pow(wallPoint2.z - mid.z, 2)));

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            tree->radiusSearch(mid, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            if (!pointIdxRadiusSearch.empty()) {

                auto minX = min(wallPoint1.x, wallPoint2.x);
                auto maxX = max(wallPoint1.x, wallPoint2.x);
                auto minZ = min(wallPoint1.z, wallPoint2.z);
                auto maxZ = max(wallPoint1.z, wallPoint2.z);

                for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {
                    // TODO implement direct calculation test if point lies inside wall rectangle

                    const auto& point = (*cloud)[*nIdxIt];

                    if (pointPlaneDistance(cloud->points[*nIdxIt], wallPlane) > wallThreshold) {
                        continue;
                    }
                    if (point.x > maxX || point.x < minX || point.z > maxZ || point.z < minZ) {
                        continue;
                    }

                    pcl::PointXYZRGBNormal normalPoint;
                    normalPoint.x = point.x + point.normal_x;
                    normalPoint.y = point.y + point.normal_y;
                    normalPoint.z = point.z + point.normal_z;
                    // check if normal is horizontal
                    auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                    auto vertLen = point.normal_y;
                    if (horLen > vertLen) {
                        if (signedPointPlaneDistance(normalPoint, wallPlane) < 0) {
                            (*cloud)[*nIdxIt].normal_x *= -1;
                            (*cloud)[*nIdxIt].normal_y *= -1;
                            (*cloud)[*nIdxIt].normal_z *= -1;
                        }
                    }
                }
            }
        }
    }
}

void
DataStructure::adaComputeSplats(float alpha, float splatGrowEpsilon, std::vector<pcl::Indices>& pointNeighbourhoods,
                                std::vector<std::vector<float>>& pointNeighbourhoodsDistance) {
    // TODO problem: some discarded points become splats (check),
    //  some points dont have valid neighbourhoods that should have one (check)

    std::vector<bool> discardPoint(cloud->points.size());
    fill(discardPoint.begin(), discardPoint.end(), false);

    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

        if (discardPoint[pointIdx]) {
            continue;
        }

        auto const& point = cloud->points[pointIdx];
        auto const& neighbourhood = pointNeighbourhoods[pointIdx];

        (*cloud)[pointIdx].curvature = 0; // this is radius
        float epsilonSum = 0;
        int epsilonCount = 0;
        int lastEpsilonNeighbourIdx = 0;
        pcl::PointXYZ normal;
        normal.x = point.normal_x;
        normal.y = point.normal_y;
        normal.z = point.normal_z;

        for (auto nIdx = 1; nIdx < neighbourhood.size(); nIdx++) {

            if (discardPoint[neighbourhood[nIdx]]) {
                continue;
            }

            auto eps = signedPointPlaneDistance(point, cloud->points[neighbourhood[nIdx]], normal);
            if (abs(eps) > splatGrowEpsilon) {
                // stop growing this neighbourhood
                // point nIdx does NOT belong to neighbourhood
//                    if (nIdx == 1){
//                        (*cloud)[pointIdx].r = 0;
//                        (*cloud)[pointIdx].g = 255;
//                        (*cloud)[pointIdx].b = 0;
//                    }
                break;
            }

            epsilonSum += eps;
            epsilonCount++;
            lastEpsilonNeighbourIdx = nIdx;

        }

        if (epsilonCount == 0) {
            // no valid neighbours (all have been discarded or nearest neighbours eps dist is too big)
//            if((*cloud)[pointIdx].g != 255) {
//                (*cloud)[pointIdx].r = 255;
//                (*cloud)[pointIdx].g = 0;
//                (*cloud)[pointIdx].b = 0;
//            }
            continue;
        }

        // compute avg of the epsilons
        float epsilonAvg = epsilonSum / static_cast<float>(lastEpsilonNeighbourIdx);

        // move splat point
        (*cloud)[pointIdx].x += epsilonAvg * normal.x;
        (*cloud)[pointIdx].y += epsilonAvg * normal.y;
        (*cloud)[pointIdx].z += epsilonAvg * normal.z;


        // compute splat radius
        const auto& lastNeighbourPoint = cloud->points[neighbourhood[lastEpsilonNeighbourIdx]];
        auto pointToNeighbourVec = vectorSubtract(lastNeighbourPoint, point);
        auto bla = dotProduct(normal, pointToNeighbourVec);
        pcl::PointXYZ rightSide;
        rightSide.x = bla * normal.x;
        rightSide.y = bla * normal.y;
        rightSide.z = bla * normal.z;
        float radius = vectorLength(vectorSubtract(pointToNeighbourVec, rightSide));
        (*cloud)[pointIdx].curvature = radius;


        // discard points
        auto neighbourhoodDistances = pointNeighbourhoodsDistance[pointIdx];
//        int randR = rand() % (255 - 0 + 1) + 0;
//        int randG = rand() % (255 - 0 + 1) + 0;
//        int randB = rand() % (255 - 0 + 1) + 0;
        for (auto nIdx = 0; nIdx < neighbourhood.size(); nIdx++) {

            if(discardPoint[neighbourhood[nIdx]])
                continue;

            auto dist = neighbourhoodDistances[nIdx];
            if (dist < alpha * radius) {
                discardPoint[neighbourhood[nIdx]] = true;
                // color debug - discarded points
                if (nIdx != 0) {
                    (*cloud)[neighbourhood[nIdx]].r = 255;
                    (*cloud)[neighbourhood[nIdx]].g = 255;
                    cloud->points[neighbourhood[nIdx]].b = 0;
                }
            } else {
                // dist values are ascending
                break;
            }
        }

        // TODO problem mit squared distances?
        // TODO einen splat einzeln ansehen am pc radius undso ob 20 prozent

        // color debug - random color for every point
//        int randR = rand() % (255 - 0 + 1) + 0;
//        int randG = rand() % (255 - 0 + 1) + 0;
//        int randB = rand() % (255 - 0 + 1) + 0;
        (*cloud)[pointIdx].r = 0;
        (*cloud)[pointIdx].g = 255;
        (*cloud)[pointIdx].b = 255;

    }
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

ShpDataIO::Point DataStructure::getUtmForWgs(ShpDataIO::Point wgsPoint) {
    return wgsPoint;
};

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

pcl::PointXYZRGBNormal* DataStructure::getVertices() {
    return cloud->data();// vertices.data();
}

void DataStructure::normalOrientation(const uint32_t& startIdx, const uint32_t& endIdx,
                                      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& treePtr) {
    // TODO use buildings to detect right normal orientation
    //  maybe also segmentation stuff?
    float thresholdDelta = 1.0f; // TODO find good value

    for (auto building: buildings) {
        // get point index of next part/ring if there are more than one, skip "walls" which connect different parts
        auto partIdx = building.parts.begin();
        uint32_t nextPartIndex = *partIdx;
        partIdx++;
        if (partIdx != building.parts.end()) {
            nextPartIndex = *partIdx;
        }
        for (auto pointIdx = 0; pointIdx < building.points.size() - 1; pointIdx++) {
//            std::cout << "yrah " << pointIdx << std::endl;

//            const auto& wallPoint1 = building.points[pointIdx];
//            const auto& wallPoint2 = building.points[pointIdx + 1];

            // if reached end of part/ring -> skip this "wall"
            if (pointIdx + 1 == nextPartIndex) {
                partIdx++;
                if (partIdx != building.parts.end()) {
                    nextPartIndex = *partIdx;
                }
                continue;
            }

            float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
            float ground = -38; // minY // TODO boden ist wegen opengl offset grad bei -38
            pcl::PointXYZRGBNormal wallPoint1, wallPoint2;
            wallPoint1.x = building.points[pointIdx].x;
            wallPoint1.y = ground + wallHeight;
            wallPoint1.z = building.points[pointIdx].z;
            wallPoint2.x = building.points[pointIdx + 1].x;
            wallPoint2.y = ground + wallHeight;
            wallPoint2.z = building.points[pointIdx + 1].z;

            // detect (and color) alle points on this wall
            pcl::PointXYZRGBNormal mid;
            mid.x = (wallPoint1.x + wallPoint2.x) / 2;
            mid.y = (ground + wallHeight) / 2;
            mid.z = (wallPoint1.z + wallPoint2.z) / 2;


            Plane wallPlane = {wallPoint1, wallPoint2, mid};

            // p2 = wp2.x wallHeight wp2.z
            // p1 = mid
            float r = sqrt(pow(wallPoint2.x - mid.x, 2) + pow(wallHeight - mid.y, 2) + pow(wallPoint2.z - mid.z, 2));

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            treePtr->radiusSearch(mid, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            if (pointIdxRadiusSearch.size() != 0) {

                int randR = rand() % (255 - 0 + 1) + 0;
                int randG = rand() % (255 - 0 + 1) + 0;
                int randB = rand() % (255 - 0 + 1) + 0;


//                std::cout << "yrah " << pointIdxRadiusSearch.size() << std::endl;

                for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {
                    // TODO implement direct calculation test if point lies inside wall rectangle
//                    if(building.points.size() == 5){
//                        randR = 255;
//                        randG = 0;
//                        randB = 0;
//                    }
                    const auto& point = (*cloud)[*nIdxIt];

                    if (pointPlaneDistance(cloud->points[*nIdxIt], wallPlane) > thresholdDelta) {
                        continue;
                    }
                    auto minX = min(wallPoint1.x, wallPoint2.x);
                    auto maxX = max(wallPoint1.x, wallPoint2.x);
                    auto minZ = min(wallPoint1.z, wallPoint2.z);
                    auto maxZ = max(wallPoint1.z, wallPoint2.z);

                    const auto& x = point.x;
                    const auto& z = point.z;
                    if (x > maxX || x < minX || z > maxZ || z < minZ) {
                        continue;
                    }
                    const auto& y = point.y;

                    cloud->points[*nIdxIt].b = randB;
                    (*cloud)[*nIdxIt].g = randG;
                    (*cloud)[*nIdxIt].r = randR;

                    pcl::PointXYZRGBNormal normalPoint;
                    normalPoint.x = point.x + point.normal_x;
                    normalPoint.y = point.y + point.normal_y;
                    normalPoint.z = point.z + point.normal_z;
                    // TODO check if normal is waagerecht
                    auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                    auto vertLen = point.normal_y;
                    if (horLen > vertLen) {
                        if (signedPointPlaneDistance(normalPoint, wallPlane) < 0) { // TODO check sign richtig
                            (*cloud)[*nIdxIt].normal_x *= -1;
                            (*cloud)[*nIdxIt].normal_y *= -1;
                            (*cloud)[*nIdxIt].normal_z *= -1;
                        }
                    }

                }
                // TODO filter points, keep only those that belong to the wall
                //  check distance to wall plane first
                //  then check if point is on wall part of plane
            }

        }
    }
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
