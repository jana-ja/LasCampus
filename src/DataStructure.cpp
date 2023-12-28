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
#include <algorithm>
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


    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);


    int k = 40;
    std::vector<pcl::Indices> pointNeighbourhoods(cloud->points.size());
    std::vector<vector<float>> pointNeighbourhoodsDistance(cloud->points.size());

    // ********** knn and compute avgRadius **********
    float avgRadiusSumNeighbourhoods = 0;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        pcl::Indices neighboursPointIdx(k);
        std::vector<float> neighboursSquaredDistance;
        if (tree->nearestKSearch(pointIdx, k, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca
            // i need not squared distance
            for (auto& item: neighboursSquaredDistance){
                item = sqrt(item);
            }

            auto const count = static_cast<float>(neighboursSquaredDistance.size());
            auto avgRadius = std::reduce(neighboursSquaredDistance.begin(), neighboursSquaredDistance.end()) / count;
            avgRadiusSumNeighbourhoods += avgRadius;

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursSquaredDistance;
        } else {// TODO was mach ich bei else?? dann hab ich an dem index bei pointNeighbourhoods nichts gesetzt
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }
    }
    float avgRadiusNeighbourhoods = avgRadiusSumNeighbourhoods / cloud->points.size();

    // ********** get neighbourhood with radius and pca normal **********
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
                // TODO destroy pointNeighbourhoodsDistance here?

                // compute avg unsigned point to plane distance for splat grow epsilon
                float uPtpDistSum = 0;
                for (auto nPointIdx = 0; nPointIdx < neighbours.size(); nPointIdx++) {
                    auto ppd = pointPlaneDistance((*cloud)[neighbours[nPointIdx]], (*cloud)[pointIdx]);
                    uPtpDistSum += ppd;
                }
                float uPtpDistAvg = uPtpDistSum / neighbours.size();
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
                // TODO what?
                pointNeighbourhoods[pointIdx] = pcl::Indices();
                // color debug
                (*cloud)[pointIdx].r = 255;
                (*cloud)[pointIdx].g = 255;
                (*cloud)[pointIdx].b = 255;
            }
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
        }
    }

    // ********** compute epsilon **********
    float splatGrowEpsilon = uPtpDistSumNeighbourhoods / pointNeighbourhoods.size();

    // ********** normal orientation **********
    float wallThreshold = 1.0;

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
            tree->radiusSearch(mid, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            if (pointIdxRadiusSearch.size() != 0) {

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
                    // TODO check if normal is waagerecht
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




    // ********** compute splats **********
    float alpha = 0.2;
    std::vector<bool> discardPoint(cloud->points.size());
    fill(discardPoint.begin(), discardPoint.end(), false);

    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

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

        for (auto nIdx = 0; nIdx < neighbourhood.size(); nIdx++) {

            if (!discardPoint[neighbourhood[nIdx]]) {
                auto eps = signedPointPlaneDistance(point, cloud->points[neighbourhood[nIdx]], normal);
                epsilonSum += eps;
                epsilonCount++;
                lastEpsilonNeighbourIdx = nIdx;


                if (abs(eps) > splatGrowEpsilon) {
                    // stop growing this neighbourhood
                    // point nIdx does NOT belong to neighbourhood
                    break;
                }
            }
        }

        if (epsilonSum == 0) {
            // no valid neighbours (all have been discarded)
            float radius = 0;
            (*cloud)[pointIdx].curvature = radius;
            continue;
            // TODO regelmäßige streifen in der pointcoud bekommen keine farbe, werden also hier continued, warum?

        }

        // compute avg of the epsilons
        float epsilonAvg = epsilonSum / (lastEpsilonNeighbourIdx + 1);

        // move splat point
        (*cloud)[pointIdx].x += epsilonAvg * normal.x;
        (*cloud)[pointIdx].y += epsilonAvg * normal.y;
        (*cloud)[pointIdx].z += epsilonAvg * normal.z;


        // compute splat radius
        const auto& lastNeighbourPoint = cloud->points[neighbourhood[lastEpsilonNeighbourIdx]]; // TODO out of bounds check
        auto pointToNeighbourVec = vectorSubtract(lastNeighbourPoint, point);
        auto bla = dotProduct(normal, pointToNeighbourVec);
        pcl::PointXYZ rightSide;
        rightSide.x = bla * normal.x;
        rightSide.y = bla * normal.y;
        rightSide.z = bla * normal.z;
        float radius = vectorLength(vectorSubtract(pointToNeighbourVec, rightSide));
        (*cloud)[pointIdx].curvature = radius;

        // TODO discard all neighbours in alpha * radius from splat generation
        for (auto nIdx = 0; nIdx < neighbourhood.size(); nIdx++) {

            auto eps = signedPointPlaneDistance(point, cloud->points[neighbourhood[nIdx]], normal);
            if (eps < alpha * radius) {
                discardPoint[neighbourhood[nIdx]] = true;
            }

        }

        // color debug
        int randR = rand() % (255 - 0 + 1) + 0;
        int randG = rand() % (255 - 0 + 1) + 0;
        int randB = rand() % (255 - 0 + 1) + 0;
        (*cloud)[pointIdx].r = randR;
        (*cloud)[pointIdx].g = randG;
        (*cloud)[pointIdx].b = randB;

    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished ada in " << duration.count() << "s" << std::endl;

    // TODO im datensatz de intensity der punkte ansehen ob ich darüber was filtern kann?= zB bäume raus



    // splatting nach linsen et al [26]
    // point cloud mit punkten pi, N ist anzahl der punkte
    // R ist avg radius of points in knn of every point, k=40
    // Npi ist smallest neighbourhood of pi zwischen knn und radius R.
    // auf dieser nachbarschaft pca machen um normale zu bekommen, reorientieren (die machen mit lidar sensor posi)

    // splats bauen: Si hat center normale und radius
    // init center und normale von punkt pi und radius = 0
    // radius erhöhen: nach und nach punkte (nach distanz aufsteigend) zu Npi hinzufügen bis signed point-to-plane distance von nächstem kandidat punkt > epsilon ist
    // splat center anpassen: entlang der normale bewegen um avg signed point-to-plane distance der vewendeten nachbarn
    // radius setzen auf projected distance zu dem letzten hinzugefügten nachbar punkt
    // alle verwendeten punkte innerhalb von radius*alpha von splat generation ausschließen

    // vor generation durchschnitt unsigned point-to-plane distance von punkten in allen Npi ausrechnen, das ist error bound
    // dadurch bekommt man m splats die nen radius > 0 haben. m << N
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
                                      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr treePtr) {
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

