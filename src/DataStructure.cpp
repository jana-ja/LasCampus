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

    // preprocess buildings to walls
    float resolution = 8.0f; // TODO find good value
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> wallOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>(
            resolution);
    float maxWallRadius = preprocessWalls(wallOctree);

    // read las file
    std::string lasDir = ".." + PATH_SEPARATOR + "las" + PATH_SEPARATOR;
    LasDataIO lasIo = LasDataIO();

    uint32_t startIdx = 0;
    uint32_t endIdx;
    uint32_t pointCount = 0;//; // TODO remove
    std::cout << TAG << "begin loading data" << std::endl;
    for (const auto& file: lasFiles) {

        // get points
        lasIo.readLas(lasDir + file, cloud, &pointCount, xOffset, yOffset, zOffset, walls, wallOctree, maxWallRadius);
//        lasIo.random(cloud);

        endIdx = pointCount;

        // get normals
        std::string normalFile = file;
        normalFile.replace(normalFile.end() - 3, normalFile.end(), "features");
//        adaSplats();
//        if (!lasIo.readFeaturesFromCache(lasDir + normalFile, cloud, startIdx, endIdx)) {
//            auto treePtr = kdTreePcaNormalEstimation(startIdx, endIdx);
//            normalOrientation(startIdx, endIdx, treePtr);
//            lasIo.writeFeaturesToCache(lasDir + normalFile, cloud, startIdx, endIdx);
//        }

        startIdx += pointCount;
    }
    std::cout << TAG << "loading data successful" << std::endl;
}

float DataStructure::preprocessWalls(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree) {
    // preprocessing of buildings
    // save all walls (min, mid, max point & radius)
    // dann beim normalen orientieren  spatial search nach mid point mit max radius von allen walls
    float maxR = 0;
    for (auto building: buildings) {
        // get point index of next part/ring if there are more than one, skip "walls" which connect different parts
        auto partIdx = building.parts.begin();
        uint32_t nextPartIndex = *partIdx;
        partIdx++;
        if (partIdx != building.parts.end()) {
            nextPartIndex = *partIdx;
        }
        // for all walls
        float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
        float ground = -38; // minY // TODO boden ist wegen opengl offset grad bei -38
        for (auto pointIdx = 0; pointIdx < building.points.size() - 1; pointIdx++) {

            // if reached end of part/ring -> skip this "wall"
            if (pointIdx + 1 == nextPartIndex) {
                partIdx++;
                if (partIdx != building.parts.end()) {
                    nextPartIndex = *partIdx;
                }
                continue;
            }

            Wall wall;


            pcl::PointXYZRGBNormal wallPoint1, wallPoint2;
            wallPoint1.x = building.points[pointIdx].x;
            wallPoint1.y = ground;
            wallPoint1.z = building.points[pointIdx].z;
            wallPoint2.x = building.points[pointIdx + 1].x;
            wallPoint2.y = ground + wallHeight;
            wallPoint2.z = building.points[pointIdx + 1].z;

            // detect (and color) alle points on this wall
            wall.mid.x = (wallPoint1.x + wallPoint2.x) / 2;
            wall.mid.y = (ground + wallHeight) / 2;
            wall.mid.z = (wallPoint1.z + wallPoint2.z) / 2;

            auto vec1 = vectorSubtract(wallPoint1, wallPoint2);
            auto vec2 = vectorSubtract(wallPoint1, wall.mid);
            auto planeNormal = normalize(crossProduct(vec1, vec2));
            wall.mid.normal_x = planeNormal.x;
            wall.mid.normal_y = planeNormal.y;
            wall.mid.normal_z = planeNormal.z;

            float r = sqrt(pow(wallPoint2.x - wall.mid.x, 2) + pow(wallHeight - wall.mid.y, 2) + pow(wallPoint2.z - wall.mid.z, 2));
            if (r > maxR) {
                maxR = r;
            }

            wall.minX = min(wallPoint1.x, wallPoint2.x);
            wall.maxX = max(wallPoint1.x, wallPoint2.x);
            wall.minZ = min(wallPoint1.z, wallPoint2.z);
            wall.maxZ = max(wallPoint1.z, wallPoint2.z);

            walls.push_back(wall);
            wallMidPoints->push_back(wall.mid);


        }
    }

    wallOctree.setInputCloud(wallMidPoints);
    wallOctree.defineBoundingBox();
    wallOctree.addPointsFromInputCloud();

    return maxR;
}

void DataStructure::adaSplats() {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start ada" << std::endl;

    tangent1Vec = std::vector<pcl::PointXYZ>((*cloud).size());
    tangent2Vec = std::vector<pcl::PointXYZ>((*cloud).size());

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);

    int k = 40;
    std::vector<pcl::Indices> pointNeighbourhoods(cloud->points.size());
    std::vector<vector<float>> pointNeighbourhoodsDistance(cloud->points.size());
    std::vector<int> pointClasses(cloud->points.size());
    fill(pointClasses.begin(), pointClasses.end(), 2);

    // ********** knn and compute avgRadius **********
    float avgRadiusNeighbourhoods = adaKnnAndRadius(k, tree, pointNeighbourhoods, pointNeighbourhoodsDistance);
    std::cout << TAG << "avg radius R is: " << avgRadiusNeighbourhoods << std::endl;

    // ********** get neighbourhood with radius, use pca for classification and compute epsilon **********
    float splatGrowEpsilon = adaNeighbourhoodsClassificationAndEpsilon(avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);
    std::cout << TAG << "splat grow epsilon is: " << splatGrowEpsilon << std::endl;

    // ********** get new neighbourhood with new k and new radius (depending on classification) and use pca for normal **********
    adaNewNeighbourhoods(k, tree, avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);

    // ********** normal orientation **********
    float wallThreshold = 1.0;
    adaNormalOrientation(wallThreshold, tree);

    // ********** compute splats **********
    float alpha = 0.2;
    adaComputeSplats(alpha, splatGrowEpsilon, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);


    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished ada in " << duration.count() << "s" << std::endl;

    // TODO im datensatz de intensity der punkte ansehen ob ich darüber was filtern kann?= zB bäume raus

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
                                                    std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses){
    float uPtpDistSumNeighbourhoods = 0;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

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

                switch (mainDings) {
                    case 0:
                        pointClasses[pointIdx] = 0;
                        // linearity is main
                        if (colorClasses) {
                            (*cloud)[pointIdx].r = 255;
                            (*cloud)[pointIdx].g = 0;
                            (*cloud)[pointIdx].b = 0;
                        }
                        break;
                    case 1:
                        pointClasses[pointIdx] = 1;
                        // planarity is main
                        if (colorClasses) {
                            (*cloud)[pointIdx].r = 0;
                            (*cloud)[pointIdx].g = 255;
                            (*cloud)[pointIdx].b = 0;
                        }
                        break;
                    default:
                        pointClasses[pointIdx] = 2;
                        // sphericity is main
                        if (colorClasses) {
                            (*cloud)[pointIdx].r = 0;
                            (*cloud)[pointIdx].g = 0;
                            (*cloud)[pointIdx].b = 255;
                        }
                        break;
                }

                // splat grow epsilon
                // compute avg unsigned point to plane distance for splat grow epsilon
                float uPtpDistSum = 0;
                // first neighbour is the point itself
                auto bla2 = std::vector<float>(neighbourhood.size());
                for (auto nPointIdx = 1; nPointIdx < neighbourhood.size(); nPointIdx++) {
                    auto ppd = pointPlaneDistance((*cloud)[neighbourhood[nPointIdx]], (*cloud)[pointIdx]);
                    uPtpDistSum += ppd;

                    bla2[nPointIdx] = ppd;
                }
                float uPtpDistAvg = uPtpDistSum / static_cast<float>(neighbourhood.size() - 1);
                uPtpDistSumNeighbourhoods += uPtpDistAvg;

            } else {
                tangent1Vec[pointIdx] = pcl::PointXYZ(0,0,0);
                tangent2Vec[pointIdx] = pcl::PointXYZ(0,0,0);
            }
        } else {
            tangent1Vec[pointIdx] = pcl::PointXYZ(0,0,0);
            tangent2Vec[pointIdx] = pcl::PointXYZ(0,0,0);
        }
    }

    // ********** compute epsilon **********
    return uPtpDistSumNeighbourhoods / static_cast<float>(pointNeighbourhoods.size());
}

// TODO resampling? inspiration durch ada paper
// TODO datensatz filtern anschauen: bäume raus. nochmal den "schlechteren" datensatz ansehen, da müssten dann auch noch die autos undso rausgefiltert werden aber falls das nicht schwer ist wäre das vllt ne gute option weil aufgefüllte böden unter den gebäuden undso brauche ich ja eh nicht.
// TODO uv eliptical splats ansehen
// TODO das ada paper nochmal zu ende lesen und schauen ob noch was relevantes kommt

// farbwerte von jpeg2000 nehmen

void DataStructure::adaNewNeighbourhoods(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, float avgRadiusNeighbourhoods, std::vector<pcl::Indices>& pointNeighbourhoods,
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

            auto r = static_cast<float>(sqrt(pow(wallPoint2.x - mid.x, 2) + pow(wallHeight - mid.y, 2) + pow(wallPoint2.z - mid.z, 2)));

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

                            // also flip tangents
                            auto temp = tangent1Vec[*nIdxIt];
                            tangent1Vec[*nIdxIt] = tangent2Vec[*nIdxIt];
                            tangent2Vec[*nIdxIt] = temp;
                        }
                    }
                }
            }
        }
    }
}

void
DataStructure::adaComputeSplats(float alpha, float splatGrowEpsilon, std::vector<pcl::Indices>& pointNeighbourhoods,
                                std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses) {
    // 1 → 0°, 0.7 → 45°, 0 → 90°(pi/2). i guess: -0.7 → 135°, -1 → 180°(pi). (arccos kann nur zwischen 0° und 180° zeigen, richtung nicht beachtet)
    float angleThreshold = 0.86; // ~30°

    std::vector<bool> discardPoint(cloud->points.size());
    fill(discardPoint.begin(), discardPoint.end(), false);

    float currentSplatGrowEpsilon = splatGrowEpsilon;

    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

//        if (pointIdx % 1000 != 0){
//            continue;
//        }

        if (discardPoint[pointIdx]) {
            tangent1Vec[pointIdx] = pcl::PointXYZ(0,0,0);
            continue;
        }

        switch (pointClasses[pointIdx]) {
            case 0: // linearity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 2;//0.33;
                break;
            case 1: // planarity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 2;
                break;
            default: // sphericity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 0.25;
                break;
        }
// TODO vllt kann ich linearity punkte die eig ne fläche sein sollten einfach auch große nachbarschaft  geben und dann endet der splat wenn er auf ne fläche trifft??
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

        for (auto nIdx = 1; nIdx < neighbourhood.size(); nIdx++) {

            if (!growTangent1 && !growTangent2) {
                break;
            }

            // first check if neighbour point concerns tangent1 growth or tangent2 growth
            // project vector onto plane:
            auto neighbourVec = vectorSubtract(cloud->points[neighbourhood[nIdx]], point);
            auto projectedDistance = dotProduct(normal, neighbourVec);
            auto normalProjectedVec = pcl::PointXYZ(projectedDistance * normal.x, projectedDistance * normal.y, projectedDistance * normal.z);
            auto planeProjectedVec = vectorSubtract(neighbourVec, normalProjectedVec);
            // angle
            auto vecLen = vectorLength(planeProjectedVec);
            float cosAngle = dotProduct(tangent1Vec[pointIdx], planeProjectedVec) / vecLen;
            if (abs(cosAngle) > 0.7) { // 45°
                // concerns tangent1 -> less then 45° between tangent1 and projected neighbour vector
                concernsTangent1 = true;
                if(!growTangent1){
                    continue;
                }
            } else {
                concernsTangent1 = false;
                if(!growTangent2) {
                    continue;
                }
            }

            // TODO ich teste jetzt abbruchbedingungen auch bei discardeten punkten zu checken
            // stop growing when neighbour has different class
            if(pointClasses[pointIdx] != pointClasses[neighbourhood[nIdx]]){
                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            // stop growing when angle between point normal and neighbour normal is too big
            pcl::PointXYZ neighbourNormal;
            neighbourNormal.x = (*cloud)[neighbourhood[nIdx]].normal_x;
            neighbourNormal.y = (*cloud)[neighbourhood[nIdx]].normal_y;
            neighbourNormal.z = (*cloud)[neighbourhood[nIdx]].normal_z;

            float angle = dotProduct(normal, neighbourNormal);
            if (angle < angleThreshold){
                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            auto eps = signedPointPlaneDistance(point, cloud->points[neighbourhood[nIdx]], normal);
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
                if((*cloud)[pointIdx].g != 255) {
                    (*cloud)[pointIdx].r = 255;
                    (*cloud)[pointIdx].g = 0;
                    (*cloud)[pointIdx].b = 255;
                }
            }
            tangent1Vec[pointIdx] = pcl::PointXYZ(0,0,0);
            continue;
        }

        // compute avg of the epsilons
        float epsilonAvg = epsilonSum / (epsilonCount1 + epsilonCount2);

        // move splat point
        (*cloud)[pointIdx].x += epsilonAvg * normal.x;
        (*cloud)[pointIdx].y += epsilonAvg * normal.y;
        (*cloud)[pointIdx].z += epsilonAvg * normal.z;

        // compute splat radii
        // tangent 1
        const auto& lastNeighbourPoint1 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx1]];
        auto pointToNeighbourVec1 = vectorSubtract(lastNeighbourPoint1, point);
        auto bla1 = dotProduct(normal, pointToNeighbourVec1);
        auto  rightSide1 = pcl::PointXYZ(bla1 * normal.x, bla1 * normal.y, bla1 * normal.z);
        float radius1 = vectorLength(vectorSubtract(pointToNeighbourVec1, rightSide1));
        // tangent 2
        const auto& lastNeighbourPoint2 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx2]];
        auto pointToNeighbourVec2 = vectorSubtract(lastNeighbourPoint2, point);
        auto bla2 = dotProduct(normal, pointToNeighbourVec2);
        auto  rightSide2 = pcl::PointXYZ(bla2 * normal.x, bla2 * normal.y, bla2 * normal.z);
        float radius2 = vectorLength(vectorSubtract(pointToNeighbourVec2, rightSide2));
        // length of axes has to be 1/radius
        tangent1Vec[pointIdx] = pcl::PointXYZ(tangent1Vec[pointIdx].x / radius1, tangent1Vec[pointIdx].y / radius1, tangent1Vec[pointIdx].z / radius1);
        tangent2Vec[pointIdx] = pcl::PointXYZ(tangent2Vec[pointIdx].x / radius2, tangent2Vec[pointIdx].y / radius2, tangent2Vec[pointIdx].z / radius2);
//        (*cloud)[pointIdx].curvature = radius;


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

