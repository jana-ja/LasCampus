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
        if (!lasIo.readFeaturesFromCache(lasDir + normalFile, cloud, startIdx, endIdx)) {
            robustNormalEstimation(startIdx, endIdx);
//            auto treePtr = kdTreePcaNormalEstimation(startIdx, endIdx);
//            normalOrientation(startIdx, endIdx, treePtr);
            lasIo.writeFeaturesToCache(lasDir + normalFile, cloud, startIdx, endIdx);
        }

        startIdx += pointCount;
    }
    std::cout << TAG << "loading data successful" << std::endl;
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

        auto avg = std::reduce(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end() - diff) / (count - diff);
        (*it).curvature = avg;

    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished radius calculation in " << duration.count() << "s" << std::endl;

    return tree;
}

void DataStructure::robustNormalEstimation(const uint32_t& startIdx, const uint32_t& endIdx) { // TODO use indices
    // TODO wenn das wieder benutzt wird die normalen beim einlesen wieder alle auf (-1, -1, -1) setzen
    // TODO und die radii bestimmen und in curvature field vom punkt schreiben
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start octree" << std::endl;


    // find consistent neighborhoods

    // ****** OCTREE ******

    // minimum scale threshold - specified by referring to the sampling density of P
    // length of smallest voxel at lowest octree level
    float resolution = 8.0f; // TODO find good value
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>(
            resolution);

    // assign bounding box to octree
    octree.setInputCloud(cloud);
    octree.defineBoundingBox();
    // octree.defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ); // TODO get bounding box values, should be easy from las file data, for testing get while reading points from subset
    octree.addPointsFromInputCloud();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished octree in " << duration.count() << "s. " << "depth: " << octree.getTreeDepth()
              << std::endl;
    start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start normal calculation" << std::endl;


    // ****** NORMAL ESTIMATION ******

    vector<Neighborhood> consNeighborhoods;

    int okay[10]{0};
    int spur[10]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    // depth first traversal
    for (auto it = octree.breadth_begin(); it != octree.breadth_end(); it++) {

        // get bounds for voxel
        Eigen::Vector3f voxel_min, voxel_max;
        octree.getVoxelBounds(it, voxel_min, voxel_max);

        // calc voxel center from bounds
        pcl::PointXYZRGBNormal voxel_center;
        voxel_center.x = (voxel_max[0] + voxel_min[0]) / 2.0f;
        voxel_center.y = (voxel_max[1] + voxel_min[1]) / 2.0f;
        voxel_center.z = (voxel_max[2] + voxel_min[2]) / 2.0f;


        // calc search radius r from bounds
        float voxelSize = abs(voxel_max[0] - voxel_min[0]); // voxel is a cube
        float r = sqrt(2.0) * (voxelSize / 2.0);

        double minX, minY, minZ, maxX, maxY, maxZ;
        octree.getBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);

        // radius search
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (octree.radiusSearch(voxel_center, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // algorithm 1
            auto bla = *this;
            Neighborhood neighborhood = DataStructure::algo1(r, pointIdxRadiusSearch, cloud,
                                                             it.getCurrentOctreeDepth(), spur, okay);
            if (!neighborhood.pointIdc.empty()) {
                // found a consistent neighborhood, points have already been
                consNeighborhoods.push_back(neighborhood);
            }
        }
    }

    int okaySum = 0, spurSum = 0;

    for (auto i = 0; i <=
                     octree.getTreeDepth(); i++) { // TODO treedepth = 5, aber okay und spur sind gefüllt bis index 5, der kommt oben von getCurrentOctreeDepth... also <=
        std::cout << i << ": okay - " << okay[i] << ". spur - " << spur[i] << std::endl;
        okaySum += okay[i];
        spurSum += spur[i];
    }
    std::cout << ": okay - " << okaySum << ". spur - " << spurSum << std::endl;


    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal calculation in " << duration.count() << "s" << std::endl;




    // ******  NORMAL OPTIMIZATION ******

    float r = 5.0f; // TODO find good r

    // build map pointIdc -> neighborhoodIdx?
    std::map<int, int> pointNeighborhoodMap{};
    for (auto i = 0; i < consNeighborhoods.size(); i++) {
        for (auto pointIdx: consNeighborhoods[i].pointIdc) {
            pointNeighborhoodMap[pointIdx] = i;
        }
    }
    std::cout << TAG << "finished building map " << std::endl;


    // remove non-planar points (ex trees) from cons neighborhoods (c.N.)
    // TODO problem here: punkte werden entfernt die eigentlich nicht entfernt werden sollen. zB am rand einer runden c.N. und darüber sind nicht klassif. wandpunkte.

    // for every c.N.:
    // for every non-planar point in c.N. (sind aktuell insgesamt nur so 230 von 10k... kann ich mir dann vllt grad auch sparen die rauszulassen
    // get r neighborhood (r.N.) from point
    // check set of points from r.N. that are in the same c.N. and set of those who are not in the same c.N.
    // if less of neighbouring points are inside c.N. than outside -> point is non-planar, remove from c.N.
//    for (auto cnIdx = 0; cnIdx < consNeighborhoods.size(); cnIdx++) {
//        std::cout << TAG << "remove points in " << cnIdx << std::endl;
//
//        int removeCount = 0;
//
//        auto pointIt = consNeighborhoods[cnIdx].pointIdc.begin();
//        while (pointIt != consNeighborhoods[cnIdx].pointIdc.end()) { // TODO maybe filter only non-planar points
//
//
//            auto pointIdx = *pointIt;
//            const auto& point = (*cloud)[pointIdx];
//            std::vector<int> pointIdxRadiusSearch;
//            std::vector<float> pointRadiusSquaredDistance;
//            if (octree.radiusSearch(point, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
//                int insideCount = 0, outsideCount = 0;
//                for (auto npIdx: pointIdxRadiusSearch) {
//                    const auto& nPoint = (*cloud)[npIdx];
//                    // lots of points are removed unnecessarily
//                    //  -> try this check: if most neighbour points have no cN then remove this point
////                    if (pointNeighborhoodMap[npIdx] == cnIdx) { // [] operator performs insertion if map doesn't contain key!
//                    if (pointNeighborhoodMap.find(npIdx) != pointNeighborhoodMap.end()) {
//                        insideCount++;
//                    } else {
//                        outsideCount++;
//                    }
//                }
//
//
//                if (insideCount < outsideCount) {
////                    std::cout << TAG << "remove point from " << cnIdx << std::endl;
//                    removeCount++;
//
//                    pointNeighborhoodMap.erase(pointIdx);
//                    pointIt = consNeighborhoods[cnIdx].pointIdc.erase(pointIt);
//                } else ++pointIt;
//            } else ++pointIt;
//        }
//        std::cout << TAG << "removed " << removeCount << " points from " << cnIdx << std::endl;
//    }
//    std::cout << TAG << "finished remove non planar points " << std::endl;


    // move points to better fitting c.N.
    // TODO problem hier: auf großen flächen sind die c.N.s sehr ähnlich und punkte werden relativ willkürlich dazwischen rumgeschoben. die c.N.s sind dann viel kleinere fransigere patch work dinger als vorher.

    // for every c.N.
    // for every planar point in c.N. (glaube nach step 1 sind da nur noch planar points drin)
    // get r neighborhood from point
    // for all c.N.s that contain at least one point from r's neighborhood (including that from the outermost loop)
    // check distance of point to plane of the neighborhood, find c.N. with the smallest distance
    // if c.N. with smallest distance is not the curretn one, move point.
//    for (auto cnIdx = 0; cnIdx < consNeighborhoods.size(); cnIdx++) {
//        std::cout << TAG << "move points from " << cnIdx << std::endl;
//
//        int moveCount = 0;
//
//        auto pointIt = consNeighborhoods[cnIdx].pointIdc.begin();
//        while (pointIt != consNeighborhoods[cnIdx].pointIdc.end()) {
//
//            auto pointIdx = *pointIt;
//            const auto& point = (*cloud)[pointIdx];
//            std::vector<int> pointIdxRadiusSearch;
//            std::vector<float> pointRadiusSquaredDistance;
//            if (octree.radiusSearch(point, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
//                // TODO bis hier ist alle same wie oben -> optimieren möglich? oben werden ja noch punkt verschoben
//                std::set<int> neighborhoodIdc = {};
//                // find all neighborhoods from neighbors
//                for (auto npIdx: pointIdxRadiusSearch) {
//                    neighborhoodIdc.insert(pointNeighborhoodMap[npIdx]);
//                }
//                // test if they are closer to point
//                double minDist = INFINITY;
//                int minDistIdx = -1;
//                for (auto neighborhoodIdx: neighborhoodIdc) {
//                    auto dist = pointPlaneDistance(point, consNeighborhoods[neighborhoodIdx].plane);
//                    if (dist < minDist) {
//                        minDist = dist;
//                        minDistIdx = neighborhoodIdx;
//                    }
//                }
//                if (minDistIdx != cnIdx) {
////                    std::cout << TAG << "move point from " << cnIdx << " to " << minDistIdx << std::endl;
//                    moveCount++;
//
//                    // move point
//                    pointNeighborhoodMap[pointIdx] = minDistIdx;
//
//                    pointIt = consNeighborhoods[cnIdx].pointIdc.erase(pointIt);
//                    consNeighborhoods[minDistIdx].pointIdc.push_back(pointIdx);
//                } else ++pointIt;
//            } else ++pointIt;
//        }
//        std::cout << TAG << "moved " << moveCount << " points from " << cnIdx << std::endl;
//    }
//    std::cout << TAG << "finished other thing " << std::endl;




    // for each c.N.
    // calculate normal (formula 3 with covariance matrix)
    // TODO
    for (auto cnIdx = 0; cnIdx < consNeighborhoods.size(); cnIdx++) {
        // color debug
        int randR = rand() % (255 - 0 + 1) + 0;//100;//rand() % (255 - 0 + 1) + 0;
        int randG = rand() % (255 - 0 + 1) + 0;//100;//rand() % (255 - 0 + 1) + 0;
        int randB = rand() % (255 - 0 + 1) + 0;//100;//rand() % (255 - 0 + 1) + 0;

        // normal
        const auto& plane = consNeighborhoods[cnIdx].plane;
        auto vec1 = vectorSubtract(plane[0], plane[1]);
        auto vec2 = vectorSubtract(plane[0], plane[2]);
        auto planeNormal = normalize(crossProduct(vec1, vec2));
        // if vertical make sure normal is oriented up
        auto horLen = sqrt(pow(planeNormal.x, 2) + pow(planeNormal.z, 2));
        auto vertLen = abs(planeNormal.y);
        if (horLen < vertLen) {
//            randR = 255;
//            randG = 0;
//            randB = 0;
            // alle senkrechten nach oben orientieren
            if (planeNormal.y < 0) {
//                randR = 255;
//                randG = 255;
//                randB = 0;
                planeNormal.x *= -1;
                planeNormal.y *= -1;
                planeNormal.z *= -1;
            }
        } else {
//            randR = 0;
//            randG = 0;
//            randB = 255;
        }

        for (auto pointIt = consNeighborhoods[cnIdx].pointIdc.begin();
             pointIt != consNeighborhoods[cnIdx].pointIdc.end(); pointIt++) {
            auto pointIdx = *pointIt;
            // normal
            (*cloud)[pointIdx].normal_x = planeNormal.x;
            (*cloud)[pointIdx].normal_y = planeNormal.y;
            (*cloud)[pointIdx].normal_z = planeNormal.z;
            // color
            (*cloud)[pointIdx].b = randB;
            (*cloud)[pointIdx].g = randG;
            (*cloud)[pointIdx].r = randR;
        }
    }
    std::cout << TAG << "finished calculating normal per consistent neighbourhood " << std::endl;


    // all remaining points with no c.N. get uniform normal
    // TODO
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        if (pointNeighborhoodMap.find(pointIdx) == pointNeighborhoodMap.end()) {
            // point has no neighboorhood
            // check with maps data and give a good normal if it lies on wall
        }
    }


    // *****************************************************************

    // TODO use buildings to detect right normal orientation
    //  maybe also segmentation stuff?
    float thresholdDelta = 1.5f; // TODO find good value
    // TODO vllt zum flippen großen threshold, zum normal assignen bei kleineren threshold??

    std::vector<bool> cnVisited(consNeighborhoods.size(), false);
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
            if (pointIdx + 1 == nextPartIndex){
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
            octree.radiusSearch(mid, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            if(pointIdxRadiusSearch.size() != 0) {

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
                    // point is on plane of wall
                    auto minX = min(wallPoint1.x, wallPoint2.x);
                    auto maxX = max(wallPoint1.x, wallPoint2.x);
                    auto minZ = min(wallPoint1.z, wallPoint2.z);
                    auto maxZ = max(wallPoint1.z, wallPoint2.z);

                    const auto& x = point.x;
                    const auto& z = point.z;
                    if (x > maxX || x < minX || z > maxZ || z < minZ) {
                        continue;
                    }
                    // point is on wall rectangle
                    const auto& y = point.y;
//
//                    cloud->points[*nIdxIt].b = randB;
//                    (*cloud)[*nIdxIt].g = randG;
//                    (*cloud)[*nIdxIt].r = randR;

                    // check if point doesn't belong to a c.N. -> assign a normal
                    if (pointNeighborhoodMap.find(*nIdxIt) == pointNeighborhoodMap.end()) {
                        // TODO assign normal of this wall

                        auto vec1 = vectorSubtract(wallPlane[0], wallPlane[1]);
                        auto vec2 = vectorSubtract(wallPlane[0], wallPlane[2]);
                        auto planeNormal = normalize(crossProduct(vec1, vec2));

                        (*cloud)[*nIdxIt].normal_x = planeNormal.x;
                        (*cloud)[*nIdxIt].normal_y = planeNormal.y;
                        (*cloud)[*nIdxIt].normal_z = planeNormal.z;

                        (*cloud)[*nIdxIt].r = 255;
                        (*cloud)[*nIdxIt].g = 255;
                        (*cloud)[*nIdxIt].b = 255;

                    } else {
                        // check if normal has to be flipped

                        pcl::PointXYZRGBNormal normalPoint;
                        normalPoint.x = point.x + point.normal_x;
                        normalPoint.y = point.y + point.normal_y;
                        normalPoint.z = point.z + point.normal_z;
                        // check if normal is waagerecht
                        auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                        auto vertLen = point.normal_y;
                        if (horLen > vertLen) { // to avoid ground and roof
                            if (signedPointPlaneDistance(normalPoint, wallPlane) < 0) { // TODO check sign richtig
                                // TODO flip normal of all points of this c.N. and mark as visited

                                const auto& cnIdx = pointNeighborhoodMap[*nIdxIt];
                                // TODO check if this cn was already visited
                                if(cnVisited[cnIdx]){ // TODO diesen check weiter nach oben packen
                                    // this cN was already visited
                                } else {
                                    const auto& cN = *(consNeighborhoods.begin() + cnIdx);
                                    for (auto cnPointIdxIt = cN.pointIdc.begin();
                                         cnPointIdxIt != cN.pointIdc.end(); cnPointIdxIt++) {
                                        (*cloud)[*cnPointIdxIt].normal_x *= -1;
                                        (*cloud)[*cnPointIdxIt].normal_y *= -1;
                                        (*cloud)[*cnPointIdxIt].normal_z *= -1;
                                    }
                                    // TODO mark as visited
                                    cnVisited[cnIdx] = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // *****************************************************************

    // compute radii
    for (auto it = cloud->points.begin(); it != cloud->points.end(); it++) {
        const auto& bla = *it;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        octree.radiusSearch(bla, 1.5, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        auto const count = static_cast<float>(pointRadiusSquaredDistance.size()); // TODO warum sidn die radii hier plötzlich so groß (im vergleih zu bei dem kd tree dings verfahren)

        auto diff = std::max((count - 7.0f), 0.0f);

        auto avg = std::reduce(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end() - diff) / (count - diff);
        (*it).curvature = avg;
    }

}

DataStructure::Neighborhood DataStructure::algo1(const float& r, const std::vector<int>& pointIdxRadiusSearch,
                                                 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int level,
                                                 int (& spur)[10], int (& okay)[10]) {

    // check if voxel/radius search is "empty"
    if (pointIdxRadiusSearch.size() < 3) {
        std::cout << "skipping empty voxel on level " << level << std::endl;
        return {};
    }

    float thresholdDelta = 0.3f; // TODO find good threshold

    Plane bestPlane;
    long bestPlaneInSum = 0;
    long bestPlaneOutSum = 0;
    // find best plane
    for (auto i = 0; i < 3 * r; i++) { // TODO wie viele random ebenen testen?? abhängig von voxel point count machen
        long inSum = 0;
        long outSum = 0;

        std::vector<int> randSample;
        size_t nelems = 3;
        std::sample(
                pointIdxRadiusSearch.begin(),
                pointIdxRadiusSearch.end(),
                std::back_inserter(randSample),
                nelems,
                std::mt19937{std::random_device{}()}
        );


        Plane currPlane = {(*cloud)[randSample[0]], (*cloud)[randSample[1]], (*cloud)[randSample[2]]};

        for (std::size_t p = 0; p < pointIdxRadiusSearch.size(); ++p) {
            const auto& point = (*cloud)[pointIdxRadiusSearch[p]];

            if (point.normal_x == -10) { // normale wur
                continue;
            }

            if (pointPlaneDistance(point, currPlane) > thresholdDelta) {
                outSum++;
                // TODO bei der suche nach möglicher bester ebene: wenn ouside points > als bei aktuell bester direkt skip
//                        if(outSum > bestPlaneOutSum)
//                            break;
            } else {
                inSum++;
            }
        }

        // TODO testen wie es ist zwei/drei best planes abzuchecken
        // check if currPlane is bestPlane
        if (inSum > bestPlaneInSum) {
            std::memcpy(bestPlane, currPlane, sizeof(pcl::PointXYZRGBNormal[3]));
            bestPlaneInSum = inSum;
            bestPlaneOutSum = outSum;
        }
    }

    Neighborhood neighborhood;
    // is best plane "spurious"?
    if (bestPlaneInSum > bestPlaneOutSum) {
        // plane is not "spurious"
        // found consistent neighborhood of voxelCenter
        // voxelCenter is considered as a planar point
        // TODO ist es wohl schneller die in points oben immer neu hinzuzufügen oder hier nochmal alle zu durchlaufen einmal?

        std::memcpy(neighborhood.plane, bestPlane, sizeof(Plane));

        // set normal, functions as unavailability detection too
//        auto vec1 = vectorSubtract(bestPlane[0], bestPlane[1]);
//        auto vec2 = vectorSubtract(bestPlane[0], bestPlane[2]);
//
//        auto planeNormal = normalize(crossProduct(vec1, vec2));

//        // color debug
//        int randR = rand() % (255 - 0 + 1) + 0;
//        int randG = rand() % (255 - 0 + 1) + 0;
//        int randB = rand() % (255 - 0 + 1) + 0;


//        std::cout << randR << " " << randG << " " << randB << std::endl;
        for (std::size_t p = 0; p < pointIdxRadiusSearch.size(); ++p) {

            const auto& point = (*cloud)[pointIdxRadiusSearch[p]];
            if (point.normal_x == -10 ) { // this point already belongs to a c.N.
                continue;
            }
            auto dist = pointPlaneDistance(point, bestPlane);
            if (dist <= thresholdDelta) {
                neighborhood.pointIdc.push_back(pointIdxRadiusSearch[p]);


                (*cloud)[pointIdxRadiusSearch[p]].normal_x = -10; // mark as visited
//                (*cloud)[pointIdxRadiusSearch[p]].normal_y = planeNormal.y;
//                (*cloud)[pointIdxRadiusSearch[p]].normal_z = planeNormal.z;


//                (*cloud)[pointIdxRadiusSearch[p]].b = randB;
//                (*cloud)[pointIdxRadiusSearch[p]].g = randG;
//                (*cloud)[pointIdxRadiusSearch[p]].r = randR;

            }
        }
//        std::cout << "best plane okay on level " << level << std::endl;
        okay[level]++;

    } else {
//        std::cout << "best plane is spurious on level " << level << std::endl;
        spur[level]++;
    }
    return neighborhood;
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

void DataStructure::normalOrientation(const uint32_t& startIdx, const uint32_t& endIdx, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr treePtr) {
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
            if (pointIdx + 1 == nextPartIndex){
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

            if(pointIdxRadiusSearch.size() != 0) {

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
