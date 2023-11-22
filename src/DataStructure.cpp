//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include <set>
#include "util.h"
#include "DataStructure.h"


using namespace std;

DataStructure::DataStructure(const std::vector<std::string>& files) {

    LasDataIO io = LasDataIO();


    std::string dir = ".." + PATH_SEPARATOR + "las" + PATH_SEPARATOR;
    // in cache muss: pointrecformat, anzahl points, offset

    uint32_t startIdx = 0;
    uint32_t endIdx;
    uint32_t pointCount = 0;//; // TODO remove
    std::cout << TAG << "begin loading data" << std::endl;
    for (const auto& file: files) {

        // get points
        io.readLas(dir + file, cloud, &pointCount);
//        io.random(cloud);

        endIdx = pointCount;

        // get normals
        std::string normalFile = file;
        normalFile.replace(normalFile.end() - 3, normalFile.end() - 1, "normal");
        if (!io.readNormalsFromCache(dir + normalFile, cloud, startIdx, endIdx)) {
            robustNormalEstimation(startIdx, endIdx);
//            io.writeNormalsToCache(dir + normalFile, cloud, startIdx, endIdx); // TODO temporarily not used
        }

        startIdx += pointCount;
    }
    std::cout << TAG << "loading data successful" << std::endl;
}


void DataStructure::robustNormalEstimation(const uint32_t& startIdx, const uint32_t& endIdx) { // TODO use indices
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
    int spur[10]{0,0,0,0,0,0,0,0,0,0};


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

    for ( auto i = 0; i <= octree.getTreeDepth(); i++ ){ // TODO treedepth = 5, aber okay und spur sind gefüllt bis index 5, der kommt oben von getCurrentOctreeDepth... also <=
        std::cout << i << ": okay - " << okay[i] << ". spur - " << spur[i] << std::endl;
        okaySum += okay[i];
        spurSum += spur[i];
    }
    std::cout << ": okay - " << okaySum << ". spur - " << spurSum << std::endl;


    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal calculation in " << duration.count() << "s" << std::endl;




    // ******  NORMAL OPTIMIZATION ******

    float r = 1.0f; // TODO find good r

    // build map pointIdc -> neighborhoodIdx?
    std::map<int, int> pointNeighborhoodMap{};
    for (auto i = 0; i < consNeighborhoods.size(); i++) {
        for (auto pointIdx: consNeighborhoods[i].pointIdc) {
            pointNeighborhoodMap[pointIdx] = i;
        }
    }
    std::cout << TAG << "finished building map " << std::endl;


    // remove non-planar points (ex trees) from cons neighborhoods (c.N.)

    // for every c.N.:
    // for every non-planar point in c.N. (sind aktuell insgesamt nur so 230 von 10k... kann ich mir dann vllt grad auch sparen die rauszulassen
        // get r neighborhood (r.N.) from point
        // check set of points from r.N. that are in the same c.N. and set of those who are not in the same c.N.
        // if less of neighbouring points are inside c.N. than outside -> point is non-planar, remove from c.N.
    for (auto cnIdx = 0; cnIdx < consNeighborhoods.size(); cnIdx++) {
        std::cout << TAG << "remove points in " << cnIdx << std::endl;

        int removeCount = 0;

        auto pointIt = consNeighborhoods[cnIdx].pointIdc.begin();
        while(pointIt != consNeighborhoods[cnIdx].pointIdc.end()) { // TODO maybe filter only non-planar points


            auto pointIdx = *pointIt;
            const auto& point = (*cloud)[pointIdx];
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if (octree.radiusSearch(point, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                int insideCount = 0, outsideCount = 0;
                for (auto npIdx: pointIdxRadiusSearch) {
                    const auto& nPoint = (*cloud)[npIdx];
                    if (pointNeighborhoodMap[npIdx] == cnIdx){
                        insideCount++;
                    } else {
                       outsideCount++;
                    }
                }


                if (insideCount < outsideCount){
//                    std::cout << TAG << "remove point from " << cnIdx << std::endl;
                    removeCount++;

                    pointNeighborhoodMap.erase(pointIdx);
                    pointIt = consNeighborhoods[cnIdx].pointIdc.erase(pointIt);
                }
                else ++pointIt;
            }
            else ++pointIt;
        }
        std::cout << TAG << "removed " << removeCount << " points from " << cnIdx << std::endl;
    }
    std::cout << TAG << "finished remove non planar points " << std::endl;


    // move points to better fitting c.N.

    // for every c.N.
    // for every planar point in c.N. (glaube nach step 1 sind da nur noch planar points drin)
    // get r neighborhood from point
    // for all c.N.s that contain at least one point from r's neighborhood (including that from the outermost loop)
    // check distance of point to plane of the neighborhood, find c.N. with the smallest distance
    // if c.N. with smallest distance is not the curretn one, move point.
    for (auto cnIdx = 0; cnIdx < consNeighborhoods.size(); cnIdx++) {
        std::cout << TAG << "move points from " << cnIdx << std::endl;

        int moveCount = 0;

        auto pointIt = consNeighborhoods[cnIdx].pointIdc.begin();
        while(pointIt != consNeighborhoods[cnIdx].pointIdc.end()) {

            auto pointIdx = *pointIt;
            const auto& point = (*cloud)[pointIdx];
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if (octree.radiusSearch(point, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                // TODO bis hier ist alle same wie oben -> optimieren möglich? oben werden ja noch punkt verschoben
                std::set<int> neighborhoodIdc = {};
                // find all neighborhoods from neighbors
                for (auto npIdx: pointIdxRadiusSearch) {
                    neighborhoodIdc.insert(pointNeighborhoodMap[npIdx]);
                }
                // test if they are closer to point
                double minDist = INFINITY;
                int minDistIdx = -1;
                for (auto neighborhoodIdx: neighborhoodIdc) {
                    auto dist = pointPlaneDistance(point, consNeighborhoods[neighborhoodIdx].plane);
                    if (dist < minDist){
                        minDist = dist;
                        minDistIdx = neighborhoodIdx;
                    }
                }
                if (minDistIdx != cnIdx) {
//                    std::cout << TAG << "move point from " << cnIdx << " to " << minDistIdx << std::endl;
                    moveCount++;

                    // move point
                    pointNeighborhoodMap[pointIdx] = minDistIdx;

                    pointIt = consNeighborhoods[cnIdx].pointIdc.erase(pointIt);
                    consNeighborhoods[minDistIdx].pointIdc.push_back(pointIdx);
                }
                else ++pointIt;
            }
            else ++pointIt;
        }
        std::cout << TAG << "moved " << moveCount << " points from " << cnIdx << std::endl;
    }
    std::cout << TAG << "finished other thing " << std::endl;



    // for each c.N.
        // calculate normal (formula 3 with covariance matrix)
        // TODO


    // all remaining points with no c.N. get uniform normal
    // TODO


    // add colors
    for (auto cnIdx = 0; cnIdx < consNeighborhoods.size(); cnIdx++) {
        // color debug
        int randR = rand() % (255 - 0 + 1) + 0;
        int randG = rand() % (255 - 0 + 1) + 0;
        int randB = rand() % (255 - 0 + 1) + 0;
        for (auto pointIt = consNeighborhoods[cnIdx].pointIdc.begin(); pointIt != consNeighborhoods[cnIdx].pointIdc.end(); pointIt++) {
            auto pointIdx = *pointIt;
            (*cloud)[pointIdx].b = randB;
            (*cloud)[pointIdx].g = randG;
            (*cloud)[pointIdx].r = randR;
        }
    }
    std::cout << TAG << "finished adding colors " << std::endl;


}

DataStructure::Neighborhood DataStructure::algo1(const float& r, const std::vector<int>& pointIdxRadiusSearch,
                                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int level, int (&spur)[10], int (&okay)[10]) {

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
    for (auto i = 0; i < 3*r; i++) { // TODO wie viele random ebenen testen?? abhängig von voxel point count machen
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

            if (point.normal_x != -1 || point.normal_y != -1 || point.normal_z != -1) { // normale wur
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
        auto vec1 = vectorSubtract(bestPlane[0], bestPlane[1]);
        auto vec2 = vectorSubtract(bestPlane[0], bestPlane[2]);

        auto planeNormal = normalize(crossProduct(vec1, vec2));

//        // color debug
//        int randR = rand() % (255 - 0 + 1) + 0;
//        int randG = rand() % (255 - 0 + 1) + 0;
//        int randB = rand() % (255 - 0 + 1) + 0;

//    switch (level) {
//        case 0:
//            randR = 255;
//            randG = 0;
//            randB = 0;
//            break;
//        case 1:
//            randR = 0;
//            randG = 255;
//            randB = 0;
//            break;
//        case 2:
//            randR = 0;
//            randG = 0;
//            randB = 255;
//            break;
//        case 3:
//            randR = 255;
//            randG = 255;
//            randB = 0;
//            break;
//        case 4:
//            randR = 0;
//            randG = 255;
//            randB = 255;
//            break;
//        case 5:
//            randR = 255;
//            randG = 0;
//            randB = 255;
//            break;
//
//    }

//        std::cout << randR << " " << randG << " " << randB << std::endl;
        for (std::size_t p = 0; p < pointIdxRadiusSearch.size(); ++p) {

            const auto& point = (*cloud)[pointIdxRadiusSearch[p]];
            if (point.normal_x != -1 || point.normal_y != -1 || point.normal_z != -1) { // normale wur
                continue;
            }
            auto dist = pointPlaneDistance(point, bestPlane);
            if (dist <= thresholdDelta) {
                neighborhood.pointIdc.push_back(pointIdxRadiusSearch[p]);


                (*cloud)[pointIdxRadiusSearch[p]].normal_x = planeNormal.x;
                (*cloud)[pointIdxRadiusSearch[p]].normal_y = planeNormal.y;
                (*cloud)[pointIdxRadiusSearch[p]].normal_z = planeNormal.z;

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
