//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include "util.h"
#include "DataStructure.h"


using namespace std;

DataStructure::DataStructure(const std::vector<std::string> &files) {

    LasDataIO io = LasDataIO();


    std::string dir = ".." + PATH_SEPARATOR + "las" + PATH_SEPARATOR;
    // in cache muss: pointrecformat, anzahl points, offset

    uint32_t startIdx = 0;
    uint32_t endIdx;
    uint32_t pointCount;
    std::cout << TAG << "begin loading data" << std::endl;
    for (const auto &file: files) {

        // get points
        io.readLas(dir + file, cloud, &pointCount);

        endIdx = pointCount;

        // get normals
        std::string normalFile = file;
        normalFile.replace(normalFile.end() - 3, normalFile.end() - 1, "normal");
        if (!io.readNormalsFromCache(dir + normalFile, cloud, startIdx, endIdx)) {
            calculateNormals(startIdx, endIdx);
//            io.writeNormalsToCache(dir + normalFile, cloud, startIdx, endIdx); // TODO temporarily not used
        }

        startIdx += pointCount;
    }
    std::cout << TAG << "loading data successful" << std::endl;
}


void DataStructure::calculateNormals(const uint32_t &startIdx, const uint32_t &endIdx) { // TODO use indices
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start octree" << std::endl;


    // find consistent neighborhoods

    // ****** OCTREE ******

    // minimum scale threshold - specified by referring to the sampling density of P
    // length of smallest voxel at lowest octree level
    float resolution = 10.0f; // TODO find good value

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>(resolution);

    // assign bounding box to octree
    // octree.defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ); // TODO get bounding box values, should be easy from las file data, for testing get while reading points from subset

    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished octree in " << duration.count() << "s" << std::endl;
    start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start normal calculation" << std::endl;

    // ****** NORMAL CALCULATION ******

    octree.robustNormalEstimation(cloud);


    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal calculation in " << duration.count() << "s" << std::endl;
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

pcl::PointXYZRGBNormal *DataStructure::getVertices() {
    return cloud->data();// vertices.data();
}

std::vector<int> DataStructure::algo1(const float& r, const std::vector<int>& pointIdxRadiusSearch, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {


        Plane bestPlane;
        long bestPlaneInSum = 0;
        long bestPlaneOutSum = 0;
        // find best plane
        for (auto i = 0; i < 100; i++) { // TODO wie viele rnandom ebenen testen??
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


            Plane currPlane = { (*cloud)[randSample[0]], (*cloud)[randSample[1]], (*cloud)[randSample[2]] };

                for (std::size_t p = 0; p < pointIdxRadiusSearch.size(); ++p) {
                    const auto& point = (*cloud)[pointIdxRadiusSearch[p]];

                    if (point.normal_x != -1 || point.normal_y != -1 || point.normal_z != -1){ // normale wur
                        continue;
                    }

                    if(pointPlaneDistance(point, currPlane) > 0.5f){ //thresholdDelta){ // TODO find good threshold
                        outSum++;
                        // TODO bei der suche nach möglicher bester ebene: wenn ouside points > als bei aktuell bester direkt skip
//                        if(outSum > bestPlaneOutSum)
//                            break;
                    } else {
                        inSum++;
                    }
                }

                // check if currPlane is bestPlane
                if (inSum > bestPlaneInSum){
                    std::memcpy(bestPlane, currPlane, sizeof(pcl::PointXYZRGBNormal[3]));
                    bestPlaneInSum = inSum;
                    bestPlaneOutSum = outSum;
                }


        }
        Neighborhood neighborhood;
        // is best plane "spurious"?
        if(bestPlaneInSum > bestPlaneOutSum){
            // plane is not "spurious"
            // found consistent neighborhood of voxelCenter
            // voxelCenter is considered as a planar point
            // TODO ist es wohl schneller die in points oben immer neu hinzuzufügen oder hier nochmal alle zu durchlaufen einmal?

            // TODO mark points as unavailable for following detection - extra vector? are points just pointers? then assign normal from detected plane!
            // set normal, functions as unavailable detection too
            auto vec1 = vectorSubtract(bestPlane[0], bestPlane[1]);
            auto vec2 = vectorSubtract(bestPlane[0], bestPlane[2]);

            auto planeNormal = normalize(crossProduct(vec1, vec2));

            // color debug
            int randR = rand()%(255-0 + 1) + 0;
            int randG = rand()%(255-0 + 1) + 0;
            int randB = rand()%(255-0 + 1) + 0;
            for (std::size_t p = 0; p < pointIdxRadiusSearch.size(); ++p) {

                const auto& point = (*cloud)[pointIdxRadiusSearch[p]];
                if(pointPlaneDistance(point, bestPlane) <= 0.5f){//} thresholdDelta) { // TODO thresholdDelta richtig bestimmen
                    neighborhood.push_back(pointIdxRadiusSearch[p]);


                    (*cloud)[pointIdxRadiusSearch[p]].normal_x = planeNormal.x;
                    (*cloud)[pointIdxRadiusSearch[p]].normal_y = planeNormal.y;
                    (*cloud)[pointIdxRadiusSearch[p]].normal_z = planeNormal.z;



                    (*cloud)[pointIdxRadiusSearch[p]].b = randR;
                    (*cloud)[pointIdxRadiusSearch[p]].g = randG;
                    (*cloud)[pointIdxRadiusSearch[p]].r = randB;


                }
            }

        }

        return neighborhood;


}


template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::robustNormalEstimation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) const  // TODO je nachdem ob ich normalen assignen kann void oder neighborhoods zurückgeben
{

    OctreeKey key;
    key.x = key.y = key.z = 0;


    robustNormalEstimationRecursive(this->root_node_,
                                      key,
                                      1, cloud);

}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::robustNormalEstimationRecursive(
                                  const BranchNode* node,
                                  const OctreeKey& key,
                                  uindex_t tree_depth,
                                  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) const
{

    using Neighborhood = std::vector<int>;


    // TODO if voxel.count < 3 -> empty, skip

    vector<Neighborhood> consNeighborhoods;


    // iterate over all children
    for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
        if (!this->branchHasChild(*node, child_idx))
            continue;

        const OctreeNode* child_node;
        child_node = this->getBranchChildPtr(*node, child_idx);



        OctreeKey new_key;
        PointT voxel_center;

        // generate new key for current branch voxel
        new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
        new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
        new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

        // generate voxel center point for voxel at key
        this->genVoxelCenterFromOctreeKey(new_key, tree_depth, voxel_center);

        // *************************
        float voxelSize = sqrt(this->getVoxelSquaredSideLen(tree_depth)); // TODO richtig?
        float r = sqrt(2.0) * (voxelSize/2.0);


        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (this->radiusSearch(voxel_center, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // algorithm 1
            auto bla = *this;
            Neighborhood neighborhood = DataStructure::algo1(r, pointIdxRadiusSearch, cloud);
            if(!neighborhood.empty()){
                consNeighborhoods.push_back(neighborhood);
                // TODO schwierig die punkte in dieser neighborhood müssen alle als vergeben markiert werden
            } else {
                // TODO try to detect a valid plane from a small neighborhood of pi - einfach weiter in die voxel oder spezifisch dafür suchen??
            }
        }

        // *************************

        if (child_node->getNodeType() == BRANCH_NODE) {
            // we have not reached maximum tree depth
            robustNormalEstimationRecursive(static_cast<const BranchNode*>(child_node),
                                              new_key,
                                              tree_depth + 1, cloud);
        }
    }
}

