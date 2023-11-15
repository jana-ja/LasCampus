//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <chrono>
#include "util.h"
#include "PointCloud.h"


using namespace std;

PointCloud::PointCloud(const std::vector<std::string> &files) {

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
            calculateNormals(startIdx, endIdx); // TODO use idx
//            io.writeNormalsToCache(dir + normalFile, cloud, startIdx, endIdx); // TODO temporarily not used
        }

        startIdx += pointCount;
    }
    std::cout << TAG << "loading data successful" << std::endl;
}


void PointCloud::calculateNormals(const uint32_t &startIdx, const uint32_t &endIdx) { // TODO use indices
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start octree" << std::endl;


    // find consistent neighborhoods

    // ****** OCTREE ******
    srand((unsigned int) time(NULL));

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

    pcl::PointXYZRGBNormal searchPoint = cloud->points[1000];

    // TODO problem: start radius search in biggest voxel and move down the tree.
    //  how to access all voxels of tree?
    //  idee: getNeighborsWithinRadiusRecursive überschreiben/umschreiben sodass man max depth angeben kann? wie dann die kinder nicht doppelt machen?

    // TODO for voxel in tree:
    {
        // TODO if voxel.count < 3 -> empty, skip

        float voxelSize = 10.0f; // TODO
        float r = sqrt(2.0) * (voxelSize/2.0);

        // algorithm 1
        algo1(r, octree);
    }

    // Neighbors within radius search

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 3.0f;

    // TODO hier kann mana uch suche starten mit index von nem puntk aus cloud
    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;


    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].y
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }


    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal calculation in " << duration.count() << "s" << std::endl;
}




//Vertex PointCloud::getUTMForOpenGL(Vertex *vertexOpenGL) {
//    // TODO offset is float, losing precision
//    return Vertex{vertexOpenGL->x + xOffset, vertexOpenGL->y + yOffset, vertexOpenGL->z + zOffset};
//}
//
//Vertex PointCloud::getWGSForOpenGL(Vertex *vertex) {
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

uint32_t PointCloud::getVertexCount() {
    return (uint32_t) cloud->width;
}

pcl::PointXYZRGBNormal *PointCloud::getVertices() {
    return cloud->data();// vertices.data();
}

std::vector<int> PointCloud::algo1(const float& r, const std::vector<int>& pointIdxRadiusSearch) {
    using Neighborhood = std::vector<int>;


        plane bestPlane;
        long bestPlaneInSum;
        long bestPlaneOutSum;
        // find best plane
        for (auto i = 0; i < 10; i++) { // TODO wie viele rnandom ebenen testen??
            long inSum = 0;
            long outSum = 0;
            plane currPlane = buildPlane(3 random points);

                for (std::size_t p = 0; p < pointIdxRadiusSearch.size(); ++p) {
                    const auto& point = (*cloud)[pointIdxRadiusSearch[p]];

                    if(dist(point, plane) > thresholdDelta){
                        outSum++;
                        // TODO bei der suche nach möglicher bester ebene: wenn ouside points > als bei aktuell bester direkt skip
//                        if(outSum > bestPlaneOutSum)
//                            continue;
                    } else {
                        inSum++;
                    }
                }

                // check if currPlane is bestPlane
                if (inSum > bestPlaneInSum){
                    bestPlane = plane;
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
            for (std::size_t p = 0; p < pointIdxRadiusSearch.size(); ++p) {

                const auto& point = (*cloud)[pointIdxRadiusSearch[p]];
                if(dist(point, bestPlane) <= thresholdDelta) {
                    neighborhood.push_back(pointIdxRadiusSearch[p]);
                    // TODO mark points as unavailable for following detection

                }
            }

        }

        return neighborhood;


}


template <typename PointT, typename LeafContainerT, typename BranchContainerT>
pcl::uindex_t pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::robustNormalEstimation(
        const PointT& p_q,
        const double radius,
        Indices& k_indices,
        std::vector<float>& k_sqr_distances,
        uindex_t max_nn) const
{



    assert(isFinite(p_q) &&
           "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
    OctreeKey key;
    key.x = key.y = key.z = 0;

    k_indices.clear();
    k_sqr_distances.clear();

    getNeighborsWithinRadiusRecursive(p_q,
                                      radius * radius,
                                      this->root_node_,
                                      key,
                                      1,
                                      k_indices,
                                      k_sqr_distances,
                                      max_nn);

    return k_indices.size();
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::robustNormalEstimationRecursive(const PointT& point,
                                  const double radiusSquared,
                                  const BranchNode* node,
                                  const OctreeKey& key,
                                  uindex_t tree_depth,
                                  Indices& k_indices,
                                  std::vector<float>& k_sqr_distances,
                                  uindex_t max_nn) const
{

    using Neighborhood = std::vector<int>;

    // TODO if voxel.count < 3 -> empty, skip

    vector<Neighborhood> consNeighborhoods;


    // get spatial voxel information
    double voxel_squared_diameter = this->getVoxelSquaredDiameter(tree_depth);

    // iterate over all children
    for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
        if (!this->branchHasChild(*node, child_idx))
            continue;

        const OctreeNode* child_node;
        child_node = this->getBranchChildPtr(*node, child_idx);



        OctreeKey new_key;
        PointT voxel_center;
        float squared_dist;

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

        if (this->radiusSearch(voxelCenter, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // algorithm 1
            Neighborhood neighborhood = algo1(r, pointIdxRadiusSearch);
            if(!neighborhood.empty()){
                consNeighborhoods.push_back(neighborhood);
            } else {
                // TODO try to detect a valid plane from a small neighborhood of pi
            }
        }

        // TODO schwierig die punkte in dieser neighborhood müssen alle als vergeben markiert werden
        // *************************

    }
}

