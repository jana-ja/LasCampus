//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <assert.h>
#include <Eigen/Core>
#include <chrono>
#include "util.h"
#include "PointCloud.h"


using namespace std;

PointCloud::PointCloud(const std::vector<std::string> &files) {
    std::string dir = ".." + PATH_SEPARATOR + "las" + PATH_SEPARATOR;

    for (const auto &file: files) {
        read(dir + file);
    }
    std::cout << TAG << "read data successful" << std::endl;

//    tree = KdTree(vertices);
//    buildTree(vertices);

    calculateNormals();
}

void PointCloud::read(const string &path) {
    ifstream inf(path, ios::binary);

    if (inf.is_open()) {

        // header
        Header header = Header();
        // fill in header ref with read tree of size of header
        inf.read((char *) &header,
                 sizeof(header)); // cast to (char *) -> tell cpp we have some amount of bytes here/char array

        std::cout << TAG << "File: " << path << std::endl;
        std::cout << TAG << "Version: " << +header.versionMaj << "." << +header.versionMin << std::endl;
        std::cout << TAG << "Point Data Record Format: " << +header.pointDataRecordFormat << std::endl;


        // version checks
        if (header.versionMaj != 1 || header.versionMin != 2) {
            throw std::invalid_argument("Can't handle given LAS file. Only LAS version 1.2 allowed.");
        }
        assert(header.headerSize == sizeof(header));
        if (header.pointDataRecordFormat != 1 && header.pointDataRecordFormat != 2) {
            throw std::invalid_argument("Can't handle given LAS file. Only point tree record format 1 or 2 allowed.");
        }


        // point tree record format of multiple files must match
        if (firstFile) {
            // first file
            pointRecFormat = header.pointDataRecordFormat;

            // mittelpunkt von x, y, z - to center pointcloud
            auto midX = (header.maxX + header.minX) / 2.0f;
            auto midY = (header.maxY + header.minY) / 2.0f;
            auto midZ = (header.maxZ + header.minZ) / 2.0f;
            xOffset = (float) midX;
            yOffset = (float) midZ;
            zOffset = (float) midY;

            firstFile = false;
        } else {
            // following files
            if (header.pointDataRecordFormat != 1 && header.pointDataRecordFormat != 2) {
                throw std::invalid_argument("All given LAS files need to have the same point tree record format.");
            }
        }


//        // var length records
//        VarLenRecHeader varLenRecHeaders[header.numVarLenRecords]; // size two in this case
//        GeoKeyDirectoryTag geoKeyDirectoryTag; // is required
//
//        for (int i = 0; i < header.numVarLenRecords; i++) {
//
//            // read header
//            auto& currentHeader = varLenRecHeaders[i]; // ref
//            inf.read((char*) &currentHeader, sizeof currentHeader);
//
//            if (strcmp(currentHeader.userid, "LASF_Projection") == 0 && currentHeader.recordId == 34735) {
//                //  this var length record is the GeoKeyDirectoryTag
//
//                // read info
//                inf.read((char*) &geoKeyDirectoryTag, 8);//sizeof geoKeyDirectoryTag);
//
//                // resize entry vector of geo key directory tag
//                geoKeyDirectoryTag.entries.resize(geoKeyDirectoryTag.wNumberOfKeys);
//                // read entries
//                inf.read((char*) &geoKeyDirectoryTag.entries[0],
//                         geoKeyDirectoryTag.wNumberOfKeys * sizeof(GeoKeyEntry));
//            }
//        }

        // points
        int pointsUsed = 5000000;  //header.numberOfPoints

        std::cout << TAG << "Num of points: " << pointsUsed << std::endl;
        inf.seekg(header.pointDataOffset); // skip to point tree

        // init cloud
        cloud->width = pointsUsed;
        cloud->height = 1;

        if (header.pointDataRecordFormat == 1) {
            for (uint32_t i = 0; i < pointsUsed; i++) {//header.numberOfPoints; i++) {
                PointDRF1 point;
                inf.read((char *) (&point), sizeof(PointDRF1));

                // convert to opengl friendly thing
                // Xcoordinate = (Xrecord * Xscale) + Xoffset

                // center pointcloud - offset is in opengl coord system!
                pcl::PointNormal v;
                v.x = (float) (point.x * header.scaleX + header.offX - xOffset);
                v.y = (float) (point.z * header.scaleZ + header.offZ - yOffset);
                v.z = -(float) (point.y * header.scaleY + header.offY - zOffset);

                cloud->push_back(v);
            }
        }
//        else if (header.pointDataRecordFormat == 2) {
//            for (uint32_t i = 0; i < header.numberOfPoints; i++) {
//                PointDRF2 point;
//                inf.read((char*) (&point), sizeof(PointDRF2));
//
//                // convert to opengl friendly thing
//                // Xcoordinate = (Xrecord * Xscale) + Xoffset
//
//                auto redFloat = (point.red / 65536.0f);
//                auto greenFloat = (point.green / 65536.0f);
//                auto blueFloat = (point.blue / 65536.0f);
//
//                // center pointcloud - offset is in opengl coord system!
//                ColorVertex v;
//                v.x = (float) (point.x * header.scaleX + header.offX - xOffset);
//                v.y = (float) (point.z * header.scaleZ + header.offZ - yOffset);
//                v.z = -(float) (point.y * header.scaleY + header.offY - zOffset);
//                v.red = redFloat;
//                v.green = greenFloat;
//                v.blue = blueFloat;
//
//                colorVertices.push_back(v);
//            }
//        }

        if (!inf.good())
            throw runtime_error("Reading LAS ran into error");

    } else {
        throw runtime_error("Can't find LAS file");
    }
}

uint32_t PointCloud::getVertexCount() {
//    if (hasColor())
//        return (uint32_t) colorVertices.size();
//    else
    return (uint32_t) cloud->width;
}

pcl::PointNormal *PointCloud::getVertices() {
    return cloud->data();// vertices.data();
}

//Vertex* PointCloud::getColorVertices() {
//    return colorVertices.tree();
//}

Vertex PointCloud::getUTMForOpenGL(Vertex *vertexOpenGL) {
    // TODO offset is float, losing precision
    return Vertex{vertexOpenGL->x + xOffset, vertexOpenGL->y + yOffset, vertexOpenGL->z + zOffset};
}

Vertex PointCloud::getWGSForOpenGL(Vertex *vertex) {
    // TODO offset is float, losing precision

    // wert in utm holen, dann:

    // zone number: 60 zones, each 6 degrees of longitude (horizontal stripes), number is consistent in horizontal stripes
    // zone letter: 20 zones, each 8 degrees of latitude (vertical stripes), letter is consistent in vertical stripes
    // x wert zwischen 100.000 und 899.999 meter in zone
    // y wert ist entfernugn vom äquator (zumindest auf nordhalbkugel)
    return Vertex();
}

void PointCloud::kNN(const Vertex &point, size_t k, std::vector<KdTreeNode> *result) {
    tree.kNN(point, k, result);
}

void PointCloud::calculateNormals() {
    auto start = std::chrono::high_resolution_clock::now();


    std::cout << TAG << "start normal calc" << std::endl;

//    // Placeholder for the 3x3 covariance matrix at each surface patch
//    Eigen::Matrix3f covariance_matrix;
//    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
//    Eigen::Vector4f xyz_centroid;
//    // Estimate the XYZ centroid
//    pcl::compute3DCentroid(cloud, xyz_centroid);
//    // Compute the 3x3 covariance matrix
//    pcl::computeCovarianceMatrix(cloud, xyz_centroid, covariance_matrix);
//    // TODO solve normal orientation

//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
//    cloudPtr.pu;


    // Create the normal estimation class, and pass the input dataset to it
//    pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
//    pcl::NormalEstimation<pcl::PointNormal, pcl::Normal> ne2;
    //ne.setNumberOfThreads(8);
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "tree in " << duration.count() << std::endl;
    start = std::chrono::high_resolution_clock::now();
    ne.setSearchMethod(tree);


    // Use all neighbors in a sphere of radius 300cm
//    ne.setRadiusSearch(3);
    pcl::PointNormal bla;

    bla = cloud->points[0];
    ne.setKSearch(6);

    // Compute the features
//    ne.compute(*normals);
    ne.compute(*cloud);


    // normals->size () should have the same size as the input cloud->size ()*


    bla = cloud->points[0];
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "normal calc in " << duration.count() << std::endl;
}

void PointCloud::buildTree(std::vector<Vertex> vertices) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud.
//
//
//
//    // Create the normal estimation class, and pass the input dataset to it
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    ne.setInputCloud(cloud);
//
//    // Create an empty kdtree representation, and pass it to the normal estimation object.
//    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//    ne.setSearchMethod(tree);
//
//    // Output datasets
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//
//    // Use all neighbors in a sphere of radius 3cm
//    ne.setRadiusSearch(0.03);
//
//    // Compute the features
//    ne.compute(*cloud_normals);
//
//    // cloud_normals->size () should have the same size as the input cloud->size ()*

}

pcl::Normal *PointCloud::getNormals() {
    return normals->data();
}

//bool PointCloud::hasColor() {
//    return pointRecFormat == 2;
//}
