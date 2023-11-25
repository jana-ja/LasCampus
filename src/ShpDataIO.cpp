//
// Created by Jana on 25.11.2023.
//

#include <iostream>
#include "ShpDataIO.h"


void ShpDataIO::readShp(const std::string &path) {

    std::cout << TAG << "read shp file..." << std::endl;

    std::ifstream inf(path, std::ios::binary);

    if (inf.is_open()) {

        // read header
        Header header = Header();
        inf.read((char *) &header,
                 sizeof(header)); // cast to (char *) -> tell cpp we have some amount of bytes here/char array

        std::cout << TAG << "File: " << path << std::endl;
        std::cout << TAG << "Shape Type: " << +header.shapeType << std::endl;


        // version checks
        if (header.shapeType != 5) {
            throw std::invalid_argument("Can't handle given SHP file. Only shapeType 5 (Polygon -> buildings) is allowed.");
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

//        // points
//        int pointsUsed = 200000; //header.numberOfPoints;
//        *pointCount = pointsUsed;
//
//        std::cout << TAG << "Num of points: " << pointsUsed << std::endl;
//        inf.seekg(header.pointDataOffset); // skip to point tree
//
//        // init cloud
//        inf.seekg(11500000 * sizeof(PointDRF1), std::ios_base::cur); // skip to point tree
//        cloud->width = pointsUsed; // TODO help noch anpassen für mehrere files
//        cloud->height = 1;
//
//        if (header.pointDataRecordFormat == 1) {
//            for (uint32_t i = 0; i < pointsUsed; i++) {//header.numberOfPoints; i++) {
//                PointDRF1 point;
//                inf.read((char *) (&point), sizeof(PointDRF1));
//
//                // convert to opengl friendly thing
//                // Xcoordinate = (Xrecord * Xscale) + Xoffset
//
//                // center pointcloud - offset is in opengl coord system!
//                pcl::PointXYZRGBNormal v;
//                v.x = (float) (point.x * header.scaleX + header.offX - xOffset);
//                v.y = (float) (point.z * header.scaleZ + header.offZ - yOffset);
//                v.z = -(float) (point.y * header.scaleY + header.offY - zOffset);
////                v.normal_x = -1;
////                v.normal_y = -1;
////                v.normal_z = -1;
//                // pcl library switched r and b component
//                v.b = 100; // r
//                v.g = 55; // g
//                v.r = 28; // b
//                v.a = 255;
//
//                cloud->push_back(v);
//            }
//        }

        if (!inf.good())
            throw std::runtime_error("Reading .las ran into error");

        std::cout << TAG << "finished reading las file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .las file");
    }
}
