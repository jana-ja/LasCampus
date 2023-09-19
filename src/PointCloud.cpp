//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <assert.h>
#include "PointCloud.h"

using namespace std;

PointCloud::PointCloud(const string &path) {
    read(path);
}

void PointCloud::read(const string &path) {
    ifstream inf(path, ios::binary);

    if (inf.is_open()) {


        // header
        Header header;

        // fill in header ref with read data of size of header
        inf.read((char *) &header,
                 sizeof(header)); // cast to (char *) -> tell cpp we have some amount of bytes here/char array

        std::cout << "version " << +header.versionMaj << "." << +header.versionMin << std::endl;
        std::cout << "scale factors " << header.scaleX << " " << header.scaleY << " " << header.scaleZ << std::endl;

        // can only read las version 1.2
        assert(header.versionMaj == 1 && header.versionMin == 2);
        assert(header.headerSize == sizeof(header));
        assert(header.pointDataRecordFormat == 1);


        // var length records
        VarLenRecHeader varLenRecHeaders[header.numVarLenRecords]; // size two in this case
        GeoKeyDirectoryTag geoKeyDirectoryTag; // is required

        for(int i = 0; i < header.numVarLenRecords; i++){

            // read header
            auto& currentHeader = varLenRecHeaders[i]; // ref
            inf.read((char *) &currentHeader, sizeof currentHeader);

            if(strcmp(currentHeader.userid, "LASF_Projection") == 0 && currentHeader.recordId == 34735){
                //  this var length record is the GeoKeyDirectoryTag

                // read info
                inf.read((char *) &geoKeyDirectoryTag, 8);//sizeof geoKeyDirectoryTag);

                // resize entry vector of geo key directory tag
                geoKeyDirectoryTag.entries.resize(geoKeyDirectoryTag.wNumberOfKeys);
                // read entries
                inf.read((char *) &geoKeyDirectoryTag.entries[0], geoKeyDirectoryTag.wNumberOfKeys * sizeof(GeoKeyEntry));
            }


        }

        // TODO assert GeoKeyDirectoryTag is there

        geoKeyDirectoryTag.entries[0].wValueOffset;
        // CRS = coordinate reference system (?)
        // geo tiff specification key -> value
        // 1024 -> 1    -   GTModelTypeGeoKey
            // the model CRS is a 2D projected coordinate reference system, indicated by the value of ProjectedCRSGeoKey
        // 3072 -> 25832    -   v1.0 ProjectedCSTypeGeoKey / v1.1? ProjectedCRSGeoKey
            // for 2D projected CRS ("map grids"). if model crs is pseudo-3D compund crs consisting of projected 2D crs with vertical crs, the code of vertical component is given through VerticalGeoKey
            // shall be EPSG projected crs codes - finde 25832 nicht in der liste mit codes??
            // UTM Zone 32N
        // 3076 -> 9001 -   ProjLinearUnitsGeoKey
            // linear units -   9001 = meter (surprise surprise)
        // 4099 -> 9001 -   VerticalUnitsGeoKey
            // ebenfalls meter
        // 4096 -> 7837 -   VerticalCSTypeGeoKey / VerticalGeoKey
            // wegen dem was bei ProjectedCTSGeoKey steht schaue ich bei EPSG Vertical CRS Codes nach
            // DHHN2016 height


        // points
        std::cout << "num of points: " << header.numberOfPoints << std::endl;

        inf.seekg(header.pointDataOffset); // skip to point data

        // mittelpunkt von x, y, z - to center pointcloud
        auto midX = (header.maxX + header.minX) / 2.0f;
        auto midY = (header.maxY + header.minY) / 2.0f;
        auto midZ = (header.maxZ + header.minZ) / 2.0f;
        xOffset = (float)midX;
        yOffset = (float)midZ;
        zOffset = (float)midY;

        for (uint32_t i = 0; i < header.numberOfPoints; i++) {
            PointDRF1 point;
            inf.read((char *) (&point), sizeof(PointDRF1));

            // convert to opengl friendly thing
            // Xcoordinate = (Xrecord * Xscale) + Xoffset

            // center pointcloud
            Vertex v = {
                    (float) (point.x * header.scaleX + header.offX - midX),
                    (float) (point.z * header.scaleZ + header.offZ - midZ),
                    -(float) (point.y * header.scaleY + header.offY - midY)

            };

            //cout << v.x << ", " << v.y << ", " << v.z << endl;
            vertices.push_back(v);
        }


        cout << "\t min\t max" << endl;

        cout << "x:\t" << header.minX - midX << "\t" << header.maxX - midX << endl;
        cout << "y:\t" << header.minY - midY << "\t" << header.maxY - midY << endl;
        cout << "z:\t" << header.minZ - midZ << "\t" << header.maxZ - midZ << endl;


        if (!inf.good())
            throw runtime_error("Reading LAS ran into error");

    } else {
        throw runtime_error("Cant find LAS file");
    }
}

uint32_t PointCloud::getVerticesCount() {
    return (uint32_t) vertices.size();
}

Vertex *PointCloud::getVertices() {
    return vertices.data();
}

Vertex PointCloud::getUTMForOpenGL(Vertex* vertexOpenGL) {
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
