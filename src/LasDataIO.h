//
// Created by Jana on 11.11.2023.
//

#ifndef LASCAMPUS_LASDATAIO_H
#define LASCAMPUS_LASDATAIO_H

#include <pcl/octree/octree_search.h>
#include "stb_image.h"

class LasDataIO {

public:

    // TODO denke das sollte woanders leben?
    struct Wall {
        pcl::PointXYZRGBNormal mid;
        float minX, maxX;
        float minZ, maxZ;

    };

    // ********** las **********
    void readLas(const std::string& path, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Wall>& walls,
                 pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, float& maxWallRadius);

    // ********** shp **********
    //ShpDataIO(double maxX, double maxY, double minX, double minY); TODO get values from las
    struct Point {
        double x, z;
    };
    /*
     * Polygon may contain one or more rings.
     * The rings are closed (the first and last vertex of a ring MUST be the same).
     * Waling along the ring in vertex order -> inside of polygon is on the right hand side.
     *  -> Polygons with only one ring always in clockwise order. Holes in Polygons are counterclockwise.
     */
    struct Polygon {
        std::vector<uint32_t> parts; // size = numParts
        std::vector<Point> points; // size = numPoints
    };
    void readShp(const std::string& path, std::vector<Polygon>* buildings, const float& xOffset, const float& zOffset, double maxX, double maxY, double minX, double minY);

    // ********** cache **********
    bool readFeaturesFromCache(const std::string &normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud);
    void writeFeaturesToCache(const std::string &normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud);



//    LasDataIO(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& imgFile): {}
    bool readData(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& imgFile, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& buildings);

private:
    const std::string TAG = "LasIO\t";

    // ########## structs ##########
#pragma pack(push, 1) // win - tightly pack the bytes and dont start at new power of two things

// ********** las **********
    struct Header {
        char magic[4];
        uint16_t fileSourceId; // unsigned short - 2 bytes
        uint16_t globalEncoding;
        uint32_t guidData1; // unsigned long - 4 bytes
        uint16_t guidData2;
        uint16_t guidData3;
        uint8_t guidData4[8]; // chars that are numbers
        uint8_t versionMaj, versionMin;
        char systemIdentifier[32];
        char genSoftware[32];
        uint16_t creationDay, creationYear;
        uint16_t headerSize;
        uint32_t pointDataOffset;
        uint32_t numVarLenRecords = 2;
        uint8_t pointDataRecordFormat;
        uint16_t pointDataRecordLen;
        uint32_t numberOfPoints;
        uint32_t numPointsByReturn[5];
        double scaleX, scaleY, scaleZ;
        double offX, offY, offZ;
        double maxX, minX;
        double maxY, minY; // z in opengl system
        double maxZ, minZ; // y in opengl system
    };

    struct VarLenRecHeader {
        uint16_t reserved; // unsigned short - 2 bytes
        char userid[16]; // user which created the var len rec. // can be LASF_Spec or LASF_Projection in this case
        uint16_t recordId; // depends on userid. 34735 for userId LASF_Projection is GeoKeyDirectoryTag and is mandatory here.
        uint16_t recordLenAfterHeader;
        char description[32];
    };

    struct GeoKeyEntry {
        uint16_t wKeyId; // id from GeoTiff specification
        uint16_t wTiffTagLocation; // 0 -> tree is in wValueOffset.
        // 34736 -> wValueOffset is index of tree in GeoDoubleParamsTag record.
        // 34767 -> wValueOffset is index of tree in GeoAsciiParamsTag record.
        uint16_t wCount; // only relevant for GeoAsciiParamsTag, 1 otherwise
        uint16_t wValueOffset; // content depends on wTiffTagLocation
    };

    struct GeoKeyDirectoryTag {
        uint16_t wKeyDirectoryVersion; // always 1
        uint16_t wKeyRevision; // always 1
        uint16_t wMinorRevision; // always 0
        uint16_t wNumberOfKeys;
        std::vector<GeoKeyEntry> entries;
    };

// Point Data Record Format 1
    struct PointDRF1 {
        uint32_t x, y, z;
        uint16_t intensity;
        uint8_t flags; // multiple bits that are not needed and add up to eight
        uint8_t classification;
        uint8_t scanAngleRank;
        uint8_t userData;
        uint16_t pointSourceId;
        double gpsTime;
    };

    struct FeatureCacheHeader {
        uint32_t numberOfPoints;
        uint8_t version;
    };


    // ********** shp **********
    struct ShpHeader {
        // big endianess
        uint32_t fileCode;
        uint32_t unused[5];
        uint32_t fileLength; // in 16 bit words, including the header. header is 100 bytes = 50 16-bit words
        // little endianess
        uint32_t version;
        uint32_t shapeType;
        double minX, minY, maxX, maxY; // minimum bounding rectangle of all shapes in dataset,
        double minZ, maxZ; //
        double minM, maxM; //

    };

    struct ShpVarLenRecHeader {
        uint32_t recNumber; // 1-based
        uint32_t contentLen; // in 16-bit words
    };

    // variable length record content for shape type 5
    struct ShpPolygonRecContent {
        uint32_t shapeType;
        double xMin, yMin, xMax, yMax;
        uint32_t numParts; // "rings" of polygon.
        uint32_t numPoints;
        // those will be handled in vectors
//        uint32_t parts[numParts];
//        Point points[numPoints];
    };
#pragma pack(pop)


    // ########## FIELDS & METHODS ##########



    // ********** las **********
    float xOffset;
    float yOffset;
    float zOffset;
    bool colorReturnNumberClasses = false;
    int pointRecFormat;
    bool firstFile = true;

    // ********** shp **********
    double boundsMaxX, boundsMaxY, boundsMinX, boundsMinY;
    bool isPolygonInBounds(ShpPolygonRecContent& polygon){
        if (polygon.xMin > boundsMaxX || polygon.xMax < boundsMinX) {
            return false; // out of bounds in x direction
        }
        if (polygon.yMin > boundsMaxY || polygon.yMax < boundsMinY) {
            return false; // out of bounds in y direction
        }
        return true;
    }
    float preprocessWalls(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, std::vector<Polygon>& buildings);
    std::vector<LasDataIO::Wall> walls;

    // ********** img **********
    bool load_image(std::vector<unsigned char>& image, const std::string& filename, int& x, int&y)
    {
        int desiredChannels = 3; // if file has less channels, remaining fields will be 255
        int actualChannels;
        unsigned char* data = stbi_load(filename.c_str(), &x, &y, &actualChannels, desiredChannels);
        if (data != nullptr)
        {
            image = std::vector<unsigned char>(data, data + x * y * desiredChannels);
        }
        stbi_image_free(data);
        return (data != nullptr);
    }

    // ********** cache **********
    const uint8_t FEATURE_CACHE_VERSION = 2;

};



#endif //LASCAMPUS_LASDATAIO_H
