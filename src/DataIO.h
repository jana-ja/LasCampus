//
// Created by Jana on 11.11.2023.
//

#ifndef LASCAMPUS_DATAIO_H
#define LASCAMPUS_DATAIO_H

#include <pcl/octree/octree_search.h>
#include "stb_image.h"
#include <map>
#include "util.h"

class DataIO {

public:

    // ********** shp **********
    struct ShpPoint {
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
        std::vector<ShpPoint> points; // size = numPoints
    };
//    struct Wall {
//        pcl::PointXYZRGBNormal mid;
//        float minX, maxX;
//        float minZ, maxZ;
//        int buildingIdx;
//    };

    // ********** cache **********
    void writeFeaturesToCache(const std::string &normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud);



//    DataIO(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& imgFile): {}
    bool readData(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& imgFile,
                  const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<pcl::PointXY>& texCoords,
                  std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec, int& wallPointsStartIndex);

private:
    const std::string TAG = "DataIO\t";

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

// ShpPoint Data Record Format 1
    struct PointDRF1 {
        uint16_t intensity;
        uint8_t flags; // multiple bits that are not needed and add up to eight
        uint8_t classification;
        uint8_t scanAngleRank;
        uint8_t userData;
        uint16_t pointSourceId;
        double gpsTime;
        double x, y, z; // originally uint32_t at first position in struct, but i want double

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
//        ShpPoint points[numPoints];
    };
#pragma pack(pop)


    // ########## FIELDS & METHODS ##########

    bool colorReturnNumberClasses = true;

    bool buildingCheck(const pcl::PointXYZRGBNormal& v, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, const float& maxWallRadius);

    // ********** las **********
    // in opengl coord sys
    float xOffset;
    float yOffset;
    float zOffset;
    int pointRecFormat;
    bool firstFile = true;
    int numOfPoints;

    std::vector<PointDRF1> lasPoints;
    void readLas(const std::string& path);
    void filterAndColorPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
                              const float& maxWallRadius, const std::string& imgFile, std::vector<bool>& lasWallPoints, std::vector<bool>& lasGroundPoints, std::vector<pcl::PointXY>& texCoords);
    void detectWalls(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& buildings, std::vector<bool>& lasWallPoints, std::vector<bool>& lasGroundPoints,
                     const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<pcl::PointXY>& texCoords,
                     std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec, int& wallPointsStartIndex);
    void findXYZMedian(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices, float& xMedian, float& yMedian, float& zMedian);
    void findYMinMax(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices, float& yMin, float& yMax);
    float getMaxY(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, float& x, float& z, float& yMin, float& yMax, float& stepWidth, std::vector<bool>& removePoints, const pcl::PointXYZ& wallNormal, const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree);

    // ********** shp **********
    // these are to only read buildings that match the las file, because shp file covers whole regierungsbezirk arnsberg
    double boundsMaxX, boundsMaxY, boundsMinX, boundsMinY; // in wgs84 lat lon in degrees
    bool isPolygonInBounds(ShpPolygonRecContent& polygon){
        if (polygon.xMin > boundsMaxX || polygon.xMax < boundsMinX) {
            return false; // out of bounds in x direction
        }
        if (polygon.yMin > boundsMaxY || polygon.yMax < boundsMinY) {
            return false; // out of bounds in y direction
        }
        return true;
    }
    void readShp(const std::string& path, std::vector<Polygon>* buildings);
    float preprocessWalls(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, std::vector<Polygon>& buildings);
    std::vector<Util::Wall> walls;
    std::map<int, std::vector<int>> buildingWallMap;

    // ********** img **********
    bool readImg(std::vector<unsigned char>& image, const std::string& filename, const int& desiredChannels, int& width, int& height);

    // ********** cache **********
    const uint8_t FEATURE_CACHE_VERSION = 2;
    bool readFeaturesFromCache(const std::string &normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud);

};



#endif //LASCAMPUS_DATAIO_H
