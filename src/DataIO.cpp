//
// Created by Jana on 12.11.2023.
//

#include "DataIO.h"
#include <numeric>
#include <fstream>
#include "UTM.h"
#include "util.h"

bool DataIO::readData(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& imgFile,
                      const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& buildings) {


    std::cout << TAG << "begin loading data" << std::endl;


// read shape file
// TODO hard coded coordinates from current test las file - maybe read las header, then read shp and the rest of las??
// these are to only read buildings that match the las file, because shp file covers whole regierungsbezirk arnsberg
    double maxX = 7.415424;
    double maxY = 51.494428;
    double minX = 7.401340;
    double minY = 51.485245;
    boundsMaxX = maxX;
    boundsMaxY = maxY,
    boundsMinX = minX;
    boundsMinY = minY;
    std::string shpDir = ".." + Util::PATH_SEPARATOR + "shp" + Util::PATH_SEPARATOR;
    float xOffset2 = 389500;
    float zOffset2 = 5705500; // TODO get from las file
    readShp(shpDir + shpFile, &buildings, xOffset2, zOffset2, maxX, maxY, minX, minY);

// preprocess buildings to walls
    float resolution = 8.0f; // TODO find good value
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> wallOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>(
            resolution);
    // TODO add error handling if buildings are empty
    float maxWallRadius = preprocessWalls(wallOctree, buildings);

    // read las file
    std::string lasDir = ".." + Util::PATH_SEPARATOR + "las" + Util::PATH_SEPARATOR;
    const auto& file = lasFiles[0];
    // get points
    readLas(lasDir + file, cloud, walls, wallOctree, maxWallRadius);

    // get normals
//    std::string cacheFile = file;
//    cacheFile.replace(cacheFile.end() - 3, cacheFile.end(), "features");
//    bool loadedCachedFeatures = readFeaturesFromCache(lasDir + cacheFile, cloud);
//    return loadedCachedFeatures;
    return false;


    std::cout << TAG << "loading data successful" << std::endl;

}



// ********** las **********

void DataIO::readLas(const std::string& path, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Wall>& walls,
                     pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, float& maxWallRadius) {

    std::cout << TAG << "read las file..." << std::endl;

    std::ifstream inf(path, std::ios::binary);

    if (inf.is_open()) {

        // header
        Header header = Header();
        // fill in header ref with read tree of size of header
        inf.read((char*) &header,
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
            if (header.pointDataRecordFormat != pointRecFormat) {
                throw std::invalid_argument("All given LAS files need to have the same point data record format.");
            }
        }


//        // var length records
//        ShpVarLenRecHeader varLenRecHeaders[header.numVarLenRecords]; // size two in this case
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


        // read image
        std::string filename = "../img/dop10rgbi_32_389_5705_1_nw_2021.jpg"; // TODO
        int width, height;
        std::vector<unsigned char> image;
        bool success = load_image(image, filename, width, height);
        if (!success) {
            std::cout << "Error loading image\n";
            // TODO stop
        }
        std::cout << "Image width = " << width << '\n';
        std::cout << "Image height = " << height << '\n';
        const size_t RGBI = 3;
        int x = 3;
        int y = 4;
        size_t index = RGBI * (y * width + x);
//        std::cout << "RGBI pixel @ (x=3, y=4): "
//                  << static_cast<int>(image[index + 0]) << " "
//                  << static_cast<int>(image[index + 1]) << " "
//                  << static_cast<int>(image[index + 2]) << " "
//                  << static_cast<int>(image[index + 3]) << '\n';


        // points
        int pointsUsed = 200000; //header.numberOfPoints;

        std::cout << TAG << "Num of points: " << pointsUsed << std::endl;
        inf.seekg(header.pointDataOffset); // skip to point tree

        // init cloud
        inf.seekg(11500000 * sizeof(PointDRF1), std::ios_base::cur); // skip to point tree
        cloud->width = pointsUsed; // TODO help noch anpassen für mehrere files
        cloud->height = 1;

        if (header.pointDataRecordFormat == 1) {
//            int count = 0;
            for (uint32_t i = 0; i < pointsUsed; i++) {//header.numberOfPoints; i++) {
                PointDRF1 point;
                inf.read((char*) (&point), sizeof(PointDRF1));

                // convert to opengl friendly thing
                // Xcoordinate = (Xrecord * Xscale) + Xoffset
                float pointX = point.x * header.scaleX + header.offX;
                float pointY = point.y * header.scaleY + header.offY;
                float pointZ = point.z * header.scaleZ + header.offZ;


                pcl::PointXYZRGBNormal v; // TODO nach unten später
                // center pointcloud - offset is in opengl coord system!
                v.x = (float) (pointX - xOffset);
                v.y = (float) (pointZ - yOffset);
                v.z = -(float) (pointY - zOffset);
                v.normal_x = -1;
                v.normal_y = -1;
                v.normal_z = -1;

                // pcl library switched r and b component
                v.b = 100; // r
                v.g = 100; // g
                v.r = 100; // b
                v.a = 255;

                // filter stuff
                float wallThreshold = 1.5;

                // get info out of 8 bit classification:
                // classification, synthetic, keypoint, withheld
                // little endian
                // w k s c c c c c
                int8_t classification = point.classification & 31;
                bool synthetic = (point.classification >> 5) & 1;
//                bool keyPoint = (point.classification >> 6) & 1;
//                bool withheld = (point.classification >> 7) & 1;
                if (synthetic) { // point.pointSourceId == 2
                    continue;
                }
                // get info out of 8 bit flags:
                // return number, number of returns, stuff, stuff
                // little endian
                // _ _ n n n r r r
                int8_t returnNumber = point.flags & 7;
                int8_t numOfReturns = (point.flags >> 3) & 7;
                if (numOfReturns > 1) {
                    if (returnNumber == 1) {
                        // first of many
                        if (colorReturnNumberClasses) {
                            v.b = 255;
                            v.g = 0;
                            v.r = 0;
                        }
                        // bäume oberer teil, teile von wänden, ein dach?
                        // keep wall points, skip others
                        bool belongsToWall = false;
                        std::vector<int> wallIdxRadiusSearch;
                        std::vector<float> wallRadiusSquaredDistance;
                        if (wallOctree.radiusSearch(v, maxWallRadius, wallIdxRadiusSearch, wallRadiusSquaredDistance) > 0) {
                            for (auto wallIdx = 0; wallIdx < wallIdxRadiusSearch.size(); wallIdx++) {
                                const auto& wall = walls[wallIdxRadiusSearch[wallIdx]];

                                float dist = Util::pointPlaneDistance(v, wall.mid);
                                if (dist > wallThreshold) {
                                    continue;
                                }
                                if (v.x > wall.maxX || v.x < wall.minX || v.z > wall.maxZ || v.z < wall.minZ) {
                                    continue;
                                }
                                // belongs to wall
                                belongsToWall = true;
                                break;
                            }
                        }
                        if (!belongsToWall) {
                            continue;
                        }
                    } else if (returnNumber != numOfReturns) {
                        // intermediate points
                        // viel baum, ganz wenig wand -> raus
                        continue;
                        if (colorReturnNumberClasses) {
                            v.b = 0;
                            v.g = 0;
                            v.r = 255;
                        }
                    } else {
                        // last of many
                        // boden, bisschen wände, kein baum. einfach lassen
                        if (classification != 2) { // not ground
                            bool belongsToWall = false;
                            std::vector<int> wallIdxRadiusSearch;
                            std::vector<float> wallRadiusSquaredDistance;
                            if (wallOctree.radiusSearch(v, maxWallRadius, wallIdxRadiusSearch, wallRadiusSquaredDistance) > 0) {
                                for (auto wallIdx = 0; wallIdx < wallIdxRadiusSearch.size(); wallIdx++) {
                                    const auto& wall = walls[wallIdxRadiusSearch[wallIdx]];

                                    float dist = Util::pointPlaneDistance(v, wall.mid);
                                    if (dist > wallThreshold) {
                                        continue;
                                    }
                                    if (v.x > wall.maxX || v.x < wall.minX || v.z > wall.maxZ || v.z < wall.minZ) {
                                        continue;
                                    }
                                    // belongs to wall
                                    belongsToWall = true;
                                    break;
                                }
                            }
                            if (colorReturnNumberClasses && belongsToWall) {
                                v.b = 0;
                                v.g = 255;
                                v.r = 0;
                            }
                        }
                    }
                }


                // get color from image
                int imageX = (pointX - 389000.05) * 10; // data from jp2 world file
                int imageY = (pointY - 5705999.95) * -10;
                // werte sollten immer zwischen 0 und 999 (oder 1 und 1000?) sein.
                size_t index = RGBI * (imageY * width + imageX);
                v.b = static_cast<int>(image[index + 0]);
                v.g = static_cast<int>(image[index + 1]);
                v.r = static_cast<int>(image[index + 2]);
//                int intensity = image[index + 3];


                cloud->push_back(v);
//                count++;
            }
            // resize cloud
            //cloud->resize(count);
            std::cout << TAG << "Num of points: " << pointsUsed << std::endl;


        }

        if (!inf.good())
            throw std::runtime_error("Reading .las ran into error");

        std::cout << TAG << "finished reading las file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .las file");
    }
}


// ********** shp **********
template<typename T>
T swap_endian(T u) {
    static_assert(CHAR_BIT == 8, "CHAR_BIT != 8");

    union {
        T u;
        unsigned char u8[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

// TODO idee: hier direkt bounding box mitgeben und nur die shapes innerhalb speichern
void
DataIO::readShp(const std::string& path, std::vector<Polygon>* buildings, const float& xOffset, const float& zOffset, double maxX, double maxY, double minX,
                double minY) {

    std::cout << TAG << "read shp file..." << std::endl;

    std::ifstream inf(path, std::ios::binary);

    if (inf.is_open()) {

        uint32_t bytesRead = 0;

        // read header
        ShpHeader header = ShpHeader();
        inf.read((char*) &header,
                 sizeof(header)); // cast to (char *) -> tell cpp we have some amount of bytes here/char array
        bytesRead += sizeof(header);
        // swap big endian fields
        header.fileCode = swap_endian<uint32_t>(header.fileCode);
        header.fileLength = swap_endian<uint32_t>(header.fileLength);
        auto fileLengthBytes = header.fileLength * 2; //* 16 / 8;

        std::cout << TAG << "File: " << path << std::endl;
        std::cout << TAG << "Shape Type: " << +header.shapeType << std::endl;


        // version checks
        if (header.shapeType != 5) {
            throw std::invalid_argument(
                    "Can't handle given SHP file. Only shapeType 5 (Polygon -> buildings) is allowed.");
        }


        bool readContents = true;
        while (readContents) {

            // read var length record header - TODO gleich vector erstellen wo die ganzen var len records reinkommen??
            ShpVarLenRecHeader varLenRecHeader;
            inf.read((char*) &varLenRecHeader, sizeof(varLenRecHeader));
            bytesRead += sizeof(varLenRecHeader);
            // swap big endian fields
            varLenRecHeader.recNumber = swap_endian<uint32_t>(varLenRecHeader.recNumber);
            varLenRecHeader.contentLen = swap_endian<uint32_t>(varLenRecHeader.contentLen);
            auto contentLenBytes = varLenRecHeader.contentLen * 2; //* 16 / 8;
//            std::cout << TAG << "var len rec len: " << +contentLenBytes << std::endl;


            // read var length record content
            ShpPolygonRecContent polygonRecContent;

            // read shape type
            // all shapes in file have the same shape type (5 here) or null (0)
            inf.read((char*) &polygonRecContent, sizeof(ShpPolygonRecContent::shapeType));
            bytesRead += sizeof(ShpPolygonRecContent::shapeType);
//            std::cout << TAG << "var len rec shape type: " << +polygonRecContent.shapeType << std::endl;
            // skip if null shape
            if (polygonRecContent.shapeType == 0) {
                std::cout << TAG << "Skipping Null Shape" << std::endl;
                continue;
            }
            // read polygon content
            inf.read((char*) &polygonRecContent + sizeof(ShpPolygonRecContent::shapeType),
                     sizeof(polygonRecContent) - sizeof(ShpPolygonRecContent::shapeType));
            bytesRead += sizeof(polygonRecContent) - sizeof(ShpPolygonRecContent::shapeType);


            // check if polygon lies within bounds of las data
            if (isPolygonInBounds(polygonRecContent)) {

                // read part indices and points of all parts
                Polygon polygon;
                // part indices
                uint32_t partIndex;
                for (auto i = 0; i < polygonRecContent.numParts; i++) {
                    inf.read((char*) &partIndex, sizeof(uint32_t));
                    bytesRead += sizeof(uint32_t);
                    polygon.parts.push_back(partIndex); // push back makes copy
                }
                // points of all parts
                Point point;
                for (auto i = 0; i < polygonRecContent.numPoints; i++) {
                    inf.read((char*) &point, sizeof(Point));
                    bytesRead += sizeof(Point);
                    // convert point from w84 to utm zone 32n TODO is precise enough?
                    double utmX, utmZ;
                    LatLonToUTMXY(point.z, point.x, 32, utmX, utmZ);
                    // TODO - offset for opengl coord sys
                    point.x = utmX - xOffset;
                    point.z = -(utmZ - zOffset); // TODO negative z for opengl coord sys
                    polygon.points.push_back(point);
                }

                buildings->push_back(polygon);

            } else {

                // skip this polygon
                auto remainingBytesPolygon = contentLenBytes - sizeof(ShpPolygonRecContent);
                inf.seekg(remainingBytesPolygon, std::ios_base::cur); // skip to point tree
                bytesRead += remainingBytesPolygon;

            }

            // test if file is finished
//            std::cout << TAG << "bytesRead: " << +bytesRead << std::endl;
            if (bytesRead >= fileLengthBytes) {
                readContents = false;
            }

        }

        if (!inf.good())
            throw std::runtime_error("Reading .shp ran into error");

        std::cout << TAG << "finished reading shp file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .shp file");
    }
}

float DataIO::preprocessWalls(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, std::vector<Polygon>& buildings) {
    // preprocessing of buildings
    // save all walls (min, mid, max point & radius)
    // dann beim normalen orientieren  spatial search nach mid point mit max radius von allen walls
    float maxR = 0;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallMidPoints = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
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

            auto vec1 = Util::vectorSubtract(wallPoint1, wallPoint2);
            auto vec2 = Util::vectorSubtract(wallPoint1, wall.mid);
            auto planeNormal = Util::normalize(Util::crossProduct(vec1, vec2));
            wall.mid.normal_x = planeNormal.x;
            wall.mid.normal_y = planeNormal.y;
            wall.mid.normal_z = planeNormal.z;

            float r = sqrt(pow(wallPoint2.x - wall.mid.x, 2) + pow(wallHeight - wall.mid.y, 2) + pow(wallPoint2.z - wall.mid.z, 2));
            if (r > maxR) {
                maxR = r;
            }

            wall.minX = std::min(wallPoint1.x, wallPoint2.x);
            wall.maxX = std::max(wallPoint1.x, wallPoint2.x);
            wall.minZ = std::min(wallPoint1.z, wallPoint2.z);
            wall.maxZ = std::max(wallPoint1.z, wallPoint2.z);

            walls.push_back(wall);
            wallMidPoints->push_back(wall.mid);


        }
    }

    wallOctree.setInputCloud(wallMidPoints);
    wallOctree.defineBoundingBox();
    wallOctree.addPointsFromInputCloud();

    return maxR;
}


// ********** cache **********
bool DataIO::readFeaturesFromCache(const std::string& normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) {

    std::cout << TAG << "try to read features from cache" << std::endl;

    std::ifstream inf(normalPath, std::ios::binary);
    if (!inf.good()) {
        std::cout << TAG << "no cache file found" << std::endl;
        return false;
    }

    if (inf.is_open()) {
        std::cout << TAG << "cache file found" << std::endl;

        // read header
        FeatureCacheHeader normalHeader;
        inf.read((char*) (&normalHeader), sizeof(FeatureCacheHeader));


        // check if right version
        if (normalHeader.version != FEATURE_CACHE_VERSION) {
            throw std::runtime_error("Feature Cache File has wrong version");
        }


        // read normals
        for (auto it = cloud->points.begin(); it != cloud->points.end(); it++) {
            float normal[3];
            inf.read((char*) (&normal), 3 * sizeof(float));

            (*it).normal_x = normal[0];
            (*it).normal_y = normal[1];
            (*it).normal_z = normal[2];

            float radius;
            inf.read((char*) (&radius), sizeof(float));
            (*it).curvature = radius;

        }

        if (!inf.good())
            throw std::runtime_error("Reading .normal ran into error");

        std::cout << TAG << "finished reading normals from cache" << std::endl;
        return true;

    } else {
        throw std::runtime_error("Can't find .normal file");
    }
}

/**
 *
 * @param normalPath
 * @param cloud
 * @param startIdx
 * @param endIdx exclusive
 */
void DataIO::writeFeaturesToCache(const std::string& normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) {
    std::ofstream out(normalPath, std::ios::binary);

    std::cout << TAG << "writing normals to cache" << std::endl;

    if (out.is_open()) {
        // write header
        FeatureCacheHeader normalHeader;
        normalHeader.numberOfPoints = cloud->size();
        normalHeader.version = FEATURE_CACHE_VERSION;

        out.write((char*) (&normalHeader), sizeof(FeatureCacheHeader));

        // write normals and radii
        for (auto it = cloud->points.begin(); it != cloud->points.end(); it++) {
            out.write((char*) (&*it->normal), 3 * sizeof(float));
            out.write((char*) (&it->curvature), sizeof(float));
        }

        if (!out.good())
            throw std::runtime_error("Writing .normal ran into error");

        out.close();
        std::cout << TAG << "finished writing normals to cache" << std::endl;


    } else {
        throw std::runtime_error("Can't find .normal file");
    }
}

