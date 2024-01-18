//
// Created by Jana on 12.11.2023.
//

#include "LasDataIO.h"


void LasDataIO::readLas(const std::string& path, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, uint32_t* pointCount,
                        float& xOffset, float& yOffset, float& zOffset, std::vector<DataStructure::Wall>& walls,
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


        // read image
        std::string filename = "../las/dop10rgbi_32_389_5705_1_nw_2021.jpg";
        int width, height;
        std::vector<unsigned char> image;
        bool success = load_image(image, filename, width, height);
        if (!success)
        {
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
        *pointCount = pointsUsed;

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
                        if(colorClasses) {
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

                                float dist = DataStructure::pointPlaneDistance(v, wall.mid);
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
                        if(colorClasses) {
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

                                    float dist = DataStructure::pointPlaneDistance(v, wall.mid);
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
                            if(colorClasses && belongsToWall) {
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
            *pointCount = cloud->size();
            std::cout << TAG << "Num of points: " << *pointCount << std::endl;


        }

        if (!inf.good())
            throw std::runtime_error("Reading .las ran into error");

        std::cout << TAG << "finished reading las file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .las file");
    }
}


bool LasDataIO::readFeaturesFromCache(const std::string& normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const uint32_t& startIdx,
                                      const uint32_t& endIdx) { // TODO endindex muss exklusiv sein!

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
        for (auto it = cloud->points.begin() + startIdx; it != cloud->points.begin() + endIdx; it++) {
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
void LasDataIO::writeFeaturesToCache(const std::string& normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const uint32_t& startIdx,
                                     const uint32_t& endIdx) { // TODO end index is exclusive
    std::ofstream out(normalPath, std::ios::binary);

    std::cout << TAG << "writing normals to cache" << std::endl;

    if (out.is_open()) {
        // write header
        FeatureCacheHeader normalHeader;
        normalHeader.numberOfPoints = endIdx - startIdx;
        normalHeader.version = FEATURE_CACHE_VERSION;

        out.write((char*) (&normalHeader), sizeof(FeatureCacheHeader));

        // write normals and radii
        for (auto it = cloud->points.begin() + startIdx; it != cloud->points.begin() + endIdx; it++) {
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

void LasDataIO::random(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) {
    for (int i = 0; i < 10000; ++i) {
        pcl::PointXYZRGBNormal p;
        p.x = (float) (rand() % 100);
        p.y = (float) (rand() % 100);
        p.z = (float) (rand() % 100);

        cloud->push_back(p);

        //std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
    }
}
