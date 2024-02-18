//
// Created by Jana on 12.11.2023.
//

#include "DataIO.h"
#include <numeric>
#include <fstream>
#include <pcl/common/pca.h>
#include <cstdlib>
#include <random>
#include "UTM.h"

bool DataIO::readData(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& imgFile,
                      const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<pcl::PointXY>& texCoords,
                      std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec, int& wallPointsStartIndex) {

    std::vector<DataIO::Polygon> osmPolygons;

    std::cout << TAG << "begin loading data" << std::endl;

    // read las file
    std::string lasDir = ".." + Util::PATH_SEPARATOR + "las" + Util::PATH_SEPARATOR;
    const auto& file = lasFiles[0];
    // sets: las to opengl offsets, bounds for shp and numOfPoints
    readLas(lasDir + file);

    // read shape file
    std::string shpDir = ".." + Util::PATH_SEPARATOR + "shp" + Util::PATH_SEPARATOR;
    readShp(shpDir + shpFile, &osmPolygons);

    std::cout << TAG << "begin processing shp data" << std::endl;
    // preprocess osmPolygons to osmWalls
    float resolution = 8.0f; // TODO find good value
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> wallOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>(
            resolution);
    // TODO add error handling if osmPolygons are empty
    float maxWallRadius = preprocessWalls(wallOctree, osmPolygons);

    // get cached features
    // TODO probleme mit cache files da ich ja jetzt schon vorher punkte rauswerfe, muss dann exakt übereinstimmen also vllt verwendete params (zB wallthreshold im cache speichern)
//    std::string cacheFile = file;
//    cacheFile.replace(cacheFile.end() - 3, cacheFile.end(), "features");
//    bool loadedCachedFeatures = readFeaturesFromCache(lasDir + cacheFile, cloud);
//    return loadedCachedFeatures;

    std::cout << TAG << "begin filtering and coloring points" << std::endl;
    std::vector<bool> lasWallPoints;
    std::vector<bool> lasGroundPoints;
    filterAndColorPoints(cloud, wallOctree, maxWallRadius, imgFile, lasWallPoints, lasGroundPoints, texCoords);

    // up to this index are points from las file, from this index on are synthetic wall points
    wallPointsStartIndex = cloud->size();
    tangent1Vec = std::vector<pcl::PointXYZ>((*cloud).size());
    tangent2Vec = std::vector<pcl::PointXYZ>((*cloud).size());

    std::cout << TAG << "begin detecting walls" << std::endl;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);
    detectWalls(cloud, osmPolygons, lasWallPoints, lasGroundPoints, tree, texCoords, tangent1Vec, tangent2Vec, wallPointsStartIndex);
    tree->setInputCloud(cloud);


    std::cout << TAG << "loading data successful" << std::endl;
    return false;
}



// ********** las **********

/**
 * sets offsets (to convert UTM to my OpenGl system), bounds (in WGS to cut out shp osm data matching las file space), numberOfPoints
 * @param path
 */
void DataIO::readLas(const std::string& path) {

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
        std::cout << TAG << "ShpPoint Data Record Format: " << +header.pointDataRecordFormat << std::endl;


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

        // set bounds to read shp file later
        double maxLat, maxLon;
        UTMXYToLatLon(header.maxX, header.maxY, 32, false, maxLat, maxLon);
        double minLat, minLon;
        UTMXYToLatLon(header.minX, header.minY, 32, false, minLat, minLon);
        boundsMaxX = maxLon;
        boundsMaxY = maxLat,
                boundsMinX = minLon;
        boundsMinY = minLat;


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


        // points
        numOfPoints = 400000;
//        numOfPoints = header.numberOfPoints;

        std::cout << TAG << "Num of points: " << numOfPoints << std::endl;
        inf.seekg(header.pointDataOffset); // skip to point tree
        // skip to point tree TODO because i dont use all points for testing
        inf.seekg(11300000 * (sizeof(PointDRF1) - 3 * sizeof(double) + 3 * sizeof(uint32_t)), std::ios_base::cur);

        if (header.pointDataRecordFormat == 1) {
            for (uint32_t i = 0; i < numOfPoints; i++) {//header.numberOfPoints; i++) {
                PointDRF1 point;
                uint32_t x, y, z;
                inf.read((char*) (&x), sizeof(uint32_t));
                inf.read((char*) (&y), sizeof(uint32_t));
                inf.read((char*) (&z), sizeof(uint32_t));
                inf.read((char*) (&point), sizeof(PointDRF1) - 3 * sizeof(double));

                // Xcoordinate = (Xrecord * Xscale) + Xoffset
                point.x = (x * header.scaleX + header.offX);
                point.y = (y * header.scaleY + header.offY);
                point.z = (z * header.scaleZ + header.offZ);

                lasPoints.push_back(point);
            }
            std::cout << TAG << "Num of points: " << numOfPoints << std::endl;


        }

        if (!inf.good())
            throw std::runtime_error("Reading .las ran into error");

        std::cout << TAG << "finished reading las file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .las file");
    }
}


bool DataIO::buildingCheck(const pcl::PointXYZRGBNormal& point, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
                           const float& maxWallRadius) {

    // create ray for this point
    auto rayPoint = pcl::PointXYZ(point.x, point.y, point.z);
    auto rayDir = pcl::PointXYZ(-0.5, 0, 0.5); // TODO choose sth smart?
    for (const auto& building: buildings) {
        if (isPointInBuildingBbox(building, point)) {

            // check if belongs to building
            int intersectionCount = 0;
            for (auto& bWall: building.osmWalls) {

                // check if near wall
                float dist = Util::pointPlaneDistance(point, bWall.mid);
                if (dist <= osmWallThreshold) {
                    if (Util::horizontalDistance(point, bWall.mid) <= bWall.length / 2) {
//                    if (point.x <= bWall.maxX && point.x >= bWall.minX && point.z <= bWall.maxZ && point.z >= bWall.minZ) {
                        // belongs to wall -> belongs to building
                        return true;
                    }
                }

                // intersect ray with wall
                intersectionCount += Util::intersectWall(bWall, rayPoint,
                                                         rayDir);// { // return 0 if no intersection, -1 for intersection from outside to inside, 1 for intersection in - out
            }

            if (intersectionCount % 2 != 0) {
                // point is inside of building
                return true;
            }
            // else continue search with other buildings
        }
    }
    // point belongs to no building
    return false;
}

void DataIO::filterAndColorPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                                  const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
                                  const float& maxWallRadius, const std::string& imgFile, std::vector<bool>& lasWallPoints,
                                  std::vector<bool>& lasGroundPoints, std::vector<pcl::PointXY>& texCoords) {

    // init cloud
    cloud->width = numOfPoints;
    cloud->height = 1;

    // read image
    std::vector<unsigned char> image;
    int width, height;
    const int channels = 3;
    if (!readImg(image, imgFile, channels, width, height)) {
        std::cout << "Error loading image\n";
        // TODO stop
    }

    // filter stuff
    int idxxx = 0;
    int sizeo = lasPoints.size();
    for (const auto& point: lasPoints) {

        bool belongsToWall = false;

        pcl::PointXYZRGBNormal v; // TODO nach unten später
        // center pointcloud - offset is in opengl coord system!
        // switch coord system: opengl (x, y, z) = engineer (x, z, -y)
        v.x = (float) (point.x - xOffset);
        v.y = (float) (point.z - yOffset);
        v.z = -(float) (point.y - zOffset);
        v.normal_x = -1;
        v.normal_y = -1;
        v.normal_z = -1;

        // pcl library switched r and b component
        v.b = 100; // r
        v.g = 100; // g
        v.r = 100; // b
        v.a = 255;

        v.curvature = 0;


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


        //region filter multiple return points

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
                belongsToWall = buildingCheck(v, wallOctree, maxWallRadius);

                if (!belongsToWall) {
                    continue;
                }
            } else if (returnNumber != numOfReturns) {
                // intermediate points
                // viel baum, ganz wenig wand -> raus
                if (colorReturnNumberClasses) {
                    v.b = 0;
                    v.g = 0;
                    v.r = 255;
                }
                continue;
            } else {
                // last of many
                // boden, bisschen wände, kein baum. einfach lassen
                if (classification != 2) { // not ground
                    if (colorReturnNumberClasses) {//} && belongsToWall) {
                        v.b = 0;
                        v.g = 255;
                        v.r = 0;
                    }
                    belongsToWall = buildingCheck(v, wallOctree, maxWallRadius);
                    if (!belongsToWall) {
                        continue;
                    }
                }
            }
        }
        //endregion

        int imageX = (point.x - 389000.05) * 10; // data from jp2 world file
        int imageY = (point.y - 5705999.95) * -10;
        texCoords.emplace_back(static_cast<float>(imageX) / 10000, static_cast<float>(imageY) / 10000);

        cloud->push_back(v);
        lasWallPoints.push_back(belongsToWall);
        lasGroundPoints.push_back(classification == 2);
    }
}


void DataIO::detectWalls(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& polygons, std::vector<bool>& lasWallPoints,
                         std::vector<bool>& lasGroundPoints,
                         const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<pcl::PointXY>& texCoords,
                         std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec, int& wallPointsStartIndex) {
    bool colorOsmWall = false;
    bool colorCertainLasWall = false;
    bool colorCertainLasWallRandom = false;
    bool colorFinalLasWall = false;
    bool colorFinalLasWallWithoutGround = false;
    bool removeOldWallPoints = false;

    // TODO im currently searching for each wall with lasPoint tree here, and searching for each point with wallTree in DataIO.
    //  -> make more efficient?

    pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(cloud);

    auto removePoints = std::vector<bool>(cloud->size());
    std::fill(removePoints.begin(), removePoints.end(), false);

    auto usedLasWallPoints = std::vector<bool>(lasWallPoints.size());
    std::fill(usedLasWallPoints.begin(), usedLasWallPoints.end(), false);




    // match osm and las walls
    for (auto bIdx = 0; bIdx < buildings.size(); bIdx++) {

        auto& building = buildings[bIdx];
        building.lasWalls = std::vector<std::optional<Util::Wall>>(building.osmWalls.size());

        std::map<int, pcl::Indices> osmWallSearchResults;
        std::map<int, pcl::Indices> lasCertainWallPoints;

        // for each wall
        for (auto osmWallIdx = 0; osmWallIdx < building.osmWalls.size(); osmWallIdx++) {

            // get wall
            const auto& osmWall = building.osmWalls[osmWallIdx];

//            int randR = 0;//rand() % (256);
//            int randG = 255;//rand() % (256);
//            int randB = 0;//rand() % (256);
            int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
            int randG = rand() % (156) + 100;
            int randB = rand() % (156) + 100;


            //region search points from osmWall mid-point with big radius

            pcl::Indices& pointIdxRadiusSearch = osmWallSearchResults[osmWallIdx];
            std::vector<float> pointRadiusSquaredDistance;
            float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
            auto searchRadius = static_cast<float>(sqrt(pow(osmWall.point2.x - osmWall.mid.x, 2) +
                                                        pow(wallHeight - osmWall.mid.y, 2) + pow(osmWall.point2.z - osmWall.mid.z, 2)));;
            if (tree->radiusSearch(osmWall.mid, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0) {
                continue;
            }
            //endregion

            auto& certainWallPoints = lasCertainWallPoints[osmWallIdx];
            //region filter search results for certain las wall points

            // only take osm wall points that are also las wall points
            for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {

                const auto& nIdx = *nIdxIt;
                const auto& point = (*cloud)[*nIdxIt];
                if (Util::pointPlaneDistance(cloud->points[*nIdxIt], osmWall.mid) > osmWallThreshold) {
                    continue;
                }
                if (Util::horizontalDistance(point, osmWall.mid) > osmWall.length / 2) {
                    continue;
                }
                if (colorOsmWall) {
                    (*cloud)[nIdx].b = randR;
                    (*cloud)[nIdx].g = randG;
                    (*cloud)[nIdx].r = randB;
                }
                if (!lasWallPoints[nIdx]) {
                    continue;
                }


                if (colorCertainLasWall) {
                    (*cloud)[nIdx].b = 0;
                    (*cloud)[nIdx].g = 255;
                    (*cloud)[nIdx].r = 0;
                }
                if (colorCertainLasWallRandom) {
                    (*cloud)[nIdx].b = randR;
                    (*cloud)[nIdx].g = randG;
                    (*cloud)[nIdx].r = randB;
                }
                certainWallPoints.push_back(nIdx);
                removePoints[nIdx] = true;
                usedLasWallPoints[nIdx] = true;

            }
            //endregion

            if (certainWallPoints.size() < 3)
                continue;

            Util::Wall lasWall;
            // TODO test different approaches to get wall plane (some 2d line fitting? least squares regression was bad)
            //region fit *vertical* plane through certain wall points

            pcl::IndicesPtr certainWallPointsPtr = std::make_shared<pcl::Indices>(certainWallPoints);
            pca.setIndices(certainWallPointsPtr);
            Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
            Eigen::Vector3f eigenValues = pca.getEigenValues();

            // create plane
            // get median point of certain wall points
            float xMedian, yMedian, zMedian;
            findXYZMedian(cloud, certainWallPoints, xMedian, yMedian, zMedian);
            lasWall.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
//                cloud->push_back(pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian, 0, 0, 255));
            // normal
            // check if normal is more vertical
            auto vertLen = abs(eigenVectors(1, 2));
            if (vertLen > 0.5) { // length adds up to 1
                continue; //TODO skip this wall for now
            } else {
                // make wall vertical
                auto lasWallNormal = Util::normalize(pcl::PointXYZ(eigenVectors(0, 2), 0, eigenVectors(2, 2)));
                lasWall.mid.normal_x = lasWallNormal.x;
                lasWall.mid.normal_y = lasWallNormal.y;
                lasWall.mid.normal_z = lasWallNormal.z;
            }
            // project wall start and end point to certain wall point plane
            // border points
            auto dist1 = Util::signedPointPlaneDistance(osmWall.point1, lasWall.mid);
            auto dist2 = Util::signedPointPlaneDistance(osmWall.point2, lasWall.mid);
            // move point um distance along plane normal (point - (dist * normal))
            lasWall.point1 = Util::vectorSubtract(osmWall.point1, pcl::PointXYZRGBNormal(dist1 * lasWall.mid.normal_x,
                                                                                         dist1 * lasWall.mid.normal_y, dist1 * lasWall.mid.normal_z));
            lasWall.point2 = Util::vectorSubtract(osmWall.point2, pcl::PointXYZRGBNormal(dist2 * lasWall.mid.normal_x,
                                                                                         dist2 * lasWall.mid.normal_y, dist2 * lasWall.mid.normal_z));
            auto lasWallVec = Util::vectorSubtract(lasWall.point2, lasWall.point1);
            auto horPerpVec = Util::normalize(lasWallVec); // horizontal
            // right orientation
            auto lasWallNormal = Util::crossProduct(horPerpVec, pcl::PointXYZ(0, -1, 0));
            lasWall.mid.normal_x = lasWallNormal.x;
            lasWall.mid.normal_y = lasWallNormal.y;
            lasWall.mid.normal_z = lasWallNormal.z;
            // new mid that is in the middle // TODO important
            lasWall.mid.x = (lasWall.point1.x + lasWall.point2.x) / 2.0f;
//            lasWall.mid.y = (lasWall.point1.y + lasWall.point2.y) / 2.0f; // TODO das macht keinen sinnd weil beide puntke bei -38 liegen, bleibt jetzt einfach beim meidan erstmal
            lasWall.mid.z = (lasWall.point1.z + lasWall.point2.z) / 2.0f;

            building.lasWalls[osmWallIdx] = lasWall; // TODO kann ich oben direkt value nehmen und da rein schreiben oder wie geht das mit optional?

            //endregion
        }


//        //region draw old las wall
//
//        for (auto osmWallIdx = 0; osmWallIdx < building.osmWalls.size(); osmWallIdx++) {
//
//            auto& lasWallOpt = building.lasWalls[osmWallIdx];
//            if (!lasWallOpt.has_value())
//                continue;
////            const auto& osmWall = osmWalls[osmWallIdx];
//            auto& lasWall = lasWallOpt.value();
//
//            int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
//            int randG = rand() % (156) + 100;
//            int randB = rand() % (156) + 100;
//
//
//            pcl::Indices finalWallPoints;
//            std::vector<pcl::PointXYZ> finalWallPointsNotGround;
//            //region get all las wall points with lasWallPlane and las border points   +   project these points onto plane
//
//            // get min max wall borders
//            auto lasMinX = std::min(lasWall.point1.x, lasWall.point2.x);
//            auto lasMaxX = std::max(lasWall.point1.x, lasWall.point2.x);
//            auto lasMinZ = std::min(lasWall.point1.z, lasWall.point2.z);
//            auto lasMaxZ = std::max(lasWall.point1.z, lasWall.point2.z);
//
//            if (osmWallSearchResults[osmWallIdx].size() == 0)
//                continue;
//            const auto& pointIdxRadiusSearch = osmWallSearchResults[osmWallIdx];
//
//            // select points with las wall plane with smaller threshold
//            for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {
//
//                const auto& nIdx = *nIdxIt;
//                const auto& point = (*cloud)[*nIdxIt];
//
//                if (Util::pointPlaneDistance(cloud->points[*nIdxIt], lasWall.mid) > lasWallThreshold) {
//                    continue;
//                }
//
//                if (point.x > lasMaxX || point.x < lasMinX || point.z > lasMaxZ || point.z < lasMinZ) {
//                    continue;
//                }
//
//                if (colorFinalLasWall) {
//                    (*cloud)[nIdx].b = randR;
//                    (*cloud)[nIdx].g = randG;
//                    (*cloud)[nIdx].r = randB;
//                }
//
//                // dont need to project these, because they are only used to determine y values and projection normal is horizontal
//                finalWallPoints.push_back(nIdx);
//
//                if (!lasGroundPoints[nIdx]) {
//                    if (colorFinalLasWallWithoutGround) {
//                        (*cloud)[nIdx].b = randR;
//                        (*cloud)[nIdx].g = randG;
//                        (*cloud)[nIdx].r = randB;
//                    }
//                    // project wall points onto las wallpoint plane
//                    auto pointDist = Util::signedPointPlaneDistance(point, lasWall.mid);
//                    auto newPosi = Util::vectorSubtract(point, pcl::PointXYZRGBNormal(pointDist * lasWall.mid.normal_x, pointDist * lasWall.mid.normal_y,
//                                                                                      pointDist * lasWall.mid.normal_z));
//                    finalWallPointsNotGround.push_back(newPosi);
//                    removePoints[nIdx] = true;
//                }
//            }
//            //endregion
//
//            if (finalWallPoints.empty())
//                continue;
//
//            //region fill wall with points
//
//            // get y min and max from finalWallPoints to cover wall from bottom to top
//            float yMin, yMax;
//            findYMinMax(cloud, finalWallPoints, yMin, yMax);
//            lasWall.point1.y = yMin; // TODo muss ich dashier üerhaupt setzen? sonst wieder const machen
//            lasWall.point2.y = yMin;
//
//            // draw plane
//            float stepWidth = 0.5;
//            // get perp vec
//            auto lasWallVec = Util::vectorSubtract(lasWall.point2, lasWall.point1);
//            auto horPerpVec = Util::normalize(lasWallVec); // horizontal
//            auto lasWallNormal = Util::crossProduct(horPerpVec, pcl::PointXYZ(0, -1, 0)); // TODO use stuff from wall struct
//
//            float lasWallLength = Util::vectorLength(lasWallVec);
//            float x = lasWall.point1.x;
//            float z = lasWall.point1.z;
//            float distanceMoved = 0;
//
//            // move horizontal
//            while (distanceMoved < lasWallLength) {
//                float y = yMin;
//                float xCopy = x;
//                float zCopy = z;
//                float currentMaxY = getMaxY(cloud, x, z, yMin, yMax, stepWidth, removePoints, lasWallNormal, tree);
////                if (currentMaxY > y + stepWidth) { // only build wall if more than init point
//                while (y < currentMaxY) {
//                    auto v = pcl::PointXYZRGBNormal(x, y, z, 0, 100, 0);//randR, randG, randB));
//                    // set normal
//                    v.normal_x = lasWallNormal.x;
//                    v.normal_y = lasWallNormal.y;
//                    v.normal_z = lasWallNormal.z;
//                    // also set tangents
//                    tangent1Vec.push_back(horPerpVec);
//                    tangent2Vec.emplace_back(0, 1, 0);
//                    texCoords.emplace_back(0, 0);
//
//                    cloud->push_back(v);
//
//                    y += stepWidth;
//                }
////                }
//                x = xCopy + stepWidth * horPerpVec.x;
//                z = zCopy + stepWidth * horPerpVec.z;
//                distanceMoved += stepWidth;
//            }
//            //endregion
//
//        }
//        //endregion




        float epsilon = 0.4;
        // find walls with high scattering
        for (auto osmWallIdx = 0; osmWallIdx < building.osmWalls.size(); osmWallIdx++) {

            auto& lasWallOpt = building.lasWalls[osmWallIdx];
            if (!lasWallOpt.has_value())
                continue;
            auto& lasWall = lasWallOpt.value();

            auto& certainWallPoints = lasCertainWallPoints[osmWallIdx];

            float scatter = 0;
            for (int certainWallPointIdx : certainWallPoints) {
                scatter += Util::pointPlaneDistance((*cloud)[certainWallPointIdx], lasWall.mid);
            }
            scatter /= certainWallPoints.size();

            if (scatter > epsilon) {
                auto bla = "groß";
                certainWallPoints.clear();
            } else {
                auto bla = "klein";
            }
        }

        // try to update las walls with high scatter and draw las walls
        for (auto osmWallIdx = 0; osmWallIdx < building.osmWalls.size(); osmWallIdx++) {

            auto& lasWallOpt = building.lasWalls[osmWallIdx];
            if (!lasWallOpt.has_value())
                continue;
            auto& lasWall = lasWallOpt.value();

            int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
            int randG = rand() % (156) + 100;
            int randB = rand() % (156) + 100;

            float r = 200;
            float g = 200;
            float b = 200;

            //region update wall points for walls with high scatter if neighbour walls are gut
            const auto& certainWallPoints = lasCertainWallPoints[osmWallIdx];
            if (certainWallPoints.empty()) {

                g = 255; // grün wand schief
                r = 0;
                b = 0;

                auto prevIdx = (osmWallIdx - 1) % building.osmWalls.size();
                if (building.parts.size() != 1) {
                    // check if leaving part
                    auto partIt = find(building.parts.begin(), building.parts.end(), osmWallIdx);
                    if (partIt != building.parts.end()) {
                        // leaving part at the start -> go back to last point of part
                        // skip to next part, go back one -> last wall of current part
                        partIt++;
                        if (partIt == building.parts.end()) {
                            // current part is last part -> use last wall
                            prevIdx = building.osmWalls.size() - 1;
                        } else {
                            prevIdx = *partIt - 1;
                        }
                    }
                }

                auto nextIdx = (osmWallIdx + 1) % building.osmWalls.size();
                if (building.parts.size() != 1) {
                    // if nextIdx is start of a part -> skip back to prev part. can be from first ring (idx0) to last part
                    auto partIt = find(building.parts.begin(), building.parts.end(), nextIdx);
                    if (partIt != building.parts.end()) {
                        // leaving part at the end -> go back to first point of that part
                        if (partIt == building.parts.begin()) {
                            // nextIdx is at start of walls -> current part is last part, use first wall of last part
                            nextIdx = building.parts.back();
                        } else {
                            // go back one part
                            partIt--;
                            nextIdx = *partIt;
                        }
                    }
                }

                // wenn nachbar wand nicht auch doof ist und existiert dann von der den punkt nehmen
                if (building.lasWalls[prevIdx].has_value() && !lasCertainWallPoints[prevIdx].empty()) {
                    auto& prevWall = building.lasWalls[prevIdx].value();
                    lasWall.point1 = prevWall.point2;
                    // update length
                    lasWall.length = Util::horizontalDistance(lasWall.point1, lasWall.point2);
                    g = 0;
                    r = 255; // blau wand gefixt
                    b = 0;
                }
                if (building.lasWalls[nextIdx].has_value() && !lasCertainWallPoints[nextIdx].empty()) {
                    auto& nextWall = building.lasWalls[nextIdx].value();
                    lasWall.point2 = nextWall.point1;
                    // update length
                    lasWall.length = Util::horizontalDistance(lasWall.point1, lasWall.point2);
                    g = 0;
                    r = 255; // blau wand gefixt
                    b = 0;
                }

            }
            //endregion

            pcl::Indices finalWallPoints; // TODO vllt sind die hier nicht so gut weil doch von den certain las wall points weggeschoben?
            std::vector<pcl::PointXYZ> finalWallPointsNotGround;
            //region get all las wall points with lasWallPlane and las border points   +   project these points onto plane

            // get min max wall borders
            auto lasMinX = std::min(lasWall.point1.x, lasWall.point2.x);
            auto lasMaxX = std::max(lasWall.point1.x, lasWall.point2.x);
            auto lasMinZ = std::min(lasWall.point1.z, lasWall.point2.z);
            auto lasMaxZ = std::max(lasWall.point1.z, lasWall.point2.z);

            if (osmWallSearchResults[osmWallIdx].empty())
                continue;
            const auto& pointIdxRadiusSearch = osmWallSearchResults[osmWallIdx];

            // select points with las wall plane with smaller threshold
            for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {

                const auto& nIdx = *nIdxIt;
                const auto& point = (*cloud)[*nIdxIt];

                if (Util::pointPlaneDistance(cloud->points[*nIdxIt], lasWall.mid) > lasWallThreshold) {
                    continue;
                }

                if (point.x > lasMaxX || point.x < lasMinX || point.z > lasMaxZ || point.z < lasMinZ) {
                    continue;
                }

                if (colorFinalLasWall) {
                    (*cloud)[nIdx].b = randR;
                    (*cloud)[nIdx].g = randG;
                    (*cloud)[nIdx].r = randB;
                }

                // dont need to project these, because they are only used to determine y values and projection normal is horizontal
                finalWallPoints.push_back(nIdx);

                if (!lasGroundPoints[nIdx]) {
                    if (colorFinalLasWallWithoutGround) {
                        (*cloud)[nIdx].b = randR;
                        (*cloud)[nIdx].g = randG;
                        (*cloud)[nIdx].r = randB;
                    }
                    // project wall points onto las wallpoint plane
                    auto pointDist = Util::signedPointPlaneDistance(point, lasWall.mid);
                    auto newPosi = Util::vectorSubtract(point, pcl::PointXYZRGBNormal(pointDist * lasWall.mid.normal_x, pointDist * lasWall.mid.normal_y,
                                                                                      pointDist * lasWall.mid.normal_z));
                    finalWallPointsNotGround.push_back(newPosi);
                    removePoints[nIdx] = true;
                }


//                    // project wall points onto las wallpoint plane
//                    auto pointDist = Util::signedPointPlaneDistance(point, lasWallPlane);
//                    auto newPosi = Util::vectorSubtract(point, pcl::PointXYZRGBNormal(pointDist * lasWallPlane.normal_x, pointDist * lasWallPlane.normal_y,
//                                                                                      pointDist * lasWallPlane.normal_z));
//                    (*cloud)[nIdx].x = newPosi.x;
//                    (*cloud)[nIdx].y = newPosi.y;
//                    (*cloud)[nIdx].z = newPosi.z;
            }
            //endregion

            if (finalWallPoints.empty())
                continue;


            //region fill wall with points

            // get y min and max from finalWallPoints to cover wall from bottom to top
            float yMin, yMax;
            // TODO vllt minY pro gebäude finden?
            findYMinMax(cloud, finalWallPoints, yMin, yMax);
            lasWall.point1.y = yMin;
            lasWall.point2.y = yMin;

            // draw plane
            float stepWidth = 0.5;
            // get perp vec
            auto lasWallVec = Util::vectorSubtract(lasWall.point2, lasWall.point1);
            auto horPerpVec = Util::normalize(lasWallVec); // horizontal
            auto lasWallNormal = Util::crossProduct(horPerpVec, pcl::PointXYZ(0, -1, 0)); // TODO use stuff from wall struct

            float lasWallLength = Util::vectorLength(lasWallVec);
            float x = lasWall.point1.x;
            float z = lasWall.point1.z;
            float distanceMoved = 0;


            // move horizontal
            while (distanceMoved < lasWallLength) {
                float y = yMin;
                float xCopy = x;
                float zCopy = z;
                float currentMaxY = getMaxY(cloud, x, z, yMin, yMax, stepWidth, removePoints, lasWallNormal, tree);
                if (currentMaxY > y + stepWidth) { // only build wall if more than init point
                    while (y < currentMaxY) {
                        auto v = pcl::PointXYZRGBNormal(x, y, z, r, g, b);//randR, randG, randB));

                        // set normal
                        v.normal_x = lasWallNormal.x;
                        v.normal_y = lasWallNormal.y;
                        v.normal_z = lasWallNormal.z;
                        // also set tangents
                        tangent1Vec.push_back(horPerpVec);
                        tangent2Vec.emplace_back(0, 1, 0);
                        texCoords.emplace_back(0, 0);

                        cloud->push_back(v);

                        y += stepWidth;
                    }
                }
                x = xCopy + stepWidth * horPerpVec.x;
                z = zCopy + stepWidth * horPerpVec.z;
                distanceMoved += stepWidth;
            }
            //endregion
        }
    }

//    // find walls that have no corresponding osm wall
//    for (auto pIdx = 0; pIdx < lasWallPoints.size(); pIdx++) {
//        if (lasWallPoints[pIdx] && !usedLasWallPoints[pIdx]) {
//            auto& point = (*cloud)[pIdx];
//            point.r = 255;
//        }
//    }

    if (removeOldWallPoints) {
        auto newPoints = std::vector<pcl::PointXYZRGBNormal>();
        auto newTexCoords = std::vector<pcl::PointXY>();
        // only keep old points that do not belong to walls
        for (auto pIdx = 0; pIdx < removePoints.size(); pIdx++) {
            if (!removePoints[pIdx]) {
                newPoints.push_back((*cloud)[pIdx]);
                newTexCoords.push_back(texCoords[pIdx]);
            }
        }
        wallPointsStartIndex = newPoints.size();

        // move partition of wall tangents in tangent1Vec and tangent2Vec to match point cloud by inserting or removing points
        int initCloudLength = removePoints.size();
        int pointCountDif = initCloudLength - wallPointsStartIndex; // is always >=0 because points just got removed
        // fewer points than before, remove points
        tangent1Vec.erase(tangent1Vec.begin(), tangent1Vec.begin() + pointCountDif);
        tangent2Vec.erase(tangent2Vec.begin(), tangent2Vec.begin() + pointCountDif);

        // keep all new points
        int tangentIdx = 0;
        for (auto pIdx = removePoints.size(); pIdx < (*cloud).size(); pIdx++) {
            newPoints.push_back((*cloud)[pIdx]);
            newTexCoords.emplace_back(0, 0); // TODO give texture to walls
        }

        (*cloud).clear();
        (*cloud).insert((*cloud).end(), newPoints.begin(), newPoints.end());
        texCoords.clear();
        texCoords.insert(texCoords.end(), newTexCoords.begin(), newTexCoords.end());
    }
}

bool xComparator(pcl::PointXYZRGBNormal& p1, pcl::PointXYZRGBNormal& p2) {
    return p1.x < p2.x;
}

bool yComparator(pcl::PointXYZRGBNormal& p1, pcl::PointXYZRGBNormal& p2) {
    return p1.y < p2.y;
}

bool zComparator(pcl::PointXYZRGBNormal& p1, pcl::PointXYZRGBNormal& p2) {
    return p1.z < p2.z;
}

void DataIO::findXYZMedian(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices, float& xMedian, float& yMedian,
                           float& zMedian) {
    auto points = std::vector<pcl::PointXYZRGBNormal>(pointIndices.size());
    for (auto i = 0; i < pointIndices.size(); i++) {
        const auto& pointIdx = pointIndices[i];
        points[i] = (*cloud)[pointIdx];
    }
    int n = pointIndices.size() / 2;
    std::nth_element(points.begin(), points.begin() + n, points.end(), xComparator);
    xMedian = points[n].x;
    std::nth_element(points.begin(), points.begin() + n, points.end(), yComparator);
    yMedian = points[n].y;
    std::nth_element(points.begin(), points.begin() + n, points.end(), zComparator);
    zMedian = points[n].z;
}

void DataIO::findYMinMax(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices, float& yMin, float& yMax) {
    auto points = std::vector<pcl::PointXYZRGBNormal>(pointIndices.size());
    for (auto i = 0; i < pointIndices.size(); i++) {
        const auto& pointIdx = pointIndices[i];
        points[i] = (*cloud)[pointIdx];
    }
    std::nth_element(points.begin(), points.begin(), points.end(), yComparator);
    yMin = points[0].y;
    std::nth_element(points.begin(), points.end() - 1, points.end(), yComparator);
    yMax = points[points.size() - 1].y;
}

/**
 *
 * @param x
 * @param z
 * @param yMin
 * @param yMax
 * @param stepWidth should be bigger then las wall radius
 * @param tree
 * @return
 */
float DataIO::getMaxY(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, float& x, float& z, float& yMin, float& yMax, float& stepWidth,
                      std::vector<bool>& removePoints, const pcl::PointXYZ& wallNormal, const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree) {

    // search points from (x,midY,z)
    float newMaxY = yMin;
    pcl::PointXYZ wallPlane = Util::crossProduct(wallNormal, pcl::PointXYZ(0, 1, 0));
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    auto searchPoint = pcl::PointXYZRGBNormal(x, (yMax + yMin) / 2.0, z);
    if (tree->radiusSearch(searchPoint, (yMax - yMin) / 2.0 * 1.5, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (auto pIdxIdx = 0; pIdxIdx < pointIdxRadiusSearch.size(); pIdxIdx++) {
            auto& point = (*cloud)[pointIdxRadiusSearch[pIdxIdx]];
            if (removePoints[pointIdxRadiusSearch[pIdxIdx]])
                continue;
            // distance to wall plane > 1.5
            float distToWall = abs(wallNormal.x * (searchPoint.x - point.x) + wallNormal.z * (searchPoint.z - point.z));
            if (distToWall > 1.5)
                continue;
            // distance to wall normal > stepWidth (n * searchPoint_point)
            float distToNormal = abs(wallPlane.x * (searchPoint.x - point.x) + wallPlane.z * (searchPoint.z - point.z));
            if (distToNormal > stepWidth * 1.1)
                continue;
            if (point.y > newMaxY) {
                newMaxY = point.y;
            }
        }
    }
    return newMaxY;
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

void DataIO::readShp(const std::string& path, std::vector<Polygon>* polygons) {

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
                    "Can't handle given SHP file. Only shapeType 5 (Polygon -> polygons) is allowed.");
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
                ShpPoint point;
                for (auto i = 0; i < polygonRecContent.numPoints; i++) {
                    inf.read((char*) &point, sizeof(ShpPoint));
                    bytesRead += sizeof(ShpPoint);
                    // convert point from w84 to utm zone 32n TODO is precise enough?
                    double utmX, utmZ;
                    LatLonToUTMXY(point.z, point.x, 32, utmX, utmZ);
                    // TODO - offset for opengl coord sys
                    point.x = utmX - xOffset;
                    point.z = -(utmZ - zOffset); // TODO negative z for opengl coord sys
                    polygon.points.push_back(point);
                }

                double utmMinX, utmMinZ;
                LatLonToUTMXY(polygonRecContent.yMin, polygonRecContent.xMin, 32, utmMinX, utmMinZ);
                double utmMaxX, utmMaxZ;
                LatLonToUTMXY(polygonRecContent.yMax, polygonRecContent.xMax, 32, utmMaxX, utmMaxZ);
                polygon.xMin = utmMinX - xOffset;
                polygon.xMax = utmMaxX - xOffset;
                polygon.zMin = -(utmMaxZ - zOffset); // swap min max because of -
                polygon.zMax = -(utmMinZ - zOffset);
                polygons->push_back(polygon);

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

float DataIO::preprocessWalls(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, std::vector<Polygon>& polygons) {
    // preprocessing of polygons to buildings
    // save all walls (min, mid, max point & radius)
    // dann beim normalen orientieren spatial search nach mid point mit max radius von allen walls
    float maxR = 0;
    buildings = std::vector<Building>(polygons.size());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallMidPoints = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for (auto bIdx = 0; bIdx < polygons.size(); bIdx++) {
        const auto& polygon = polygons[bIdx];
        auto& building = buildings[bIdx];
        building.xMin = polygon.xMin; // TODO read shp und preprocess walls verschmelzen
        building.xMax = polygon.xMax;
        building.zMin = polygon.zMin;
        building.zMax = polygon.zMax;

        // get point index of next part/ring if there are more than one, skip "walls" which connect different parts
        auto partIdxIt = polygon.parts.begin();
        uint32_t nextPartIndex = *partIdxIt;
        partIdxIt++;
        if (partIdxIt != polygon.parts.end()) {
            nextPartIndex = *partIdxIt;
        }
        building.parts.emplace_back(0); // first ring starts at first wall

        // for all walls
        float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
        float ground = -38; // minY // TODO boden ist wegen opengl offset grad bei -38
        for (auto pointIdx = 0; pointIdx < polygon.points.size() - 1; pointIdx++) {

            // if reached end of part/ring -> skip this "wall"
            if (pointIdx + 1 == nextPartIndex) {
                partIdxIt++;
                building.parts.emplace_back(building.osmWalls.size());
                if (partIdxIt != polygon.parts.end()) {
                    // there is another ring after the next
                    nextPartIndex = *partIdxIt;
                }
                continue;
            }

            building.osmWalls.emplace_back();
            auto& wall = building.osmWalls[building.osmWalls.size() - 1];

            // must have same height
            wall.point1.x = polygon.points[pointIdx].x;
            wall.point1.y = ground;
            wall.point1.z = polygon.points[pointIdx].z;
            wall.point2.x = polygon.points[pointIdx + 1].x;
            wall.point2.y = ground;
            wall.point2.z = polygon.points[pointIdx + 1].z;

            wall.mid.x = (wall.point1.x + wall.point2.x) / 2;
            wall.mid.y = (ground + wallHeight) / 2;
            wall.mid.z = (wall.point1.z + wall.point2.z) / 2;

            auto vec1 = Util::vectorSubtract(wall.point1, wall.point2);
            auto vec2 = Util::vectorSubtract(wall.mid, wall.point1);
            auto planeNormal = Util::normalize(Util::crossProduct(vec1, vec2));
            wall.mid.normal_x = planeNormal.x;
            wall.mid.normal_y = planeNormal.y;
            wall.mid.normal_z = planeNormal.z;

            float r = sqrt(pow(wall.point2.x - wall.mid.x, 2) + pow(wallHeight - wall.mid.y, 2) + pow(wall.point2.z - wall.mid.z, 2));
            if (r > maxR) {
                maxR = r;
            }

            wall.length = Util::horizontalDistance(wall.point1, wall.point2);

            wallMidPoints->push_back(wall.mid);

        }
    }

    wallOctree.setInputCloud(wallMidPoints);
    wallOctree.defineBoundingBox();
    wallOctree.addPointsFromInputCloud();

    return maxR;
}

// ********** img **********
bool DataIO::readImg(std::vector<unsigned char>& image, const std::string& imgFile, const int& desiredChannels, int& width, int& height) {
    std::string imgDir = ".." + Util::PATH_SEPARATOR + "img" + Util::PATH_SEPARATOR;
    std::string imgPath = imgDir + imgFile;
    // if file has less then desired channels, remaining fields will be 255
    int actualChannels;
    unsigned char* data = stbi_load(imgPath.c_str(), &width, &height, &actualChannels, desiredChannels);
    if (data != nullptr) {
        image = std::vector<unsigned char>(data, data + width * height * desiredChannels);
    }
    stbi_image_free(data);
    return (data != nullptr);
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

