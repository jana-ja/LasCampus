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
                      std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec,
                      int& wallPointsStartIndex) {

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
    // preprocess osmPolygons to osm walls
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
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);
    detectWalls(cloud, osmPolygons, lasWallPoints, lasGroundPoints, tree, texCoords, tangent1Vec, tangent2Vec,
                wallPointsStartIndex);
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


bool DataIO::buildingCheck(const pcl::PointXYZRGBNormal& point,
                           const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
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
                                  const float& maxWallRadius, const std::string& imgFile,
                                  std::vector<bool>& lasWallPoints,
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


void DataIO::detectWalls(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& polygons,
                         std::vector<bool>& lasWallPoints,
                         std::vector<bool>& lasGroundPoints,
                         const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree,
                         std::vector<pcl::PointXY>& texCoords,
                         std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec,
                         int& wallPointsStartIndex) {
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


    for (auto bIdx = 0; bIdx < buildings.size(); bIdx++) {

        auto& building = buildings[bIdx];
        //region match osm and las walls

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
                                                        pow(wallHeight - osmWall.mid.y, 2) +
                                                        pow(osmWall.point2.z - osmWall.mid.z, 2)));;
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
                                                                                         dist1 * lasWall.mid.normal_y,
                                                                                         dist1 * lasWall.mid.normal_z));
            lasWall.point2 = Util::vectorSubtract(osmWall.point2, pcl::PointXYZRGBNormal(dist2 * lasWall.mid.normal_x,
                                                                                         dist2 * lasWall.mid.normal_y,
                                                                                         dist2 * lasWall.mid.normal_z));
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

            building.lasWalls[osmWallIdx] = lasWall;

            //endregion
        }
        //endregion

        // skip if building has no las walls
        if (!std::any_of(building.lasWalls.begin(), building.lasWalls.end(),
                         [](std::optional<Util::Wall> wall) { return wall.has_value(); }))
            continue;


        simpleStableWalls(building, lasCertainWallPoints, cloud);
//        complexStableWalls(building);

        // region draw las walls

        for (auto osmWallIdx = 0; osmWallIdx < building.osmWalls.size(); osmWallIdx++) {

            auto& lasWallOpt = building.lasWalls[osmWallIdx];
            if (!lasWallOpt.has_value())
                continue;
            auto& lasWall = lasWallOpt.value();


            int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
            int randG = rand() % (156) + 100;
            int randB = rand() % (156) + 100;


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
                    auto newPosi = Util::vectorSubtract(point, pcl::PointXYZRGBNormal(pointDist * lasWall.mid.normal_x,
                                                                                      pointDist * lasWall.mid.normal_y,
                                                                                      pointDist *
                                                                                      lasWall.mid.normal_z));
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
            auto lasWallNormal = Util::crossProduct(horPerpVec,
                                                    pcl::PointXYZ(0, -1, 0)); // TODO use stuff from wall struct

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
                        auto v = pcl::PointXYZRGBNormal(x, y, z, 100, 100, 100);//randR, randG, randB));
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
        //endregion

    }


    wallsWithoutOsm(lasWallPoints, usedLasWallPoints, removePoints, tree, cloud, texCoords, tangent1Vec, tangent2Vec);


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

void DataIO::findStartEnd(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices,
                          pcl::PointXYZ& startPoint, pcl::PointXYZ& endPoint,
                          float& yMin, float& yMax) {
    auto points = std::vector<pcl::PointXYZRGBNormal>(pointIndices.size());
    for (auto i = 0; i < pointIndices.size(); i++) {
        const auto& pointIdx = pointIndices[i];
        points[i] = (*cloud)[pointIdx];
    }
    // randpunkte holen nach x achse // TODO was wenn wand parallel zu z achse?
    std::nth_element(points.begin(), points.begin(), points.end(), xComparator);
    startPoint.x = points[0].x;
    startPoint.y = points[0].y;
    startPoint.z = points[0].z;
    std::nth_element(points.begin(), points.end() - 1, points.end(), xComparator);
    endPoint.x = (points[points.size() - 1]).x;
    endPoint.y = (points[points.size() - 1]).y;
    endPoint.z = (points[points.size() - 1]).z;

    std::nth_element(points.begin(), points.begin(), points.end(), yComparator);
    yMin = points[0].y;
    std::nth_element(points.begin(), points.end() - 1, points.end(), yComparator);
    yMax = points[points.size() - 1].y;
}

void DataIO::findXYZMedian(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices,
                           float& xMedian, float& yMedian,
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

void DataIO::findYMinMax(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices,
                         float& yMin, float& yMax) {
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
float
DataIO::getMaxY(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, float& x, float& z, float& yMin, float& yMax,
                float& stepWidth,
                std::vector<bool>& removePoints, const pcl::PointXYZ& wallNormal,
                const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree) {

    // search points from (x,midY,z)
    float newMaxY = yMin;
    pcl::PointXYZ wallPlane = Util::crossProduct(wallNormal, pcl::PointXYZ(0, 1, 0));
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    auto searchPoint = pcl::PointXYZRGBNormal(x, (yMax + yMin) / 2.0, z);
    if (tree->radiusSearch(searchPoint, (yMax - yMin) / 2.0 * 1.5, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
        0) {
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

float DataIO::preprocessWalls(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
                              std::vector<Polygon>& polygons) {
    // preprocessing of polygons to buildings
    // save all walls (min, mid, max point & radius)
    // dann beim normalen orientieren spatial search nach mid point mit max radius von allen walls
    float maxR = 0;
    buildings = std::vector<Building>(polygons.size());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallMidPoints = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
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

            float r = sqrt(pow(wall.point2.x - wall.mid.x, 2) + pow(wallHeight - wall.mid.y, 2) +
                           pow(wall.point2.z - wall.mid.z, 2));
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
bool
DataIO::readImg(std::vector<unsigned char>& image, const std::string& imgFile, const int& desiredChannels, int& width,
                int& height) {
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
bool DataIO::readFeaturesFromCache(const std::string& normalPath,
                                   const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) {

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
void
DataIO::writeFeaturesToCache(const std::string& normalPath, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) {
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

void DataIO::simpleStableWalls(DataIO::Building& building, std::map<int, pcl::Indices>& lasCertainWallPoints,
                               const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) {
    float epsilon = 0.4;
    // find walls with high scattering
    for (auto osmWallIdx = 0; osmWallIdx < building.osmWalls.size(); osmWallIdx++) {

        auto& lasWallOpt = building.lasWalls[osmWallIdx];
        if (!lasWallOpt.has_value())
            continue;
        auto& lasWall = lasWallOpt.value();

        auto& certainWallPoints = lasCertainWallPoints[osmWallIdx];

        float scatter = 0;
        for (int certainWallPointIdx: certainWallPoints) {
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
    }
}

void DataIO::complexStableWalls(DataIO::Building& building) {


    if (building.parts.size() > 1) {
        auto bla = "blub";
    }

    // do it per building part
    for (auto pIdx = 0; pIdx < building.parts.size(); pIdx++) {
        const int& partStart = building.parts[pIdx];
        // letzter part ? -> end = building walls size : next part start
        const int& partEnd = (pIdx == building.parts.size() - 1) ? static_cast<int>(building.osmWalls.size())
                                                                 : building.parts[pIdx + 1];

        // matrix: cos angle, sin angle, transl x, transl z
        auto matrices = std::vector<std::array<float, 4>>(partEnd - partStart);
        std::vector<int> osmWallsWithLasWallIndices; // TODO weiter oben anlegen und darüber check machen ob es las walls gibt
        //region calculate transform matrices for corresponding osm and las walls

        float scaleFactor = 0.99;

        for (auto osmWallIdx = partStart; osmWallIdx < partEnd; osmWallIdx++) {

            // get all wall planes for this building and compute transf matrix from osm wall to las wall.
            //  then ransac to find best matrix with least error for all the osmWalls
            //  use this matrix to recompute the las walls, continue with those
            //  -> avoid single skewed osmWalls caused by outliers (from trees near osmWalls)

            int matrixIdx = osmWallIdx - partStart;
            auto& lasWallOpt = building.lasWalls[osmWallIdx];
            if (!lasWallOpt.has_value())
                continue;
            auto& lasWall = lasWallOpt.value();
            const auto& osmWall = building.osmWalls[osmWallIdx];

            osmWallsWithLasWallIndices.emplace_back(osmWallIdx);

            // comp matrix from osm wall to las wall
            // need angle between normals and transform from osm point to las point
            // horizontal
            auto osmToLasVec = pcl::PointXYZ(lasWall.mid.x - osmWall.mid.x, 0, lasWall.mid.z - osmWall.mid.z);
            // counter clockwise from osm to las, range [-180, 180]
            auto osmToLasAngle = atan2(
                    osmWall.mid.normal_x * lasWall.mid.normal_z - osmWall.mid.normal_z * lasWall.mid.normal_x,
                    osmWall.mid.normal_x * lasWall.mid.normal_x + osmWall.mid.normal_z * lasWall.mid.normal_z);

            auto cosAngle = cos(osmToLasAngle);
            auto sinAngle = sin(osmToLasAngle);

            auto& matrix = matrices[matrixIdx];
            matrix[0] = cosAngle;
            matrix[1] = sinAngle;
            matrix[2] = osmToLasVec.x;
            matrix[3] = osmToLasVec.z;

            // TODo vllt dann für las am anfagn doch mittelwert nehmen statt median?
        }
        //endregion


        float minError = INFINITY;
        int minErrorIdx;
        //region find matrix with smallest error

        // get index sample;
        std::vector<int> out;
        size_t nelems = osmWallsWithLasWallIndices.size();// / 2; //TODO wie oft ransac?
        std::sample(
                osmWallsWithLasWallIndices.begin(),
                osmWallsWithLasWallIndices.end(),
                std::back_inserter(out),
                nelems,
                std::mt19937{std::random_device{}()}
        );

        for (auto testWallIdx: out) {

            // get random index
//            int testOsmWallMapIdx = std::rand() % (buildingOsmWallMap[bIdx].size());

            // matrix: cos angle, sin angle, transl x, transl z
            auto testMatrixIdx = testWallIdx - partStart;
            const auto& matrix = matrices[testMatrixIdx];

            const auto& cosAngle = matrix[0];
            const auto& sinAngle = matrix[1];
            const auto& translateX = matrix[2];
            const auto& translateZ = matrix[3];

            float error = 0;
            // sum up error for walls of this part
            for (auto osmWallIdx = partStart; osmWallIdx < partEnd; osmWallIdx++) {

                auto& lasWallOpt = building.lasWalls[osmWallIdx];
                if (!lasWallOpt.has_value())
                    continue;
                auto& lasWall = lasWallOpt.value();
                const auto& osmWall = building.osmWalls[osmWallIdx];

                // check matrix
                pcl::PointXYZ newNormal;
                newNormal.x = cosAngle * osmWall.mid.normal_x - sinAngle * osmWall.mid.normal_z;
                newNormal.y = 0;
                newNormal.z = sinAngle * osmWall.mid.normal_x + cosAngle * osmWall.mid.normal_z;
                auto lasWallNormal = pcl::PointXYZ(lasWall.mid.normal_x, lasWall.mid.normal_y, lasWall.mid.normal_z);

                float transfNewMidPointX = (osmWall.mid.x - osmWall.mid.x) * scaleFactor;
                float transfNewMidPointZ = (osmWall.mid.z - osmWall.mid.z) * scaleFactor;
                float newMidPointX =
                        cosAngle * transfNewMidPointX - sinAngle * transfNewMidPointZ + osmWall.mid.x + translateX;
                float newMidPointZ =
                        sinAngle * transfNewMidPointX + cosAngle * transfNewMidPointZ + osmWall.mid.z + translateZ;
                auto newMidPoint = pcl::PointXYZRGBNormal(newMidPointX, 0, newMidPointZ);
//
//                float newMidPointX = cosAngle * osmWall.mid.x - sinAngle * osmWall.mid.z + translateX;
//                float newMidPointZ = sinAngle * osmWall.mid.x + cosAngle * osmWall.mid.z + translateZ;
//                auto newMidPoint = pcl::PointXYZRGBNormal(newMidPointX, 0, newMidPointZ);

                // TODO bei normalen vllt winkel? dann iwie relativieren mit 90° oder so?
                float normalError = Util::vectorLength(Util::vectorSubtract(newNormal, lasWallNormal));
                // TODO ebenen abstand von matrix*osm mid zu korrespondierende las wall ebene
                float distError = Util::horizontalDistance(newMidPoint, lasWall.mid);
                // TODO vllt auch error der rand punkte mit nehmen? damit mid verschiebungen bei zu kurzen wänden rausfallen?  könnte sein dass die sowieso rausfallen
                error += normalError + distError; // TODO wie gewichten?
                if (osmWallIdx == testWallIdx) {
                    auto ble = 2;
                }
            } // TODO warum hat transf mit matrix von wand 0 bei wand0 nen dist error > 0??

            if (error < minError) {
                minError = error;
                minErrorIdx = testWallIdx;
            }
        }
        //endregion

        if (minError == INFINITY)
            return; //TODo notlösung: manche gebäude liegen nur teilweise in der las da ta range, dadurch haben nur manche wände davon eine entsprechende las wand.
        //  da kanns dann sein dass die oben bei den random dingern nicht dabei ist, dadurch der min error nicht gesetzt wird und dann crasht es unten.
        //  ich muss mir überlegen was ich dann machen will? vermutlich oben den random index so wählen dass er nur aus tatsächlichen las wänden gewählt wird

        // region use this matrix to transform all osm walls to new las walls

        // TODO danach vllt nochmal punkte rauswerfen die nicht an las wall sind, falls die outlier probleme machen
        for (auto osmWallIdx = partStart; osmWallIdx < partEnd; osmWallIdx++) {

            auto& lasWallOpt = building.lasWalls[osmWallIdx];
            if (!lasWallOpt.has_value())
                continue;
            auto& lasWall = lasWallOpt.value();
            const auto& osmWall = building.osmWalls[osmWallIdx];


            const auto& transformOsmWall = building.osmWalls[minErrorIdx];
            // matrix: cos angle, sin angle, transl x, transl z
            const auto& matrix = matrices[minErrorIdx - partStart];

            const auto& cosAngle = matrix[0];
            const auto& sinAngle = matrix[1];
            const auto& translateX = matrix[2];
            const auto& translateZ = matrix[3];

//            auto scaleFactor = 0.99f;

            // transform osm wall to las wall
            // update las wall
            float transfNewPoint1X = (osmWall.point1.x - transformOsmWall.mid.x) * scaleFactor;
            float transfNewPoint1Z = (osmWall.point1.z - transformOsmWall.mid.z) * scaleFactor;
            float newPoint1X =
                    cosAngle * transfNewPoint1X - sinAngle * transfNewPoint1X + transformOsmWall.mid.x + translateX;
            float newPoint1Z =
                    sinAngle * transfNewPoint1Z + cosAngle * transfNewPoint1Z + transformOsmWall.mid.z + translateZ;

            float transfNewPoint2X = (osmWall.point2.x - transformOsmWall.mid.x) * scaleFactor;
            float transfNewPoint2Z = (osmWall.point2.z - transformOsmWall.mid.z) * scaleFactor;
            float newPoint2X =
                    cosAngle * transfNewPoint2X - sinAngle * transfNewPoint2X + transformOsmWall.mid.x + translateX;
            float newPoint2Z =
                    sinAngle * transfNewPoint2Z + cosAngle * transfNewPoint2Z + transformOsmWall.mid.z + translateZ;

            lasWall.point1.x = newPoint1X;
            lasWall.point1.z = newPoint1Z;
            lasWall.point2.x = newPoint2X;
            lasWall.point2.z = newPoint2Z;

            pcl::PointXYZ newNormal;
            newNormal.x = cosAngle * osmWall.mid.normal_x - sinAngle * osmWall.mid.normal_z;
            newNormal.y = 0;
            newNormal.z = sinAngle * osmWall.mid.normal_x + cosAngle * osmWall.mid.normal_z;

            lasWall.mid.normal_x = newNormal.x;
            lasWall.mid.normal_z = newNormal.z;

            float transfNewMidPointX = (osmWall.mid.x - transformOsmWall.mid.x) * scaleFactor;
            float transfNewMidPointZ = (osmWall.mid.z - transformOsmWall.mid.z) * scaleFactor;
            float newMidPointX =
                    cosAngle * transfNewMidPointX - sinAngle * transfNewMidPointZ + transformOsmWall.mid.x + translateX;
            float newMidPointZ =
                    sinAngle * transfNewMidPointX + cosAngle * transfNewMidPointZ + transformOsmWall.mid.z + translateZ;

            lasWall.mid.x = newMidPointX;
            lasWall.mid.z = newMidPointZ;

        }
        //endregion
    }
}

void DataIO::wallsWithoutOsm(std::vector<bool>& lasWallPoints, std::vector<bool>& usedLasWallPoints,
                             std::vector<bool>& removePoints,
                             const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& allPointsTree,
                             const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& allPointsCloud,
                             std::vector<pcl::PointXY>& texCoords,
                             std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec) {

    int colorCount = 7;
    int colors[][3] = {
            {255, 255, 0},
            {0,   255, 255},
            {255, 0,   255},
            {255, 125, 125},
            {125, 255, 125},
            {125, 125, 255},
            {255, 125, 0}
    };
    int colorIndex = 0;


    std::map<int, int> idxMap;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr remainingWallsCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::Indices pointSearchIndices;
    pcl::IndicesPtr pointSearchIndicesPtr = std::make_shared<pcl::Indices>(pointSearchIndices);

    //region find wall points that have no corresponding osm wall

    for (auto lasPointIdx = 0; lasPointIdx < lasWallPoints.size(); lasPointIdx++) {
        if (lasWallPoints[lasPointIdx] && !usedLasWallPoints[lasPointIdx]) {
            auto& point = (*allPointsCloud)[lasPointIdx];
            point.r = 0;
            point.g = 255;
            point.b = 255;
//            point.r = 100;
//            point.g = 100;
//            point.b = 100;

            remainingWallsCloud->push_back(point);
            idxMap[remainingWallsCloud->size() - 1] = lasPointIdx;
            pointSearchIndicesPtr->emplace_back(remainingWallsCloud->size() - 1);

        } else {
            auto& point = (*allPointsCloud)[lasPointIdx];
            point.r = 100;
            point.g = 100;
            point.b = 100;
        }
    }
    // endregion

    //region only use linear points
    //endregion

    std::vector<Util::Wall> wallPatches;
    std::vector<pcl::Indices> wallPatchPointIdc;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallPatchMids = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //region find small wall patches

    // tree will get updated indices when points are assigned to a wall patch
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr remainingWallsTree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    remainingWallsTree->setInputCloud(remainingWallsCloud);

    auto wallPointSkip = std::vector<bool>(remainingWallsCloud->size());
    std::fill(wallPointSkip.begin(), wallPointSkip.end(), false);
    // prepare pca
    pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(remainingWallsCloud);

    // for every wall point
    for (auto pIdx = 0; pIdx < remainingWallsCloud->size(); pIdx++) {
        if (wallPointSkip[pIdx])
            continue;

        auto& point = (*remainingWallsCloud)[pIdx];

        // radius search // TODO nachbarn können grade auch nicht linear sein, will ich das?
        pcl::Indices searchResultIdx;
        std::vector<float> searchResultDist;
        if (remainingWallsTree->radiusSearch(point, 1.0f, searchResultIdx, searchResultDist) < 3) {
            continue;
        }

        // prepare pca for neighbour points
        pcl::IndicesPtr searchResultIdxPtr = std::make_shared<pcl::Indices>(searchResultIdx);
        pca.setIndices(searchResultIdxPtr);

        bool isLinear;
        //region check if linear

        Eigen::Vector3f eigenValues = pca.getEigenValues();
        // local descriptors
        const auto& l1 = eigenValues(0);
        const auto& l2 = eigenValues(1);
        const auto& l3 = eigenValues(2);
        float linearity = (l1 - l2) / l1;
        float planarity = (l2 - l3) / l1;
        float sphericity = l3 / l1;
        std::array bla = {linearity, planarity, sphericity};
        int mainDings = std::distance(bla.begin(), std::max_element(bla.begin(), bla.end()));
        isLinear = mainDings == 0;
//        switch (mainDings) {
//            case 0:
//                // linearity is main
//                    (*allPointsCloud)[idxMap[pIdx]].r = 255;
//                    (*allPointsCloud)[idxMap[pIdx]].g = 0;
//                    (*allPointsCloud)[idxMap[pIdx]].b = 0;
//                break;
//            case 1:
//                // planarity is main
//                    (*allPointsCloud)[idxMap[pIdx]].r = 0;
//                    (*allPointsCloud)[idxMap[pIdx]].g = 255;
//                    (*allPointsCloud)[idxMap[pIdx]].b = 0;
//                break;
//            default:
//                // sphericity is main
//                    (*allPointsCloud)[idxMap[pIdx]].r = 0;
//                    (*allPointsCloud)[idxMap[pIdx]].g = 0;
//                    (*allPointsCloud)[idxMap[pIdx]].b = 255;
//                break;
//        }
//endregion
        if (!isLinear)
            continue;

        Util::Wall newWall;
        //region get mid and normal if strongest eigenvector is horizontal

        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();

        auto eigen1 = pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0));
        auto eigen2 = pcl::PointXYZ(eigenVectors(0, 1), eigenVectors(1, 1), eigenVectors(2, 1));
        auto eigen3 = pcl::PointXYZ(eigenVectors(0, 2), eigenVectors(1, 2), eigenVectors(2, 2));

        // für lineare sachen nicht normale nehmen sondern stärksten eigenvektor, nur der ist eindeutig, zweiter und die normale können um die lineare achse rotieren
        // TODO vllt auch bei dem normalen wand ding schauen wenn die normale vertikal ist dann über linearity und ersten eigenvektor gehen?
        // normal
        // continue if first eigenvector is too vertical
        if (abs(eigenVectors(1, 0)) > 0.3) { //TODO welcher wert?
            continue;
        }
        auto lasWallNormal = Util::normalize(
                Util::crossProduct(pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0)),
                                   pcl::PointXYZ(0, 1, 0)));

        float xMedian, yMedian, zMedian;
        findXYZMedian(remainingWallsCloud, searchResultIdx, xMedian, yMedian, zMedian);
        newWall.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
        newWall.mid.normal_x = lasWallNormal.x;
        newWall.mid.normal_y = lasWallNormal.y;
        newWall.mid.normal_z = lasWallNormal.z;
        //endregion


        // TODO nochmal größere suche before ich es zu patch ernenne?
//        int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
//        int randG = rand() % (156) + 100;
//        int randB = rand() % (156) + 100;

        // this is wall patch
        //region mark neighbours and get bounds of patch

        float minX, minY, minZ = INFINITY;
        float maxX, maxY, maxZ = -INFINITY;
        for (const auto& nIdx: searchResultIdx) {
            wallPointSkip[nIdx] = true;
            auto bla = std::find(pointSearchIndicesPtr->begin(), pointSearchIndicesPtr->end(), nIdx);
            if (bla != pointSearchIndicesPtr->end())
                pointSearchIndicesPtr->erase(bla);

//                (*allPointsCloud)[idxMap[nIdx]].b = randR;
//                (*allPointsCloud)[idxMap[nIdx]].g = randG;
//                (*allPointsCloud)[idxMap[nIdx]].r = randB;
            (*allPointsCloud)[idxMap[nIdx]].b = 200;
            (*allPointsCloud)[idxMap[nIdx]].g = 200;
            (*allPointsCloud)[idxMap[nIdx]].r = 200;

            (*allPointsCloud)[idxMap[nIdx]].normal_x = lasWallNormal.x;
            (*allPointsCloud)[idxMap[nIdx]].normal_y = lasWallNormal.y;
            (*allPointsCloud)[idxMap[nIdx]].normal_z = lasWallNormal.z;
        }
        //endregion

        // save point indices of this patch
        wallPatchPointIdc.push_back(searchResultIdx);
        // update tree -> points of this patch will not be included in next search
        remainingWallsTree->setInputCloud(remainingWallsCloud, pointSearchIndicesPtr);
        // add to patches
        wallPatches.push_back(newWall);
        wallPatchMids->push_back(newWall.mid);

    }
    //endregion

    std::vector<Util::Wall> finalWalls;
    //region combine wall patches

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr wallPatchTree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    wallPatchTree->setInputCloud(wallPatchMids);
    auto wallPatchSkip = std::vector<bool>(wallPatchMids->size());
    std::fill(wallPatchSkip.begin(), wallPatchSkip.end(), false);

//    auto wallPointSkip = std::vector<bool>(remainingWallsCloud->size());
//    std::fill(wallPointSkip.begin(), wallPointSkip.end(), false);

    int lookAt = 125; //165
    for (int patchIdx = 0; patchIdx < wallPatches.size(); patchIdx++) {

//        if(patchIdx > lookAt || patchIdx < lookAt) {
//            continue;
//        }

        if (wallPatchSkip[patchIdx])
            continue;


        const auto& wallPatch = wallPatches[patchIdx];
        // radius search
        pcl::Indices wallPatchSearchResultIdx;
        std::vector<float> wallPatchSearchResultDist;
        float searchRadius = 20.0f;
        if (wallPatchTree->radiusSearch(wallPatch.mid, searchRadius, wallPatchSearchResultIdx,
                                        wallPatchSearchResultDist) <= 0) {
            continue; //TODO was dann? auch aus einem patch ne wand machen?
        }
        // radius search points
        pcl::Indices wallPointSearchResultIdx;
        std::vector<float> wallPointSearchResultDist;
//        float searchRadius = 20.0f;
        remainingWallsTree->radiusSearch(wallPatch.mid, searchRadius, wallPointSearchResultIdx,
                                         wallPointSearchResultDist);


        pcl::Indices wallCandidatePointIdc;
        // die punkte vom patch selbst sind immer dabei
        wallCandidatePointIdc.insert(wallCandidatePointIdc.end(), wallPatchPointIdc[patchIdx].begin(),
                                     wallPatchPointIdc[patchIdx].end());
        // remove first point of radius search weil das ist der patch selbst
        wallPatchSearchResultIdx.erase(wallPatchSearchResultIdx.begin());
        wallPatchSearchResultDist.erase(wallPatchSearchResultDist.begin());
        // bounds
//        float combineWallBounds[6] = {INFINITY, -INFINITY, INFINITY, -INFINITY, INFINITY, -INFINITY}; //xmin, xmax, ymin, ymax, zmin, zmax

        auto patchNormal = pcl::PointXYZ(wallPatch.mid.normal_x, wallPatch.mid.normal_y, wallPatch.mid.normal_z);
        // TODO ich könnte merken zu welchem gebäude ich die punkte erkannt hab -> nicht punkte/patches von versch gebäuden mixen

        // remove skip patches
        auto newPatchEnd = std::remove_if(wallPatchSearchResultIdx.begin(), wallPatchSearchResultIdx.end(),
                                          [&wallPatchSkip](int idx) {
                                              return (wallPatchSkip[idx]);
                                          });
        wallPatchSearchResultIdx.erase(newPatchEnd, wallPatchSearchResultIdx.end());
        // remove skip points
        auto newPointEnd = std::remove_if(wallPointSearchResultIdx.begin(), wallPointSearchResultIdx.end(),
                                          [&wallPointSkip](int idx) {
                                              return (wallPointSkip[idx]);
                                          });
        wallPointSearchResultIdx.erase(newPointEnd, wallPointSearchResultIdx.end());


        if (wallPatchSearchResultIdx.empty()) {
            continue;
        }


        auto wallCandidate = Util::Wall();
        wallCandidate.mid = wallPatch.mid;

        // TODO versuch 1: grow suche nach patches und einzelpunkten: nur mit patches die wall anpassen
        //  versuch 2: von patch aus suchen damit man startwerte hat, dann einfach punkte suchen und hinzufügen und neue params


        float maxDist = -INFINITY;
        while (true) {
            // TODO maxdist vllt hier auf -inf setzen? damit bei größerer dist wenn ich von wann candidate mid suche nicht unnötig weiter suche wenn keine neuen eingefügt wurden
            if (wallPatchSearchResultIdx.empty() && wallPointSearchResultIdx.empty()) {
                // wenn einer hinzugefügt wurde, der nah am rand ist → search again with bigger radius
                if (maxDist > searchRadius * 0.8) {
                    // search again with bigger radius
                    searchRadius += 10;
                    // TODO macht es sinn radius search auch von wall candidate mid aus zu machen??
                    wallPatchTree->radiusSearch(wallPatch.mid, searchRadius, wallPatchSearchResultIdx,
                                                wallPatchSearchResultDist);
                    // remove patches that have already been taken
                    // it's useful to look at patches again that weren't near enough, because a bridge could be built by other patches
                    newPatchEnd = std::remove_if(wallPatchSearchResultIdx.begin(), wallPatchSearchResultIdx.end(),
                                                 [&wallPatchSkip](int idx) {
                                                     return (wallPatchSkip[idx]);
                                                 });
                    wallPatchSearchResultIdx.erase(newPatchEnd, wallPatchSearchResultIdx.end());

                    // points
                    remainingWallsTree->radiusSearch(wallPatch.mid, searchRadius, wallPointSearchResultIdx,
                                                     wallPointSearchResultDist);
                    // remove patches that have already been taken
                    // it's useful to look at patches again that weren't near enough, because a bridge could be built by other patches
                    newPointEnd = std::remove_if(wallPointSearchResultIdx.begin(), wallPointSearchResultIdx.end(),
                                                 [&wallPointSkip](int idx) {
                                                     return (wallPointSkip[idx]);
                                                 });
                    wallPointSearchResultIdx.erase(newPointEnd, wallPointSearchResultIdx.end());

                } else {
                    break;
                }


            }
            // TODO doch erst die mit schlechter normal rauswerfen?

            // check if there is a neighbour patch that is near the wall combi
            auto nearPatchIt = wallPatchSearchResultIdx.end();
            float nearPatchDist = INFINITY;
            for (auto wallPatchNeighIdxIt = wallPatchSearchResultIdx.begin();
                 wallPatchNeighIdxIt != wallPatchSearchResultIdx.end(); wallPatchNeighIdxIt++) {
                const auto& wallPatchNeighIdx = *wallPatchNeighIdxIt;
                auto& neighbourPatchPoint = wallPatchMids->points[wallPatchNeighIdx];

                // wenn es nah an irgendeinem punkt aus der combi ist dann go
                bool near = false;
                for (auto& cPointIdx: wallCandidatePointIdc) {
                    const auto& cPoint = (*remainingWallsCloud)[cPointIdx];
                    float dist = Util::distance(neighbourPatchPoint, cPoint); // TODO hor or generell?
                    if (dist <= 4.0f) {
                        near = true;
                        nearPatchDist = dist;
                        break;
                    }
                }
                if (near) {
                    nearPatchIt = wallPatchNeighIdxIt;
                    break;
                }
            }
            // punkt suche
            // check if there is a neighbour point that is near the wall combi
            auto nearPointIt = wallPointSearchResultIdx.end();
            float nearPointDist = INFINITY;
            for (auto wallPointNeighIdxIt = wallPointSearchResultIdx.begin();
                 wallPointNeighIdxIt != wallPointSearchResultIdx.end(); wallPointNeighIdxIt++) {
                const auto& wallPointNeighIdx = *wallPointNeighIdxIt;
                auto& neighbourPoint = remainingWallsCloud->points[wallPointNeighIdx];

                // wenn es nah an irgendeinem punkt aus der combi ist dann go
                bool near = false;
                for (auto& cPointIdx: wallCandidatePointIdc) {
                    const auto& cPoint = (*remainingWallsCloud)[cPointIdx];
                    float dist = Util::distance(neighbourPoint, cPoint); // TODO hor or generell?
                    if (dist <= 3.0f) {
                        near = true;
                        nearPointDist = dist;
                        break;
                    }
                }
                if (near) {
                    nearPointIt = wallPointNeighIdxIt;
                    break;
                }
            }

            // check if any neighbour was found
            if (nearPatchIt == wallPatchSearchResultIdx.end() && nearPointIt == wallPointSearchResultIdx.end()) {
                // found no near neighbour
//                for (auto wallPatchNeighIdxIt = wallPatchSearchResultIdx.begin(); wallPatchNeighIdxIt != wallPatchSearchResultIdx.end(); wallPatchNeighIdxIt++) {
//                    const auto& wallPatchNeighIdx = *wallPatchNeighIdxIt;
//                    auto& neighbourPatchPoint = wallPatchMids->points[wallPatchNeighIdx];
//                    auto& neighbourPatchAllPoints = wallPatchPointIdc[wallPatchNeighIdx];
//                    for (const auto& nIdx: neighbourPatchAllPoints) {
////                        (*allPointsCloud)[idxMap[nIdx]].b = 255;
////                        (*allPointsCloud)[idxMap[nIdx]].r = 0;
////                        (*allPointsCloud)[idxMap[nIdx]].g = 0;
//                    }
//                }
                // remove remaining points
                wallPatchSearchResultIdx.clear();
                wallPointSearchResultIdx.clear();
                continue;
            }

            // patch oder punkt, das mit der kleinsten dist hinzufügen


            if (nearPatchDist <= nearPointDist) {
                // nearest is patch
                // check if near neighbour also has good normal angle
                auto& neighbourPatchPoint = wallPatchMids->points[*nearPatchIt];
                auto neighbourNormal = pcl::PointXYZ(neighbourPatchPoint.normal_x, neighbourPatchPoint.normal_y,
                                                     neighbourPatchPoint.normal_z);
                float normalAngle = acos(
                        Util::dotProduct(patchNormal, neighbourNormal)); // TODO normal angle util func machen
                if (normalAngle <= 0.78f) {//0.78f) { // 45°

                    // check plane distance
                    auto ppd = Util::pointPlaneDistance(neighbourPatchPoint, wallCandidate.mid);
                    if (ppd < 1.0f) {
                        // found a patch for the combi
                        wallCandidatePointIdc.insert(wallCandidatePointIdc.end(),
                                                     wallPatchPointIdc[*nearPatchIt].begin(),
                                                     wallPatchPointIdc[*nearPatchIt].end());
                        // remove it from patch search
                        wallPatchSkip[*nearPatchIt] = true;
                        // save distance
                        auto dist = Util::distance(neighbourPatchPoint, wallPatch.mid);
                        if (dist > maxDist) {
                            maxDist = dist;
                        }
                        // update mid point
                        float xMedian, yMedian, zMedian;
                        findXYZMedian(remainingWallsCloud, wallCandidatePointIdc, xMedian, yMedian, zMedian);
                        wallCandidate.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);

                        // update normal
                        auto newNormal = pcl::PointXYZ(0, 0, 0);

                        pcl::IndicesPtr wallCandidatePointIdcPtr = std::make_shared<pcl::Indices>(
                                wallCandidatePointIdc);
                        pca.setIndices(wallCandidatePointIdcPtr);
                        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
                        newNormal = pcl::PointXYZ(eigenVectors(0, 2), 0, eigenVectors(2, 2));
                        newNormal = Util::normalize(newNormal);
                        wallCandidate.mid.normal_x = newNormal.x;
                        wallCandidate.mid.normal_y = newNormal.y;
                        wallCandidate.mid.normal_z = newNormal.z;

//                        // TODO für debug zum anschauen
//                        for (const auto& combWallIdx: wallCandidatePatchIdc) {
//                            auto& combWall = wallPatches[combWallIdx];
//                            combWall.mid.normal_x = newNormal.x;
//                            combWall.mid.normal_y = newNormal.y;
//                            combWall.mid.normal_z = newNormal.z;
//                        }
                    }
                }
                // remove patch from neighbours regardless if it has good angle and belongs to combi or not
                wallPatchSearchResultIdx.erase(nearPatchIt);
//                continue;

            } else {
                // nearest is point
                auto& neighbourPoint = remainingWallsCloud->points[*nearPointIt];

                // check plane distance
                auto ppd = Util::pointPlaneDistance(neighbourPoint, wallCandidate.mid);
                if (ppd < 1.0f) {
                    // found a patch for the combi
                    wallCandidatePointIdc.push_back(*nearPointIt);
                    // remove it from patch search
                    wallPointSkip[*nearPointIt] = true;
                    // save distance
                    auto dist = Util::distance(neighbourPoint, wallPatch.mid);
                    if (dist > maxDist) {
                        maxDist = dist;
                    }
                    // update mid point
                    float xMedian, yMedian, zMedian;
                    findXYZMedian(remainingWallsCloud, wallCandidatePointIdc, xMedian, yMedian, zMedian);
                    wallCandidate.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);

                    // update normal
                    auto newNormal = pcl::PointXYZ(0, 0, 0);

                    pcl::IndicesPtr wallCandidatePointIdcPtr = std::make_shared<pcl::Indices>(
                            wallCandidatePointIdc);
                    pca.setIndices(wallCandidatePointIdcPtr);
                    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
                    newNormal = pcl::PointXYZ(eigenVectors(0, 2), 0, eigenVectors(2, 2));
                    newNormal = Util::normalize(newNormal);
                    wallCandidate.mid.normal_x = newNormal.x;
                    wallCandidate.mid.normal_y = newNormal.y;
                    wallCandidate.mid.normal_z = newNormal.z;

//                        // TODO für debug zum anschauen
//                        for (const auto& combWallIdx: wallCandidatePatchIdc) {
//                            auto& combWall = wallPatches[combWallIdx];
//                            combWall.mid.normal_x = newNormal.x;
//                            combWall.mid.normal_y = newNormal.y;
//                            combWall.mid.normal_z = newNormal.z;
//                        }
                }

                // remove point from neighbours regardless if it has good angle and belongs to combi or not
                wallPointSearchResultIdx.erase(nearPointIt);
//                continue;
            }
        }




//        if (wallCandidatePatchIdc.size() < 3) { // TODO was mache ich jetzt hier?
//            continue;
//        }
        wallPatchSkip[patchIdx] = true; // sonst kann leitender patch noch wonaders mit reinkommen
        // patches
        int randB = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
        int randG = rand() % (156) + 100;
        int randR = rand() % (156) + 100;

//        auto& color = colors[colorIndex];
//        colorIndex = (colorIndex + 1) % colorCount;

        // ############## draw combined patches
        for (const auto& index: wallCandidatePointIdc) {

            (*allPointsCloud)[idxMap[index]].r = randR;
            (*allPointsCloud)[idxMap[index]].g = randG;
            (*allPointsCloud)[idxMap[index]].b = randB;
//                (*allPointsCloud)[idxMap[index]].r = color[0];
//                (*allPointsCloud)[idxMap[index]].g = color[1];
//                (*allPointsCloud)[idxMap[index]].b = color[2];

        }
        // den patch von dem die patch combi ausgeht anders anmalen
        auto blee2 = wallPatchPointIdc[patchIdx][1];
        (*allPointsCloud)[idxMap[blee2]].r = 0;
        (*allPointsCloud)[idxMap[blee2]].g = 0;
        (*allPointsCloud)[idxMap[blee2]].b = 255;
//            for (auto blee: wallPatchPointIdc[patchIdx]) {
//                (*allPointsCloud)[idxMap[blee]].r = 0;
//                (*allPointsCloud)[idxMap[blee]].g = 0;
//                (*allPointsCloud)[idxMap[blee]].b = 255;
//            }

        // TODO normale ist jetzt durchschnitt der patches, gut oder nochmal pca?

// TODO wall malen, danach weiter optimieren, scheinbar sind ein par patches nicht near die es sein sollten, und plane dist grenze vllt zu groß ajf sind da komisch entfernte patches scheinbar mit drin
//

        // region draw wall

        // wall: combWall
        // patches davon: wallCandidatePatchIdc
        // punkte davon: wallPatchPointIdc

        // TODO einzelne puntke die nicht zu patch geworden sind auch mit einsammeln um die borders und y min max besser zu machen
        //  wände nah an anderen wänden rauskicken ( vllt von hier alle neuen wände returnen und dann vor dem malen in der mutter funktion aussortieren?)
        //  manchen bullshit aufräumen (komscihe wände die entstehen)

        // merge pointIdx of all patches
//        pcl::Indices allWallCandidatePointIdc;
        for (auto& pointIdx: wallCandidatePointIdc) {

            // TODO wenn ich hier drüber loope könnte ich die eig auch direkt abspeichern und in findStartEnd geben anstatt da wieder zu loopen
            // project onto wall
            auto& point = (*remainingWallsCloud)[pointIdx];
            auto pointDist = Util::signedPointPlaneDistance(point, wallCandidate.mid);
            auto newPosi = Util::vectorSubtract(point, pcl::PointXYZRGBNormal(pointDist * wallCandidate.mid.normal_x,
                                                                              pointDist * wallCandidate.mid.normal_y,
                                                                              pointDist * wallCandidate.mid.normal_z));
            (*remainingWallsCloud)[pointIdx].x = newPosi.x;
            (*remainingWallsCloud)[pointIdx].y = newPosi.y;
            (*remainingWallsCloud)[pointIdx].z = newPosi.z;

        }
        // get start and end point of wall

        float yMin, yMax;
        findStartEnd(remainingWallsCloud, wallCandidatePointIdc, wallCandidate.point1, wallCandidate.point2, yMin,
                     yMax);
        wallCandidate.point1.y = yMin;
        wallCandidate.point2.y = yMin;
        wallCandidate.length = Util::horizontalDistance(wallCandidate.point1, wallCandidate.point2);


        // get y min and max from finalWallPoints to cover wall from bottom to top


        // draw plane
        float stepWidth = 0.5;
        // get perp vec
        auto wallVec = Util::vectorSubtract(wallCandidate.point2, wallCandidate.point1);
        auto horPerpVec = Util::normalize(wallVec); // horizontal
        auto wallNormal = Util::crossProduct(horPerpVec,
                                             pcl::PointXYZ(0, -1, 0)); // TODO use stuff from wall struct

        float lasWallLength = Util::vectorLength(wallVec);
        float x = wallCandidate.point1.x;
        float z = wallCandidate.point1.z;
        float distanceMoved = 0;


        // move horizontal
        while (distanceMoved < lasWallLength) {
            float y = yMin;
            float xCopy = x;
            float zCopy = z;
            float currentMaxY = yMax;//getMaxY(cloud, x, z, yMin, yMax, stepWidth, removePoints, wallNormal, tree);
            if (currentMaxY > y + stepWidth) { // only build wall if more than init point
                while (y < currentMaxY) {
                    auto v = pcl::PointXYZRGBNormal(x, y, z, 150, 150, 150);//randR, randG, randB));
                    // set normal
                    v.normal_x = wallNormal.x;
                    v.normal_y = wallNormal.y;
                    v.normal_z = wallNormal.z;
                    // also set tangents
                    tangent1Vec.push_back(horPerpVec);
                    tangent2Vec.emplace_back(0, 1, 0);
                    texCoords.emplace_back(0, 0);

                    allPointsCloud->push_back(v);

                    y += stepWidth;
                }
            }
            x = xCopy + stepWidth * horPerpVec.x;
            z = zCopy + stepWidth * horPerpVec.z;
            distanceMoved += stepWidth;
        }
        // mid point
        auto v = pcl::PointXYZRGBNormal(wallCandidate.mid.x, wallCandidate.mid.y, wallCandidate.mid.z, 255, 0,
                                        255);//randR, randG, randB));
        // set normal
        v.normal_x = wallNormal.x;
        v.normal_y = wallNormal.y;
        v.normal_z = wallNormal.z;
        // also set tangents
        tangent1Vec.push_back(horPerpVec);
        tangent2Vec.emplace_back(0, 1, 0);
        texCoords.emplace_back(0, 0);

        allPointsCloud->push_back(v);

        //endregion

//
//        if (wallCandidatePatchIdc.size() < 3) { // TODO needed for pca, what to do with samller dingsß
//            continue;
//        }
//        // found wall
//
//        Util::Wall finalWall;
//        //region build wall from patches
//
//        // prepare pca
//        pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
//        pca.setInputCloud(wallPatchMids);
//        pcl::IndicesPtr combineWallPatchIdxPtr = std::make_shared<pcl::Indices>(wallCandidatePatchIdc);
//        pca.setIndices(combineWallPatchIdxPtr);
//
//        //region get mid and normal if strongest eigenvector is horizontal
//
//        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
//
//        auto eigen1 = pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0));
//        auto eigen2 = pcl::PointXYZ(eigenVectors(0, 1), eigenVectors(1, 1), eigenVectors(2, 1));
//        auto eigen3 = pcl::PointXYZ(eigenVectors(0, 2), eigenVectors(1, 2), eigenVectors(2, 2));
//
//        // normal
//        // hier wieder über normale aus 3. eigenvektor machen weil nicht linear ist vermutlich
//        if (abs(eigenVectors(1, 2) > 0.5)) {// length adds up to 1
//            continue; //TODO skip this wall for now
//        }
////        auto finalWallNormal = Util::normalize( pcl::PointXYZ(eigenVectors(0, 2), 0, eigenVectors(2, 2)));
//        auto finalWallNormal = pcl::PointXYZ( wallPatch.mid.normal_x, 0, wallPatch.mid.normal_z);
////        // continue if first eigenvector is too vertical
////        if (abs(eigenVectors(1, 0)) > 0.3) { //TODO welcher wert?
////            continue;
////        }
////        auto finalWallNormal = Util::normalize(
////                Util::crossProduct(pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0)), pcl::PointXYZ(0, 1, 0)));
//
//        float xMedian, yMedian, zMedian;
//        findXYZMedian(wallPatchMids, wallCandidatePatchIdc, xMedian, yMedian, zMedian);
//        finalWall.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
//        finalWall.mid.normal_x = finalWallNormal.x;
//        finalWall.mid.normal_y = finalWallNormal.y;
//        finalWall.mid.normal_z = finalWallNormal.z;
//        // TODO radius berechnen, größter abstand (hor?) in den punkten?
//finalWall.length = 10.0f;
//        // TODO check if already wall near here
//        //  maybe just return all the final walls and draw and check in mother method
//        //endregion
//
//        // draw
//        // radius search on all points
//        pcl::Indices searchResultAllIdx;
//        std::vector<float> searchResultAllDist;
//        if (allPointsTree->radiusSearch(finalWall.mid, finalWall.length, searchResultAllIdx, searchResultAllDist) <= 0) {
//            continue; //TODO was dann? auch aus einem patch ne wand machen?
//        }
////        int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
////        int randG = rand() % (156) + 100;
////        int randB = rand() % (156) + 100;
//        // select points with final wall plane
//        for (const auto& fIdx: searchResultAllIdx) {
//
//            const auto& point = (*allPointsCloud)[fIdx];
//
//            if (Util::pointPlaneDistance(allPointsCloud->points[fIdx], finalWall.mid) > 1.0) {// TODO threshold
//                continue;
//            }
//
//            if (Util::horizontalDistance(point, finalWall.mid) > finalWall.length / 2.0f) {
//                continue;
//            }
//
//
////                (*allPointsCloud)[fIdx].b = randR;
////                (*allPointsCloud)[fIdx].g = randG;
////                (*allPointsCloud)[fIdx].r = randB;
//
//            // dont need to project these, because they are only used to determine y values and projection normal is horizontal
//
//
//        }
//
//        //endregion

    }

    //endregion
}

