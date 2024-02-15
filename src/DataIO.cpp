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

    std::vector<DataIO::Polygon> buildings;

    std::cout << TAG << "begin loading data" << std::endl;

    // read las file
    std::string lasDir = ".." + Util::PATH_SEPARATOR + "las" + Util::PATH_SEPARATOR;
    const auto& file = lasFiles[0];
    // sets: las to opengl offsets, bounds for shp and numOfPoints
    readLas(lasDir + file);

    // read shape file
    std::string shpDir = ".." + Util::PATH_SEPARATOR + "shp" + Util::PATH_SEPARATOR;
    readShp(shpDir + shpFile, &buildings);

    std::cout << TAG << "begin processing shp data" << std::endl;
    // preprocess buildings to osmWalls
    float resolution = 8.0f; // TODO find good value
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> wallOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>(
            resolution);
    // TODO add error handling if buildings are empty
    float maxWallRadius = preprocessWalls(wallOctree, buildings);

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

    std::cout << TAG << "begin detecting osmWalls" << std::endl;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);
    detectWalls(cloud, buildings, lasWallPoints, lasGroundPoints, tree, texCoords, tangent1Vec, tangent2Vec, wallPointsStartIndex);
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
        numOfPoints = 400000; //header.numberOfPoints;

        std::cout << TAG << "Num of points: " << numOfPoints << std::endl;
        inf.seekg(header.pointDataOffset); // skip to point tree
        inf.seekg(11300000 * (sizeof(PointDRF1) - 3 * sizeof(double) + 3 * sizeof(uint32_t)),
                  std::ios_base::cur); // skip to point tree TODO because i dont use all points for testing

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

    std::vector<int> visitedBuildings;

    std::vector<int> wallIdxRadiusSearch;
    std::vector<float> wallRadiusSquaredDistance;
    if (wallOctree.radiusSearch(point, maxWallRadius, wallIdxRadiusSearch, wallRadiusSquaredDistance) > 0) {

        // create ray for this point
        auto rayPoint = pcl::PointXYZ(point.x, point.y, point.z);
        auto rayDir = pcl::PointXYZ(-0.5, 0, 0.5); // TODO choose sth smart?

        // TODO mit building bbox suchen?
        for (auto searchWallIdx = 0; searchWallIdx < wallIdxRadiusSearch.size(); searchWallIdx++) {
            const auto& searchWall = osmWalls[wallIdxRadiusSearch[searchWallIdx]];


            // skip building belonging to this wall, if it has already been visited because of another wall
            if (std::find(visitedBuildings.begin(), visitedBuildings.end(), searchWall.buildingIdx) != visitedBuildings.end()) {
                continue;
            }

            visitedBuildings.push_back(searchWall.buildingIdx);

            // check if belongs to building
            int intersectionCount = 0;
            auto& bWalls = buildingOsmWallMap[searchWall.buildingIdx];
            for (auto bWallIdx: bWalls) {
                const auto& bWall = osmWalls[bWallIdx];

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


void DataIO::detectWalls(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& buildings, std::vector<bool>& lasWallPoints,
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

    // TODO draw osm walls for testing

    pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(cloud);

    auto removePoints = std::vector<bool>(cloud->size());
    std::fill(removePoints.begin(), removePoints.end(), false);

    auto usedLasWallPoints = std::vector<bool>(lasWallPoints.size());
    std::fill(usedLasWallPoints.begin(), usedLasWallPoints.end(), false);




    // match osm and las walls
    for (auto bIdx = 0; bIdx < buildingOsmWallMap.size(); bIdx++) {







//        //region draw osm walls
//
//        for (auto osmWallIdx: buildingOsmWallMap[bIdx]) {
//
//            const auto& osmWall = osmWalls[osmWallIdx];
//
//            //region fill wall with points
//
//            float yMin = -2;
//
//            // draw plane
//            float stepWidth = 0.5;
//            // get perp vec
//            auto lasWallVec = Util::vectorSubtract(osmWall.point2, osmWall.point1);
//            auto horPerpVec = Util::normalize(lasWallVec); // horizontal
//            auto lasWallNormal = Util::crossProduct(horPerpVec, pcl::PointXYZ(0, -1, 0)); // TODO use stuff from wall struct
//
//            float lasWallLength = Util::vectorLength(lasWallVec);
//            float x = osmWall.point1.x;
//            float z = osmWall.point1.z;
//            float distanceMoved = 0;
//
//            // move horizontal
//            while (distanceMoved < lasWallLength) {
//                float y = yMin;
//                float xCopy = x;
//                float zCopy = z;
//                while (y < 15) {
//                    auto v = pcl::PointXYZRGBNormal(x, y, z, 0, 0, 100);//randR, randG, randB));
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
//            auto v = pcl::PointXYZRGBNormal(osmWall.mid.x, 5, osmWall.mid.z, 0, 0, 255);//randR, randG, randB));
//            // set normal
//            v.normal_x = lasWallNormal.x;
//            v.normal_y = lasWallNormal.y;
//            v.normal_z = lasWallNormal.z;
//            // also set tangents
//            tangent1Vec.push_back(horPerpVec);
//            tangent2Vec.emplace_back(0, 1, 0);
//            texCoords.emplace_back(0, 0);
//            cloud->push_back(v);
//
//        }
//        //endregion












        auto const& building = buildings[bIdx];

        std::map<int, pcl::Indices> osmWallSearchResults;

        // for each wall
        for (auto osmWallIdx: buildingOsmWallMap[bIdx]) {

            // get wall
            const auto& osmWall = osmWalls[osmWallIdx];

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

            pcl::Indices certainWallPoints;
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

            // TODO vllt nicht median sondern mittelwert nehmen, dann den wall vec nehmen und border points berechnen mit richtiger länge?
            //  ne dafür sind die daten nicht gleichmäßig genug verteilt.. würde wand zu sehr verschieben
            lasWalls.push_back(lasWall);
            osmWallLasWallMap[osmWallIdx] = lasWalls.size()-1;
            //endregion
        }

        // TODO wenn die osmWallLasWallMap für den bIdx leer ist, kann ich continuen


        // get matrices and indices of corresponding las walls
        // matrix: cos angle, sin angle, transl x, transl z
        auto matrices = std::vector<std::array<float, 4>>(buildingOsmWallMap[bIdx].size());
        std::vector<int> osmWallsWithLasMapIndices;

        for (auto osmWallMapIdx = 0; osmWallMapIdx < buildingOsmWallMap[bIdx].size(); osmWallMapIdx++) {

            const auto osmWallIdx = buildingOsmWallMap[bIdx][osmWallMapIdx];

            // get all wall planes for this building and compute transf matrix from osm wall to las wall.
            //  then ransac to find best matrix with least error for all the osmWalls
            //  use this matrix to recompute the las walls, continue with those
            //  -> avoid single skewed osmWalls caused by outliers (from trees near osmWalls)

            const auto& lasWallIt = osmWallLasWallMap.find(osmWallIdx);
            if (lasWallIt == osmWallLasWallMap.end())
                continue;
            const auto& osmWall = osmWalls[osmWallIdx];
            auto& lasWall = lasWalls[lasWallIt->second];
            osmWallsWithLasMapIndices.push_back(osmWallMapIdx);

            // comp matrix from osm wall to las wall
            // need angle between normals and transform from osm point to las point
            // horizontal
            auto osmToLasVec = pcl::PointXYZ(lasWall.mid.x - osmWall.mid.x, 0, lasWall.mid.z - osmWall.mid.z);
            // counter clockwise from osm to las, range [-180, 180]
            auto osmToLasAngle = atan2(osmWall.mid.normal_x * lasWall.mid.normal_z - osmWall.mid.normal_z * lasWall.mid.normal_x, osmWall.mid.normal_x * lasWall.mid.normal_x + osmWall.mid.normal_z * lasWall.mid.normal_z);

            auto cosAngle = cos(osmToLasAngle);
            auto sinAngle = sin(osmToLasAngle);

            auto& matrix = matrices[osmWallMapIdx];
            matrix[0] = cosAngle;
            matrix[1] = sinAngle;
            matrix[2] = osmToLasVec.x;
            matrix[3] = osmToLasVec.z;

            // TODo vllt dann für las am anfagn doch mittelwert nehmen statt median?
        }

        // find matrix with smallest error
        float minError = INFINITY;
        int minErrorIdx;
        // get index sample;
        std::vector<int> out;
        size_t nelems = osmWallsWithLasMapIndices.size();// / 2; //TODO wie oft ransac?
        std::sample(
                osmWallsWithLasMapIndices.begin(),
                osmWallsWithLasMapIndices.end(),
                std::back_inserter(out),
                nelems,
                std::mt19937{std::random_device{}()}
        );

        for (auto testOsmWallMapIdx: out) {

            // get random index
//            int testOsmWallMapIdx = std::rand() % (buildingOsmWallMap[bIdx].size());

            // matrix: cos angle, sin angle, transl x, transl z
            const auto& matrix = matrices[testOsmWallMapIdx];

            const auto& cosAngle = matrix[0];
            const auto& sinAngle = matrix[1];
            const auto& translateX = matrix[2];
            const auto& translateZ = matrix[3];

            float error = 0;
            // sum up error for walls of this building
            for (auto osmWallMapIdx = 0; osmWallMapIdx < buildingOsmWallMap[bIdx].size(); osmWallMapIdx++) {
                const auto osmWallIdx = buildingOsmWallMap[bIdx][osmWallMapIdx];
                const auto& lasWallIt = osmWallLasWallMap.find(osmWallIdx);
                if (lasWallIt == osmWallLasWallMap.end())
                    continue;
                const auto& osmWall = osmWalls[osmWallIdx];
                auto& lasWall = lasWalls[lasWallIt->second];
                // check matrix
                pcl::PointXYZ newNormal;
                newNormal.x = cosAngle * osmWall.mid.normal_x - sinAngle * osmWall.mid.normal_z;
                newNormal.y = 0;
                newNormal.z = sinAngle * osmWall.mid.normal_x + cosAngle * osmWall.mid.normal_z;
                auto lasWallNormal = pcl::PointXYZ(lasWall.mid.normal_x, lasWall.mid.normal_y, lasWall.mid.normal_z);

                float newMidPointX = cosAngle * osmWall.mid.x - sinAngle * osmWall.mid.z + translateX;
                float newMidPointZ = sinAngle * osmWall.mid.x + cosAngle * osmWall.mid.z + translateZ;
                auto newMidPoint = pcl::PointXYZRGBNormal(newMidPointX, 0, newMidPointZ);

                // TODO bei normalen vllt winkel? dann iwie relativieren mit 90° oder so?
                float normalError = Util::vectorLength(Util::vectorSubtract(newNormal, lasWallNormal));
                // TODO ebenen abstand von matrix*osm mid zu korrespondierende las wall ebene
                float distError = Util::horizontalDistance(newMidPoint, lasWall.mid);
                // TODO vllt auch error der rand punkte mit nehmen? damit mid verschiebungen bei zu kurzen wänden rausfallen?  könnte sein dass die sowieso rausfallen
                error += normalError + distError; // TODO wie gewichten?
            }

            if (error < minError) {
                minError = error;
                minErrorIdx = testOsmWallMapIdx;
            }
        }

        if(minError == INFINITY)
            continue; //TODo notlösung: manche gebäude liegen nur teilweise in der las da ta range, dadurch haben nur manche wände davon eine entsprechende las wand.
            //  da kanns dann sein dass die oben bei den random dingern nicht dabei ist, dadurch der min error nicht gesetzt wird und dann crasht es unten.
            //  ich muss mir überlegen was ich dann machen will? vermutlich oben den random index so wählen dass er nur aus tatsächlichen las wänden gewählt wird






            //region draw old las wall

        for (auto osmWallIdx: buildingOsmWallMap[bIdx]) {

            const auto& lasWallIt = osmWallLasWallMap.find(osmWallIdx);
            if (lasWallIt == osmWallLasWallMap.end())
                continue;
//            const auto& osmWall = osmWalls[osmWallIdx];
            auto& lasWall = lasWalls[lasWallIt->second];

            int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
            int randG = rand() % (156) + 100;
            int randB = rand() % (156) + 100;


            pcl::Indices finalWallPoints;
            std::vector<pcl::PointXYZ> finalWallPointsNotGround;
            //region get all las wall points with lasWallPlane and las border points   +   project these points onto plane

            // get min max wall borders
            auto lasMinX = std::min(lasWall.point1.x, lasWall.point2.x);
            auto lasMaxX = std::max(lasWall.point1.x, lasWall.point2.x);
            auto lasMinZ = std::min(lasWall.point1.z, lasWall.point2.z);
            auto lasMaxZ = std::max(lasWall.point1.z, lasWall.point2.z);

            if (osmWallSearchResults[osmWallIdx].size() == 0)
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
            findYMinMax(cloud, finalWallPoints, yMin, yMax);
            lasWall.point1.y = yMin; // TODo muss ich dashier üerhaupt setzen? sonst wieder const machen
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
//                if (currentMaxY > y + stepWidth) { // only build wall if more than init point
                while (y < currentMaxY) {
                    auto v = pcl::PointXYZRGBNormal(x, y, z, 0, 100, 0);//randR, randG, randB));
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
//                }
                x = xCopy + stepWidth * horPerpVec.x;
                z = zCopy + stepWidth * horPerpVec.z;
                distanceMoved += stepWidth;
            }
            //endregion

            auto v = pcl::PointXYZRGBNormal(lasWall.mid.x, lasWall.mid.y, lasWall.mid.z, 0, 255, 0);//randR, randG, randB));
            // set normal
            v.normal_x = lasWallNormal.x;
            v.normal_y = lasWallNormal.y;
            v.normal_z = lasWallNormal.z;
            // also set tangents
            tangent1Vec.push_back(horPerpVec);
            tangent2Vec.emplace_back(0, 1, 0);
            texCoords.emplace_back(0, 0);

            cloud->push_back(v);

        }
        //endregion







        // use this matrix to transform all osm walls to new las walls
        // TODO danach vllt nochmal punkte rauswerfen die nicht an las wall sind, falls die outlier probleme machen

        for (auto osmWallMapIdx = 0; osmWallMapIdx < buildingOsmWallMap[bIdx].size(); osmWallMapIdx++) {

            const auto osmWallIdx = buildingOsmWallMap[bIdx][osmWallMapIdx];
            const auto& lasWallIt = osmWallLasWallMap.find(osmWallIdx);
            if (lasWallIt == osmWallLasWallMap.end()) // TODo die wände hier einfach neu dazu tun?
                continue;
            const auto& osmWall = osmWalls[osmWallIdx];
            auto& lasWall = lasWalls[lasWallIt->second];


            // matrix: cos angle, sin angle, transl x, transl z
            const auto& matrix = matrices[minErrorIdx];

            const auto& cosAngle = matrix[0];
            const auto& sinAngle = matrix[1];
            const auto& translateX = matrix[2];
            const auto& translateZ = matrix[3];

            // transform osm wall to las wall
            // update las wall
            float newPoint1X = osmWall.point1.x - osmWall.mid.x;
            float newPoint1Z = osmWall.point1.z - osmWall.mid.z;
            newPoint1X = cosAngle * newPoint1X - sinAngle * newPoint1X  +  osmWall.mid.x  +  translateX;
            newPoint1Z = sinAngle * newPoint1Z + cosAngle * newPoint1Z  +  osmWall.mid.z  +  translateZ;

            float newPoint2X = osmWall.point2.x - osmWall.mid.x;
            float newPoint2Z = osmWall.point2.z - osmWall.mid.z;
            newPoint2X = cosAngle * newPoint2X - sinAngle * newPoint2X  +  osmWall.mid.x  +  translateX;
            newPoint2Z = sinAngle * newPoint2Z + cosAngle * newPoint2Z  +  osmWall.mid.z  +  translateZ;

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

            float newMidPointX = cosAngle * osmWall.mid.x - sinAngle * osmWall.mid.z + translateX;
            float newMidPointZ = sinAngle * osmWall.mid.x + cosAngle * osmWall.mid.z + translateZ;
            lasWall.mid.x = newMidPointX;
            lasWall.mid.z = newMidPointZ;

        }

        for (auto osmWallIdx: buildingOsmWallMap[bIdx]) {

            const auto& lasWallIt = osmWallLasWallMap.find(osmWallIdx);
            if (lasWallIt == osmWallLasWallMap.end())
                continue;
//            const auto& osmWall = osmWalls[osmWallIdx];
            auto& lasWall = lasWalls[lasWallIt->second];

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

            if (osmWallSearchResults[osmWallIdx].size() == 0)
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
            lasWall.point1.y = yMin; // TODo muss ich dashier üerhaupt setzen? sonst wieder const machen
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
//                if (currentMaxY > y + stepWidth) { // only build wall if more than init point
                    while (y < currentMaxY) {
                        auto v = pcl::PointXYZRGBNormal(x, y, z, 200, 200, 200);//randR, randG, randB));
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
//                }
                x = xCopy + stepWidth * horPerpVec.x;
                z = zCopy + stepWidth * horPerpVec.z;
                distanceMoved += stepWidth;
            }
            //endregion

            auto v = pcl::PointXYZRGBNormal(lasWall.mid.x, lasWall.mid.y, lasWall.mid.z, 255, 0, 0);//randR, randG, randB));
            // set normal
            v.normal_x = lasWallNormal.x;
            v.normal_y = lasWallNormal.y;
            v.normal_z = lasWallNormal.z;
            // also set tangents
            tangent1Vec.push_back(horPerpVec);
            tangent2Vec.emplace_back(0, 1, 0);
            texCoords.emplace_back(0, 0);
            cloud->push_back(v);

            auto p1 = pcl::PointXYZRGBNormal(lasWall.point1.x, lasWall.mid.y, lasWall.point1.z, 0, 255, 0);//randR, randG, randB));
            // set normal
            p1.normal_x = lasWallNormal.x;
            p1.normal_y = lasWallNormal.y;
            p1.normal_z = lasWallNormal.z;
            // also set tangents
            tangent1Vec.push_back(horPerpVec);
            tangent2Vec.emplace_back(0, 1, 0);
            texCoords.emplace_back(0, 0);
            cloud->push_back(p1);

            auto p2 = pcl::PointXYZRGBNormal(lasWall.point2.x, lasWall.mid.y, lasWall.point2.z, 0, 0, 255);//randR, randG, randB));
            // set normal
            p2.normal_x = lasWallNormal.x;
            p2.normal_y = lasWallNormal.y;
            p2.normal_z = lasWallNormal.z;
            // also set tangents
            tangent1Vec.push_back(horPerpVec);
            tangent2Vec.emplace_back(0, 1, 0);
            texCoords.emplace_back(0, 0);
            cloud->push_back(p2);

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
            newTexCoords.emplace_back(0, 0); // TODO give texture to osmWalls
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

void DataIO::readShp(const std::string& path, std::vector<Polygon>* buildings) {

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
    // save all osmWalls (min, mid, max point & radius)
    // dann beim normalen orientieren spatial search nach mid point mit max radius von allen osmWalls
    float maxR = 0;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallMidPoints = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for (auto bIdx = 0; bIdx < buildings.size(); bIdx++) {
        const auto& building = buildings[bIdx];
        // get point index of next part/ring if there are more than one, skip "osmWalls" which connect different parts
        auto partIdx = building.parts.begin();
        uint32_t nextPartIndex = *partIdx;
        partIdx++;
        if (partIdx != building.parts.end()) {
            nextPartIndex = *partIdx;
        }
        // for all osmWalls
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

            Util::Wall wall;

            // must have same height
            wall.point1.x = building.points[pointIdx].x;
            wall.point1.y = ground;
            wall.point1.z = building.points[pointIdx].z;
            wall.point2.x = building.points[pointIdx + 1].x;
            wall.point2.y = ground;
            wall.point2.z = building.points[pointIdx + 1].z;

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

            wall.buildingIdx = bIdx;

            wall.length = Util::horizontalDistance(wall.point1, wall.point2);

            osmWalls.push_back(wall);
            wallMidPoints->push_back(wall.mid);

            buildingOsmWallMap[bIdx].push_back(osmWalls.size() - 1);

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

