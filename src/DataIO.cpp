//
// Created by Jana on 12.11.2023.
//

#include "DataIO.h"
#include <numeric>
#include <fstream>
#include <pcl/common/pca.h>
#include <cstdlib>
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
    // preprocess buildings to walls
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

    std::cout << TAG << "begin detecting walls" << std::endl;
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
        inf.seekg(11300000 * (sizeof(PointDRF1) - 3 * sizeof(double) + 3 * sizeof(uint32_t)), std::ios_base::cur); // skip to point tree TODO because i dont use all points for testing

        if (header.pointDataRecordFormat == 1) {
            for (uint32_t i = 0; i < numOfPoints; i++) {//header.numberOfPoints; i++) {
                PointDRF1 point;
                uint32_t x,y,z;
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


bool DataIO::buildingCheck(const pcl::PointXYZRGBNormal& point, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree, const float& maxWallRadius) {

    std::vector<int> visitedBuildings;

    std::vector<int> wallIdxRadiusSearch;
    std::vector<float> wallRadiusSquaredDistance;
    if (wallOctree.radiusSearch(point, maxWallRadius, wallIdxRadiusSearch, wallRadiusSquaredDistance) > 0) {

        // create ray for this point
        auto rayPoint = pcl::PointXYZ(point.x, point.y, point.z);
        auto rayDir = pcl::PointXYZ(-0.5, 0, 0.5); // TODO choose sth smart?

        // TODO mit building bbox suchen?
        for (auto searchWallIdx = 0; searchWallIdx < wallIdxRadiusSearch.size(); searchWallIdx++) {
            const auto& searchWall = walls[wallIdxRadiusSearch[searchWallIdx]];


            // skip building belonging to this wall, if it has already been visited because of another wall
            if (std::find(visitedBuildings.begin(), visitedBuildings.end(), searchWall.buildingIdx) != visitedBuildings.end()) {
                continue;
            }

            visitedBuildings.push_back(searchWall.buildingIdx);

            // check if belongs to building
            int intersectionCount = 0;
            auto& bWalls = buildingWallMap[searchWall.buildingIdx];
            for (auto bWallIdx: bWalls) {
                const auto& bWall = walls[bWallIdx];

                // check if near wall
                float dist = Util::pointPlaneDistance(point, bWall.mid);
                if (dist <= osmWallThreshold) {
                    if (point.x <= bWall.maxX && point.x >= bWall.minX && point.z <= bWall.maxZ && point.z >= bWall.minZ) {
                        // belongs to wall -> belongs to building
                       return true;
                    }
                }

                // intersect ray with wall
                intersectionCount += Util::intersectWall(bWall, rayPoint, rayDir);// { // return 0 if no intersection, -1 for intersection from outside to inside, 1 for intersection in - out
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

void DataIO::filterAndColorPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
                                  const float& maxWallRadius, const std::string& imgFile, std::vector<bool>& lasWallPoints,
                                  std::vector<bool>& lasGroundPoints, std::vector<pcl::PointXY>& texCoords){

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
                    if (colorReturnNumberClasses){//} && belongsToWall) {
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
        texCoords.emplace_back(static_cast<float>(imageX)/10000, static_cast<float>(imageY)/10000);

        cloud->push_back(v);
        lasWallPoints.push_back(belongsToWall);
        lasGroundPoints.push_back(classification == 2);
    }
}


void DataIO::detectWalls(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& buildings, std::vector<bool>& lasWallPoints, std::vector<bool>& lasGroundPoints,
                         const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<pcl::PointXY>& texCoords,
                         std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec, int& wallPointsStartIndex) {
    bool colorOsmWall = false;
    bool colorCertainLasWall = false;
    bool colorCertainLasWallRandom = false;
    bool colorFinalLasWall = false;
    bool colorFinalLasWallWithoutGround = false;
    bool removeOldWallPoints = true;

    // TODO im currently searching for each wall with lasPoint tree here, and searching for each point with wallTree in DataIO.
    //  -> make more efficient?

    bool oneWall = false;

    pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(cloud);

    auto removePoints = std::vector<bool>(cloud->size());
    std::fill(removePoints.begin(), removePoints.end(), false);

    auto usedLasWallPoints = std::vector<bool>(lasWallPoints.size());
    std::fill(usedLasWallPoints.begin(), usedLasWallPoints.end(), false);


    int buildingCount = 0;
    int wallCount = 0;
    int buildingsNumber = 184;
    int wallNumber = 12; // ich will wand 12 und 14, 13 ist iwie son dummer stumpf??


    // match osm and las walls
    for (auto bIdx = 0; bIdx < buildingWallMap.size(); bIdx++) {
//    for (auto building: buildings) {
        auto const& building = buildings[bIdx];

        //region skippi

        if (oneWall) {
            if (buildingCount > buildingsNumber)
                break;
            buildingCount++;
            if (buildingCount < buildingsNumber)
                continue;
        }
        // get point index of next part/ring if there are more than one, skip "walls" which connect different parts
        auto partIdx = building.parts.begin();
        uint32_t nextPartIndex = *partIdx;
        partIdx++;
        if (partIdx != building.parts.end()) {
            nextPartIndex = *partIdx;
        }
        //endregion

        // for each wall
        wallCount = 0;
        for (auto bPointIdx = 0; bPointIdx < building.points.size() - 1; bPointIdx++) {

            //region skippi

            if (oneWall) {
                if (wallCount - 2 > wallNumber) // will 3 walls
                    break;
                wallCount++;
                if (wallCount - 1 < wallNumber)
                    continue;
            }
            // if reached end of part/ring -> skip this "wall"
            if (bPointIdx + 1 == nextPartIndex) {
                partIdx++;
                if (partIdx != building.parts.end()) {
                    nextPartIndex = *partIdx;
                }
                continue;
            }
            //endregion

            auto flippi = false;
//            int randR = 0;//rand() % (256);
//            int randG = 255;//rand() % (256);
//            int randB = 0;//rand() % (256);
            int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
            int randG = rand() % (156) + 100;
            int randB = rand() % (156) + 100;


            pcl::PointXYZRGBNormal osmWallPoint1, osmWallPoint2; // must have same height
            pcl::PointXYZRGBNormal osmWallPlane;
            float searchRadius;
            //region set osm wall end points, set osm wall plane with mid point and normal, set radius for search

            float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
            float ground = -38; // minY // TODO boden ist wegen opengl offset grad bei -38
            osmWallPoint1.x = static_cast<float>(building.points[bPointIdx].x);
            osmWallPoint1.y = ground + wallHeight;
            osmWallPoint1.z = static_cast<float>(building.points[bPointIdx].z);
            osmWallPoint2.x = static_cast<float>(building.points[bPointIdx + 1].x);
            osmWallPoint2.y = ground + wallHeight;
            osmWallPoint2.z = static_cast<float>(building.points[bPointIdx + 1].z);
            // compute mid point for osmWallPlane
            osmWallPlane.x = (osmWallPoint1.x + osmWallPoint2.x) / 2;
            osmWallPlane.y = (ground + wallHeight) / 2;
            osmWallPlane.z = (osmWallPoint1.z + osmWallPoint2.z) / 2;
            // set normal of osmWallPlane
            auto vec1 = Util::vectorSubtract(osmWallPoint1, osmWallPoint2);
            auto vec2 = Util::vectorSubtract(osmWallPoint1, osmWallPlane);
            auto osmPlaneNormal = Util::normalize(Util::crossProduct(vec1, vec2));
            osmWallPlane.normal_x = osmPlaneNormal.x;
            osmWallPlane.normal_y = osmPlaneNormal.y;
            osmWallPlane.normal_z = osmPlaneNormal.z;
            // compute search radius for osmWallPlane
            searchRadius = static_cast<float>(sqrt(
                    pow(osmWallPoint2.x - osmWallPlane.x, 2) + pow(wallHeight - osmWallPlane.y, 2) + pow(osmWallPoint2.z - osmWallPlane.z, 2)));

            //endregion

            // search points from osmWallPlane point with big radius
            pcl::Indices pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if (tree->radiusSearch(osmWallPlane, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {

                pcl::Indices certainWallPoints;
                //region filter search results for certain las wall points

                auto osmMinX = std::min(osmWallPoint1.x, osmWallPoint2.x);
                auto osmMaxX = std::max(osmWallPoint1.x, osmWallPoint2.x);
                auto osmMinZ = std::min(osmWallPoint1.z, osmWallPoint2.z);
                auto osmMaxZ = std::max(osmWallPoint1.z, osmWallPoint2.z);

                // only take osm wall points that are also las wall points
                for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {

                    const auto& nIdx = *nIdxIt;
                    const auto& point = (*cloud)[*nIdxIt];
                    if (Util::pointPlaneDistance(cloud->points[*nIdxIt], osmWallPlane) > osmWallThreshold) {
                        continue;
                    }
                    if (point.x > osmMaxX || point.x < osmMinX || point.z > osmMaxZ || point.z < osmMinZ) {
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

                    // TODO nochmal die sicheren wandpunkte nach wänden einfärben und schauen welche zsm gehören, warum ist lange wand trotz median nur links??
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

                pcl::PointXYZRGBNormal lasWallPlane;
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
                lasWallPlane = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
//                cloud->push_back(pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian, 0, 0, 255));
                // normal
                // check if normal is more vertical
                auto vertLen = abs(eigenVectors(1, 2));
                if (vertLen > 0.5) { // length adds up to 1
                    continue; //TODO skip this wall for now
                } else {
                    // make wall vertical
                    auto lasWallNormal = Util::normalize(pcl::PointXYZ(eigenVectors(0, 2), 0, eigenVectors(2, 2)));
                    lasWallPlane.normal_x = lasWallNormal.x;
                    lasWallPlane.normal_y = lasWallNormal.y;
                    lasWallPlane.normal_z = lasWallNormal.z;
                }
                //endregion

                pcl::PointXYZ lasWallPoint1, lasWallPoint2;
                //region get border points for las wall by projecting osm border points onto lasWallPlane

                // project wall start and end point to certain wall point plane
                // get distance to plane
                auto dist1 = Util::signedPointPlaneDistance(osmWallPoint1, lasWallPlane);
                auto dist2 = Util::signedPointPlaneDistance(osmWallPoint2, lasWallPlane);
                // move point um distance along plane normal (point - (dist * normal))
                lasWallPoint1 = Util::vectorSubtract(osmWallPoint1, pcl::PointXYZRGBNormal(dist1 * lasWallPlane.normal_x,
                                                                                           dist1 * lasWallPlane.normal_y, dist1 * lasWallPlane.normal_z));
                lasWallPoint2 = Util::vectorSubtract(osmWallPoint2, pcl::PointXYZRGBNormal(dist2 * lasWallPlane.normal_x,
                                                                                           dist2 * lasWallPlane.normal_y, dist2 * lasWallPlane.normal_z));
                //endregion

                pcl::Indices finalWallPoints;
                std::vector<pcl::PointXYZ> finalWallPointsNotGround;
                //region get all las wall points with lasWallPlane and las border points   +   project these points onto plane

                // get min max wall borders
                auto lasMinX = std::min(lasWallPoint1.x, lasWallPoint2.x);
                auto lasMaxX = std::max(lasWallPoint1.x, lasWallPoint2.x);
                auto lasMinZ = std::min(lasWallPoint1.z, lasWallPoint2.z);
                auto lasMaxZ = std::max(lasWallPoint1.z, lasWallPoint2.z);

                // select points with las wall plane with smaller threshold
                for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {

                    const auto& nIdx = *nIdxIt;
                    const auto& point = (*cloud)[*nIdxIt];

                    if (Util::pointPlaneDistance(cloud->points[*nIdxIt], lasWallPlane) > lasWallThreshold) {
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
                        auto pointDist = Util::signedPointPlaneDistance(point, lasWallPlane);
                        auto newPosi = Util::vectorSubtract(point, pcl::PointXYZRGBNormal(pointDist * lasWallPlane.normal_x, pointDist * lasWallPlane.normal_y,
                                                                                          pointDist * lasWallPlane.normal_z));
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
                lasWallPoint1.y = yMin;
                lasWallPoint2.y = yMin;

                // draw plane
                float stepWidth = 0.5;
                // get perp vec
                auto lasWallVec = Util::vectorSubtract(lasWallPoint2, lasWallPoint1);
                auto horPerpVec = Util::normalize(lasWallVec); // horizontal
                auto lasWallNormal = Util::crossProduct(horPerpVec, pcl::PointXYZ(0,-1,0));

                float lasWallLength = Util::vectorLength(lasWallVec);
                float x = lasWallPoint1.x;
                float z = lasWallPoint1.z;
                float distanceMoved = 0;

                // move horizontal
                while (distanceMoved < lasWallLength) {
                    float y = yMin;
                    float xCopy = x;
                    float zCopy = z;
                    float currentMaxY = getMaxY(cloud, x, z, yMin, yMax, stepWidth, removePoints, lasWallNormal, tree);
                    if (currentMaxY > y + stepWidth) { // only build wall if more than init point
                        while (y < currentMaxY) {
                            auto v = pcl::PointXYZRGBNormal(x, y, z, 100,100,100);//randR, randG, randB));
                            // set normal
                            v.normal_x = lasWallNormal.x;
                            v.normal_y = lasWallNormal.y;
                            v.normal_z = lasWallNormal.z;
                            // also set tangents
                            tangent1Vec.push_back(horPerpVec);
                            tangent2Vec.emplace_back(0,1,0);

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
    }

    // find walls that have no corresponding osm wall
    for (auto pIdx = 0; pIdx < lasWallPoints.size(); pIdx++) {
        if (lasWallPoints[pIdx] && !usedLasWallPoints[pIdx]) {
            auto& point = (*cloud)[pIdx];
            point.r = 255;
        }
    }

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
            newTexCoords.emplace_back(0,0); // TODO give texture to walls
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

void DataIO::findXYZMedian(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<int>& pointIndices, float& xMedian, float& yMedian, float& zMedian) {
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
float DataIO::getMaxY(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, float& x, float& z, float& yMin, float& yMax, float& stepWidth, std::vector<bool>& removePoints, const pcl::PointXYZ& wallNormal, const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree) {

    // search points from (x,midY,z)
    float newMaxY = yMin;
    pcl::PointXYZ wallPlane = Util::crossProduct(wallNormal, pcl::PointXYZ(0,1,0));
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
    // save all walls (min, mid, max point & radius)
    // dann beim normalen orientieren spatial search nach mid point mit max radius von allen walls
    float maxR = 0;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallMidPoints = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for (auto bIdx = 0; bIdx < buildings.size(); bIdx++) {
        const auto& building = buildings[bIdx];
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

            Util::Wall wall;


            pcl::PointXYZRGBNormal wallPoint1, wallPoint2;
            wallPoint1.x = building.points[pointIdx].x;
            wallPoint1.y = ground;
            wallPoint1.z = building.points[pointIdx].z;
            wallPoint2.x = building.points[pointIdx + 1].x;
            wallPoint2.y = ground + wallHeight;
            wallPoint2.z = building.points[pointIdx + 1].z;

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
            if (bIdx == 59) {
                    auto bal = 33;
            }
            wall.maxX = std::max(wallPoint1.x, wallPoint2.x);
            wall.minZ = std::min(wallPoint1.z, wallPoint2.z);
            wall.maxZ = std::max(wallPoint1.z, wallPoint2.z);

            wall.buildingIdx = bIdx;

            wall.length = Util::horizontalDistance(wallPoint1, wallPoint2);

            walls.push_back(wall);
            wallMidPoints->push_back(wall.mid);

            buildingWallMap[bIdx].push_back(walls.size()-1);

            const auto& lol = buildingWallMap[bIdx];
            auto bjkuaf = 3;


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

