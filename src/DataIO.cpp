//
// Created by Jana on 12.11.2023.
//

#include "DataIO.h"
#include <numeric>
#include <fstream>
#include <cstdlib>
#include <random>
#include "UTM.h"
#include <pcl/filters/statistical_outlier_removal.h>

bool DataIO::readData(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& gmlFile, const std::string& imgFile,
                      const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<pcl::PointXY>& texCoords,
                      std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec,
                      int& wallPointsStartIndex, std::vector<int>& pointClasses) {

    std::vector<DataIO::Polygon> osmPolygons;

    std::cout << TAG << "begin loading data" << std::endl;
    std::string lasDir = ".." + Util::PATH_SEPARATOR + "las" + Util::PATH_SEPARATOR;
    const auto& file = lasFiles[0];

    // get cached points

    auto cacheType = readCache(lasDir + file, cloud, texCoords, tangent1Vec, tangent2Vec, wallPointsStartIndex, pointClasses);
    if (cacheType == "splat"){
        return true;
    }
    if (cacheType == "point") {
        return false;
    }

    // read las file
//    std::string lasDir = ".." + Util::PATH_SEPARATOR + "las" + Util::PATH_SEPARATOR;
//    const auto& file = lasFiles[0];
    // sets: las to opengl offsets, bounds for shp and numOfPoints
    readLas(lasDir + file);

    // read gml file
    std::string gmlDir = ".." + Util::PATH_SEPARATOR + "gml" + Util::PATH_SEPARATOR;
    readGml(gmlDir + gmlFile);

    // read shape file
    std::string shpDir = ".." + Util::PATH_SEPARATOR + "shp" + Util::PATH_SEPARATOR;
    readShp(shpDir + shpFile, &osmPolygons);

    std::cout << TAG << "begin processing shp data" << std::endl;
    // preprocess osmPolygons to osm walls
    float resolution = 8.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> wallOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>(
            resolution);
    // TODO add error handling if osmPolygons are empty
    float maxWallRadius = preprocessWalls(wallOctree, osmPolygons);



    std::cout << TAG << "begin filtering and coloring points" << std::endl;
    std::vector<bool> lasWallPoints;
    std::vector<bool> lasGroundPoints;
    filterAndColorPoints(cloud, wallOctree, maxWallRadius, imgFile, lasWallPoints, lasGroundPoints, texCoords);

    // up to this index are points from las file, from this index on are synthetic wall points
    wallPointsStartIndex = cloud->size();
    tangent1Vec = std::vector<pcl::PointXYZ>((*cloud).size());
    tangent2Vec = std::vector<pcl::PointXYZ>((*cloud).size());

    std::cout << TAG << "begin processing walls" << std::endl;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    pointClasses = std::vector<int>(cloud->points.size());
    fill(pointClasses.begin(), pointClasses.end(), 2);
    tree->setInputCloud(cloud);
    insertWalls(cloud, osmPolygons, lasWallPoints, lasGroundPoints, tree, wallOctree, texCoords, tangent1Vec, tangent2Vec,
                wallPointsStartIndex, pointClasses);
    tree->setInputCloud(cloud);

    writeCache(lasDir + file, false, cloud, texCoords, tangent1Vec, tangent2Vec, wallPointsStartIndex, pointClasses);


    std::cout << TAG << "loading data successful" << std::endl;
    return false;
}



// ********** las **********

/**
 * sets offsets (to convert UTM to my OpenGl system), bounds (in WGS to cut out shp osm data matching las file space), numberOfPoints
 * @param path
 */
void DataIO::readLas(const std::string& path) {
    std::string TAG = DataIO::TAG + "readLas\t";

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
            throw std::invalid_argument("Can't handle given LAS file. Only point record format 1 or 2 allowed.");
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



        // points
//        numOfPoints = 800000;
        numOfPoints = header.numberOfPoints;
//
        std::cout << TAG << "Num of points: " << numOfPoints << std::endl;
        inf.seekg(header.pointDataOffset); // skip to point tree
        // skip to point tree when using smaller number of points for testing
//        inf.seekg(11300000 * (sizeof(PointDRF1) - 3 * sizeof(double) + 3 * sizeof(uint32_t)), std::ios_base::cur);

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


        }


        if (!inf.good())
            throw std::runtime_error("Reading .las ran into error");

        std::cout << TAG << "finished reading las file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .las file");
    }
}

bool polygonCheck(const pcl::PointXYZRGBNormal& point, std::vector<pcl::PointXYZ>& points) {
    // create ray for this point
    auto rayPoint = pcl::PointXYZ(point.x, point.y, point.z);
    auto rayDir = pcl::PointXYZ(0, -1, 0);

    int intersectionCount = 0;
    for (auto i = 0; i < points.size() - 1; i++) {
        // intersect ray with line
        intersectionCount += Util::intersectLine(points[i], points[i + 1], rayPoint, rayDir);
    }

    if (intersectionCount % 2 != 0) {
        // point is inside of polygon
        return true;
    }

    // point is not inside polygon
    return false;
}

bool DataIO::isPointInBuilding(const pcl::PointXYZRGBNormal& v,
                               const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
                               const float& maxWallRadius) {

    // create ray for this point
    auto rayPoint = pcl::PointXYZ(v.x, v.y, v.z);
    auto rayDir = pcl::PointXYZ(-0.5, 0, 0.5); // TODO choose sth smart?
    for (const auto& building: buildings) {
        if (isPointInBuildingBbox(building, v)) {

            // check if belongs to building
            int intersectionCount = 0;
            for (auto& bWall: building.osmWalls) {

                // check if near wall
                float dist = Util::pointPlaneDistance(v, bWall.mid);
                if (dist <= osmWallThreshold) {
                    if (Util::horizontalDistance(v, bWall.mid) <= bWall.length / 2) {
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



    std::string TAG = DataIO::TAG + "filterAndColorPoints\t";
    // init cloud
    cloud->width = numOfPoints;
    cloud->height = 1;

    // read image
    std::vector<unsigned char> image;
    int width, height;
    const int channels = 3;
    if (!readImg(image, imgFile, channels, width, height)) {
        std::cout << "Error loading image\n";
        // TODO error handling
    }

    std::cout << TAG << "remove tree/vegetation points"  << std::endl;
    std::cout << TAG << "+ mark wall points" << std::endl;
    std::cout << TAG << "+ assign texture coordinates to points" << std::endl;

    // wall points go in extra cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudMulti = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::vector<bool> lasGroundPointsMulti;
    std::vector<bool> lasWallPointsMulti;
    std::vector<pcl::PointXY> texCoordsMulti;

    // filter stuff
    int idxxx = 0;
    int sizeo = lasPoints.size();
    for (const auto& point: lasPoints) {

        bool belongsToWall = false;

        pcl::PointXYZRGBNormal v; // TODO move down
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
                belongsToWall = isPointInBuilding(v, wallOctree, maxWallRadius);

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
//                    v.b = 255;
//                    v.g = 0;
//                    v.r = 0;
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
//                        v.b = 255;
//                        v.g = 0;
//                        v.r = 0;
                    }
                    belongsToWall = isPointInBuilding(v, wallOctree, maxWallRadius);
                    if (!belongsToWall) {
                        continue;
                    }
                }
            }
        }
        //endregion

        int imageX = (point.x - 389000.05) * 10; // data from jp2 world file
        int imageY = (point.y - 5705999.95) * -10;


        if (numOfReturns != 1) {
            // if multi return points -> wall cloud
            texCoordsMulti.emplace_back(static_cast<float>(imageX) / 10000, static_cast<float>(imageY) / 10000);

            cloudMulti->push_back(v);
            lasWallPointsMulti.push_back(belongsToWall);
            lasGroundPointsMulti.push_back(classification == 2);

        } else {
            // else -> normal cloud
            texCoords.emplace_back(static_cast<float>(imageX) / 10000, static_cast<float>(imageY) / 10000);

            cloud->push_back(v);
            lasWallPoints.push_back(belongsToWall);
            lasGroundPoints.push_back(classification == 2);
        }

    }

    std::cout << TAG << "new number of points: " << (*cloud).size() << std::endl;

    std::cout << TAG << "filter outliers... " << std::endl;


    // filter single return points for outliers, multi return points werden eh später entfernt
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
// Create the filtering object
    auto sor = pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal>(true);
    sor.setInputCloud (cloud);
    sor.setMeanK (40);
    sor.setStddevMulThresh (2.0);
    sor.filter (*cloud_filtered);
    auto removedIndices = sor.getRemovedIndices();

    // copy cloud points here because they get filtered as well because the set indices before filtering
    auto newPoints = std::vector<pcl::PointXYZRGBNormal>();
    std::vector<bool> newLasGroundPoints;
    std::vector<pcl::PointXY> newTexCoords;
    std::vector<bool> newLasWallPoints;
    auto idxIdx = 0;
    for (auto i = 0; i < lasGroundPoints.size(); i++) {
        if (idxIdx < removedIndices->size()) {
            // check
            auto cloudIdx = (*removedIndices)[idxIdx];
            if (i == cloudIdx) {
                idxIdx++;
                // skip this, dont copy
                continue;
            }
        }

        newPoints.push_back((*cloud)[i]);
        newLasGroundPoints.push_back(lasGroundPoints[i]);
        newLasWallPoints.push_back(lasWallPoints[i]);
        newTexCoords.push_back(texCoords[i]);
    }

    (*cloud).clear();
    (*cloud).insert((*cloud).end(), newPoints.begin(), newPoints.end());
    lasWallPoints = newLasWallPoints;
    lasGroundPoints = newLasGroundPoints;
    texCoords = newTexCoords;

    // warum gehen textur koords kaputt?

    // attach multi points to end
    (*cloud).insert((*cloud).end(), (*cloudMulti).begin(), (*cloudMulti).end());
    lasWallPoints.insert(lasWallPoints.end(), lasWallPointsMulti.begin(), lasWallPointsMulti.end());
    lasGroundPoints.insert(lasGroundPoints.end(), lasGroundPointsMulti.begin(), lasGroundPointsMulti.end());
    texCoords.insert(texCoords.end(), texCoordsMulti.begin(), texCoordsMulti.end());




    std::cout << TAG << "finished filtering points" << std::endl;

}


void DataIO::insertWalls(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<Polygon>& polygons,
                         std::vector<bool>& lasWallPoints,
                         std::vector<bool>& lasGroundPoints,
                         const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& lasPointTree,
                         const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& osmWallOctree,
                         std::vector<pcl::PointXY>& texCoords,
                         std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec,
                         int& wallPointsStartIndex, std::vector<int>& pointClasses) {
    std::string TAG = DataIO::TAG + "insertWalls\t";

    bool colorOsmWall = false;
    bool colorCertainLasWall = false;
    bool colorCertainLasWallRandom = false;
    bool colorFinalLasWall = false;
    bool colorFinalLasWallWithoutGround = false;
    bool removeOldWallPoints = true;

    pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(cloud);

    auto removePoints = std::vector<bool>(cloud->size());
    std::fill(removePoints.begin(), removePoints.end(), false);

    // mark certain wall points that are used by gml or osm walls -> check if i can/should build walls with the remaining points (maybe unnecessary because of gml walls now)
    auto usedLasWallPoints = std::vector<bool>(lasWallPoints.size());
    std::fill(usedLasWallPoints.begin(), usedLasWallPoints.end(), false);

    auto osmWallMidPoints = osmWallOctree.getInputCloud();

    // mark osm walls that are already covered by gml walls -> only draw remaining osm walls
    auto usedOsmWalls = std::vector<bool>(osmWallMidPoints->size());
    std::fill(usedOsmWalls.begin(), usedOsmWalls.end(), false);

    std::cout << TAG << "match gml walls to osm walls" << std::endl;
    std::cout << TAG << "+ draw gml walls" << std::endl;

    std::map<int, std::vector<std::pair<int, int>>> osmToGmlMap;
    std::map<std::pair<int, int>, std::vector<int>> gmlToOsmMap;

    for (auto bIdx = 0; bIdx < gmlBuildings.size(); bIdx++) {

        auto& gmlBuilding = gmlBuildings[bIdx];


        //region match osm to gml walls of this building

        for (auto gmlWallIdx = 0; gmlWallIdx < gmlBuilding.osmWalls.size(); gmlWallIdx++) {
            auto gmlWall = gmlBuilding.osmWalls[gmlWallIdx];

            pcl::Indices wallIdxRadiusSearch;
            std::vector<float> wallRadiusSquaredDistance;
            float wallHeight = 80; // TODO get dynamically from las data
            auto osmSearchRadius = static_cast<float>(sqrt(pow(gmlWall.length / 2, 2) + pow(wallHeight / 2, 2)));
            // search for near osm walls
            if (osmWallOctree.radiusSearch(gmlWall.mid, osmSearchRadius, wallIdxRadiusSearch, wallRadiusSquaredDistance) > 0) {
                for (const auto& osmWallIdx: wallIdxRadiusSearch) {
                    // check if wall is similar
                    auto& osmWall = osmWalls[osmWallIdx];

                    // point to plane distance
                    auto ppd = Util::pointPlaneDistance(osmWall.mid, gmlWall.mid);
                    if (ppd > 2.0) // 2.5 seems to be a good value, fixes some problems where the difference between gml and osm walls is rather big
                        continue;
                    // normal angle
                    auto normalAngle = Util::normalAngle(gmlWall.mid, osmWall.mid);
                    auto normalReverse = pcl::PointXYZ(-osmWall.mid.normal_x, -osmWall.mid.normal_y, -osmWall.mid.normal_z);
                    float normalAngleAndersrum = Util::normalAngle(gmlWall.mid, normalReverse);
                    if (normalAngle > 0.26 && normalAngleAndersrum > 0.26) // 15 deg
                        continue;

                    // walls don't match if they don't overlap anywhere (+ buffer)   (osm.p1 zu gml.p1 < max length and osm.p2 to gml.p2 smaller than max length -> match)
                    if (Util::horizontalDistance(osmWall.point1, gmlWall.point1) > std::max(osmWall.length, gmlWall.length)
                        || Util::horizontalDistance(osmWall.point2, gmlWall.point2) > std::max(osmWall.length, gmlWall.length)) {
                        continue;
                    }

                    // found matching osm wall
                    osmToGmlMap[osmWallIdx].emplace_back(bIdx, gmlWallIdx);
                    gmlToOsmMap[std::pair(bIdx, gmlWallIdx)].emplace_back(osmWallIdx);
                }
            }
        }
        //endregion
    }
    for (auto bIdx = 0; bIdx < gmlBuildings.size(); bIdx++) {
        auto& gmlBuilding = gmlBuildings[bIdx];

        // region draw gml walls after cover check

        for (auto gmlWallIdx = 0; gmlWallIdx < gmlBuilding.osmWalls.size(); gmlWallIdx++) {
            auto gmlWall = gmlBuilding.osmWalls[gmlWallIdx];

            pcl::Indices pointIdxRadiusSearch;
            //region radius search points for gmlWall

            std::vector<float> pointRadiusSquaredDistance;
            auto searchRadius = Util::distance(gmlWall.mid, gmlWall.point2);
            searchRadius *= 1.2; // want to collect certain wall points on top wall edges
            if (lasPointTree->radiusSearch(gmlWall.mid, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0) {
                continue;
            }
            //endregion

            pcl::Indices wallPoints;
            int certainWallPointsCount = 0;
            bool hasTopPoints = false;
            //region collect wall points from search results
            for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {

                const auto& nIdx = *nIdxIt;
                const auto& point = (*cloud)[nIdx];

                if (Util::pointPlaneDistance(point, gmlWall.mid) > 0.5) {
                    continue;
                }
                if (Util::horizontalDistance(point, gmlWall.mid) > gmlWall.length / 2) {
                    continue;
                }
                if (point.y < gmlWall.point1.y - 1.0 || point.y > gmlWall.point2.y + 1.0) {
                    continue;
                }
                if (lasWallPoints[nIdx]) {
                    certainWallPointsCount++;
                }

                wallPoints.push_back(nIdx);

            }
            //endregion

            int gmlR = 100, gmlG = 200, gmlB = 100;

            if (wallPoints.size() < 3) {
                continue;
                gmlR = 0; // rot
                gmlG = 0;
                gmlB = 255;
            }


            //region check if there is a near osm wall and do cover check.
            // if osm wall is covered -> mark as visited
            // else -> skip gml wall

            for (const auto& osmWallIdx: gmlToOsmMap[std::pair(bIdx,gmlWallIdx)]) {
                if (usedOsmWalls[osmWallIdx])
                    continue;
                const auto& osmWall = osmWalls[osmWallIdx];
                // if this osm wall is covered by gml walls (so that there is at max a 3m hole) -> mark this osm wall (don't need to draw it later)

                float matchingGmlWallLengthSum = 0;
                for (const auto& matchingGmlWallIdxPair: osmToGmlMap[osmWallIdx]) { // works because the match maps are built per building
                    auto& matchingBuilding = gmlBuildings[matchingGmlWallIdxPair.first];
                    const auto& matchingGmlWall = matchingBuilding.osmWalls[matchingGmlWallIdxPair.second];
                    matchingGmlWallLengthSum += matchingGmlWall.length;
                }
                // this gml wall does cover a osm wall (maybe together with more gml walls) -> don't skip it
                    if (matchingGmlWallLengthSum /*+ 3.0*/ > osmWall.length * 0.70  || matchingGmlWallLengthSum + 3.0 > osmWall.length) {
                    usedOsmWalls[osmWallIdx] = true;
                }
            }
            //endregion

            // is a valid wall
            //region mark as visited

            for (const auto& pIdx: wallPoints) {
                if (lasWallPoints[pIdx]) {
                    usedLasWallPoints[pIdx] = true;
                }
                if (!lasGroundPoints[pIdx]) {
                    removePoints[pIdx] = true;

                }
            }
            //endregion


            //region draw gml wall

            // get y min and max to cover wall from bottom to top
            float yMin = gmlWall.point1.y, yMax = gmlWall.point2.y;

            // draw plane
            float stepWidth = 0.5;
            auto wallVec = Util::normalize(Util::horizontalVector(gmlWall.point2, gmlWall.point1)); // horizontal

            float x = gmlWall.point1.x;
            float z = gmlWall.point1.z;
            float distanceMoved = 0;

            bool horMajor = yMax - yMin < gmlWall.length;

            // move horizontal
            while (distanceMoved < gmlWall.length) {
                float y = yMin;
                float xCopy = x;
                float zCopy = z;
                while (y < yMax) {



                    auto v = pcl::PointXYZRGBNormal(x, y, z, gmlR, gmlG, gmlB);//randR, randG, randB));

                    // if the wall is not a rectangle I have to check if the new vertex is inside the polygon
                    if (!gmlWall.isRect && !polygonCheck(v, gmlWall.points)) {
                        y += stepWidth;
                        continue;
                    }

                    if (!gmlWall.isRect) {
                        auto f = 3;
                    }

                    // set normal
                    v.normal_x = gmlWall.mid.normal_x;
                    v.normal_y = gmlWall.mid.normal_y;
                    v.normal_z = gmlWall.mid.normal_z;

                    texCoords.emplace_back(0, 0);


                    // set class
                    // also set tangents
                    // set major and minor
                    // inner (planar points) get according to wall ratio
                    // border points (linear) get according to border
                    if (gmlWall.isRect) {
                        if (distanceMoved == 0 || distanceMoved + stepWidth > gmlWall.length) {

                            pointClasses.emplace_back(0);
                            // left / right column
                            // vertical is major
                            tangent1Vec.emplace_back(0, 1, 0); // swap sign for right orientation
                            tangent2Vec.push_back(wallVec);
                        } else if (y == yMin || y+stepWidth > yMax ){
                            pointClasses.emplace_back(0);
                            // bottom / top row
                            // horizontal is major
                            tangent1Vec.push_back(wallVec);
                            tangent2Vec.emplace_back(0, -1, 0);
                        } else {
                            pointClasses.emplace_back(1);
                            if (horMajor) {
                                tangent1Vec.push_back(wallVec);
                                tangent2Vec.emplace_back(0, -1, 0);
                            } else {
                                tangent1Vec.emplace_back(0, 1, 0); // swap sign for right orientation
                                tangent2Vec.push_back(wallVec);
                            }
                        }
                    } else {
                        // left / right
                        if (distanceMoved == 0 || distanceMoved + stepWidth > gmlWall.length
                            || !polygonCheck(pcl::PointXYZRGBNormal(x - stepWidth * wallVec.x, y, z - stepWidth * wallVec.z), gmlWall.points)
                            || !polygonCheck(pcl::PointXYZRGBNormal(x + stepWidth * wallVec.x, y, z + stepWidth * wallVec.z), gmlWall.points)
                            ) {
                            pointClasses.emplace_back(0);
                            // vertical is major
                            tangent1Vec.emplace_back(0, 1, 0); // swap sign for right orientation
                            tangent2Vec.push_back(wallVec);
                        }
                        // bottom / top
                        else if (y == yMin || y+stepWidth > yMax
                            || !polygonCheck(pcl::PointXYZRGBNormal(x, y - stepWidth, z), gmlWall.points)
                            || !polygonCheck(pcl::PointXYZRGBNormal(x, y + stepWidth, z), gmlWall.points)
                            ) {
                            pointClasses.emplace_back(0);
                            // horizontal is major
                            tangent1Vec.push_back(wallVec);
                            tangent2Vec.emplace_back(0, -1, 0);
                        } else {
                            pointClasses.emplace_back(1);
                            if (horMajor) {
                                tangent1Vec.push_back(wallVec);
                                tangent2Vec.emplace_back(0, -1, 0);
                            } else {
                                tangent1Vec.emplace_back(0, 1, 0); // swap sign for right orientation
                                tangent2Vec.push_back(wallVec);
                            }
                        }
                    }

                    cloud->push_back(v);

                    y += stepWidth;
                }
                x = xCopy + stepWidth * wallVec.x;
                z = zCopy + stepWidth * wallVec.z;
                distanceMoved += stepWidth;
            }
            //endregion
        }
        //endregion
    }

    std::cout << TAG << "draw osm walls that are not covered by gml walls" << std::endl;

    //region draw remaining osm walls

    for (auto osmWallIdx = 0; osmWallIdx < osmWalls.size(); osmWallIdx++) {
        if (usedOsmWalls[osmWallIdx])
            continue;

        // get wall
        const auto& osmWall = osmWalls[osmWallIdx];

        int randR = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
        int randG = rand() % (156) + 100;
        int randB = rand() % (156) + 100;


        pcl::Indices pointIdxRadiusSearch;
        //region search points from osmWall mid-point with big radius

        std::vector<float> pointRadiusSquaredDistance;
        float wallHeight = 80; // TODO get dynamically from las data
        auto searchRadius = static_cast<float>(sqrt(pow(osmWall.point2.x - osmWall.mid.x, 2) +
                                                    pow(wallHeight - osmWall.mid.y, 2) +
                                                    pow(osmWall.point2.z - osmWall.mid.z, 2)));;
        lasPointTree->radiusSearch(osmWall.mid, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        //endregion

        if (pointIdxRadiusSearch.size() < 3) {
            continue;
        }

        pcl::Indices certainWallPoints;
        //region filter search results for certain las wall points

        // only take osm wall points that are also las wall points
        for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {

            const auto& nIdx = *nIdxIt;
            const auto& point = (*cloud)[*nIdxIt];
            if (Util::horizontalDistance(point, osmWall.mid) > osmWall.length / 2) {
                continue;
            }
            auto ppd = Util::pointPlaneDistance(cloud->points[*nIdxIt], osmWall.mid);
            if (ppd > osmWallThreshold) {
                continue;
            }

            if (lasWallPoints[nIdx]) {
                certainWallPoints.push_back(nIdx);
            }
        }
        //endregion

        if (certainWallPoints.size() < 3) {
            continue;
        }

        Util::Wall lasWall;
        // fit *vertical* plane through certain wall points
        if (!fitPlane(cloud, osmWall, certainWallPoints, pca, lasWall))
            continue;


        int osmR = 255;
        int osmG = 255;
        int osmB = 0;

        int debugR = 100;
        int debugG = 100;
        int debugB = 255;


        pcl::Indices finalWallPoints;
        //region get final wall points from new wall plane

        // get min max wall borders
        auto lasMinX = std::min(lasWall.point1.x, lasWall.point2.x);
        auto lasMaxX = std::max(lasWall.point1.x, lasWall.point2.x);
        auto lasMinZ = std::min(lasWall.point1.z, lasWall.point2.z);
        auto lasMaxZ = std::max(lasWall.point1.z, lasWall.point2.z);
        std::vector<pcl::PointXYZ> finalWallPointsNotGround;

        for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {

            const auto& nIdx = *nIdxIt;
            const auto& point = (*cloud)[*nIdxIt];
            if (point.x > lasMaxX || point.x < lasMinX || point.z > lasMaxZ || point.z < lasMinZ) {
                continue;
            }
            auto ppd = Util::pointPlaneDistance(cloud->points[*nIdxIt], lasWall.mid);
            if (ppd > lasWallThreshold) {
                continue;
            }
            finalWallPoints.push_back(nIdx);


            if (!lasGroundPoints[nIdx]) {
                if (colorFinalLasWallWithoutGround) {
                    (*cloud)[nIdx].b = randR;
                    (*cloud)[nIdx].g = randG;
                    (*cloud)[nIdx].r = randB;
                }
                // project wall points onto las wallpoint plane because they are used to determine start and end point?
                auto pointDist = Util::signedPointPlaneDistance(point, lasWall.mid);
                auto newPosi = Util::vectorSubtract(point, pcl::PointXYZRGBNormal(pointDist * lasWall.mid.normal_x,
                                                                                  pointDist * lasWall.mid.normal_y,
                                                                                  pointDist *
                                                                                  lasWall.mid.normal_z));
                finalWallPointsNotGround.push_back(newPosi);
                removePoints[nIdx] = true;
            }
        }
        //endregion
        finalWallPoints.insert(finalWallPoints.end(), certainWallPoints.begin(), certainWallPoints.end());

        if (finalWallPoints.size() < 3) {
            continue;
        }

        // get y min and max from finalWallPoints to cover wall from bottom to top
        float yMin, yMax;
        findYMinMax(cloud, finalWallPoints, yMin, yMax);
        lasWall.point1.y = yMin;
        lasWall.point2.y = yMin;

        if (yMax <= yMin + 0.5) {
            continue;
        }

        // this is valid las wall
        // mark certain wall points
        for (auto& nIdx: certainWallPoints) {
            removePoints[nIdx] = true;
            usedLasWallPoints[nIdx] = true;
        }

        // region draw wall

        // fill wall with points
        // draw plane
        float stepWidth = 0.5;
        // get perp vec
        // update values
        pcl::PointXYZ lasWallVec = Util::vectorSubtract(lasWall.point2, lasWall.point1);
        pcl::PointXYZ horPerpVec = Util::normalize(lasWallVec); // horizontal
        pcl::PointXYZ lasWallNormal = Util::crossProduct(horPerpVec, pcl::PointXYZ(0, -1, 0)); // TODO use data from wall struct

        float lasWallLength = Util::vectorLength(lasWallVec);
        float x = lasWall.point1.x;
        float z = lasWall.point1.z;
        float distanceMoved = 0;

        auto newWallStartIndex = cloud->size() - 1;
        std::map<int, int> counters;
        std::vector<int> columnHeights;
        // move horizontal
        while (distanceMoved < lasWallLength) {
            float y = yMin;
            float xCopy = x;
            float zCopy = z;
            float currentMaxY = getMaxY(cloud, x, z, yMin, yMax, stepWidth, lasWallPoints, lasWallNormal, lasPointTree);
            int columnHeight = static_cast<int>((currentMaxY - yMin) / stepWidth);
            columnHeights.push_back(columnHeight);
            counters[columnHeight]++;

            x = xCopy + stepWidth * horPerpVec.x;
            z = zCopy + stepWidth * horPerpVec.z;
            distanceMoved += stepWidth;
        }
        //endregion

        auto mostCommon = std::max_element(counters.begin(), counters.end(), [](const auto& p1, const auto& p2) {
            return p1.second < p2.second;
        });
        if (mostCommon == counters.end()) {
            continue;
        }

        // debug
        auto columnHeightsCopy = columnHeights;

        if (mostCommon->first < 3) {
            // skip weil zu niedrig
            continue;
        }


        // look at jumping/outlier values
        int jumpCount = 0;
        if (columnHeights.size() > 1) {
            // jump count
            for (auto it = columnHeights.begin(); it != columnHeights.end(); it++) {
                if (it != columnHeights.begin()) {
                    if (abs(*(it - 1) - *it) > 2) {
                        jumpCount++;
                    }
                }
            }

            // fix outliers/jumps - replace with most common value
            for (auto it = columnHeights.begin(); it != columnHeights.end(); it++) {
                // end edge case
                if (it == columnHeights.begin()) {
                    if (abs(*(it + 1) - *it) > 2) {
                        *it = mostCommon->first;
                    }
                } else {
                    if (abs(*(it - 1) - *it) > 2) {
                        *it = mostCommon->first;
                    }
                }
            }
            for (auto it = columnHeights.rbegin(); it != columnHeights.rend(); it++) {
                // end edge case
                if (it == columnHeights.rbegin()) {
                    if (abs(*(it + 1) - *it) > 2) {
                        *it = mostCommon->first;
                    }
                } else {
                    if (abs(*(it - 1) - *it) > 2) {
                        *it = mostCommon->first;
                    }
                }
            }
        }


        bool horMajor = (yMax - yMin) < lasWallLength;


        x = lasWall.point1.x;
        z = lasWall.point1.z;
        for (int j = 0; j < columnHeights.size(); j++) {
            auto columnHeight = columnHeights[j];
            float y = yMin;
            float xCopy = x;
            float zCopy = z;
            for (int i = 0; i <= columnHeight; i++) {
                auto v = pcl::PointXYZRGBNormal(x, y, z, osmR, osmG, osmB);//randR, randG, randB)); // türkis
                // set normal
                v.normal_x = lasWallNormal.x;
                v.normal_y = lasWallNormal.y;
                v.normal_z = lasWallNormal.z;


                // set class
                // also set tangents
                // set major and minor
                // inner (planar points) get according to wall ratio
                // border points (linear) get according to border
                if (j == 0 || j + 1 == columnHeights.size()) {

                    pointClasses.emplace_back(0);
                    // left / right column
                    // vertical is major
                    tangent1Vec.emplace_back(0, -1, 0); // swap sign for right orientation
                    tangent2Vec.push_back(horPerpVec);
                } else if (i == 0 || i + 1 > columnHeight){
                    pointClasses.emplace_back(0);
                    // bottom / top row
                    // horizontal is major
                    tangent1Vec.push_back(horPerpVec);
                    tangent2Vec.emplace_back(0, 1, 0);
                } else {
                    pointClasses.emplace_back(1);

                    if (horMajor) {
                        tangent1Vec.push_back(horPerpVec);
                        tangent2Vec.emplace_back(0, 1, 0);
                    } else {
                        tangent1Vec.emplace_back(0, -1, 0); // swap sign for right orientation
                        tangent2Vec.push_back(horPerpVec);
                    }

                }

                texCoords.emplace_back(0, 0);

                cloud->push_back(v);

                y += stepWidth;
            }
            x = xCopy + stepWidth * horPerpVec.x;
            z = zCopy + stepWidth * horPerpVec.z;
        }
        //endregion
    }
    //endregion

    auto insertedWallPointCount = (*cloud).size() - removePoints.size();
    std::cout << TAG << "added " << insertedWallPointCount << " new wall points" << std::endl;
    std::cout << TAG << "new number of points: " << (*cloud).size() << std::endl;


    if (removeOldWallPoints) {
        std::cout << TAG << "removing old wall points.." << std::endl;

        auto newPoints = std::vector<pcl::PointXYZRGBNormal>();
        auto newTexCoords = std::vector<pcl::PointXY>();
        // only keep old points that do not belong to walls
        for (auto pIdx = 0; pIdx < removePoints.size(); pIdx++) {
            if (removePoints[pIdx])
                continue;
            if (lasWallPoints[pIdx])
                continue;

            newPoints.push_back((*cloud)[pIdx]);
            newTexCoords.push_back(texCoords[pIdx]);
        }
        wallPointsStartIndex = newPoints.size();

        // move partition of wall tangents in tangent1Vec and tangent2Vec to match point cloud by removing points
        int initCloudLength = removePoints.size();
        int pointCountDif = initCloudLength - wallPointsStartIndex; // is always >=0 because points just got removed
        // fewer points than before, remove points
        tangent1Vec.erase(tangent1Vec.begin(), tangent1Vec.begin() + pointCountDif);
        tangent2Vec.erase(tangent2Vec.begin(), tangent2Vec.begin() + pointCountDif);
        pointClasses.erase(pointClasses.begin(), pointClasses.begin() + pointCountDif);

        // keep all new points
        int tangentIdx = 0;
        for (auto pIdx = removePoints.size(); pIdx < (*cloud).size(); pIdx++) {
            newPoints.push_back((*cloud)[pIdx]);
            newTexCoords.emplace_back(0, 0); // TODO assign texture coords to walls
        }

        (*cloud).clear();
        (*cloud).insert((*cloud).end(), newPoints.begin(), newPoints.end());
        texCoords.clear();
        texCoords.insert(texCoords.end(), newTexCoords.begin(), newTexCoords.end());

        std::cout << TAG << "removed " << pointCountDif << " old points" << std::endl;
        std::cout << TAG << "new number of points: " << (*cloud).size() << std::endl;


    }
}

bool xComparator2(pcl::PointXYZ& p1, pcl::PointXYZ& p2) {
    return p1.x < p2.x;
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

            auto distThreshold = osmWallThreshold * 1.5;
            float distToWall = abs(wallNormal.x * (searchPoint.x - point.x) + wallNormal.z * (searchPoint.z - point.z));
            if (distToWall > distThreshold) // back to 1.5, fix wall after introducing remove point strategy for osm walls
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


// ********** gml **********
inline void trim(std::string& s) {
    // left trim
    s.erase(s.begin(), s.begin() + s.find_first_not_of(" \n\r\t"));
    // right trim
    s.erase(s.find_last_not_of(" \n\r\t") + 1);
}

void DataIO::readGml(const std::string& path) {
    std::string TAG = DataIO::TAG + "readGml\t";


    std::cout << TAG << "read gml file..." << std::endl;

    std::ifstream inf(path);
    gmlBuildings = std::vector<Building>();
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallMidPoints = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
//            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//    float maxR = -1;

    if (inf.is_open()) {

        std::cout << TAG << "File: " << path << std::endl;

        int buildingCount = 0;
        std::string line;
        std::string firstToken;


        auto lineCount = 0;
        while (std::getline(inf, line)) {
            lineCount++;
            trim(line);
            if (line == "<core:cityObjectMember>") {

                // get all walls
                auto newBuilding = Building();
                float buildingMinX = INFINITY, buildingMinY = INFINITY, buildingMinZ = INFINITY;
                float buildingMaxX = -INFINITY, buildingMaxY = -INFINITY, buildingMaxZ = -INFINITY;
                // read walls
                while (std::getline(inf, line)) {
                    lineCount++;
                    trim(line);
                    if (line == "</bldg:Building>") {
                        // end of building
                        newBuilding.xMax = buildingMaxX;
                        newBuilding.xMin = buildingMinX;
                        newBuilding.zMax = buildingMaxZ;
                        newBuilding.zMin = buildingMinZ;
                        newBuilding.parts.emplace_back(0); // first ring starts at first wall

                        gmlBuildings.push_back(newBuilding);
                        break;
                    }
                    if (line == "<bldg:boundedBy>") {
                        std::getline(inf, line);
                        lineCount++;
                        trim(line);
                        std::istringstream iss(line);
                        iss >> firstToken;
                        if (firstToken == "<bldg:WallSurface" || firstToken == "<bldg:WallSurface>") { // sometimes has id
                            // new wall
                            // read until posi
                            while (std::getline(inf, line)) {
                                lineCount++;
                                trim(line);
                                if (line == "</bldg:WallSurface>") {
                                    break;
                                }
                                std::istringstream iss(line);
                                iss >> firstToken;
                                if (firstToken == "<gml:posList") {
                                    auto newWall = Util::Wall();
                                    // remove tags from line
                                    line.erase(line.begin() + line.find('<'), line.begin() + line.find('>') + 1);
                                    line.erase(line.begin() + line.find('<'), line.begin() + line.find('>') + 1);
                                    std::istringstream iss(line);

                                    //read positions
                                    std::string x, y, z;
                                    std::vector<pcl::PointXYZ> points;
                                    float minX = INFINITY, minY = INFINITY, minZ = INFINITY;
                                    float maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;
                                    auto invalid = false;
                                    while (iss >> x >> y >> z) {
                                        float glX = std::stof(x) - xOffset;
                                        float glY = std::stof(z) - yOffset;
                                        float glZ = -(std::stof(y) - zOffset);

                                        minX = std::min(minX, glX);
                                        minY = std::min(minY, glY);
                                        minZ = std::min(minZ, glZ);

                                        maxX = std::max(maxX, glX);
                                        maxY = std::max(maxY, glY);
                                        maxZ = std::max(maxZ, glZ);

                                        buildingMinX = std::min(minX, buildingMinX);
                                        buildingMinY = std::min(minY, buildingMinY);
                                        buildingMinZ = std::min(minZ, buildingMinZ);

                                        buildingMaxX = std::max(maxX, buildingMaxX);
                                        buildingMaxY = std::max(maxY, buildingMaxY);
                                        buildingMaxZ = std::max(maxZ, buildingMaxZ);


                                        points.emplace_back(glX, glY, glZ);
                                        if (points.size() > 1) {
                                            if (Util::distance(points[points.size()-1], points[points.size()-2]) < 0.1) {
                                                // skip this wall because its too thin
                                                invalid = true;
                                                break;
                                            }
                                            // check is this wall is rectangular
                                            bool difX = points[points.size() - 1].x != points[points.size() - 2].x;
                                            bool difY = points[points.size() - 1].y != points[points.size() - 2].y;
                                            bool difZ = points[points.size() - 1].z != points[points.size() - 2].z;
                                            if (difX && difY || difZ && difY) {
                                                // if neighbour points differ on y and xz plane -> no rectangle
                                                newWall.isRect = false;
                                            }
                                        }
                                    }
                                    if (invalid)
                                        continue;
                                    float wallHeight = maxY - minY;
                                    if (wallHeight < 0.2f) {
                                        continue; // manche walls haben mehrere polygone
                                    }

                                    if (points.size() != 5) {
                                        newWall.isRect = false;
                                    }

                                    // MARK1
                                    auto vec1 = Util::normalize(Util::vectorSubtract(points[0], points[1]));
                                    auto vec2 = Util::normalize(Util::vectorSubtract(points[1], points[2]));
                                    auto normal = Util::normalize(Util::crossProduct(vec1, vec2));

                                    auto pointsCopy = points;
                                    // randpunkte holen nach x achse // TODO make this a function
                                    std::nth_element(pointsCopy.begin(), pointsCopy.begin(), pointsCopy.end(), xComparator2);
                                    newWall.point1 = pcl::PointXYZ(pointsCopy[0].x, minY, pointsCopy[0].z);
                                    std::nth_element(pointsCopy.begin(), pointsCopy.end() - 1, pointsCopy.end(), xComparator2);
                                    newWall.point2 = pcl::PointXYZ(pointsCopy[pointsCopy.size() - 1].x, maxY, pointsCopy[pointsCopy.size() - 1].z);


                                    newWall.mid.x = (minX + maxX) / 2;
                                    newWall.mid.y = (minY + maxY) / 2;
                                    newWall.mid.z = (minZ + maxZ) / 2;

                                    newWall.mid.normal_x = normal.x;
                                    newWall.mid.normal_y = normal.y;
                                    newWall.mid.normal_z = normal.z;

                                    // reorient point1 and point 2 to match normal
                                    auto vec1b = Util::vectorSubtract(newWall.point1, pcl::PointXYZ(newWall.point2.x, minY, newWall.point2.z));
                                    auto vec2b = Util::vectorSubtract(newWall.mid, newWall.point1);
                                    auto testNormal = Util::normalize(Util::crossProduct(vec1b, vec2b));
                                    // if sign is switched for x or z -> swap
                                    if (signbit(normal.x) != signbit(testNormal.x) || signbit(normal.z) != signbit(testNormal.z)) {
                                        // swap
                                        auto temp = newWall.point1;
                                        newWall.point1.x = newWall.point2.x;
                                        newWall.point1.z = newWall.point2.z;
                                        newWall.point2.x = temp.x;
                                        newWall.point2.z = temp.z;
                                    }

                                    newWall.length = Util::horizontalDistance(newWall.point1, newWall.point2);
                                    newWall.points = points;

                                    newBuilding.osmWalls.push_back(newWall);

                                }
                            }

                        }
                    }
                }
            }
        }

        std::cout << TAG << "read data of " << gmlBuildings.size() << " buildings" << std::endl;
        std::cout << TAG << "finished reading gml file" << std::endl;

    } else {
        throw std::runtime_error("Can't find .gml file");
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

void DataIO::readShp(const std::string& path, std::vector<Polygon>* polygons) {
    std::string TAG = DataIO::TAG + "readShp\t";

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

            // read var length record header
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
                    // convert point from w84 to utm zone 32n
                    double utmX, utmZ;
                    LatLonToUTMXY(point.z, point.x, 32, utmX, utmZ);
                    point.x = utmX - xOffset;
                    point.z = -(utmZ - zOffset); // negative z for opengl coord sys
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

        std::cout << TAG << "read data of " << polygons->size() << " polygons" << std::endl;
        std::cout << TAG << "finished reading shp file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .shp file");
    }
}

float DataIO::preprocessWalls(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal>& wallOctree,
                              std::vector<Polygon>& polygons) {
    std::string TAG = DataIO::TAG + "preprocessWalls\t";
    // preprocessing of polygons to buildings
    // save all walls (min, mid, max point & radius)
    // dann beim normalen orientieren spatial search nach mid point mit max radius von allen walls
    std::cout << TAG << "preprocess polygons to buildings/walls" << std::endl;
    float maxR = 0;
    buildings = std::vector<Building>(polygons.size());
    auto wallMidPoints = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for (auto bIdx = 0; bIdx < polygons.size(); bIdx++) {
        const auto& polygon = polygons[bIdx];
        auto& building = buildings[bIdx];
        building.xMin = polygon.xMin; // TODO merge readShp und preprocessWalls
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
        float wallHeight = 80; // TODO get dynamically from las data
        float ground = -38;
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
            osmWalls.push_back(wall);

        }
    }
    std::cout << TAG << "processed " << wallMidPoints->size() << " walls" << std::endl;

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
std::string DataIO::readCache(const std::string& lasFile,
                                   const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, std::vector<pcl::PointXY>& texCoords,
                                   std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec, int& wallPointsStartIndex, std::vector<int>& pointClasses) {

    std::string TAG = DataIO::TAG + "readCache\t";
    std::cout << TAG << "try to find cache file..." << std::endl;

    std::string cacheFile = lasFile;
    cacheFile.replace(cacheFile.end() - 3, cacheFile.end(), "splat_cache");

    std::string cacheType = "splat";

    std::ifstream inf(cacheFile, std::ios::binary);
    if (!inf.good()) {
        std::cout << TAG << "no splat cache file found" << std::endl;
        cacheType = "point";

        cacheFile = lasFile;
        cacheFile.replace(cacheFile.end() - 3, cacheFile.end(), "point_cache");
        inf = std::ifstream(cacheFile, std::ios::binary);

        if (!inf.good()) {
            std::cout << TAG << "no point cache file found" << std::endl;
            return "none";
        }
    }


    if (inf.is_open()) {
        std::cout << TAG << cacheType << " cache file found" << std::endl;

        // read header
        FeatureCacheHeader cacheHeader;
        inf.read((char*) (&cacheHeader), sizeof(FeatureCacheHeader));


        // check if right version
        if (cacheHeader.version != FEATURE_CACHE_VERSION) {
            throw std::runtime_error("Feature Cache File has wrong version");
        }
        
        wallPointsStartIndex = cacheHeader.wallPointStartIndex;

        std::cout << TAG << "reading " << cacheHeader.numberOfPoints << " points from cache" << std::endl;
        
        // read features
        for (int i = 0; i < cacheHeader.numberOfPoints; i++) {

            pcl::PointXYZRGBNormal point;
            
            float coords[3];
            inf.read((char*) (&coords), 3 * sizeof(float));
            point.x = coords[0];
            point.y = coords[1];
            point.z = coords[2];
            
            float normal[3];
            inf.read((char*) (&normal), 3 * sizeof(float));
            point.normal_x = normal[0];
            point.normal_y = normal[1];
            point.normal_z = normal[2];

            float rgb;
            inf.read((char*) (&rgb), sizeof(float));
            point.rgb = rgb;
            
            cloud->push_back(point);


            float tangen1Cache[3];
            inf.read((char*) (&tangen1Cache), 3 * sizeof(float));
            auto tan1 = pcl::PointXYZ(tangen1Cache[0], tangen1Cache[1], tangen1Cache[2]);
            tangent1Vec.emplace_back(tangen1Cache[0], tangen1Cache[1], tangen1Cache[2]);

            float tangen2Cache[3];
            inf.read((char*) (&tangen2Cache), 3 * sizeof(float));
            tangent2Vec.emplace_back(tangen2Cache[0], tangen2Cache[1], tangen2Cache[2]);

            float texCoordsCache[2];
            inf.read((char*) (&texCoordsCache), 2 * sizeof(float));
            texCoords.emplace_back(texCoordsCache[0], texCoordsCache[1]);

            int pointClass;
            inf.read((char*) (&pointClass), sizeof(int));
            pointClasses.emplace_back(pointClass);

        }

        if (!inf.good())
            throw std::runtime_error("Reading .cache ran into error");

        std::cout << TAG << "finished reading points from cache" << std::endl;
        return cacheType;

    } else {
        throw std::runtime_error("Can't find .cache file");
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
DataIO::writeCache(const std::string& lasFile, bool splatCache, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const std::vector<pcl::PointXY>& texCoords,
                   std::vector<pcl::PointXYZ>& tangent1Vec, std::vector<pcl::PointXYZ>& tangent2Vec, int& wallPointsStartIndex, std::vector<int>& pointClasses) {

    std::string TAG = DataIO::TAG + "writeCache\t";

    std::cout << TAG << "writing points to cache" << std::endl;




    std::string cacheFile = lasFile;
    if (splatCache) {
        cacheFile.replace(cacheFile.end() - 3, cacheFile.end(), "splat_cache");
    } else {
        cacheFile.replace(cacheFile.end() - 3, cacheFile.end(), "point_cache");
    }

    std::ofstream out(cacheFile, std::ios::binary);
    if (out.is_open()) {
        // write header
        FeatureCacheHeader cacheHeader;
        cacheHeader.numberOfPoints = cloud->size();
        cacheHeader.version = FEATURE_CACHE_VERSION;
        cacheHeader.wallPointStartIndex = wallPointsStartIndex;

        out.write((char*) (&cacheHeader), sizeof(FeatureCacheHeader));

        // write features
        for (int i = 0; i < cloud->points.size(); i++) {
            const auto& point = (*cloud)[i];
            out.write((char*) (&point.x), sizeof(float));
            out.write((char*) (&point.y), sizeof(float));
            out.write((char*) (&point.z), sizeof(float));
            out.write((char*) (&point.normal), 3 * sizeof(float));
            out.write((char*) (&point.rgb), sizeof(float));
//            out.write((char*) (&it->curvature), sizeof(float));

            const auto& tangent1 = tangent1Vec[i];
            out.write((char*) (&tangent1.x), sizeof(float));
            out.write((char*) (&tangent1.y), sizeof(float));
            out.write((char*) (&tangent1.z), sizeof(float));

            const auto& tangent2 = tangent2Vec[i];
            out.write((char*) (&tangent2.x), sizeof(float));
            out.write((char*) (&tangent2.y), sizeof(float));
            out.write((char*) (&tangent2.z), sizeof(float));

            const auto& texCoord = texCoords[i];
            out.write((char*) (&texCoord.x), sizeof(float));
            out.write((char*) (&texCoord.y), sizeof(float));

            const auto& pointClass = pointClasses[i];
            out.write((char*) (&pointClass), sizeof(int));
        }

        if (!out.good())
            throw std::runtime_error("Writing .cache ran into error");

        out.close();
        std::cout << TAG << "finished writing cache" << std::endl;


    } else {
        throw std::runtime_error("Can't find .cache file");
    }
}

bool DataIO::fitPlane(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const Util::Wall& osmWall, pcl::Indices& certainWallPoints,
                      pcl::PCA<pcl::PointXYZRGBNormal>& pca, Util::Wall& lasWall) {

    pcl::IndicesPtr certainWallPointsPtr = std::make_shared<pcl::Indices>(certainWallPoints);
    pca.setIndices(certainWallPointsPtr);
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector3f eigenValues = pca.getEigenValues();

    // create plane
    // get median point of certain wall points
    float xMedian, yMedian, zMedian;
    findXYZMedian(cloud, certainWallPoints, xMedian, yMedian, zMedian);
    lasWall.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);

    // normal
    // check if normal is more vertical
    auto vertLen = abs(eigenVectors(1, 2));
    if (vertLen > 0.5) { // length adds up to 1
        return false;
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
    // new mid that is in the middle
    lasWall.mid.x = (lasWall.point1.x + lasWall.point2.x) / 2.0f;
    // keep y as median value
    lasWall.mid.z = (lasWall.point1.z + lasWall.point2.z) / 2.0f;

    lasWall.length = Util::horizontalDistance(lasWall.point1, lasWall.point2);
    return true;

}

// ************* experiments ****************

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
            auto debug = "too big";
            certainWallPoints.clear();
        } else {
            auto debug = "okay";
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

    // do it per building part
    for (auto pIdx = 0; pIdx < building.parts.size(); pIdx++) {
        const int& partStart = building.parts[pIdx];
        // letzter part ? -> end = building walls size : next part start
        const int& partEnd = (pIdx == building.parts.size() - 1) ? static_cast<int>(building.osmWalls.size())
                                                                 : building.parts[pIdx + 1];

        // matrix: cos angle, sin angle, transl x, transl z
        auto matrices = std::vector<std::array<float, 4>>(partEnd - partStart);
        std::vector<int> osmWallsWithLasWallIndices;
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

            // TODO use avg instead of median?
        }
        //endregion


        float minError = INFINITY;
        int minErrorIdx;
        //region find matrix with smallest error

        // get index sample;
        std::vector<int> out;
        size_t nelems = osmWallsWithLasWallIndices.size();// / 2; //TODO how often ransac?
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

                float normalError = Util::vectorLength(Util::vectorSubtract(newNormal, lasWallNormal));
                float distError = Util::horizontalDistance(newMidPoint, lasWall.mid);
                error += normalError + distError;
                if (osmWallIdx == testWallIdx) {
                    auto ble = 2;
                }
            }

            if (error < minError) {
                minError = error;
                minErrorIdx = testWallIdx;
            }
        }
        //endregion

        if (minError == INFINITY)
            return;
        //  da kanns dann sein dass die oben bei den random dingern nicht dabei ist, dadurch der min error nicht gesetzt wird und dann crasht es unten.
        //  ich muss mir überlegen was ich dann machen will? vermutlich oben den random index so wählen dass er nur aus tatsächlichen las wänden gewählt wird

        // region use this matrix to transform all osm walls to new las walls

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

    bool colorDiscard = true;
    bool colorPointGroups = false;
    bool colorPatches = false;
    bool colorWallPatchesAndPoints = true;

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

    //region find wall points that have no corresponding osm wall

    for (auto lasPointIdx = 0; lasPointIdx < lasWallPoints.size(); lasPointIdx++) {
        if (lasWallPoints[lasPointIdx] && !usedLasWallPoints[lasPointIdx]) {
            auto& point = (*allPointsCloud)[lasPointIdx];
            point.r = 255;
            point.g = 255;
            point.b = 255;
//            point.r = 100;
//            point.g = 100;
//            point.b = 100;

            remainingWallsCloud->push_back(point);
            idxMap[remainingWallsCloud->size() - 1] = lasPointIdx;
            pointSearchIndices.emplace_back(remainingWallsCloud->size() - 1);

        } else {
            auto& point = (*allPointsCloud)[lasPointIdx];
            point.r = 100;
            point.g = 100;
            point.b = 100;
        }
    }
    // endregion



    std::vector<Util::Wall> wallPatches;
    std::vector<pcl::Indices> wallPatchPointIdc;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wallPatchMids = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);


    // tree will get updated indices when points are assigned to a wall patch
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr remainingWallsTree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    remainingWallsTree->setInputCloud(remainingWallsCloud);

    auto wallPointSkip = std::vector<bool>(remainingWallsCloud->size());
    std::fill(wallPointSkip.begin(), wallPointSkip.end(), false);
    // prepare pca
    pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(remainingWallsCloud);

    //region remove horizontal planes

    pcl::Indices pointSearchIndicesCopy = pointSearchIndices;

    for (auto pIdx = 0; pIdx < remainingWallsCloud->size(); pIdx++) {

        if (wallPointSkip[pIdx])
            continue;

        auto& point = (*remainingWallsCloud)[pIdx];

        // radius search
        pcl::Indices searchResultIdx;
        std::vector<float> searchResultDist;
        if (remainingWallsTree->radiusSearch(point, 1.0f, searchResultIdx, searchResultDist) < 3) {
            continue;
        }

        // prepare pca for neighbour points
        pcl::IndicesPtr searchResultIdxPtr = std::make_shared<pcl::Indices>(searchResultIdx);
        pca.setIndices(searchResultIdxPtr);

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


        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();

        if (mainDings == 1) {
            // planarity is main
            // plane -> normale von 3. eogenvektor nehmen -> wenn zu vertikal dann weg
            auto normal = pcl::PointXYZ(eigenVectors(0, 2), eigenVectors(1, 2), eigenVectors(2, 2));
            if (abs(eigenVectors(1, 2)) > 0.5) {
                for (const auto& nIdx: searchResultIdx) {
                    wallPointSkip[nIdx] = true;
                    // remove from copy -> will be removed after looking at all points
                    auto bla = std::find(pointSearchIndicesCopy.begin(), pointSearchIndicesCopy.end(), nIdx);
                    if (bla != pointSearchIndicesCopy.end())
                        pointSearchIndicesCopy.erase(bla);

                    if (colorDiscard) {
                        (*allPointsCloud)[idxMap[nIdx]].r = 0; // gelb
                        (*allPointsCloud)[idxMap[nIdx]].g = 255;
                        (*allPointsCloud)[idxMap[nIdx]].b = 255;
                    }
                }
            }
        }
    }

    // remove those points from search
    pcl::IndicesPtr pointSearchIndicesPtr = std::make_shared<pcl::Indices>(pointSearchIndicesCopy);
    remainingWallsTree->setInputCloud(remainingWallsCloud, pointSearchIndicesPtr);
    //endregion

    //region find wall patches

    // for every wall point
    for (auto pIdx = 0; pIdx < remainingWallsCloud->size(); pIdx++) {
        if (wallPointSkip[pIdx])
            continue;

        auto& point = (*remainingWallsCloud)[pIdx];

        // radius search
        pcl::Indices searchResultIdx;
        std::vector<float> searchResultDist;
        if (remainingWallsTree->radiusSearch(point, 1.0f, searchResultIdx, searchResultDist) < 3) {
            continue;
        }

        // prepare pca for neighbour points
        pcl::IndicesPtr searchResultIdxPtr = std::make_shared<pcl::Indices>(searchResultIdx);
        pca.setIndices(searchResultIdxPtr);

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


        int patchColor[3];

        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();

        pcl::PointXYZ normal;
        switch (mainDings) {
            case 0:
                // linearity is main
                // linear -> 1. eigenvektor: wenn zu vertikal skip und dann normale bestimmen über cross mit senkrechtem vektor
                patchColor[0] = 255;
                patchColor[1] = 0;
                patchColor[2] = 0;
                // für lineare sachen nicht normale nehmen sondern stärksten eigenvektor, nur der ist eindeutig, zweiter und die normale können um die lineare achse rotieren
//                normal = pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2,0)); // debug
                normal = Util::normalize(Util::crossProduct(pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0)), pcl::PointXYZ(0, 1, 0)));
                if (abs(eigenVectors(1, 0)) > 0.5) {
                    patchColor[0] = 255; // türkis
                    patchColor[1] = 255;
                    patchColor[2] = 0;
                    continue;
                }
                break;
            case 1:
                // planarity is main
                // plane -> normale von 3. eigenvektor nehmen -> wenn zu vertikal dann weg
                patchColor[0] = 0;
                patchColor[1] = 255;
                patchColor[2] = 0;
                normal = pcl::PointXYZ(eigenVectors(0, 2), eigenVectors(1, 2), eigenVectors(2, 2));
                if (abs(eigenVectors(1, 2)) > 0.5) {
                    patchColor[0] = 255; // pink
                    patchColor[1] = 0;
                    patchColor[2] = 255;
                }
                break;
            default:
                // sphericity is main
                // sphere -> weg
                patchColor[0] = 0;
                patchColor[1] = 0;
                patchColor[2] = 255;
                continue;
                break;
        }

        for (const auto& nIdx: searchResultIdx) {
            if (wallPointSkip[nIdx])
                continue;
            wallPointSkip[nIdx] = true;
            (*allPointsCloud)[idxMap[nIdx]].normal_x = normal.x;
            (*allPointsCloud)[idxMap[nIdx]].normal_y = normal.y;
            (*allPointsCloud)[idxMap[nIdx]].normal_z = normal.z;
            if (colorPointGroups) {
                (*allPointsCloud)[idxMap[nIdx]].r = patchColor[0];
                (*allPointsCloud)[idxMap[nIdx]].g = patchColor[1];
                (*allPointsCloud)[idxMap[nIdx]].b = patchColor[2];
            }
            // remove from copy -> will be removed after looking at all points
            auto bla = std::find(pointSearchIndicesCopy.begin(), pointSearchIndicesCopy.end(), nIdx);
            if (bla != pointSearchIndicesCopy.end())
                pointSearchIndicesCopy.erase(bla);
        }


        Util::Wall patch;
        // mid
        float xMedian, yMedian, zMedian;
        findXYZMedian(remainingWallsCloud, searchResultIdx, xMedian, yMedian, zMedian);
        patch.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
        // normal
        patch.mid.normal_x = normal.x;
        patch.mid.normal_y = normal.y;
        patch.mid.normal_z = normal.z;

        // save point indices of this patch
        wallPatchPointIdc.push_back(searchResultIdx);
        // update tree -> points of this patch will not be included in next search
//        remainingWallsTree->setInputCloud(remainingWallsCloud, pointSearchIndicesPtr);
        // add to patches
        wallPatches.push_back(patch);
        wallPatchMids->push_back(patch.mid);

    }

    // remove points from search that belong to patches
    pointSearchIndicesPtr = std::make_shared<pcl::Indices>(pointSearchIndicesCopy);
    remainingWallsTree->setInputCloud(remainingWallsCloud, pointSearchIndicesPtr);
    //endregion

    std::vector<Util::Wall> finalWalls;
    //region combine wall patches

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr wallPatchTree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    wallPatchTree->setInputCloud(wallPatchMids);
    auto wallPatchSkip = std::vector<bool>(wallPatchMids->size());
    std::fill(wallPatchSkip.begin(), wallPatchSkip.end(), false);

    // debug
    int lookAt = 53; //165
    for (int patchIdx = 0; patchIdx < wallPatches.size(); patchIdx++) {

//        if (patchIdx > lookAt || patchIdx < lookAt) {
////        if (patchIdx > lookAt){
//            continue;
//        }

        if (wallPatchSkip[patchIdx])
            continue;


        const auto& wallPatch = wallPatches[patchIdx];
        // radius search
        pcl::Indices wallPatchSearchResultIdx;
        std::vector<float> wallPatchSearchResultDist;
        float searchRadius = 20.0f;
        if (wallPatchTree->radiusSearch(wallPatch.mid, searchRadius, wallPatchSearchResultIdx, wallPatchSearchResultDist) <= 0) {
            continue;
        }
        // radius search points
        pcl::Indices wallPointSearchResultIdx;
        std::vector<float> wallPointSearchResultDist;
        remainingWallsTree->radiusSearch(wallPatch.mid, searchRadius, wallPointSearchResultIdx, wallPointSearchResultDist);


        pcl::Indices wallCandidatePointIdc;
        pcl::Indices wallCandidatePatchIdc;
        // der patch selbst ist immer dabei
        wallCandidatePatchIdc.push_back(patchIdx);
        // die punkte vom patch selbst sind immer dabei
        wallCandidatePointIdc.insert(wallCandidatePointIdc.end(), wallPatchPointIdc[patchIdx].begin(), wallPatchPointIdc[patchIdx].end());
        // remove first point of radius search weil das ist der patch selbst
        wallPatchSearchResultIdx.erase(wallPatchSearchResultIdx.begin());
        wallPatchSearchResultDist.erase(wallPatchSearchResultDist.begin());
        // bounds

        auto patchNormal = pcl::PointXYZ(wallPatch.mid.normal_x, wallPatch.mid.normal_y, wallPatch.mid.normal_z);
        // TODO remember which building the points belong to -> don't mix buildings

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


        // need other patches to build a wall
        if (wallPatchSearchResultIdx.empty()) {
            continue;
        }


        auto wallCandidate = Util::Wall();
        wallCandidate.mid = wallPatch.mid;

        // TODO versuch 1: grow suche nach patches und einzelpunkten: nur mit patches die wall anpassen
        //  versuch 2: von patch aus suchen damit man startwerte hat, dann einfach punkte suchen und hinzufügen und neue params

        auto wallPointSkipCopy = wallPointSkip;
        auto wallPatchSkipCopy = wallPatchSkip;

        int patchCount = 1;

        float maxDist = -INFINITY;
        while (true) {
            // radius suche immer von gleichem patch mid aus (anstatt von wall candidate mid aus was sich verschiebt) -> sichere, nicht verfälschte suche
            if (wallPatchSearchResultIdx.empty() && wallPointSearchResultIdx.empty()) {
                // wenn einer hinzugefügt wurde, der nah am rand ist → search again with bigger radius
                if (maxDist > searchRadius * 0.8) {
                    // search again with bigger radius
                    searchRadius += 10;
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
                    // remove points that have already been taken
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
            // TODO remove points with bad normals first?

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
                    float dist = Util::horizontalDistance(neighbourPatchPoint, cPoint); // TODO which distance?
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
            // check if there is a neighbour point that is near a wall combi patch
            auto nearPointIt = wallPointSearchResultIdx.end();
            float nearPointDist = INFINITY;
            for (auto wallPointNeighIdxIt = wallPointSearchResultIdx.begin();
                 wallPointNeighIdxIt != wallPointSearchResultIdx.end(); wallPointNeighIdxIt++) {
                const auto& wallPointNeighIdx = *wallPointNeighIdxIt;
                auto& neighbourPoint = remainingWallsCloud->points[wallPointNeighIdx];

                // wenn es nah an irgendeinem patch aus der combi ist dann go
                bool near = false;
                for (auto& cPatchIdx: wallCandidatePatchIdc) {
                    const auto& cPatch = wallPatches[cPatchIdx];
                    float dist = Util::distance(neighbourPoint, cPatch.mid); // TODO which distance?
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
                float normalAngle = acos(Util::dotProduct(patchNormal, neighbourNormal));
                float normalAngleAndersrum = acos(Util::dotProduct(patchNormal, pcl::PointXYZ(-neighbourNormal.x, -neighbourNormal.y, -neighbourNormal.z)));

                if (normalAngle <= 0.78f || normalAngleAndersrum <= 0.78f) {//0.78f) { // 45°

                    // check plane distance
                    auto ppd = Util::pointPlaneDistance(neighbourPatchPoint, wallCandidate.mid);
                    if (ppd < 1.5f) {
                        // what if i added the plane? -> measure error
                        // save old value in case they need to  be restored
                        auto wallCandidatePatchIdcCopy = wallCandidatePatchIdc;
                        // error
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
//                        if (error < threshold) {
//                            // found a patch for the combi
//                            wallCandidatePatchIdc.push_back(*nearPatchIt);
//                            wallCandidatePointIdc.insert(wallCandidatePointIdc.end(),
//                                                         wallPatchPointIdc[*nearPatchIt].begin(),
//                                                         wallPatchPointIdc[*nearPatchIt].end());
//                            // remove it from patch search
//                            wallPatchSkip[*nearPatchIt] = true;
//                            // save distance
//                            auto dist = Util::distance(neighbourPatchPoint, wallPatch.mid);
//                            if (dist > maxDist) {
//                                maxDist = dist;
//                            }
//                            // update mid point
//                            float xMedian, yMedian, zMedian;
//                            findXYZMedian(remainingWallsCloud, wallCandidatePointIdc, xMedian, yMedian, zMedian);
//                            wallCandidate.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
//
//                            // update normal
//                            auto newNormal = pcl::PointXYZ(0, 0, 0);
//
//                            pcl::IndicesPtr wallCandidatePointIdcPtr = std::make_shared<pcl::Indices>(
//                                    wallCandidatePointIdc);
//                            pca.setIndices(wallCandidatePointIdcPtr);
//                            Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
//                            newNormal = pcl::PointXYZ(eigenVectors(0, 2), 0, eigenVectors(2, 2));
//                            newNormal = Util::normalize(newNormal);
//                            wallCandidate.mid.normal_x = newNormal.x;
//                            wallCandidate.mid.normal_y = newNormal.y;
//                            wallCandidate.mid.normal_z = newNormal.z;
//
//                            patchCount++;
//
////                        // TODO for debug
////                        for (const auto& combWallIdx: wallCandidatePatchIdc) {
////                            auto& combWall = wallPatches[combWallIdx];
////                            combWall.mid.normal_x = newNormal.x;
////                            combWall.mid.normal_y = newNormal.y;
////                            combWall.mid.normal_z = newNormal.z;
////                        }
//                        } else {
//                            // restore old values
//
//                        }
                    }
                }
                // remove patch from neighbours regardless if it has good angle and belongs to combi or not
                wallPatchSearchResultIdx.erase(nearPatchIt);

            } else {
                // nearest is point
                auto& neighbourPoint = remainingWallsCloud->points[*nearPointIt];

                // check plane distance
                auto ppd = Util::pointPlaneDistance(neighbourPoint, wallCandidate.mid);
                if (ppd < 1.0f) {
//                    if (error < threshold) {
//                        // found a point for the combi
//                        wallCandidatePointIdc.push_back(*nearPointIt);
//                        // remove it from patch search
//                        wallPointSkip[*nearPointIt] = true;
//                        // save distance
//                        auto dist = Util::distance(neighbourPoint, wallPatch.mid);
//                        if (dist > maxDist) {
//                            maxDist = dist;
//                        }
//                        // update mid point
//                        float xMedian, yMedian, zMedian;
//                        findXYZMedian(remainingWallsCloud, wallCandidatePointIdc, xMedian, yMedian, zMedian);
//                        wallCandidate.mid = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
//
//                        // update normal
//                        auto newNormal = pcl::PointXYZ(0, 0, 0);
//
//                        pcl::IndicesPtr wallCandidatePointIdcPtr = std::make_shared<pcl::Indices>(
//                                wallCandidatePointIdc);
//                        pca.setIndices(wallCandidatePointIdcPtr);
//                        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
//                        newNormal = pcl::PointXYZ(eigenVectors(0, 2), 0, eigenVectors(2, 2));
//                        newNormal = Util::normalize(newNormal);
//                        wallCandidate.mid.normal_x = newNormal.x;
//                        wallCandidate.mid.normal_y = newNormal.y;
//                        wallCandidate.mid.normal_z = newNormal.z;
//
////                        // TODO for debug
////                        for (const auto& combWallIdx: wallCandidatePatchIdc) {
////                            auto& combWall = wallPatches[combWallIdx];
////                            combWall.mid.normal_x = newNormal.x;
////                            combWall.mid.normal_y = newNormal.y;
////                            combWall.mid.normal_z = newNormal.z;
////                        }
//                    }
                }

                // remove point from neighbours regardless if it has good angle and belongs to combi or not
                wallPointSearchResultIdx.erase(nearPointIt);
            }
        }


        if (patchCount < 3) {
            // skippis wieder freigeben
            wallPatchSkip = wallPatchSkipCopy;
            wallPointSkip = wallPointSkipCopy;
            continue;
        }

        // valid wall
        wallPatchSkip[patchIdx] = true; // sonst kann leitender patch noch wonaders mit reinkommen
        // patches
        int randB = rand() % (156) + 100; //  rand() % (255 - 0 + 1) + 0;
        int randG = rand() % (156) + 100;
        int randR = rand() % (156) + 100;

//        auto& color = colors[colorIndex];
//        colorIndex = (colorIndex + 1) % colorCount;

        // ############## draw combined patches
        if (colorWallPatchesAndPoints) {
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
        }
        // TODO normal is avg of patches, keep or pca?


        // region draw wall

        // wall: combWall
        // patches davon: wallCandidatePatchIdc
        // punkte davon: wallPatchPointIdc


//        pcl::Indices allWallCandidatePointIdc;
        for (auto& pointIdx: wallCandidatePointIdc) {

            // TODO loop in findStartEnd?
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

        float wallR = randR, wallG = randG, wallB = randB;
//        float wallR = color[0], wallG = color[1], wallB = color[2];
        if (patchCount < 3) {
            wallR = 0;
            wallG = 0;
            wallB = 200;
        }


    }

    //endregion
}
