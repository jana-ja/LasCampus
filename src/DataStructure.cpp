//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include <set>
#include <numeric>
#include "util.h"
#include "DataStructure.h"
#include "UTM.h"
#include <pcl/common/pca.h>

using namespace std;

DataStructure::DataStructure(const std::vector<std::string>& lasFiles, const std::string& shpFile, const std::string& imgFile) {


    DataIO dataIO = DataIO();//lasFiles, shpFile, imgFile);

    std::vector<bool> lasWallPoints;
    std::vector<bool> lasGroundPoints;
    bool cachedFeatues = dataIO.readData(lasFiles, shpFile, imgFile, cloud, buildings, lasWallPoints, lasGroundPoints);

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud);


    detectWalls(lasWallPoints, lasGroundPoints, tree);
    // cloud has changed
    tree->setInputCloud(cloud);

    if (!cachedFeatues) {
//        adaSplats(tree);

//        std::string lasDir = ".." + Util::PATH_SEPARATOR + "las" + Util::PATH_SEPARATOR;
//        const auto& file = lasFiles[0];
//        std::string cacheFile = file;
//        cacheFile.replace(cacheFile.end() - 3, cacheFile.end(), "features");
//        dataIO.writeFeaturesToCache(lasDir + cacheFile, cloud);
    }

}

void DataStructure::detectWalls(vector<bool>& lasWallPoints, vector<bool>& lasGroundPoints, const pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree) {
    bool colorOsmWall = false;
    bool colorCertainLasWall = false;
    bool colorCertainLasWallRandom = false;
    bool colorFinalLasWall = true;
    bool colorFinalLasWallWithoutGround = false;

    // TODO im currently searching for each wall with lasPoint tree here, and searching for each point with wallTree in DataIO.
    //  -> make more efficient?

    bool oneWall = false;

    pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
    pca.setInputCloud(cloud);

    float lasWallThreshold = 0.5; // 0.2, 0.3 oder so?
    float osmWallThreshold = 1.0; // TODO macht eig keinen sinn das hier größer zu haben als in DataIO filter funktion? weil die punkte dann eh raus sind

    int buildingCount = 0;
    int wallCount = 0;
    int buildingsNumber = 184;
    int wallNumber = 12; // ich will wand 12 und 14, 13 ist iwie son dummer stumpf??
    for (auto building: buildings) {

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

                auto osmMinX = min(osmWallPoint1.x, osmWallPoint2.x);
                auto osmMaxX = max(osmWallPoint1.x, osmWallPoint2.x);
                auto osmMinZ = min(osmWallPoint1.z, osmWallPoint2.z);
                auto osmMaxZ = max(osmWallPoint1.z, osmWallPoint2.z);

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

                }
                //endregion

                if (certainWallPoints.size() < 3)
                    continue;

                pcl::PointXYZRGBNormal lasWallPlane;
                // TODO vllt ebene stattdessen so fitten dass die wand senkrecht ist und dann dist zu den punkten am kleinsten ist? im median?
                // TODO in 2d betrachten udn regression könnte besser sein, weil manchmal zB nur dachkante -> liefert hier ieine ebene die vllt eher waagerecht ist und dann rasugeworfen wird.
                //region fit *vertical* plane through certain wall points

                pcl::IndicesPtr certainWallPointsPtr = make_shared<pcl::Indices>(certainWallPoints);
                pca.setIndices(certainWallPointsPtr);
                Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
                Eigen::Vector3f eigenValues = pca.getEigenValues();

                // create plane
                // get median point of certain wall points
                float xMedian, yMedian, zMedian;
                findXYZMedian(certainWallPoints, xMedian, yMedian, zMedian);
                lasWallPlane = pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian);
                cloud->push_back(pcl::PointXYZRGBNormal(xMedian, yMedian, zMedian, 0, 0, 255));

                // normal
                lasWallPlane.normal_x = eigenVectors(0, 2);
                lasWallPlane.normal_y = eigenVectors(1, 2);
                lasWallPlane.normal_z = eigenVectors(2, 2);
                // should be vertical
                auto horLen = sqrt(pow(lasWallPlane.normal_x, 2) + pow(lasWallPlane.normal_z, 2));
                auto vertLen = abs(lasWallPlane.normal_y);
                if (vertLen > horLen) {
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

                // vllt puffer einbauen?
//                    float buffer = 0.5;
//                    auto wallVec = Util::normalize(Util::vectorSubtract(lasWallPoint2, lasWallPoint1)); // von 1 nach 2
//                    lasWallPoint1 = pcl::PointXYZ(lasWallPoint1.x - buffer * wallVec.x, lasWallPoint1.y - buffer * wallVec.y, lasWallPoint1.z - buffer * wallVec.z); // 1 -= vec
//                    lasWallPoint2 = pcl::PointXYZ(lasWallPoint2.x + buffer * wallVec.x, lasWallPoint2.y + buffer * wallVec.y, lasWallPoint2.z + buffer * wallVec.z); // 2 += vec
                // get min max wall borders
                auto lasMinX = min(lasWallPoint1.x, lasWallPoint2.x);
                auto lasMaxX = max(lasWallPoint1.x, lasWallPoint2.x);
                auto lasMinZ = min(lasWallPoint1.z, lasWallPoint2.z);
                auto lasMaxZ = max(lasWallPoint1.z, lasWallPoint2.z);

                // select points with las wall plane with smaller threshold
                // yeah
                // TODO find solution for borders top, bot, left, right (pca classes give top and bottom, but also linear class lines in te middle. no left/right border info)
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


                    finalWallPoints.push_back(
                            nIdx); // dont need to project these, because they are only used to determine y values and projection normal is horizontal
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
                // TODO wand ist nicht überall gleich hoch
                //  beim hochgehen radius search machen 1 m oder so, dann in richtung der wall normal schauen mit kleinem threshold, wenn ich punkt finde aufhören
                //  aber was ist dann mit orthogonalen wänden?
                findYMinMax(finalWallPoints, yMin, yMax);
                // get new border points based on finalWallPoints without ground points (so generated points don't go over the edges)
                findStartEnd(finalWallPointsNotGround, lasWallPoint1, lasWallPoint2);
                lasWallPoint1.y = yMin;
                lasWallPoint2.y = yMin;

                // draw plane
                float stepWidth = 0.5;
                // get perp vec
                auto lasWallNormal = pcl::PointXYZ(lasWallPlane.normal_x, lasWallPlane.normal_y, lasWallPlane.normal_z);
                auto lasWallVec = Util::vectorSubtract(lasWallPoint2, lasWallPoint1); // von 1 nach 2
                auto horPerpVec = Util::normalize(lasWallVec); // horizontal
//                auto vertPerpVec = Util::crossProduct(horPerpVec, lasWallNormal); // vertical -> =(0,1,0)

                float lasWallLength = Util::vectorLength(lasWallVec);
                float x = lasWallPoint1.x;
                float z = lasWallPoint1.z;
                float distanceMoved = 0;
                // move horizontal
                while (distanceMoved < lasWallLength) {
                    float y = yMin;
                    float xCopy = x;
                    float zCopy = z;
                    while (y < yMax) {
                        cloud->push_back(pcl::PointXYZRGBNormal(x, y, z, 255, 255, 255));
                        y += stepWidth;
                    }
                    x = xCopy + stepWidth * horPerpVec.x;
                    z = zCopy + stepWidth * horPerpVec.z;
                    distanceMoved += stepWidth;
                }
                //endregion

            }
        }
    }
}

void DataStructure::adaSplats(pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree) {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start ada" << std::endl;

    tangent1Vec = std::vector<pcl::PointXYZ>((*cloud).size());
    tangent2Vec = std::vector<pcl::PointXYZ>((*cloud).size());


    int k = 40;
    std::vector<pcl::Indices> pointNeighbourhoods(cloud->points.size());
    std::vector<vector<float>> pointNeighbourhoodsDistance(cloud->points.size());
    std::vector<int> pointClasses(cloud->points.size());
    fill(pointClasses.begin(), pointClasses.end(), 2);

    // ********** knn and compute avgRadius **********
    float avgRadiusNeighbourhoods = adaKnnAndRadius(k, tree, pointNeighbourhoods, pointNeighbourhoodsDistance);
    std::cout << TAG << "avg radius R is: " << avgRadiusNeighbourhoods << std::endl;

    // ********** get neighbourhood with radius, use pca for classification and compute epsilon **********
    float splatGrowEpsilon = adaNeighbourhoodsClassificationAndEpsilon(avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);
    std::cout << TAG << "splat grow epsilon is: " << splatGrowEpsilon << std::endl;

    // ********** get new neighbourhood with new k and new radius (depending on classification) and use pca for normal **********
    adaNewNeighbourhoods(k, tree, avgRadiusNeighbourhoods, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);

    // ********** normal orientation **********
    float wallThreshold = 1.0;
    adaNormalOrientation(wallThreshold, tree);

    // ********** compute splats **********
    float alpha = 0.2;
    adaComputeSplats(alpha, splatGrowEpsilon, pointNeighbourhoods, pointNeighbourhoodsDistance, pointClasses);

//    adaResampling(avgRadiusNeighbourhoods, tree, pointClasses);


    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished ada in " << duration.count() << "s" << std::endl;

    // TODO im datensatz de intensity der punkte ansehen ob ich darüber was filtern kann?= zB bäume raus

}

float
DataStructure::adaKnnAndRadius(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree,
                               std::vector<pcl::Indices>& pointNeighbourhoods,
                               std::vector<std::vector<float>>& pointNeighbourhoodsDistance) {
    // ********** knn with default k and compute avgRadius **********
    float avgRadiusSumNeighbourhoods = 0;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        pcl::Indices neighboursPointIdx(k);
        auto neighboursSquaredDistance = std::vector<float>(k);
        if (tree->nearestKSearch(pointIdx, k, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca

            auto const count = static_cast<float>(neighboursSquaredDistance.size());
            auto avgRadius = std::reduce(neighboursSquaredDistance.begin(), neighboursSquaredDistance.end()) /
                             (count - 1); // count - 1, weil die erste distance immer 0 ist
            avgRadiusSumNeighbourhoods += avgRadius;

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursSquaredDistance;
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }
    }

    return avgRadiusSumNeighbourhoods / static_cast<float>(cloud->points.size());;
}

float DataStructure::adaNeighbourhoodsClassificationAndEpsilon(float avgRadiusNeighbourhoods, std::vector<pcl::Indices>& pointNeighbourhoods,
                                                               std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses) {
    float uPtpDistSumNeighbourhoods = 0;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

        if (pointNeighbourhoods[pointIdx].size() >= 3) { // need at least 3 points for pca

            // define Npi as all points here from knn search but within avg radius?
            // find border index
            int lastNeighbour = findIndex(avgRadiusNeighbourhoods, pointNeighbourhoodsDistance[pointIdx]);
            // do stuff
            auto neighbourhood = pcl::Indices(pointNeighbourhoods[pointIdx].begin(),
                                              pointNeighbourhoods[pointIdx].begin() + lastNeighbour);
            if (neighbourhood.size() >= 3) {
                pcl::IndicesPtr neighboursPtr = make_shared<pcl::Indices>(neighbourhood);
                // pca on the neighbourhood
                pcl::PCA<pcl::PointXYZRGBNormal> pca = new pcl::PCA<pcl::PointXYZ>;
                pca.setInputCloud(cloud);
                pca.setIndices(neighboursPtr);
                Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
                Eigen::Vector3f eigenValues = pca.getEigenValues();

                // set normal
                cloud->points[pointIdx].normal_x = eigenVectors(0, 2);
                cloud->points[pointIdx].normal_y = eigenVectors(1, 2);
                cloud->points[pointIdx].normal_z = eigenVectors(2, 2);

                // also set tangents
                auto tangent1 = pcl::PointXYZ(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0));
                tangent1Vec[pointIdx] = tangent1;
                auto tangent2 = pcl::PointXYZ(eigenVectors(0, 1), eigenVectors(1, 1), eigenVectors(2, 1));
                tangent2Vec[pointIdx] = tangent2;

                // check if vertical normal is oriented up
                const auto& point = cloud->points[pointIdx];
                auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                auto vertLen = abs(point.normal_y);
                if (horLen < vertLen) {
                    if (point.normal_y < 0) {
                        (*cloud)[pointIdx].normal_x *= -1;
                        (*cloud)[pointIdx].normal_y *= -1;
                        (*cloud)[pointIdx].normal_z *= -1;

                        // also flip tangents
                        tangent1Vec[pointIdx] = tangent2;
                        tangent2Vec[pointIdx] = tangent1;
                    }
                }

                // local descriptors
                const auto& l1 = eigenValues(0);
                const auto& l2 = eigenValues(1);
                const auto& l3 = eigenValues(2);
                float linearity = (l1 - l2) / l1;
                float planarity = (l2 - l3) / l1;
                float sphericity = l3 / l1;
                std::array bla = {linearity, planarity, sphericity};
                int mainDings = std::distance(bla.begin(), std::max_element(bla.begin(), bla.end()));

                switch (mainDings) {
                    case 0:
                        pointClasses[pointIdx] = 0;
                        // linearity is main
                        if (colorClasses) {
                            (*cloud)[pointIdx].r = 255;
                            (*cloud)[pointIdx].g = 0;
                            (*cloud)[pointIdx].b = 0;
                        }
                        break;
                    case 1:
                        pointClasses[pointIdx] = 1;
                        // planarity is main
                        if (colorClasses) {
                            (*cloud)[pointIdx].r = 0;
                            (*cloud)[pointIdx].g = 255;
                            (*cloud)[pointIdx].b = 0;
                        }
                        break;
                    default:
                        pointClasses[pointIdx] = 2;
                        // sphericity is main
                        if (colorClasses) {
                            (*cloud)[pointIdx].r = 0;
                            (*cloud)[pointIdx].g = 0;
                            (*cloud)[pointIdx].b = 255;
                        }
                        break;
                }

                // splat grow epsilon
                // compute avg unsigned point to plane distance for splat grow epsilon
                float uPtpDistSum = 0;
                // first neighbour is the point itself
                auto bla2 = std::vector<float>(neighbourhood.size());
                for (auto nPointIdx = 1; nPointIdx < neighbourhood.size(); nPointIdx++) {
                    auto ppd = Util::pointPlaneDistance((*cloud)[neighbourhood[nPointIdx]], (*cloud)[pointIdx]);
                    uPtpDistSum += ppd;

                    bla2[nPointIdx] = ppd;
                }
                float uPtpDistAvg = uPtpDistSum / static_cast<float>(neighbourhood.size() - 1);
                uPtpDistSumNeighbourhoods += uPtpDistAvg;

            } else {
                tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
                tangent2Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            }
        } else {
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            tangent2Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
        }
    }

    // ********** compute epsilon **********
    return uPtpDistSumNeighbourhoods / static_cast<float>(pointNeighbourhoods.size());
}

// TODO resampling? inspiration durch ada paper
// TODO datensatz filtern anschauen: bäume raus. nochmal den "schlechteren" datensatz ansehen, da müssten dann auch noch die autos undso rausgefiltert werden aber falls das nicht schwer ist wäre das vllt ne gute option weil aufgefüllte böden unter den gebäuden undso brauche ich ja eh nicht.
// TODO uv eliptical splats ansehen
// TODO das ada paper nochmal zu ende lesen und schauen ob noch was relevantes kommt

// farbwerte von jpeg2000 nehmen

void DataStructure::adaNewNeighbourhoods(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, float avgRadiusNeighbourhoods,
                                         std::vector<pcl::Indices>& pointNeighbourhoods,
                                         std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses) {

    // get new knn
    int currentK;

    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

        // get new k-neighbourhood with adjusted k
        switch (pointClasses[pointIdx]) {
            case 0: // linearity is main
                currentK = k * 2;//0.33;
                break;
            case 1: // planarity is main
                currentK = k * 2;
                break;
            default: // sphericity is main
                currentK = k * 0.25;
                break;
        }
        pcl::Indices neighboursPointIdx(currentK);
        auto neighboursSquaredDistance = std::vector<float>(currentK);
        if (tree->nearestKSearch(pointIdx, currentK, neighboursPointIdx, neighboursSquaredDistance) >=
            3) { // need at least 3 points for pca

            pointNeighbourhoods[pointIdx] = neighboursPointIdx;
            pointNeighbourhoodsDistance[pointIdx] = neighboursSquaredDistance;
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
            pointNeighbourhoodsDistance[pointIdx] = vector<float>();
        }



        // get new neighbourhood with adjusted radius
        float currentAvgRadius;

        if (pointNeighbourhoods[pointIdx].size() >= 3) { // need at least 3 points for pca
            switch (pointClasses[pointIdx]) {
                case 0: // linearity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 2;//0.33;
                    break;
                case 1: // planarity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 2;
                    break;
                default: // sphericity is main
                    currentAvgRadius = avgRadiusNeighbourhoods * 0.25;
                    break;
            }
            // define Npi as all points here from knn search but within avg radius?
            // find border index
            int lastNeighbour = findIndex(currentAvgRadius, pointNeighbourhoodsDistance[pointIdx]);
            // do stuff
            auto neighbours = pcl::Indices(pointNeighbourhoods[pointIdx].begin(),
                                           pointNeighbourhoods[pointIdx].begin() + lastNeighbour);
            if (neighbours.size() >= 3) {
                // save new neighbourhood
                pointNeighbourhoods[pointIdx] = neighbours;
            } else {
                pointNeighbourhoods[pointIdx] = pcl::Indices();
            }
        } else {
            pointNeighbourhoods[pointIdx] = pcl::Indices();
        }
    }
}

void DataStructure::adaNormalOrientation(float wallThreshold, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree) {

    for (auto building: buildings) {
        // get point index of next part/ring if there are more than one, skip "walls" which connect different parts
        auto partIdx = building.parts.begin();
        uint32_t nextPartIndex = *partIdx;
        partIdx++;
        if (partIdx != building.parts.end()) {
            nextPartIndex = *partIdx;
        }
        for (auto pointIdx = 0; pointIdx < building.points.size() - 1; pointIdx++) {

            // if reached end of part/ring -> skip this "wall"
            if (pointIdx + 1 == nextPartIndex) {
                partIdx++;
                if (partIdx != building.parts.end()) {
                    nextPartIndex = *partIdx;
                }
                continue;
            }

            float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
            float ground = -38; // minY // TODO boden ist wegen opengl offset grad bei -38
            pcl::PointXYZRGBNormal wallPoint1, wallPoint2;
            wallPoint1.x = static_cast<float>(building.points[pointIdx].x);
            wallPoint1.y = ground + wallHeight;
            wallPoint1.z = static_cast<float>(building.points[pointIdx].z);
            wallPoint2.x = static_cast<float>(building.points[pointIdx + 1].x);
            wallPoint2.y = ground + wallHeight;
            wallPoint2.z = static_cast<float>(building.points[pointIdx + 1].z);

            // detect (and color) alle points on this wall
            pcl::PointXYZRGBNormal mid;
            mid.x = (wallPoint1.x + wallPoint2.x) / 2;
            mid.y = (ground + wallHeight) / 2;
            mid.z = (wallPoint1.z + wallPoint2.z) / 2;


            Plane wallPlane = {wallPoint1, wallPoint2, mid};

            auto r = static_cast<float>(sqrt(pow(wallPoint2.x - mid.x, 2) + pow(wallHeight - mid.y, 2) + pow(wallPoint2.z - mid.z, 2)));

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            tree->radiusSearch(mid, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            if (!pointIdxRadiusSearch.empty()) {

                auto minX = min(wallPoint1.x, wallPoint2.x);
                auto maxX = max(wallPoint1.x, wallPoint2.x);
                auto minZ = min(wallPoint1.z, wallPoint2.z);
                auto maxZ = max(wallPoint1.z, wallPoint2.z);

                for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {
                    // TODO implement direct calculation test if point lies inside wall rectangle

                    const auto& point = (*cloud)[*nIdxIt];

                    if (Util::pointPlaneDistance(cloud->points[*nIdxIt], wallPlane) > wallThreshold) {
                        continue;
                    }
                    if (point.x > maxX || point.x < minX || point.z > maxZ || point.z < minZ) {
                        continue;
                    }

                    pcl::PointXYZRGBNormal normalPoint;
                    normalPoint.x = point.x + point.normal_x;
                    normalPoint.y = point.y + point.normal_y;
                    normalPoint.z = point.z + point.normal_z;
                    // check if normal is horizontal
                    auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                    auto vertLen = point.normal_y;
                    if (horLen > vertLen) {
                        if (Util::signedPointPlaneDistance(normalPoint, wallPlane) < 0) {
                            (*cloud)[*nIdxIt].normal_x *= -1;
                            (*cloud)[*nIdxIt].normal_y *= -1;
                            (*cloud)[*nIdxIt].normal_z *= -1;

                            // also flip tangents
                            auto temp = tangent1Vec[*nIdxIt];
                            tangent1Vec[*nIdxIt] = tangent2Vec[*nIdxIt];
                            tangent2Vec[*nIdxIt] = temp;
                        }
                    }
                }
            }
        }
    }
}

void
DataStructure::adaComputeSplats(float alpha, float splatGrowEpsilon, std::vector<pcl::Indices>& pointNeighbourhoods,
                                std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses) {
    // 1 → 0°, 0.7 → 45°, 0 → 90°(pi/2). i guess: -0.7 → 135°, -1 → 180°(pi). (arccos kann nur zwischen 0° und 180° zeigen, richtung nicht beachtet)
    float angleThreshold = 0.86; // ~30°

    std::vector<bool> discardPoint(cloud->points.size());
    fill(discardPoint.begin(), discardPoint.end(), false);

    float currentSplatGrowEpsilon = splatGrowEpsilon;

    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {

//        if (pointIdx % 1000 != 0){
//            continue;
//        }

        if (discardPoint[pointIdx]) {
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            continue;
        }

        switch (pointClasses[pointIdx]) {
            case 0: // linearity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 2;//0.33;
                break;
            case 1: // planarity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 2;
                break;
            default: // sphericity is main
                currentSplatGrowEpsilon = splatGrowEpsilon * 0.25;
                break;
        }
// TODO vllt kann ich linearity punkte die eig ne fläche sein sollten einfach auch große nachbarschaft  geben und dann endet der splat wenn er auf ne fläche trifft??
        auto const& point = cloud->points[pointIdx];
        auto const& neighbourhood = pointNeighbourhoods[pointIdx];

        (*cloud)[pointIdx].curvature = 0; // this is radius

        auto normal = pcl::PointXYZ(point.normal_x, point.normal_y, point.normal_z);

        float epsilonSum = 0;
        int epsilonCount1 = 0;
        int epsilonCount2 = 0;
        int lastEpsilonNeighbourIdx1 = 0;
        int lastEpsilonNeighbourIdx2 = 0;

        bool growTangent1 = true;
        bool growTangent2 = true;

        bool concernsTangent1;

        // grow
        for (auto nIdx = 1; nIdx < neighbourhood.size(); nIdx++) {

            if (!growTangent1 && !growTangent2) {
                break;
            }

            // first check if neighbour point concerns tangent1 growth or tangent2 growth
            // project vector onto plane:
            auto neighbourVec = Util::vectorSubtract(cloud->points[neighbourhood[nIdx]], point);
            auto projectedDistance = Util::dotProduct(normal, neighbourVec);
            auto normalProjectedVec = pcl::PointXYZ(projectedDistance * normal.x, projectedDistance * normal.y, projectedDistance * normal.z);
            auto planeProjectedVec = Util::vectorSubtract(neighbourVec, normalProjectedVec);
            // angle
            auto vecLen = Util::vectorLength(planeProjectedVec);
            float cosAngle = Util::dotProduct(tangent1Vec[pointIdx], planeProjectedVec) / vecLen;
            if (abs(cosAngle) > 0.7) { // 45°
                // concerns tangent1 -> less then 45° between tangent1 and projected neighbour vector
                concernsTangent1 = true;
                if (!growTangent1) {
                    continue;
                }
            } else {
                concernsTangent1 = false;
                if (!growTangent2) {
                    continue;
                }
            }

            // TODO ich teste jetzt abbruchbedingungen auch bei discardeten punkten zu checken
            // stop growing when neighbour has different class
            if (pointClasses[pointIdx] != pointClasses[neighbourhood[nIdx]]) {
                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            // stop growing when angle between point normal and neighbour normal is too big
            // smoothness check
            pcl::PointXYZ neighbourNormal;
            neighbourNormal.x = (*cloud)[neighbourhood[nIdx]].normal_x;
            neighbourNormal.y = (*cloud)[neighbourhood[nIdx]].normal_y;
            neighbourNormal.z = (*cloud)[neighbourhood[nIdx]].normal_z;

            float angle = Util::dotProduct(normal, neighbourNormal);
            if (angle < angleThreshold) {
                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            auto eps = Util::signedPointPlaneDistance(point, cloud->points[neighbourhood[nIdx]], normal);
            if (abs(eps) > currentSplatGrowEpsilon) {
                // stop growing this neighbourhood
                // point nIdx does NOT belong to neighbourhood
//                if (nIdx == 1) {
//                    (*cloud)[pointIdx].r = 0;
//                    (*cloud)[pointIdx].g = 255;
//                    (*cloud)[pointIdx].b = 0;
//                }

                if (concernsTangent1) {
                    growTangent1 = false;
                } else {
                    growTangent2 = false;
                }
                continue;
            }

            // skip discarded points
            if (discardPoint[neighbourhood[nIdx]]) {
                continue;
            }

            epsilonSum += eps;
            if (concernsTangent1) {
                epsilonCount1++;
                lastEpsilonNeighbourIdx1 = nIdx;
            } else {
                epsilonCount2++;
                lastEpsilonNeighbourIdx2 = nIdx;
            }
        }

        // no valid neighbours in at least one direction // TODO schauen ob sinn macht getrennt zu betrachten
        // TODO PROBLEM: glaube schon dass das sinn macht, aber die ungewollten kanten sind so grade dass die hier invalid werden weil in eine richtung nichts passiert
        if (epsilonCount1 == 0 && epsilonCount2 == 0) {
            // no valid neighbours (all have been discarded or nearest neighbours eps dist is too big)
            if (colorInvalid) {
                if ((*cloud)[pointIdx].g != 255) {
                    (*cloud)[pointIdx].r = 255;
                    (*cloud)[pointIdx].g = 0;
                    (*cloud)[pointIdx].b = 255;
                }
            }
            tangent1Vec[pointIdx] = pcl::PointXYZ(0, 0, 0);
            continue;
        } else if (epsilonCount2 == 0) {
            // minor achse
            if (colorInvalid) {
                (*cloud)[pointIdx].r = 255;
                (*cloud)[pointIdx].g = 0;
                (*cloud)[pointIdx].b = 0;
            }
            epsilonCount2++;
            lastEpsilonNeighbourIdx2 = 1;
        } else if (epsilonCount1 == 0) {
            if (colorInvalid) {
                (*cloud)[pointIdx].r = 255;
                (*cloud)[pointIdx].g = 0;
                (*cloud)[pointIdx].b = 0;
            }
            epsilonCount1++;
            lastEpsilonNeighbourIdx1 = 1;
        }

        // compute avg of the epsilons
        float epsilonAvg = epsilonSum / (epsilonCount1 + epsilonCount2);

        // move splat point
        (*cloud)[pointIdx].x += epsilonAvg * normal.x;
        (*cloud)[pointIdx].y += epsilonAvg * normal.y;
        (*cloud)[pointIdx].z += epsilonAvg * normal.z;

        // compute splat radii
        // tangent 1
        const auto& lastNeighbourPoint1 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx1]];
        auto pointToNeighbourVec1 = Util::vectorSubtract(lastNeighbourPoint1, point);
        auto bla1 = Util::dotProduct(normal, pointToNeighbourVec1);
        auto rightSide1 = pcl::PointXYZ(bla1 * normal.x, bla1 * normal.y, bla1 * normal.z);
        float radius1 = Util::vectorLength(Util::vectorSubtract(pointToNeighbourVec1, rightSide1));
        // tangent 2
        const auto& lastNeighbourPoint2 = cloud->points[neighbourhood[lastEpsilonNeighbourIdx2]];
        auto pointToNeighbourVec2 = Util::vectorSubtract(lastNeighbourPoint2, point);
        auto bla2 = Util::dotProduct(normal, pointToNeighbourVec2);
        auto rightSide2 = pcl::PointXYZ(bla2 * normal.x, bla2 * normal.y, bla2 * normal.z);
        float radius2 = Util::vectorLength(Util::vectorSubtract(pointToNeighbourVec2, rightSide2));
        // length of axes has to be 1/radius
        tangent1Vec[pointIdx] = pcl::PointXYZ(tangent1Vec[pointIdx].x / radius1, tangent1Vec[pointIdx].y / radius1, tangent1Vec[pointIdx].z / radius1);
        tangent2Vec[pointIdx] = pcl::PointXYZ(tangent2Vec[pointIdx].x / radius2, tangent2Vec[pointIdx].y / radius2, tangent2Vec[pointIdx].z / radius2);
//        (*cloud)[pointIdx].curvature = radius;


        // discard points
        auto neighbourhoodDistances = pointNeighbourhoodsDistance[pointIdx];
//        int randR = rand() % (255 - 0 + 1) + 0;
//        int randG = rand() % (255 - 0 + 1) + 0;
//        int randB = rand() % (255 - 0 + 1) + 0;
        // TODO temp lösung
        float radius = max(radius1, radius2);
        for (auto nIdx = 0; nIdx < neighbourhood.size(); nIdx++) {

            if (discardPoint[neighbourhood[nIdx]])
                continue;

            auto dist = neighbourhoodDistances[nIdx];

            // TODO komplexe winkel berechnung zum discarden
            // https://math.stackexchange.com/questions/76457/check-if-a-point-is-within-an-ellipse
            // rotate tangents and neighbour point to be parallel to xz-plane or something
            // ellipse center is c
            // then i can ignore y component
            // use: if ( (n.x - c.x)² / r1²  +  (n.z - c.z)² / r2²  <= 1 ) -> isInside
            // umformen: if ( r2² * (n.x - c.x)²  +  r1² * (n.z - c.z)² <= r1² * r2² -> isInside

            if (dist < alpha * radius) {
                discardPoint[neighbourhood[nIdx]] = true;
                // color debug - discarded points
                if (colorDiscarded) {
                    if (nIdx != 0) {
                        (*cloud)[neighbourhood[nIdx]].r = 255;
                        (*cloud)[neighbourhood[nIdx]].g = 255;
                        cloud->points[neighbourhood[nIdx]].b = 0;
                    }
                }
            } else {
                // dist values are ascending
                break;
            }
        }

        if (colorSplats) {
            // color debug - random color for every point
            int randR = rand() % (255 - 0 + 1) + 0;
            int randG = rand() % (255 - 0 + 1) + 0;
            int randB = rand() % (255 - 0 + 1) + 0;
            (*cloud)[pointIdx].r = randR;
            (*cloud)[pointIdx].g = randG;
            (*cloud)[pointIdx].b = randB;
//            (*cloud)[pointIdx].r = 0;
//            (*cloud)[pointIdx].g = 255;
//            (*cloud)[pointIdx].b = 255;
        }
    }
}

// TODO die machen dann vor splat generation noch denoising
void DataStructure::adaResampling(float avgRadiusNeighbourhoods, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<int>& pointClasses) {

    // determine avg splats density
    float avgSplatsDensity = 0;
    int avgSplatCounter = 0;
    int pointSplatCount;
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        const auto& point = (*cloud)[pointIdx];
        pointSplatCount = 0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (tree->radiusSearch(point, avgRadiusNeighbourhoods, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // count points that are splats, but dont count non-surface group
            for (int searchIdx = 0; searchIdx < pointIdxRadiusSearch.size(); searchIdx++) {
                int& nPointIdx = pointIdxRadiusSearch[searchIdx];
                const auto& nPoint = (*cloud)[nPointIdx];
                // continue if not splat
                if (tangent1Vec[nPointIdx].x == 0 && tangent1Vec[nPointIdx].y == 0 && tangent1Vec[nPointIdx].z == 0) { // for elliptical splats
                    continue;
                }

                // continue if non-surface group
                if (pointClasses[nPointIdx] == 3) {
                    continue;
                }

                pointSplatCount++;
            }
            avgSplatsDensity += static_cast<float>(pointSplatCount) / static_cast<float>(pointIdxRadiusSearch.size());
            avgSplatCounter++;

        }
    }
    avgSplatsDensity /= static_cast<float>(avgSplatCounter);


    // resampling
    std::vector<pcl::PointXYZRGBNormal> newPoints;
    // 1 → 0°, 0.7 → 45°, 0 → 90°(pi/2). i guess: -0.7 → 135°, -1 → 180°(pi). (arccos kann nur zwischen 0° und 180° zeigen, richtung nicht beachtet)
    float angleThreshold = 0.6; // ~30° TODO copied from splat generation
    for (auto pointIdx = 0; pointIdx < cloud->points.size(); pointIdx++) {
        const auto& point = (*cloud)[pointIdx];
        // only for splats
        if (tangent1Vec[pointIdx].x == 0 && tangent1Vec[pointIdx].y == 0 && tangent1Vec[pointIdx].z == 0) { // for elliptical splats
            continue;
        }
//        auto bla = tangent1Vec[pointIdx];
//        auto blaNorm = Util::normalize(bla);
//        blaNorm.x = blaNorm.x / avgRadiusNeighbourhoods;
//        blaNorm.y = blaNorm.y / avgRadiusNeighbourhoods;
//        blaNorm.z = blaNorm.z / avgRadiusNeighbourhoods;
//        tangent1Vec[pointIdx] = blaNorm;
//        auto bla2 = tangent2Vec[pointIdx];
//        auto blaNorm2 = Util::normalize(bla2);
//        blaNorm2.x = blaNorm2.x / avgRadiusNeighbourhoods;
//        blaNorm2.y = blaNorm2.y / avgRadiusNeighbourhoods;
//        blaNorm2.z = blaNorm2.z / avgRadiusNeighbourhoods;
//        tangent2Vec[pointIdx] = blaNorm2;
        // TODO vllt oben speichern für splats?
        pointSplatCount = 0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (tree->radiusSearch(point, avgRadiusNeighbourhoods, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // count points that are splats, but dont count non-surface group
            for (int searchIdx = 0; searchIdx < pointIdxRadiusSearch.size(); searchIdx++) {
                int& nPointIdx = pointIdxRadiusSearch[searchIdx];
                const auto& nPoint = (*cloud)[nPointIdx];
                // continue if not splat
                if (tangent1Vec[nPointIdx].x == 0 && tangent1Vec[nPointIdx].y == 0 && tangent1Vec[nPointIdx].z == 0) { // for elliptical splats
                    continue;
                }

                // continue if non-surface group
                if (pointClasses[nPointIdx] == 3) {
                    continue;
                }

                pointSplatCount++;
            }
            float splatDensity = static_cast<float>(pointSplatCount) / static_cast<float>(pointIdxRadiusSearch.size());

            if (splatDensity < avgSplatsDensity) {
                // resample
                for (auto nPointIdxIt = pointIdxRadiusSearch.rbegin(); nPointIdxIt != pointIdxRadiusSearch.rend(); nPointIdxIt++) {
                    const auto& nPointIdx = *nPointIdxIt;
//                    if (tangent1Vec[nPointIdx].x == 0 && tangent1Vec[nPointIdx].y == 0 && tangent1Vec[nPointIdx].z == 0) { // for elliptical splats
//                        continue;
//                    }
                    // found the farthest splat

                    if (pointClasses[pointIdx] != pointClasses[nPointIdx]) {
                        continue;
                    }
                    // smoothness check
                    auto normal = pcl::PointXYZ((*cloud)[pointIdx].normal_x, (*cloud)[pointIdx].normal_y, (*cloud)[pointIdx].normal_z);
                    pcl::PointXYZ neighbourNormal;
                    neighbourNormal.x = (*cloud)[nPointIdx].normal_x;
                    neighbourNormal.y = (*cloud)[nPointIdx].normal_y;
                    neighbourNormal.z = (*cloud)[nPointIdx].normal_z;
                    float angle = Util::dotProduct(normal, neighbourNormal);
                    if (angle < angleThreshold) {
                        continue;
                    }
                    // found the farthest valid splat

                    const auto& farthestSplat = (*cloud)[nPointIdx];
                    // "interpolate a new point that lies at the center of the segment connecting the splats’ centers"
                    auto newPoint = pcl::PointXYZRGBNormal((point.x + farthestSplat.x) / 2, (point.y + farthestSplat.y) / 2, (point.z + farthestSplat.z) / 2);
                    // debug color
                    newPoint.b = 255;
                    newPoint.g = 0;
                    newPoint.r = 255;
                    // TODO braucht noch normale, tangente1 und 2, radii
                    newPoint.normal_x = point.normal_x;
                    newPoint.normal_y = point.normal_y;
                    newPoint.normal_z = point.normal_z;
                    pointClasses.push_back(pointClasses[pointIdx]);
                    tangent1Vec.push_back(pcl::PointXYZ(0, 0, 0));
                    tangent2Vec.push_back(pcl::PointXYZ(0, 0, 0));
                    newPoints.push_back(newPoint);
                    break;

                }
            }
        }

    }
    cloud->insert(cloud->end(), newPoints.begin(), newPoints.end());


}

pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr
DataStructure::kdTreePcaNormalEstimation(const uint32_t& startIdx, const uint32_t& endIdx) { // TODO use indices

    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start normal calculation" << std::endl;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    ne.setViewPoint(0.0f, 100000000.0f, 750000.0f);

    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    ne.setSearchMethod(tree);

    ne.setKSearch(16);

    // Compute the features
    ne.compute(*cloud);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal estimation in " << duration.count() << "s" << std::endl;


    // compute radii
    start = std::chrono::high_resolution_clock::now();
    for (auto it = cloud->points.begin(); it != cloud->points.end(); it++) {
        const auto& bla = *it;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        tree->radiusSearch(bla, 1.5, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        auto const count = static_cast<float>(pointRadiusSquaredDistance.size());

        auto diff = std::max((count - 7.0f), 0.0f);

        auto avg = std::reduce(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end() - diff) /
                   (count - diff);
        (*it).curvature = avg;

    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished radius calculation in " << duration.count() << "s" << std::endl;

    return tree;
}


// TODO später in util

//Vertex DataStructure::getUTMForOpenGL(Vertex *vertexOpenGL) {
//    // TODO offset is float, losing precision
//    return Vertex{vertexOpenGL->x + xOffset, vertexOpenGL->y + yOffset, vertexOpenGL->z + zOffset};
//}
//
//Vertex DataStructure::getWGSForOpenGL(Vertex *vertex) {
//    // TODO offset is float, losing precision
//
//    // wert in utm holen, dann:
//
//    // zone number: 60 zones, each 6 degrees of longitude (horizontal stripes), number is consistent in horizontal stripes
//    // zone letter: 20 zones, each 8 degrees of latitude (vertical stripes), letter is consistent in vertical stripes
//    // x wert zwischen 100.000 und 899.999 meter in zone
//    // y wert ist entfernugn vom äquator (zumindest auf nordhalbkugel)
//    return Vertex();
//}

uint32_t DataStructure::getVertexCount() {
    return (uint32_t) cloud->width;
}


void DataStructure::normalOrientation(const uint32_t& startIdx, const uint32_t& endIdx,
                                      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& treePtr) {
    // TODO use buildings to detect right normal orientation
    //  maybe also segmentation stuff?
    float thresholdDelta = 1.0f; // TODO find good value

    for (auto building: buildings) {
        // get point index of next part/ring if there are more than one, skip "walls" which connect different parts
        auto partIdx = building.parts.begin();
        uint32_t nextPartIndex = *partIdx;
        partIdx++;
        if (partIdx != building.parts.end()) {
            nextPartIndex = *partIdx;
        }
        for (auto pointIdx = 0; pointIdx < building.points.size() - 1; pointIdx++) {
//            std::cout << "yrah " << pointIdx << std::endl;

//            const auto& wallPoint1 = building.points[pointIdx];
//            const auto& wallPoint2 = building.points[pointIdx + 1];

            // if reached end of part/ring -> skip this "wall"
            if (pointIdx + 1 == nextPartIndex) {
                partIdx++;
                if (partIdx != building.parts.end()) {
                    nextPartIndex = *partIdx;
                }
                continue;
            }

            float wallHeight = 80; // mathe tower ist 60m hoch TODO aus daten nehmen
            float ground = -38; // minY // TODO boden ist wegen opengl offset grad bei -38
            pcl::PointXYZRGBNormal wallPoint1, wallPoint2;
            wallPoint1.x = building.points[pointIdx].x;
            wallPoint1.y = ground + wallHeight;
            wallPoint1.z = building.points[pointIdx].z;
            wallPoint2.x = building.points[pointIdx + 1].x;
            wallPoint2.y = ground + wallHeight;
            wallPoint2.z = building.points[pointIdx + 1].z;

            // detect (and color) alle points on this wall
            pcl::PointXYZRGBNormal mid;
            mid.x = (wallPoint1.x + wallPoint2.x) / 2;
            mid.y = (ground + wallHeight) / 2;
            mid.z = (wallPoint1.z + wallPoint2.z) / 2;


            Plane wallPlane = {wallPoint1, wallPoint2, mid};

            // p2 = wp2.x wallHeight wp2.z
            // p1 = mid
            float r = sqrt(pow(wallPoint2.x - mid.x, 2) + pow(wallHeight - mid.y, 2) + pow(wallPoint2.z - mid.z, 2));

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            treePtr->radiusSearch(mid, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            if (pointIdxRadiusSearch.size() != 0) {

                int randR = rand() % (255 - 0 + 1) + 0;
                int randG = rand() % (255 - 0 + 1) + 0;
                int randB = rand() % (255 - 0 + 1) + 0;


//                std::cout << "yrah " << pointIdxRadiusSearch.size() << std::endl;

                for (auto nIdxIt = pointIdxRadiusSearch.begin(); nIdxIt != pointIdxRadiusSearch.end(); nIdxIt++) {
                    // TODO implement direct calculation test if point lies inside wall rectangle
//                    if(building.points.size() == 5){
//                        randR = 255;
//                        randG = 0;
//                        randB = 0;
//                    }
                    const auto& point = (*cloud)[*nIdxIt];

                    if (Util::pointPlaneDistance(cloud->points[*nIdxIt], wallPlane) > thresholdDelta) {
                        continue;
                    }
                    auto minX = min(wallPoint1.x, wallPoint2.x);
                    auto maxX = max(wallPoint1.x, wallPoint2.x);
                    auto minZ = min(wallPoint1.z, wallPoint2.z);
                    auto maxZ = max(wallPoint1.z, wallPoint2.z);

                    const auto& x = point.x;
                    const auto& z = point.z;
                    if (x > maxX || x < minX || z > maxZ || z < minZ) {
                        continue;
                    }
                    const auto& y = point.y;

                    cloud->points[*nIdxIt].b = randB;
                    (*cloud)[*nIdxIt].g = randG;
                    (*cloud)[*nIdxIt].r = randR;

                    pcl::PointXYZRGBNormal normalPoint;
                    normalPoint.x = point.x + point.normal_x;
                    normalPoint.y = point.y + point.normal_y;
                    normalPoint.z = point.z + point.normal_z;
                    // TODO check if normal is waagerecht
                    auto horLen = sqrt(pow(point.normal_x, 2) + pow(point.normal_z, 2));
                    auto vertLen = point.normal_y;
                    if (horLen > vertLen) {
                        if (Util::signedPointPlaneDistance(normalPoint, wallPlane) < 0) { // TODO check sign richtig
                            (*cloud)[*nIdxIt].normal_x *= -1;
                            (*cloud)[*nIdxIt].normal_y *= -1;
                            (*cloud)[*nIdxIt].normal_z *= -1;
                        }
                    }

                }
                // TODO filter points, keep only those that belong to the wall
                //  check distance to wall plane first
                //  then check if point is on wall part of plane
            }

        }
    }
}

/**
 * returns index of first element greater then border
 * returns lastIndex+1 if all elements are smaller
 * @param border
 * @param vector1
 * @return
 */
int DataStructure::findIndex(float border, std::vector<float> vector1) {
    auto begin = vector1.begin();
    auto end = vector1.end();
    auto mid = begin + (end - begin) / 2;
    int index = (end - begin);
    while (begin <= end) {

        // check mid
        mid = begin + (end - begin) / 2;
        if (mid == vector1.end()) {
            // all elements are smaller then border
            return vector1.size();
        }
        if (*mid < border) {
            //search right side
            begin = mid + 1;
        } else {
            // search left side
            // mid element is current border candidate
            index = mid - vector1.begin();
            end = mid - 1;
        }
    }
    return index;
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

void DataStructure::findXYZMedian(vector<int>& pointIndices, float& xMedian, float& yMedian, float& zMedian) {
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

void DataStructure::findYMinMax(vector<int>& pointIndices, float& yMin, float& yMax) {
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

void DataStructure::findStartEnd(std::vector<pcl::PointXYZ>& points, pcl::PointXYZ& start, pcl::PointXYZ& end) {
    // x TODO funktioniert jetzt nicht gut wenn parallel zur x achse ist! wobei da sind die wände ja glaube ich schon geplättet? also sollte gehen?
    std::nth_element(points.begin(), points.begin(), points.end(), xComparator2);
    start = points[0];
    std::nth_element(points.begin(), points.end() - 1, points.end(), xComparator2);
    end = points[points.size() - 1];
//    //z
//    std::nth_element(points.begin(), points.begin(), points.end(), zComparator);
//    zMin = points[0].z;
//    std::nth_element(points.begin(), points.end() - 1, points.end(), zComparator);
//    zMax = points[points.size()-1].z;
}






