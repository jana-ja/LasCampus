//
// Created by Jana Jansen on 01.10.23.
//

#ifndef LASCAMPUS_UTIL_H
#define LASCAMPUS_UTIL_H

#include <pcl/features/impl/normal_3d.hpp> // make sure to include the .hpp file

namespace Util {

    const std::string PATH_SEPARATOR =
#ifdef _WIN32
            "\\";
#else
    "/";
#endif

    struct Wall {
        pcl::PointXYZRGBNormal mid;
        float minX, maxX;
        float minZ, maxZ;
        int buildingIdx;
        float length;
    };
    using Plane = pcl::PointXYZRGBNormal[3]; // three points define a plane

    // keep those functions inlined because they are small and used frequently
    inline pcl::PointXYZ vectorSubtract(const pcl::PointXYZRGBNormal& a, const pcl::PointXYZRGBNormal& b) {
        pcl::PointXYZ result;
        result.x = (a.x - b.x);
        result.y = (a.y - b.y);
        result.z = (a.z - b.z);
        return result;
    }
    inline pcl::PointXYZ vectorSubtract(const pcl::PointXYZRGBNormal& a, const pcl::PointXYZ& b) {
        pcl::PointXYZ result;
        result.x = (a.x - b.x);
        result.y = (a.y - b.y);
        result.z = (a.z - b.z);
        return result;
    }

    inline pcl::PointXYZ vectorSubtract(const pcl::PointXYZ& a, const pcl::PointXYZRGBNormal& b) {
        pcl::PointXYZ result;
        result.x = (a.x - b.x);
        result.y = (a.y - b.y);
        result.z = (a.z - b.z);
        return result;
    }

    inline pcl::PointXYZ vectorSubtract(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
        pcl::PointXYZ result;
        result.x = (a.x - b.x);
        result.y = (a.y - b.y);
        result.z = (a.z - b.z);
        return result;
    }

    inline float dotProduct(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline pcl::PointXYZ crossProduct(pcl::PointXYZ a, pcl::PointXYZ b) {
        pcl::PointXYZ result;
        result.x = (a.y * b.z) - (a.z * b.y);
        result.y = (a.z * b.x) - (a.x * b.z);
        result.z = (a.x * b.y) - (a.y * b.x);
        return result;
    }

    inline pcl::PointXYZ normalize(pcl::PointXYZ point) {
        float magnitude = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        pcl::PointXYZ result;
        result.x = point.x / magnitude;
        result.y = point.y / magnitude;
        result.z = point.z / magnitude;
        return result;
    }


    inline float
    isPointRightOfWall(pcl::PointXYZRGBNormal point, pcl::PointXYZRGBNormal wallPoint1, pcl::PointXYZRGBNormal wallPoint2) { // TODO inside/outside check
        float d = (wallPoint2.x - wallPoint1.x) * (point.y - wallPoint1.y) - (point.x - wallPoint1.x) * (wallPoint2.y - wallPoint1.y);
        return d;
    }

    inline float vectorLength(const pcl::PointXYZRGBNormal vector) {
        return sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
    }

    inline float vectorLength(const pcl::PointXYZ vector) {
        return sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
    }

    inline float signedPointPlaneDistance(const pcl::PointXYZRGBNormal& point, const pcl::PointXYZRGBNormal& planePoint) {
        pcl::PointXYZ normal = pcl::PointXYZ(planePoint.normal_x, planePoint.normal_y, planePoint.normal_z);
        return dotProduct(normal, (vectorSubtract(point, planePoint)));
    }

    inline float signedPointPlaneDistance(const pcl::PointXYZ& point, const pcl::PointXYZRGBNormal& planePoint) {
        pcl::PointXYZ normal = pcl::PointXYZ(planePoint.normal_x, planePoint.normal_y, planePoint.normal_z);
        return dotProduct(normal, (vectorSubtract(point, planePoint)));
    }

    inline float signedPointPlaneDistance(const pcl::PointXYZRGBNormal& point, const pcl::PointXYZRGBNormal& neighbourPoint, const pcl::PointXYZ& normal) {
        return dotProduct(normal, (vectorSubtract(neighbourPoint, point)));
    }

    inline float signedPointPlaneDistance(const pcl::PointXYZRGBNormal& point, const Plane& plane) {
        // calc normal for plane points
        auto vec1 = vectorSubtract(plane[0], plane[1]);
        auto vec2 = vectorSubtract(plane[0], plane[2]);

        auto planeNormal = normalize(crossProduct(vec1, vec2));

        float dist = dotProduct(planeNormal, (vectorSubtract(point, plane[0])));

        return dist;
    }

    inline float pointPlaneDistance(const pcl::PointXYZRGBNormal& point, const pcl::PointXYZRGBNormal& planePoint) {

        pcl::PointXYZ normal = pcl::PointXYZ(planePoint.normal_x, planePoint.normal_y, planePoint.normal_z);
        return abs(dotProduct(normal, (vectorSubtract(planePoint, point))));
    }
    inline float pointPlaneDistance(const pcl::PointXYZ point, const pcl::PointXYZRGBNormal& planePoint) {

        pcl::PointXYZ normal = pcl::PointXYZ(planePoint.normal_x, planePoint.normal_y, planePoint.normal_z);
        return abs(dotProduct(normal, (vectorSubtract(planePoint, point))));
    }

    inline float pointPlaneDistance(const pcl::PointXYZRGBNormal& point, const pcl::PointXYZRGBNormal& neighbourPoint, const pcl::PointXYZ& normal) {
        return abs(dotProduct(normal, (vectorSubtract(neighbourPoint, point))));
    }

    inline float pointPlaneDistance(const pcl::PointXYZRGBNormal& point, const Plane& plane) {
        // calc normal for plane points
        auto vec1 = vectorSubtract(plane[0], plane[1]);
        auto vec2 = vectorSubtract(plane[0], plane[2]);

        auto planeNormal = normalize(crossProduct(vec1, vec2));

        float dist = dotProduct(planeNormal, (vectorSubtract(point, plane[0])));

        return abs(dist);
    }

    inline float distance(const pcl::PointXYZRGBNormal& point1, const pcl::PointXYZRGBNormal& point2){
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
    }

    inline float horizontalDistance(const pcl::PointXYZ& point1, const pcl::PointXYZRGBNormal& point2) {
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.z - point2.z, 2));
    }
    inline float horizontalDistance(const pcl::PointXYZRGBNormal& point1, const pcl::PointXYZRGBNormal& point2) {
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.z - point2.z, 2));
    }

    inline int intersectPlane(const Wall& wall, const pcl::PointXYZ& rayOrigin, const pcl::PointXYZ& rayDir,
                              float& wallThreshold) // t kommt in func ray länge bis intersection punkt rein
    {

        const auto planePoint = pcl::PointXYZ(wall.mid.x, wall.mid.y, wall.mid.z);//pcl::PointXYZ(wall.mid.x + wallThreshold * wall.mid.normal_x, wall.mid.y + wallThreshold * wall.mid.normal_y, wall.mid.z + wallThreshold * wall.mid.normal_z);
        auto planeNormal = pcl::PointXYZ(wall.mid.normal_x, wall.mid.normal_y, wall.mid.normal_z);
        // assuming vectors are all normalized
        float denom = Util::dotProduct(planeNormal, rayDir);
        if (abs(denom) > 1e-6) {
            pcl::PointXYZ distVec = Util::vectorSubtract(planePoint, rayOrigin);
            float t = Util::dotProduct(distVec, planeNormal) / denom;
            if (t < 0)
                return 0;

            auto intersectionPoint = pcl::PointXYZ(rayOrigin.x + t * rayDir.x, rayOrigin.y + t * rayDir.y, rayOrigin.z + t * rayDir.z);
            auto dist = horizontalDistance(intersectionPoint, wall.mid);
                if (dist > wall.length / 2 ) {
                    auto bla = "das sollte nicht passieren";

                }
            if(intersectionPoint.x > wall.maxX  || intersectionPoint.z > wall.maxZ || intersectionPoint.x < wall.minX  || intersectionPoint.z < wall.minZ ) {
                if (dist <= wall.length / 2 ){
                    auto dist2 = pointPlaneDistance(intersectionPoint, wall.mid);
                    auto bla = "das sollte nicht passieren";
                }
                return 0;
            }

            auto signedDist = signedPointPlaneDistance(rayOrigin, wall.mid);
            if (signedDist < 0) {
                // inside to outside
                return 1;
            } else {
                // outside to inside
                return -1;
            }
        }
        return 0;
    }
}

#endif //LASCAMPUS_UTIL_H
