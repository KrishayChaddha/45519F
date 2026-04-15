// #ifndef distancereset_HPP
// #define distancereset_HPP

// #include "pros/distance.hpp"
 
// //each wall angle in degrees
// namespace WallAngles {
//     constexpr double FrontWallAngle = 0.0;    // horizontal wall at bottom
//     constexpr double RightWallAngle = 90.0;    // vertical wall on right
//     constexpr double BackWallAngle = 180.0;     // horizontal wall at top
//     constexpr double LeftWallAngle = 270.0;    // vertical wall on left
// }             

// //each wall from middle to where they are located in inches
// namespace FeildWalls {
//     constexpr double FrontWall = 72.0; // y+ inches
//     constexpr double BackWall = -72.0; // y- inches
//     constexpr double RightWall = 72.0; // x+ inches
//     constexpr double LeftWall = -72.0; // x- inches
// }

// //because sensors face 3 dif directions
// namespace HeadingOffsets {
//     constexpr double HeadingOffsetRight = 90.0;    // right ds
//     constexpr double HeadingOffsetFront = 0.0;      // front ds
//     constexpr double HeadingOffsetLeft = 270.0;      // left ds
// }

// /* 
// offsets for each sensor in inches

// front
// x =
// y = 

// left 
// x = 
// y =

// right
// x =
// y =

// */

// class DistanceSensor {

// private:
//     pros::Distance& distanceSensor;
//     double x_offset;
//     double y_offset;
//     double SensorHeading_offset;

// public:
//     DistanceSensor(pros::Distance& ds,double x_off,double y_off,double heading_off):
//     distanceSensor(ds),x_offset(x_off),y_offset(y_off),SensorHeading_offset(heading_off){} //constructor

    
//     double getDistance(double wall_angle) const;
// };

// void ResetPose(DistanceSensor& sensor1, double wall_angle1, double wall_pos1, bool is_x1,
//     DistanceSensor& sensor2, double wall_angle2, double wall_pos2, bool is_x2); //using 2 dsitance sesnors to reset robot pose

// #endif // DSR_HPP

