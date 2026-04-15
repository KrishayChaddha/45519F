// #include "distancereset.hpp"
// #include "robotConfigs.h"
// #include <cmath>



// double DistanceSensor::getDistance(double wall_angle) const{
 
        
//     double distance_reading = distanceSensor.get_distance() / 25.4; 

//     double theta = (chassis.getPose().theta + SensorHeading_offset) - wall_angle; 
//     double thetaRad = theta * M_PI / 180;

//     double sensor_component = cos(thetaRad) * distance_reading;
//     double lateral_component = cos(thetaRad) * x_offset;
//     double longitudinal_component = sin(thetaRad) * y_offset * -1;

//     double distance = sensor_component + lateral_component + longitudinal_component;
//     return distance;
    

// }

// void ResetPose(
//     DistanceSensor& sensor1, double wall_angle1, double wall_pos1, bool is_x1,
//     DistanceSensor& sensor2, double wall_angle2, double wall_pos2, bool is_x2)
// {
//     double dist1 = sensor1.getDistance(wall_angle1);
//     double dist2 = sensor2.getDistance(wall_angle2);    

//     double actual_y, actual_x;

//     if (is_x1) {
//         if(wall_angle1 == WallAngles::RightWallAngle){
//             actual_x = FeildWalls::RightWall - dist1;

//         } else if(wall_angle1 == WallAngles::LeftWallAngle){
//             actual_x = FeildWalls::LeftWall + dist1;
//         }
//     } else {
//         if (wall_angle1 == WallAngles::FrontWallAngle) {
//             actual_y = FeildWalls::FrontWall - dist1;
            
//         } else if (wall_angle1 == WallAngles::BackWallAngle) {
//             actual_y = FeildWalls::BackWall + dist1;
//         }
//     }

//     if (is_x2) {
//         if(wall_angle2 == WallAngles::RightWallAngle){
//             actual_x = FeildWalls::RightWall - dist2;

//         } else if(wall_angle2 == WallAngles::LeftWallAngle){
//             actual_x = FeildWalls::LeftWall + dist2;
//         }

//     } else {
//         if (wall_angle2 == WallAngles::FrontWallAngle) {
//             actual_y = FeildWalls::FrontWall - dist2;

//         } else if (wall_angle2 == WallAngles::BackWallAngle) {
//             actual_y = FeildWalls::BackWall + dist2;
//         }
//     }

//     chassis.setPose(actual_x, actual_y, chassis.getPose().theta); // set the pose to the calculated position and current heading
// }


