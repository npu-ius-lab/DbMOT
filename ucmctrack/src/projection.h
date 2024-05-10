#ifndef PROJECTION_H
#define PROJECTION_H

#include <iostream>

#include "dataType.h"

class Projection
{
public:
    Projection(Eigen::Vector3d translation, Eigen::Quaterniond rotation);

    State3D world2pixel(const State3D state3d);

    PROJ_RESULT pixel2world(const State2D state2d);
  
private:
    Eigen::Matrix<double, 3, 4> K_in;
    Eigen::Matrix4d K_ex_1; 
    Eigen::Matrix4d K_ex_2; 
    Eigen::Matrix4d K_ex;
    // Eigen::Matrix4d K_ex_inv;
    Eigen::Matrix3d A;
    // Eigen::Vector3d translation;
    // Eigen::Quaterniond rotation;
};



#endif  // PROJECTION_H