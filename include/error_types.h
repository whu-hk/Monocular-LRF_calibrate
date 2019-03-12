/**
 *
 */

#ifndef BOARD_POSE_ERROR_TYPES_H
#define BOARD_POSE_ERROR_TYPES_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include "config.h"

using namespace std;


class ErrorTypes
{
public:
    ErrorTypes(Eigen::Vector2d observation):observation_(observation){}

    template<typename T>
    bool operator()(const T* const quaternion,
                    const T* const translation,
                    const T* const point,
                    T* residuals)const{

        Eigen::Map<const Eigen::Quaternion<T>> q_cl(quaternion);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t_cl(translation);

        Eigen::Map<const Eigen::Matrix<T,3,1>> p_l(point);
        Eigen::Matrix<T,3,1> p_c = q_cl*p_l + t_cl;

        /// Transform the pc to image_points.
        T fx = Config::get<T>("fx");
        T fy = Config::get<T>("fy");
        T cx = Config::get<T>("cx");
        T cy = Config::get<T>("cy");
        //相机坐标系下的点乘以相机内参矩阵
        residuals[0] = T(fx * p_c(0,0) / p_c(2, 0) + cx - T(observation_(0))); 
        //x
        residuals[1] = T(fy * p_c(1,0) / p_c(2, 0) + cy - T(observation_(1)));
        //y
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d observation){
        return (new ceres::AutoDiffCostFunction<ErrorTypes,2,4,3,3>( 
         //ErrorTypes为CostFunctor模板类,2为残差个数，4为block0中的参数个数，3、3分别为block1和block2中的参数个数
                new ErrorTypes(observation)));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector2d observation_;
};


#endif //BOARD_POSE_ERROR_TYPES_H
