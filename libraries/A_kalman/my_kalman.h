#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <Filter/SlewLimiter.h>
#include <Filter/NotchFilter.h>
#include <Filter/AP_Filter.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/matrix3.h>
#include <AP_Math/vectorN.h>


class KalmanFilter_my {
public:
    KalmanFilter_my();

    bool invertMatrix3x3(const float input[3][3], float output[3][3]); 

    void init();

    void predict();

    void update(float y_my[3]);

    // float state() const { return x_hat_my; }

    // 定义一个3x3的单位矩阵
    // void identity_matrix(float mat[3][3]);

    static KalmanFilter_my *get_singleton() {
         return _singleton; 
    }


private:
    // Matrix3d A_my;  // 状态转移矩阵
    // Matrix3d H_my;  // 测量矩阵
    // Matrix3d Q_my;  // 过程噪声协方差
    // Matrix3d R_my;  // 测量噪声协方差
    // Matrix3d P_my;  // 误差协方差
    // Matrix3d P0_my;
    // Matrix3d I_my;
    // Matrix3d HPHR;
    float A_my[3][3];  // 状态转移矩阵
    float H_my[3][3];  // 测量矩阵
    float Q_my[3][3];  // 过程噪声协方差
    float R_my[3][3];  // 测量噪声协方差
    float P_my[3][3];  // 误差协方差
    float K_my[3][3];
    float P0_my[3][3];
    float I_my[3][3];
    float HPHR[3][3];
    float _HPHR[3][3];
    float last_HPHR[3][3]; 

    float dt_my = 0.005;
    float t0_my;
    float t_my;
    bool initialized_my;
    float x_hat_my[3];  // 确保 x_hat 是 3x1 向量

    float matrix3xVector3(float matrix[3][3], float vector[3]);
    float matrix3xmatrix3(float matrix1[3][3], float matrix2[3][3]);    
    float matrix3x3transpose(float matrix[3][3]);
    float matrix3x3add(float matrix1[3][3], float matrix2[3][3]);

    static KalmanFilter_my *_singleton;
};

namespace AP {
    KalmanFilter_my &KalmanFilter_My();
};