#include "my_kalman.h"
#include <cmath>  // 使用 math.h 中的函数
#include <AP_HAL/AP_HAL.h>
#include <chrono>
#include <AP_Math/matrix3.h>
#include <AP_Scheduler/AP_Scheduler.h>


KalmanFilter_my::KalmanFilter_my()
{
    _singleton = this;
}


bool KalmanFilter_my::invertMatrix3x3(const float input[3][3], float output[3][3]) {
    // 计算行列式 det(A)
    float det = input[0][0] * (input[1][1] * input[2][2] - input[1][2] * input[2][1]) -
                input[0][1] * (input[1][0] * input[2][2] - input[1][2] * input[2][0]) +
                input[0][2] * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);

    // 定义误差范围
    const float epsilon = 1e-8f;

    // 检查行列式是否接近零
    if (fabs(det) < epsilon) {
        return false;
    }
    // 计算逆矩阵的元素
    float invDet = 1.0f / det;
    output[0][0] = invDet * (input[1][1] * input[2][2] - input[1][2] * input[2][1]);
    output[0][1] = invDet * (input[0][2] * input[2][1] - input[0][1] * input[2][2]);
    output[0][2] = invDet * (input[0][1] * input[1][2] - input[0][2] * input[1][1]);

    output[1][0] = invDet * (input[1][2] * input[2][0] - input[1][0] * input[2][2]);
    output[1][1] = invDet * (input[0][0] * input[2][2] - input[0][2] * input[2][0]);
    output[1][2] = invDet * (input[0][2] * input[1][0] - input[0][0] * input[1][2]);

    output[2][0] = invDet * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);
    output[2][1] = invDet * (input[0][1] * input[2][0] - input[0][0] * input[2][1]);
    output[2][2] = invDet * (input[0][0] * input[1][1] - input[0][1] * input[1][0]);

    return true;
}

// void KalmanFilter_my::identity_matrix(float mat[3][3]) {
//     for (int i = 0; i < 3; i++) {
//         for (int j = 0; j < 3; j++) {
//             if (i == j) {
//                 mat[i][j] = 1.0;  // 对角线元素为 1
//             } else {
//                 mat[i][j] = 0.0;  // 非对角线元素为 0
//             }
//         }
//     }
// }

// // 矩阵与向量的乘法
// float KalmanFilter_my::matrix3xVector3(float matrix[3][3], float vector[3]) {
//     // 定义一个 3x1 的结果向量
//     float result[3] = {0.0, 0.0, 0.0};
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) { 
//             result[i] += matrix[i][j] * vector[j];
//         }
//     }
//     return result;
// }

// // 矩阵与向量的乘法
// float KalmanFilter_my::matrix3xmatrix3(float matrix1[3][3], float matrix2[3][3]) {
//     // 定义结果矩阵 C (存储 A * B)
//     float C[3][3] = {0.0};
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             for (int k = 0; k < 3; ++k) {
//                 C[i][j] += A[i][k] * B[k][j];
//             }
//         }
//     }
//     return C;
// }

// float KalmanFilter_my::matrix3x3transpose(float matrix[3][3]) {
//     // 定义结果矩阵 T (存储矩阵的转置)
//     float T[3][3] = {0.0};

//     // 执行转置操作
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             T[i][j] = matrix[j][i];  // 交换行和列
//         }
//     }

//     // 你可以选择返回 T 或直接操作矩阵
//     return T;
// }

// float KalmanFilter_my::matrix3x3add(float matrix1[3][3], float matrix2[3][3]) {
//     // 定义结果矩阵 C (存储 A + B)
//     float C[3][3] = {0.0};

//     // 执行加法操作
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             C[i][j] = matrix1[i][j] + matrix2[i][j];  // 对应元素相加
//         }
//     }

//     // 返回结果矩阵
//     return C;
// }

void KalmanFilter_my::init(){

            // 手动为每个元素赋值
        for (int i = 0; i < 3; ++i) {
            x_hat_my[i] = 0.0f;
        }

         const float dt = AP::scheduler().get_loop_period_s();

        // 手动初始化二维数组
        A_my[0][0] = 1; A_my[0][1] = dt; A_my[0][2] = 0.5 * dt * dt;
        A_my[1][0] = 0; A_my[1][1] = 1; A_my[1][2] = dt;
        A_my[2][0] = 0; A_my[2][1] = 0; A_my[2][2] = 1;

        H_my[0][0] = 1; H_my[0][1] = 0; H_my[0][2] = 0;
        H_my[1][0] = 0; H_my[1][1] = 1; H_my[1][2] = 0;
        H_my[2][0] = 0; H_my[2][1] = 0; H_my[2][2] = 1;

        Q_my[0][0] = 0.1f; Q_my[0][1] = 0.0f; Q_my[0][2] = 0.0f;
        Q_my[1][0] = 0.0f; Q_my[1][1] = 0.1f; Q_my[1][2] = 0.0f;
        Q_my[2][0] = 0.0f; Q_my[2][1] = 0.0f; Q_my[2][2] = 0.1f;
        
        R_my[0][0] = 0.1f; R_my[0][1] = 0.0f; R_my[0][2] = 0.0f;
        R_my[1][0] = 0.0f; R_my[1][1] = 0.1f; R_my[1][2] = 0.0f;
        R_my[2][0] = 0.0f; R_my[2][1] = 0.0f; R_my[2][2] = 0.1f;
        
        P_my[0][0] = 1.0f; P_my[0][1] = 0.0f; P_my[0][2] = 0.0f;
        P_my[1][0] = 0.0f; P_my[1][1] = 1.0f; P_my[1][2] = 0.0f;
        P_my[2][0] = 0.0f; P_my[2][1] = 0.0f; P_my[2][2] = 1.0f;

        I_my[0][0] = 1.0f; I_my[0][1] = 0.0f; I_my[0][2] = 0.0f;
        I_my[1][0] = 0.0f; I_my[1][1] = 1.0f; I_my[1][2] = 0.0f;
        I_my[2][0] = 0.0f; I_my[2][1] = 0.0f; I_my[2][2] = 1.0f;

        // kf_my.identity_matrix(Q_my) * 0.1;
        // kf_my.identity_matrix(R_my) * 0.1;
        // kf_my.identity_matrix(P_my);
}

void KalmanFilter_my::predict() {
    // x_hat_my = A_my * x_hat_my;
    // P_my = A_my * P_my * A_my.transposed() + Q_my;

    //公式1
    float C1[3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { 
            C1[i] += A_my[i][j] * x_hat_my[j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        x_hat_my[i] = C1[i];
    }
    // x_hat_my = C1;


    //公式2
    float C2[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                C2[i][j] += A_my[i][k] * P_my[k][j];
            }
        }
    }
    float C3[3][3] = {0.0};
    // 执行转置操作
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C3[i][j] = A_my[j][i];  // 交换行和列
        }
    }
    float C4[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                C4[i][j] += C2[i][k] * C3[k][j];
            }
        }
    }
    float C5[3][3] = {0.0};    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C5[i][j] = C4[i][j] + Q_my[i][j];  // 对应元素相加
        }
    }
    // P_my = C5;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_my[i][j] = C5[i][j];
        }
    }

}

void KalmanFilter_my::update(float y_my[3]) {
    // Matrix3d K_my = P_my * H_my.transpose() * (H_my * P_my * H_my.transpose() + R_my).inverse();
    // x_hat_my = x_hat_my + K_my * (y_my - H_my * x_hat_my);
    // P_my = (I_my - K_my * H_my) * P_my;

    //公式1
    float C1[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                C1[i][j] += H_my[i][k] * P_my[k][j];
            }
        }
    }
    float C2[3][3] = {0.0};
    // 执行转置操作
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C2[i][j] = H_my[j][i];  // 交换行和列
        }
    }
    float C3[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                C3[i][j] += C1[i][k] * C2[k][j];
            }
        }
    }
    float C4[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C4[i][j] = C3[i][j] + R_my[i][j];  // 对应元素相加
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            HPHR[i][j] = C4[i][j];
        }
    }  
    // HPHR = C4;
    if(!invertMatrix3x3(HPHR, _HPHR)){
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                _HPHR[i][j] = last_HPHR[i][j];
            }
        }          
    }
    // last_HPHR = _HPHR;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            last_HPHR[i][j] = _HPHR[i][j];
        }
    }  

    float temp[3][3] = {0};
    // 先计算 A * B
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                temp[i][j] += P_my[i][k] * C2[k][j];
            }
        }
    }
    // 再将结果乘以 C
    float C5[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                C5[i][j] += temp[i][k] * last_HPHR[k][j];
            }
        }
    }
    // K_my = C5;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_my[i][j] = C5[i][j];
        }
    } 


    //公式2
    float C6[3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { 
            C6[i] += H_my[i][j] * x_hat_my[j];
        }
    }
    float C7[3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        C7[i] = y_my[i] - C6[i];
    }
    float C8[3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { 
            C8[i] += K_my[i][j] * C7[j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        x_hat_my[i] = x_hat_my[i] + C8[i];
    }


    //公式3
    float C9[3][3] = {0};
    // 先计算 A * B
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                C9[i][j] += K_my[i][k] * H_my[k][j];
            }
        }
    }

    float C10[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C10[i][j] = I_my[i][j] - C9[i][j];  // 对应元素相加
        }
    }

    float C11[3][3] = {0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                C11[i][j] += C10[i][k] * P_my[k][j];
            }
        }
    }
    // P_my = C11;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_my[i][j] = C11[i][j];
        }
    }
}


// singleton instance
KalmanFilter_my *KalmanFilter_my::_singleton;

namespace AP {

KalmanFilter_my &KalmanFilter_My()
{
    return *KalmanFilter_my::get_singleton();
}

}
