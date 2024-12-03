#pragma once

/// @file	my.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

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
#define myROWS 21  // 迎角的数量
#define myCOLS 31  // 侧滑角的数量




/// @class	my
/// @brief	
class my {
public:
    my();
    CLASS_NO_COPY(my);


    float myalpha[myROWS] = {-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    float mybeta[myCOLS] = {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 
                        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    int find_index(float value, float array[], int size);
    float bilinear_interpolation(float alpha_value, float beta_value, 
                              float data[myROWS][myCOLS], float alpha[], float beta[]);
    void run();

    float Cmmde;
    float Cnmdr;
    float CnmdLa;
    float CnmdRa;
    float Clmdr;
    float ClmdLa;
    float ClmdRa;
    float Clmda;
    float Cnmda;

    float mass = 0.858f;
    float S = 0.2466f;
    float b = 1.40f;
    float c = 0.183546904568802f;
    float Ix = 0.037f;
    float Iy = 0.020f;
    float Iz = 0.055f;
    float Ixz = -0.002f;
    float rho_g = 1.225;
    float QBAR_withoutVV_Sc = 0.5*rho_g*S*c;
    float QBAR_withoutVV_Sb = 0.5*rho_g*S*b;   
    float QBAR_Sb;
    float QBAR_Sc;

    float c1 = ((Iy-Iz)*Iz - Ixz*Ixz)/((Ix*Iz) - Ixz*Ixz);
    float c2 = ((Ix+Iz-Iy)*Ixz)/((Ix*Iz) - Ixz*Ixz);
    float c3 = Iz/((Ix*Iz) - Ixz*Ixz);
    float c4 = Ixz/((Ix*Iz) - Ixz*Ixz);
    float c5 = (Iz-Ix)/Iy;
    float c6 = Ixz/Iy;
    float c7 = 1/Iy;
    float c8 = ((Ix-Iy)*Ix + Ixz*Ixz)/((Iz*Ix) - Ixz*Ixz);
    float c9 = Ix/((Iz*Ix) - Ixz*Ixz);
    float last_NI[3][3];
    float _NI[3][3];
    float last_zitai_NI[3][3];
    float _zitai_NI[3][3];

    bool invertMatrix3x3(const float input[3][3], float output[3][3]);

    // float get_Cmmde() const {
    //     return Cmmde;
    // }
    // float get_Cnmdr() const {
    //     return Cnmdr;
    // }
    // float get_CnmdLa() const {
    //     return CnmdLa;
    // }
    // float get_CnmdRa() const {
    //     return CnmdRa;
    // }
    // float get_ClmdLa() const {
    // return ClmdLa;
    // }   
    // float get_ClmdRa() const {
    // return ClmdRa;
    // }   
 
    static my *get_singleton() {
         return _singleton; 
    }


private:
    static my *_singleton;
};

namespace AP {
    my &My();
};