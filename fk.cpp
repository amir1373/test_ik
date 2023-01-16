#include "iostream"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <vector>
#include "math.h"


using namespace std;

using namespace Eigen;




#define d1 0
#define d2 0
#define d3 0
#define d4 0
#define d5 0
#define d6 0

#define alpha1 0
#define alpha2 0
#define alpha3 0
#define alpha4 0
#define alpha5 0
#define alpha6 0

#define a1 0
#define a2 0
#define a3 0
#define a4 0
#define a5 0
#define a6 0

double tetha1 = 0;
double tetha2 = 0;
double tetha3 = 0;
double tetha4 = 0;
double tetha5 = 0;
double tetha6 = 0;


# define PI           3.14159265358979323846  /* pi */

Matrix3d T0_1;
Matrix3d T1_2;
Matrix3d T2_3;
Matrix3d T3_4;
Matrix3d T4_5;
Matrix3d T5_6;
Matrix3d T0_6;






int main()
{
while(1){
    
T0_1 << cos(tetha1), (-1) * sin(tetha1)* cos(alpha1), sin(tetha1)* sin(alpha1)         , alpha1 * cos(tetha1),
        sin(tetha1), cos(tetha1) * cos(alpha1)      , (-1) * cos(tetha1) * sin(alpha1) , alpha1 * sin(tetha1),
        0          , sin(alpha1)                    , cos(alpha1)                      , d1                  ,
        0          , 0                              , 0                                , 0                   ;
T1_2 << cos(tetha2), (-1) * sin(tetha2)* cos(alpha2), sin(tetha2)* sin(alpha2)         , alpha2 * cos(tetha2),
        sin(tetha2), cos(tetha2) * cos(alpha2)      , (-1) * cos(tetha2) * sin(alpha2) , alpha2 * sin(tetha2),
        0          , sin(alpha2)                    , cos(alpha2)                      , d2                  ,
        0          , 0                              , 0                                , 0                   ;
T2_3 << cos(tetha3), (-1) * sin(tetha3)* cos(alpha3), sin(tetha3)* sin(alpha3)         , alpha3 * cos(tetha3),
        sin(tetha3), cos(tetha3) * cos(alpha3)      , (-1) * cos(tetha3) * sin(alpha3) , alpha3 * sin(tetha3),
        0          , sin(alpha3)                    , cos(alpha3)                      , d3                  ,
        0          , 0                              , 0                                , 0                   ;
T3_4 << cos(tetha4), (-1) * sin(tetha4)* cos(alpha4), sin(tetha4)* sin(alpha4)         , alpha4 * cos(tetha4),
        sin(tetha4), cos(tetha4) * cos(alpha4)      , (-1) * cos(tetha4) * sin(alpha4) , alpha4 * sin(tetha4),
        0          , sin(alpha4)                    , cos(alpha4)                      , d4                  ,
        0          , 0                              , 0                                , 0                   ;
T4_5 << cos(tetha5), (-1) * sin(tetha5)* cos(alpha5), sin(tetha5)* sin(alpha5)         , alpha5 * cos(tetha5),
        sin(tetha5), cos(tetha5) * cos(alpha5)      , (-1) * cos(tetha5) * sin(alpha5) , alpha5 * sin(tetha5),
        0          , sin(alpha5)                    , cos(alpha5)                      , d5                  ,
        0          , 0                              , 0                                , 0                   ;
T5_6 << cos(tetha6), (-1) * sin(tetha6)* cos(alpha6), sin(tetha6)* sin(alpha6)         , alpha6 * cos(tetha6),
        sin(tetha6), cos(tetha6) * cos(alpha6)      , (-1) * cos(tetha6) * sin(alpha6) , alpha6 * sin(tetha6),
        0          , sin(alpha6)                    , cos(alpha6)                      , d6                  ,
        0          , 0                              , 0                                , 0                   ;
}
T0_6 << cos(tetha6), (-1) * sin(tetha6)* cos(alpha6), sin(tetha6)* sin(alpha6)         , alpha6 * cos(tetha6),
        sin(tetha6), cos(tetha6) * cos(alpha6)      , (-1) * cos(tetha6) * sin(alpha6) , alpha2 * sin(tetha6),
        0          , sin(alpha6)                    , cos(alpha6)                      , d6                  ,
        0          , 0                              , 0                                , 0                   ;
}