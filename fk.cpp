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





int main()
{
while(1){
T0_1 << cos(tetha1), (-1) * sin(tetha1)* cos(alpha1), sin(tetha1)* sin(alpha1)         , alpha1 * cos(tetha1),
        sin(tetha1), cos(tetha1) * cos(alpha1)      , (-1) * cos(tetha1) * sin(alpha1) , alpha1 * sin(tetha1),
        0          , sin(alpha1)                    , cos(alpha1)                      , d1                  ,
        0          , 0                              , 0                                , 0                   ;




}
}