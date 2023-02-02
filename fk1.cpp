#include "iostream"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <vector>
#include "math.h"


using namespace std;

using namespace Eigen;

# define PI           3.14159265358979323846  /* pi */



#define d1 .0450
#define d2 0
#define d3 0
#define d4 0.640
#define d5 0
#define d6 0.095

#define alpha1 (-90 * (PI/180))
#define alpha2 (0   * (PI/180))
#define alpha3 (-90 * (PI/180))
#define alpha4 (90  * (PI/180))
#define alpha5 (-90 * (PI/180))
#define alpha6 (0   * (PI/180))

#define a1 0.150
#define a2 0.570
#define a3 0.130
#define a4 0
#define a5 0
#define a6 0

double tetha1 = 0;
// double tetha2 = (-90 * (PI/180));
double tetha2 = 0;
double tetha3 = 0;
double tetha4 = 0;
// double tetha5 = (90 * (PI/180));
double tetha5 = 0;
double tetha6 = 0;



Matrix4f T0_1;
Matrix4f T1_2;
Matrix4f T2_3;
Matrix4f T3_4;
Matrix4f T4_5;
Matrix4f T5_6;
Matrix4f T0_6;
Matrix3f rotMat;
MatrixXf posMat;


int main()
{
while(1){
    cout << "input tetha1: "; // Type a number and press enter
    cin >> tetha1; // Get user input from the keyboard
    tetha1 = (tetha1 * (PI/180));
        cout << "input tetha2: "; // Type a number and press enter
    cin >> tetha2; // Get user input from the keyboard
        tetha2 = (tetha2 * (PI/180));

        cout << "input tetha3: "; // Type a number and press enter
    cin >> tetha3; // Get user input from the keyboard
        tetha3 = (tetha3 * (PI/180));

        cout << "input tetha4: "; // Type a number and press enter
    cin >> tetha4; // Get user input from the keyboard
        tetha4 = (tetha4 * (PI/180));

        cout << "input tetha5: "; // Type a number and press enter
    cin >> tetha5; // Get user input from the keyboard
        tetha5 = (tetha5 * (PI/180));

        cout << "input tetha6: "; // Type a number and press enter
    cin >> tetha6; // Get user input from the keyboard
        tetha6 = (tetha6 * (PI/180));

T0_1 << cos(tetha1), (-1) * sin(tetha1)* cos(alpha1), sin(tetha1)* sin(alpha1)         , a1 * cos(tetha1),
        sin(tetha1), cos(tetha1) * cos(alpha1)      , (-1) * cos(tetha1) * sin(alpha1) , a1 * sin(tetha1),
        0          , sin(alpha1)                    , cos(alpha1)                      , d1                  ,
        0          , 0                              , 0                                , 1                  ;
//         cout << "matrix output T0_1"; // Type a number and press enter
// cout << T0_1; // Type a number and press enter


T1_2 << cos(tetha2), (-1) * sin(tetha2)* cos(alpha2), sin(tetha2)* sin(alpha2)         , a2 * cos(tetha2),
        sin(tetha2), cos(tetha2) * cos(alpha2)      , (-1) * cos(tetha2) * sin(alpha2) , a2 * sin(tetha2),
        0          , sin(alpha2)                    , cos(alpha2)                      , d2                  ,
        0          , 0                              , 0                                , 1                   ;

//           cout << "matrix output T1_2"; // Type a number and press enter
// cout << T1_2; // Type a number and press enter      


T2_3 << cos(tetha3), (-1) * sin(tetha3)* cos(alpha3), sin(tetha3)* sin(alpha3)         , a3 * cos(tetha3),
        sin(tetha3), cos(tetha3) * cos(alpha3)      , (-1) * cos(tetha3) * sin(alpha3) , a3 * sin(tetha3),
        0          , sin(alpha3)                    , cos(alpha3)                      , d3                  ,
        0          , 0                              , 0                                , 1                  ;

//           cout << "matrix output T2_3"; // Type a number and press enter
// cout << T2_3; // Type a number and press enter     

T3_4 << cos(tetha4), (-1) * sin(tetha4)* cos(alpha4), sin(tetha4)* sin(alpha4)         , a4 * cos(tetha4),
        sin(tetha4), cos(tetha4) * cos(alpha4)      , (-1) * cos(tetha4) * sin(alpha4) , a4 * sin(tetha4),
        0          , sin(alpha4)                    , cos(alpha4)                      , d4                  ,
        0          , 0                              , 0                                , 1                   ;

//           cout << "matrix output T3_4"; // Type a number and press enter
// cout << T3_4; // Type a number and press enter     
T4_5 << cos(tetha5), (-1) * sin(tetha5)* cos(alpha5), sin(tetha5)* sin(alpha5)         , a5 * cos(tetha5),
        sin(tetha5), cos(tetha5) * cos(alpha5)      , (-1) * cos(tetha5) * sin(alpha5) , a5 * sin(tetha5),
        0          , sin(alpha5)                    , cos(alpha5)                      , d5                  ,
        0          , 0                              , 0                                , 1                  ;
//           cout << "matrix output T4_5"; // Type a number and press enter
// cout << T4_5; // Type a number and press enter     

T5_6 << cos(tetha6), (-1) * sin(tetha6)* cos(alpha6), sin(tetha6)* sin(alpha6)         , a6 * cos(tetha6),
        sin(tetha6), cos(tetha6) * cos(alpha6)      , (-1) * cos(tetha6) * sin(alpha6) , a6 * sin(tetha6),
        0          , sin(alpha6)                    , cos(alpha6)                      , d6                  ,
        0          , 0                              , 0                                , 1                   ;
//           cout << "matrix output T5_6"; // Type a number and press enter
// cout << T5_6; // Type a number and press enter     


// T0_6 << cos(tetha6), (-1) * sin(tetha6)* cos(alpha6), sin(tetha6)* sin(alpha6)         , alpha6 * cos(tetha6),
//         sin(tetha6), cos(tetha6) * cos(alpha6)      , (-1) * cos(tetha6) * sin(alpha6) , alpha2 * sin(tetha6),
//         0          , sin(alpha6)                    , cos(alpha6)                      , d6                  ,
//         0          , 0                              , 0                                , 0                   ;
T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 ;
rotMat = T0_6.block<3, 3>(0, 0);
posMat = T0_6.block<3, 1>(0, 3);






cout << "T matrix output\n"; // Type a number and press enter
cout << T0_6 << "\n"; // Type a number and press enter
cout << "P matrix output\n"; // Type a number and press enter
cout << rotMat << "\n"; // Type a number and press enter

cout << "R matrix output\n"; // Type a number and press enter
cout << posMat << "\n"; // Type a number and press enter

    cout << "input tetha1: "; // Type a number and press enter
    cin >> tetha1; // Get user input from the keyboard
    tetha1 = (tetha1 * (PI/180));
        cout << "input tetha2: "; // Type a number and press enter
    cin >> tetha2; // Get user input from the keyboard
        tetha2 = (tetha2 * (PI/180));

        cout << "input tetha3: "; // Type a number and press enter
    cin >> tetha3; // Get user input from the keyboard
        tetha3 = (tetha3 * (PI/180));

        cout << "input tetha4: "; // Type a number and press enter
    cin >> tetha4; // Get user input from the keyboard
        tetha4 = (tetha4 * (PI/180));

        cout << "input tetha5: "; // Type a number and press enter
    cin >> tetha5; // Get user input from the keyboard
        tetha5 = (tetha5 * (PI/180));

        cout << "input tetha6: "; // Type a number and press enter
    cin >> tetha6; // Get user input from the keyboard
        tetha6 = (tetha6 * (PI/180));


}
}