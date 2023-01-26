#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"


#include <math.h>
#include <numeric>

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

double tetha1 ;
double tetha2 ;
double tetha3 ;
double tetha4 ;
double tetha5 ;
double tetha6 ;
////////////////////////////////////////////////////////////////
double r=0;
double s=0;
double shOffset=0; //shoulder offset
double D; // D is cos(tetha3)
double tetha3_1;// tetha3 has two answer
double tetha3_2;//tetha3 has two answer
double h1; // h1 is used to calculate tetha2
double h2; //h2 is used for calculating tetha3 

// input end effector pos(0-2) and or(3-6)
vector<double> randomPose;


//convert Quaternion to Euler
struct Quaternion_ {
    double w, x, y, z;
};

struct EulerAngles_ {
    double roll, pitch, yaw;
};

//Notice the Euler angle rotation order is ZYX
EulerAngles_ toEulerAngle(Quaternion_ q)
{
    //cerr << "inside toEulerAngle function ******************************************* " << endl;
    //cerr << "w = " << q.w << ", " << "x = " << q.x << ", " << "y = " << q.y << ", " << "z = " << q.z << endl;

    EulerAngles_ e_;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    e_.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        e_.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        e_.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    e_.yaw = atan2(siny_cosp, cosy_cosp);

    // cerr << endl;
    // cerr << "Calculated Euler angles : " << endl;
    // cerr << "roll (X axis) = " << e_.roll << ", " << "pitch (Y axis) = " << e_.pitch << ", " << "yaw (Z axis) = " << e_.yaw << endl;
    return e_;
}
///------------------Define rotation matrix between adjacent frames------------------///
    Matrix3d R0_1;
    Matrix3d R1_2;
    Matrix3d R2_3;

    ///------------------Define displacement vector between adjacent frames------------------///
    Vector3d d0_1;
    Vector3d d1_2;
    Vector3d d2_3;

    ///------------------R0_w for converting world frame to frame 0-----------------------///
    Matrix3d R0_w;
    R0_w << 0, -1, 0,
            1,  0, 0,
            0,  0, 1;

    ///------------------Rw_0 for converting frame 0 to world frame-----------------------///
    Matrix3d Rw_0;
    Rw_0 <<  0, 1, 0,
            -1, 0, 0,
             0, 0, 1;

    ///------------------Define user input variable------------------////
    ///>>>>Get the value of link6 from Rviz as the input directly<<<<<////

    //Orientation input from user in Degrees relative to the world frame

    Quaternion_ q_;
    EulerAngles_ e_;

    q_.w = randomPose_[3];
    q_.x = randomPose_[4];
    q_.y = randomPose_[5];
    q_.z = randomPose_[6];
    //cerr << "quaternion pose ---------------------- just before toEulerAngle -------------------------" << endl;
    //cerr << "w = " << q_.w << ", " << "x = " << q_.x << ", " << "y = " << q_.y << ", " << "z = " << q_.z << endl;

    e_ = toEulerAngle(q_);

    double Tp = e_.pitch; //Y axis
    double Tr = e_.roll; //X axis
    double Ty = e_.yaw; // Z axis
    //cerr << "Tr = " << Tr << ", " << "Tp = " << Tp << ", " << "Ty = " << Ty << endl;

    //Displacement input from user in meter relative to the world frame
    double Xe = randomPose_[0] * 1000; // mm
    double Ye = randomPose_[1] * 1000; // mm
    double Ze = randomPose_[2] * 1000; // mm

    ///--------------------Construct HTM for H0_6------------------------///
    //Create displacement vector relative to the world frame
    Vector3d dw_6(Xe, Ye, Ze);

    //Convert the displacement vector from world frame to frame 0, used for constructing H0_6 below
    //because here frame 0 and frame world is the same
    Vector3d d0_6;
    //d0_6 = R0_w * dw_6;
    d0_6 = dw_6;

    //prepare R0_6 rotational matrix
    Matrix3d rw_6;
    Matrix3d pw_6;
    Matrix3d yw_6;

    //X axis
    pw_6 << 1.0d, 0.0d,     0.0d,
            0.0d, cos(Tr), -sin(Tr),
            0.0d, sin(Tr),  cos(Tr);

    //Y axis
    rw_6 << cos(Tp), 0.0d, sin(Tp),
            0.0d,    1.0d, 0.0d,
           -sin(Tp), 0.0d, cos(Tp);

    //Z axis
    yw_6 << cos(Ty), -sin(Ty), 0.0d,
            sin(Ty),  cos(Ty), 0.0d,
            0.0d,     0.0d,    1.0d;


    //Getting rotational matrix Rw_6 relative to the world frame
    Matrix3d Rw_6;

    //following the Euler ZYX convention
    ///This is how ZYX works not the other way around conventionally
    //cerr << endl;
    //Rw_6 = pw_6 * rw_6 * yw_6;
    Rw_6 = yw_6 * rw_6 * pw_6;
    //cerr << "Rw_6 = " << endl;
    //cerr << Rw_6 << endl;

    ///Convert rotational matrix Rw_6 to R0_6
    Matrix3d R0_6;
    //R0_6 = R0_w * Rw_6;
    R0_6 = Rw_6;

    ///create Hw_6
    //This is the user input HTM
    MatrixXd temp_mmm(Rw_6.rows(), Rw_6.cols() + dw_6.cols());
    temp_mmm << Rw_6, dw_6;
    MatrixXd Hw_6(temp_mmm.rows() + Last_row.rows(), temp_mmm.cols());
    Hw_6 << temp_mmm, Last_row;

    ///create H0_6
    MatrixXd temp_m(R0_6.rows(), R0_6.cols() + d0_6.cols());
    temp_m << R0_6, d0_6;
    MatrixXd H0_6(temp_m.rows() + Last_row.rows(), temp_m.cols());
    H0_6 << temp_m, Last_row;

    //cerr << "User input relative to the world frame.  Hw_6 = " << endl << Hw_6 << endl;
    //cerr << "Convert direct input to frame 0.  H0_6 = " << endl << H0_6 << endl;


    ///------------------Solve IK for the first three joints---------------///
    //Getting Z0_6 from R0_6
    Vector3d Z0_6 = R0_6.col(2);
    Vector3d Zw_6 = Rw_6.col(2);

    //Getting wrist center vector relative to frame 0
    Vector3d d0_c = d0_6 - (d5 * Z0_6);

    //cerr << "d0_c = " << endl << d0_c << endl << endl;

    double X0_c = d0_c(0);
    double Y0_c = d0_c(1);
    double Z0_c = d0_c(2);

    ///Solve for T1
    //Compute Theta 1 in order to get d1_c
    tetha1 = atan2(-X0_c, Y0_c);
    //calculate tetha3 
    r=pow(pow(X0_c, 2.0) + pow(Y0_c, 2.0)-pow(shOffset,2), 0.5);
    s=Z0_c-d1;
    cos(tetha3)=((pow(r,2)+pow(s,2)-pow(a2,2)-pow(a3,2))/(2*a2*a3));
    D=cos(tetha3);
    tetha3_1=atan2(D,pow(1 -pow(D,2), 0.5));
    tetha3_2=atan2(D,-pow(1 -pow(D,2), 0.5));
    //calculating tetha2
    h1=atan2(r,s);
    h2=atan2((a2+a3*cos(tetha3_1)),(a3*sin(tetha3_1)));
    tetha2=h1-h2;
    /////calculating H3-0 h3-0=h1-0*h2-1*h3-2
    





    ////////////////calculating H3-6,,,H3-6=(transpose (H0-3))*H(0-6)


