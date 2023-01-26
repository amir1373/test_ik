#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"


#include <math.h>
#include <numeric>

using namespace std;
using namespace Eigen;

# define PI           3.14159265358979323846  /* pi */

void IkComputeFunction(vector<double> randomPose_);
void FkComputeFunction(vector<double> goalJointAngle_);
void IkTestResult(vector<double> ee_FKPose_);


vector<double> randomPose;
vector<double> goalJointAngle;
vector<double> ee_FKPose;

double valueThreshold = 0.000001;



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

//we make it global just to see the limiter
double Le;

void IkComputeFunction(vector<double> randomPose_)
{
    //clear out last set joint angle value


    ///------------------Define joint angle variable-------------------/////
    double T1;
    double T2;
    double T3;
    double T4;
    double T5;
    double T6;

    ///------------------Define link length variable------------------////
    //unit is mm
    double L2   = 760.0; //Link 2, distance between frame origin 1 and origin 2 ;
    double d4   = 795.0; //equivalent length of the wrist from origin_2/3 to origin_4/5
    double d5   = 105.0; //link 5 length

    ///------------------Create forth row to make it square matrix------------------///
    RowVector4d Last_row(0.0d, 0.0d, 0.0d, 1.0d);

    ///------------------Define some trigonometric variables------------------///

    //double Le;

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
    T1 = atan2(-X0_c, Y0_c);
    //cerr << "T1 = " << T1 << endl;

    d0_1 << -150.0 * sin(T1), 150.0d * cos(T1), 511.0d;
    //cerr << "d0_1 = " << endl << d0_1 << endl << endl;

    ///Solve for T3
    //Compute the rotational matrix R1_0
    Matrix3d R1_0;
    R1_0 << sin(T1),  cos(T1), 0,
             0,        0,       1,
             cos(T1), -sin(T1), 0;

    //static rotation matrix, frame 0 relative to frame 1
    Matrix3d R1_0s;
    R1_0s << 0, 1, 0,
             0, 0, 1,
             1, 0, 0;

    //Compute d1_c measured in frame 0
    Vector3d d1_c = d0_c - d0_1;
    //cerr << "d1_c measured in frame 0 = " << endl << d1_c << endl << endl;

    //Compute d1_c measured in frame 1
    Vector3d d1_c_temp = d1_c;
    d1_c = R1_0s * d1_c_temp;

    double X1_c = d1_c(0);
    double Y1_c = d1_c(1);
    double Z1_c = d1_c(2);
    //cerr << "d1_c measured in frame 1 = " << endl << d1_c << endl << endl;

    //Compute trigonometric length Le
    Le = pow(pow(X1_c, 2.0) + pow(Y1_c, 2.0) + pow(Z1_c, 2.0), 0.5);
    //cerr << "Le = " << Le << endl;

    double d4_p = pow(pow(140, 2.0) + pow(795, 2.0), 0.5); //hypotenuse of 140 and 795

    double T3_g = atan2(140, 795);

    double T3_p = pow(pow(900, 2.0) + pow(795, 2.0), 0.5);

    double T3_s = acos((pow(L2, 2.0) + pow(d4_p, 2.0) - pow(T3_p, 2.0)) / (2 * L2 * d4_p));

    //Compute the rotational angle theta3 T3
    double T3_temp = acos((pow(L2, 2.0) + pow(d4_p, 2.0) - pow(Le, 2.0)) / (2 * L2 * d4_p));

    //cerr << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ " << (pow(L2, 2.0) + pow(d4_p, 2.0) - pow(Le, 2.0)) / (2 * L2 * d4_p) << endl;

    T3 = abs(T3_temp) - abs(T3_s);
    // cerr << "T3_temp = " << T3_temp << endl;
    //cerr << "T3_s = " << T3_s << endl;

    ///Solve for T2
    //define two transient angles a(alpha), b(beta) in radians
    double b = acos(Y1_c / Le);
    double a = acos((pow(L2, 2.0) + pow(Le, 2.0) - pow(d4_p, 2.0)) / (2 * L2 * Le));

    //cerr << "b = " << b * 180 / PI << endl;
    //cerr << "a = " << a * 180 / PI<< endl;

    Matrix3d mp;
    mp << cos(T1), -sin(T1), 0.0d,
        sin(T1),  cos(T1), 0.0d,
        0.0d,     0.0d,    1.0d;

    //undo the rotation of joint 0 for d0_c
    Vector3d qq = mp.inverse() * d0_c;

    if(qq(1) < 150.0)
        T2 = abs(a) + abs(b);
    else
    {
        if(abs(a) > abs(b))
            T2 = abs(a) - abs(b);
        else
            T2 = -(abs(b) - abs(a));
    }


    ///--------------Create HTM for adjacent frames--------------///

    //This is the original matrix relative to frame 0, now we need to convert it to base frame

    R0_1 << -sin(T1), 0, cos(T1),
        cos(T1), 0, sin(T1),
        0,       1, 0;

    R1_2 << -sin(T2), -cos(T2), 0,
        cos(T2), -sin(T2), 0,
        0,        0,       1;

    R2_3 << cos(T3), 0,  sin(T3),
        sin(T3), 0, -cos(T3),
        0,       1,  0;

    //d0_1 already defined above
    //This is the original displacement vector
    d0_1 << -150.0 * sin(T1), 150.0d * cos(T1), 511.0d;

    //Re-define the displacement vector relative to the world frame
    //d0_1 << 150.0d * cos(T1), 150.0 * sin(T1), 511.0d;

    d1_2 << -L2 * sin(T2), L2 * cos(T2), 0.0d;

    d2_3 <<  d4_p * sin(T3 + T3_g), -d4_p * cos(T3 + T3_g), 0.0d;

    //1.--->H0_1
    MatrixXd temp_m0(R0_1.rows(), R0_1.cols() + d0_1.cols());
    temp_m0 << R0_1, d0_1;
    MatrixXd H0_1(temp_m0.rows() + Last_row.rows(), temp_m0.cols());
    H0_1 << temp_m0, Last_row;

    //2.--->H1_2
    MatrixXd temp_m1(R1_2.rows(), R1_2.cols() + d1_2.cols());
    temp_m1 << R1_2, d1_2;
    MatrixXd H1_2(temp_m1.rows() + Last_row.rows(), temp_m1.cols());
    H1_2 << temp_m1, Last_row;

    //3.--->H2_3
    MatrixXd temp_m2(R2_3.rows(), R2_3.cols() + d2_3.cols());
    temp_m2 << R2_3, d2_3;
    MatrixXd H2_3(temp_m2.rows() + Last_row.rows(), temp_m2.cols());
    H2_3 << temp_m2, Last_row;

    Matrix4d H0_2;
    Matrix4d H0_3;

    H0_2 = H0_1 * H1_2;
    H0_3 = H0_2 * H2_3;

    ///------------Solving for the last 3 joints---------------///

    //Get HTM H3_6
    Matrix4d H3_6;
    Matrix4d Temp_im = Matrix4d(H0_3.inverse());
    //cerr << "H0_3.inverse = " << endl << Temp_im << endl << endl;
    H3_6 = Temp_im * H0_6;

    //cerr << "H3_6 = " << endl << H3_6 << endl;

    //H3_6 derivation
    ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<///

    //-cos(T4)*sin(T5)*cos(T6)-sin(T4)*sin(T6),  cos(T4)*sin(T5)*sin(T6)-sin(T4)*cos(T6), -cos(T4)*cos(T5), -cos(T4)*cos(T5)*d5,
    //-sin(T4)*sin(T5)*cos(T6)+cos(T4)*sin(T6),  sin(T4)*sin(T5)*sin(T6)+cos(T4)*cos(T6), -sin(T4)*cos(T5), -sin(T4)*cos(T5)*d5,
    // cos(T5)*cos(T6),                         -cos(T5)*sin(T6),                         -sin(T5),         -sin(T5)*d5,
    // 0,                                        0,                                        0,                1;

    ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<///

    //define some variables
    double m1_3 = H3_6(0, 2); //row 1 column 3
    double m2_3 = H3_6(1, 2); //row 2 column 3
    double m3_3 = H3_6(2, 2); //row 3 column 3
    double m3_1 = H3_6(2, 0); //row 3 column 1
    double m3_2 = H3_6(2, 1); //row 3 column 2

    // if(int(m3_1 * 10000) == 0)
    //     m3_1 = 0;

    // if(int(m3_2 * 10000) == 0)
    //     m3_2 = 0;


    T5 = -asin(m3_3);
//     T4 = atan(m2_3 / m1_3);
//     int32_t temp_v1 = int(-cos(T4) * cos(T5) * 1000000);
//    int32_t temp_v2 = int(m1_3 * 1000000);


//    if(temp_v1 != temp_v2)
//    {
//        T5 = PI * (T5 / abs(T5)) - T5;
//        temp_v1 = int(-cos(T4) * cos(T5) * 1000000);
//        if(temp_v1 != temp_v2)
//        {
//            T4 = PI * (T4 / abs(T4)) + T4;
//            temp_v1 = int(-cos(T4) * cos(T5) * 1000000);
//            if(temp_v1 != temp_v2)
//                T5 = -asin(m3_3);
//        }
//    }

    T4 = atan2(m2_3, m1_3);
    double temp_v1 = -m1_3 / cos(T5); // cos(T4)
    double temp_v2 = -m2_3 / cos(T5); // sin(T4)

    double temp_a = temp_v1 / abs(temp_v1);
    double temp_b = temp_v2 / abs(temp_v2);
    //cerr << "temp_a = " << temp_a << ",    " << "temp_b = " << temp_b << endl; 

    //cerr << "T4 before if condition == " << T4 << "   ,  " << T4 * 180.0 / PI << endl;
    //cerr << "PI = " << PI << ",   " << "PI / 2.0 = " << PI / 2.0 << endl; 

    if(0.0 < T4 && T4 < double(PI / 2.0))
    {
        if(temp_a == double(-1.0) && temp_b == double(-1.0))
        {
            T4 = -(double(PI) - T4);
        }
    }
    else
    {
        if(double(PI / 2.0) < T4 && T4 < double(PI))
        {
            if(temp_a == double(1.0) && temp_b == double(-1))
            {
                T4 = -(double(PI) - T4);
            }
        }
        else
        {
            if(-double(PI / 2.0) < T4 && T4 < 0.0)
            {
                if(temp_a == double(-1.0) && temp_b == double(1.0))
                {
                    T4 = (double(PI) + T4);
                }
            }
            else
            {
                if(-double(PI) < T4 && T4 < -double(PI / 2.0))
                {
                    if(temp_a == double(1.0) && temp_b == double(1.0))
                    {
                        T4 = double(PI) + T4;
                    }
                }
            }
        }

    }






    


    // if(T4 > PI / 2)
    // {
    //     if(temp_a == 1 && temp_b == -1)
    //         T4 = -(PI - T4);
    // }
    // else
    // {
    //     if(temp_a == -1 && temp_b == -1)
    //         T4 = -(PI - T4);
    // }

    T6 = -atan(m3_2 / m3_1);
    //define sin(T6)
    double S6 = m3_2 / (-cos(T5));

    if(T6 < 0 && S6 > 0)
        T6 = PI + T6;

    if(T6 > 0 && S6 < 0)
        T6 = -PI + T6;

    //T1 - T6 is in radian unit while it's double data type
    // cerr << "T1 in degrees = " << (T1 * 180.0 / PI) << endl;
    // cerr << "T2 in degrees = " << (T2 * 180.0 / PI) << endl;
    // cerr << "T3 in degrees = " << (T3 * 180.0 / PI) << endl;
    // cerr << "T4 in degrees = " << (T4 * 180.0 / PI) << endl;
    // cerr << "T5 in degrees = " << (T5 * 180.0 / PI) << endl;
    // cerr << "T6 in degrees = " << (T6 * 180.0 / PI) << endl << endl;
    goalJointAngle.push_back(T1);
    goalJointAngle.push_back(T2);
    goalJointAngle.push_back(T3);
    goalJointAngle.push_back(T4);
    goalJointAngle.push_back(T5);
    goalJointAngle.push_back(T6);

    //cerr << "execution time ===================================== " << duration_ << endl;

    FkComputeFunction(goalJointAngle);

}


void FkComputeFunction(vector<double> goalJointAngle_)
{
    //always reset
    ee_FKPose.clear();

    static double T1;
    static double T2;
    static double T3;
    static double T4;
    static double T5;
    static double T6;

    T1 = goalJointAngle[0]; //Joint 1
    T2 = goalJointAngle[1]; //Joint 2
    T3 = goalJointAngle[2]; //Joint 3
    T4 = goalJointAngle[3]; //Joint 4
    T5 = goalJointAngle[4]; //Joint 5
    T6 = goalJointAngle[5]; //Joint 6

    Matrix3d R0_1;
    Matrix3d R1_2;
    Matrix3d R2_3;

    Vector3d d0_1;
    Vector3d d1_2;
    Vector3d d2_3;

    MatrixXd H0_6;

    RowVector4d Last_row(0.0d, 0.0d, 0.0d, 1.0d);

    //unit is mm
    double L2   = 760.0; //Link 2, distance between frame origin 1 and origin 2 ;
    double d4   = 795.0; //equivalent length of the wrist from origin_2/3 to origin_4/5
    double d5   = 105.0; //link 5 length

    double d4_p = pow(pow(140, 2.0) + pow(795, 2.0), 0.5); //hypotenuse of 140 and 795
    double T3_g = atan2(140, 795);

    R0_1 << -sin(T1), 0, cos(T1),
             cos(T1), 0, sin(T1),
             0,       1, 0;

    R1_2 << -sin(T2), -cos(T2), 0,
            cos(T2), -sin(T2), 0,
              0,        0,       1;

    R2_3 << cos(T3), 0,  sin(T3),
           sin(T3), 0, -cos(T3),
           0,       1,  0;

    //d0_1 already defined above
    //This is the original displacement vector
    d0_1 << -150.0 * sin(T1), 150.0d * cos(T1), 511.0d;

    //Re-define the displacement vector relative to the world frame
    //d0_1 << 150.0d * cos(T1), 150.0 * sin(T1), 511.0d;

    d1_2 << -L2 * sin(T2), L2 * cos(T2), 0.0d;

    d2_3 <<  d4_p * sin(T3 + T3_g), -d4_p * cos(T3 + T3_g), 0.0d;

    //1.--->H0_1
    MatrixXd temp_m0(R0_1.rows(), R0_1.cols() + d0_1.cols());
    temp_m0 << R0_1, d0_1;
    MatrixXd H0_1(temp_m0.rows() + Last_row.rows(), temp_m0.cols());
    H0_1 << temp_m0, Last_row;

    //2.--->H1_2
    MatrixXd temp_m1(R1_2.rows(), R1_2.cols() + d1_2.cols());
    temp_m1 << R1_2, d1_2;
    MatrixXd H1_2(temp_m1.rows() + Last_row.rows(), temp_m1.cols());
    H1_2 << temp_m1, Last_row;

    //3.--->H2_3
    MatrixXd temp_m2(R2_3.rows(), R2_3.cols() + d2_3.cols());
    temp_m2 << R2_3, d2_3;
    MatrixXd H2_3(temp_m2.rows() + Last_row.rows(), temp_m2.cols());
    H2_3 << temp_m2, Last_row;

    Matrix4d H0_2;
    Matrix4d H0_3;

    H0_2 = H0_1 * H1_2;
    H0_3 = H0_2 * H2_3;

    ///---------------Define rotation matrix for last three joints--------------///
    //4.--->rotation matrix between frame 3 and frame 4
    Matrix3d R3_4;
    R3_4 << cos(T4), 0,  sin(T4),
           sin(T4), 0, -cos(T4),
            0,       1,  0;

    //5.--->rotation matrix between frame 4 and frame 5
    Matrix3d R4_5;
    R4_5 << -sin(T5), 0, -cos(T5),
            cos(T5), 0, -sin(T5),
            0,       -1, 0;

    //6.--->rotation matrix between frame 5 and frame 6
    Matrix3d R5_6;
    R5_6 << cos(T6), -sin(T6), 0,
        sin(T6),  cos(T6), 0,
        0,        0,       1;

    ///------------------Define displacement vector for last three joints--------------------///
    //4.--->displacement vector between frame 3 and frame 4
    Vector3d d3_4(0.0d, 0.0d, 0.0d);

    //5.--->displacement vector between frame 4 and frame 5
    Vector3d d4_5(-d5 * cos(T5), -d5 * sin(T5), 0.0d);

    //6.--->displacement vector between frame 5 and frame 6
    Vector3d d5_6(0.0d, 0.0d, 0.0d);

    ///-------------------Create HTM for last three joints----------------------------///
    //4.--->H3_4
    MatrixXd temp_m3(R3_4.rows(), R3_4.cols() + d3_4.cols());
    temp_m3 << R3_4, d3_4;
    MatrixXd H3_4(temp_m3.rows() + Last_row.rows(), temp_m3.cols());
    H3_4 << temp_m3, Last_row;

    //5.--->H4_5
    MatrixXd temp_m4(R4_5.rows(), R4_5.cols() + d4_5.cols());
    temp_m4 << R4_5, d4_5;
    MatrixXd H4_5(temp_m4.rows() + Last_row.rows(), temp_m4.cols());
    H4_5 << temp_m4, Last_row;
    //cerr << "H4_5 = " << H4_5 << endl;

    //6.--->H5_6
    MatrixXd temp_m5(R5_6.rows(), R5_6.cols() + d5_6.cols());
    temp_m5 << R5_6, d5_6;
    MatrixXd H5_6(temp_m5.rows() + Last_row.rows(), temp_m5.cols());
    H5_6 << temp_m5, Last_row;

    Matrix4d H0_4;
    Matrix4d H0_5;

    H0_4 = H0_3 * H3_4;
    H0_5 = H0_4 * H4_5;
    H0_6 = H0_5 * H5_6;

    //cerr << "H0_6 = " << endl << H0_6 << endl << endl;
    Quaterniond orientationValue;

    orientationValue = H0_6.block<3, 3>(0, 0);

    ee_FKPose.push_back(H0_6(0, 3) / 1000.0); // displacement x
    ee_FKPose.push_back(H0_6(1, 3) / 1000.0); // displacement y
    ee_FKPose.push_back(H0_6(2, 3) / 1000.0); // displacement z
    ee_FKPose.push_back(orientationValue.w());
    ee_FKPose.push_back(orientationValue.x());
    ee_FKPose.push_back(orientationValue.y());
    ee_FKPose.push_back(orientationValue.z());

    IkTestResult(ee_FKPose);

}

vector<double> diffResult;
int counter_ = 0;
void IkTestResult(vector<double> ee_FKPose_)
{
    //for current value
    double temp_ = 0.0;
    diffResult.clear();

    for(int i = 0; i < 7; i++)
    {
        if(abs(ee_FKPose_[i] - randomPose[i]) < valueThreshold)
            temp_ = 0.0;
        else
            temp_ = (ee_FKPose_[i] - randomPose[i]);

        diffResult.push_back(temp_);
    }

    if(accumulate(diffResult.begin(), diffResult.end(), 0.0) != 0.0)
    {
        cerr << "Error happened at goal Pose: " << "    Le = " << Le << endl;
        cerr << "x = " << randomPose[0] << ", " << "y = " << randomPose[1] << ", " << "z = " << randomPose[2] << ", "
        << "w = " << randomPose[3] << ", " << "x = " << randomPose[4] << ", " << "y = " << randomPose[5] << ", "
        << "z = " << randomPose[6] << endl;
        if(Le < 1560.0)
            exit(0);
        else
            cerr << "Error due to over the range" << endl;
    }
    else
        cerr << counter_ << " : " << "OK !!! " << endl;


    counter_++;

}


int main(int argc, char** argv)
{
    while(1){





    }
    


    return 0;
}
