#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"

#include <math.h>
#include <numeric>
// alt + shift + f for indentation

using namespace std;
using namespace Eigen;

#define PI 3.14159265358979323846 /* pi */
#define d1 .6455
#define d2 0
#define d3 0
#define d4 1.245
#define d5 0
#define d6 0.240

#define alpha1 (-90 * (PI / 180))
#define alpha2 (0 * (PI / 180))
#define alpha3 (-90 * (PI / 180))
#define alpha4 (90 * (PI / 180))
#define alpha5 (-90 * (PI / 180))
#define alpha6 (0 * (PI / 180))

#define a1 0.260
#define a2 1.150
#define a3 0.230
#define a4 0
#define a5 0
#define a6 0

double tetha1;
double tetha2_2;
double tetha2_1; // tetha3 has two answer
double tetha3;
double tetha3_1; // tetha3 has two answer
double tetha3_2; // tetha3 has two answer
double tetha4;
double tetha5;
double tetha5_1;
double tetha5_2;
double tetha6;
////////////////////////////////////////////////////////////////
double r = 0;
double s = 0;
double shOffset = 0; // shoulder offset
double D;            // D is cos(tetha3)
double h1;           // h1 is used to calculate tetha2
double h2;           // h2 is used for calculating tetha3

// input end effector pos(0-2) and or(3-6)
vector<double> randomPose;
vector<double> goalJointAngle;
vector<double> ee_FKPose;

// convert Quaternion to Euler
struct Quaternion_
{
    double w, x, y, z;
};

struct EulerAngles_
{
    double roll, pitch, yaw;
};

// Notice the Euler angle rotation order is ZYX
EulerAngles_ toEulerAngle(Quaternion_ q)
{
    // cerr << "inside toEulerAngle function ******************************************* " << endl;
    // cerr << "w = " << q.w << ", " << "x = " << q.x << ", " << "y = " << q.y << ", " << "z = " << q.z << endl;

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

int main()
{

    while (1)
    {
        
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
                1, 0, 0,
                0, 0, 1;

            ///------------------Rw_0 for converting frame 0 to world frame-----------------------///
            Matrix3d Rw_0;
            Rw_0 << 0, 1, 0,
                -1, 0, 0,
                0, 0, 1;

            ///------------------Define user input variable------------------////
            ///>>>>Get the value of link6 from Rviz as the input directly<<<<<////

            // Orientation input from user in Degrees relative to the world frame

            Quaternion_ q_;
            EulerAngles_ e_;

            // q_.w = randomPose_[3];
            // q_.x = randomPose_[4];
            // q_.y = randomPose_[5];
            // q_.z = randomPose_[6];
            cout << "input q.w: "; // Type a number and press enter
            cin >> q_.w;           // Get user input from the keyboard
            cout << "input q.x: "; // Type a number and press enter
            cin >> q_.x;           // Get user input from the keyboard
            cout << "input q.y: "; // Type a number and press enter
            cin >> q_.y;           // Get user input from the keyboard

            cout << "input q.z: "; // Type a number and press enter
            cin >> q_.z;           // Get user input from the keyboard

            // cerr << "quaternion pose ---------------------- just before toEulerAngle -------------------------" << endl;
            // cerr << "w = " << q_.w << ", " << "x = " << q_.x << ", " << "y = " << q_.y << ", " << "z = " << q_.z << endl;
            RowVector4d Last_row(0.0, 0.0, 0.0, 1.0);
            e_ = toEulerAngle(q_);

            double Tp = e_.pitch; // Y axis
            double Tr = e_.roll;  // X axis
            double Ty = e_.yaw;   // Z axis
            // cerr << "Tr = " << Tr << ", " << "Tp = " << Tp << ", " << "Ty = " << Ty << endl;

            // Displacement input from user in meter relative to the world frame
            // double Xe = randomPose_[0] * 1000; // mm
            // double Ye = randomPose_[1] * 1000; // mm
            // double Ze = randomPose_[2] * 1000; // mm
            double Xe = 0;        // mm
            double Ye = 0;        // mm
            double Ze = 0;        // mm
            cout << "input Xe: "; // Type a number and press enter
            cin >> Xe;            // Get user input from the keyboard

            cout << "input Ye: "; // Type a number and press enter
            cin >> Ye;            // Get user input from the keyboard
            cout << "input Ze: "; // Type a number and press enter
            cin >> Ze;            // Get user input from the keyboard

            ///--------------------Construct HTM for H0_6------------------------///
            // Create displacement vector relative to the world frame
            Vector3d dw_6(Xe, Ye, Ze);

            // Convert the displacement vector from world frame to frame 0, used for constructing H0_6 below
            // because here frame 0 and frame world is the same
            Vector3d d0_6;
            // d0_6 = R0_w * dw_6;
            d0_6 = dw_6;

            // prepare R0_6 rotational matrix
            Matrix3d rw_6;
            Matrix3d pw_6;
            Matrix3d yw_6;
            double u, v1, v2, w1, w2;

            Matrix3d R3_0;
            Matrix3d R0_3;
            Matrix3d R3_6;

            double r11, r12, r13, r21, r22, r23, r31, r32, r33;

            R0_3 << cos(u) * cos(v1 + w1), sin(u), cos(u) * sin(v1 + w1),
                cos(v1 + w1) * sin(u), -1 * cos(u), sin(u) * sin(v1 + w1),
                sin(v1 + w1), 0, -1 * cos(v1 + w1);
            // X axis
            pw_6 << 1.0, 0.0, 0.0,
                0.0, cos(Tr), -sin(Tr),
                0.0, sin(Tr), cos(Tr);

            // Y axis
            rw_6 << cos(Tp), 0.0, sin(Tp),
                0.0, 1.0, 0.0,
                -sin(Tp), 0.0, cos(Tp);

            // Z axis
            yw_6 << cos(Ty), -sin(Ty), 0.0,
                sin(Ty), cos(Ty), 0.0,
                0.0, 0.0, 1.0;

            // Getting rotational matrix Rw_6 relative to the world frame
            Matrix3d Rw_6;

            // following the Euler ZYX convention
            /// This is how ZYX works not the other way around conventionally
            // cerr << endl;
            // Rw_6 = pw_6 * rw_6 * yw_6;
            Rw_6 = yw_6 * rw_6 * pw_6;
            // cerr << "Rw_6 = " << endl;
            // cerr << Rw_6 << endl;

            /// Convert rotational matrix Rw_6 to R0_6
            Matrix3d R0_6;
            // R0_6 = R0_w * Rw_6;
            R0_6 = Rw_6;

            /// create Hw_6
            // This is the user input HTM
            MatrixXd temp_mmm(Rw_6.rows(), Rw_6.cols() + dw_6.cols());
            temp_mmm << Rw_6, dw_6;
            MatrixXd Hw_6(temp_mmm.rows() + Last_row.rows(), temp_mmm.cols());
            Hw_6 << temp_mmm, Last_row;

            /// create H0_6
            MatrixXd temp_m(R0_6.rows(), R0_6.cols() + d0_6.cols());
            temp_m << R0_6, d0_6;
            MatrixXd H0_6(temp_m.rows() + Last_row.rows(), temp_m.cols());
            H0_6 << temp_m, Last_row;

            // cerr << "User input relative to the world frame.  Hw_6 = " << endl << Hw_6 << endl;
            // cerr << "Convert direct input to frame 0.  H0_6 = " << endl << H0_6 << endl;

            ///------------------Solve IK for the first three joints---------------///
            // Getting Z0_6 from R0_6
            Vector3d Z0_6 = R0_6.col(2);
            Vector3d Zw_6 = Rw_6.col(2);

            // Getting wrist center vector relative to frame 0
            Vector3d d0_c = d0_6 - (d5 * Z0_6);

            // cerr << "d0_c = " << endl << d0_c << endl << endl;

            double X0_c = d0_c(0);
            double Y0_c = d0_c(1);
            double Z0_c = d0_c(2);

            /// Solve for T1
            // Compute Theta 1 in order to get d1_c
            tetha1 = atan2(-1 * X0_c, Y0_c);
            // calculate tetha3
            r = pow(pow(X0_c, 2.0) + pow(Y0_c, 2.0) - pow(shOffset, 2), 0.5);
            s = Z0_c - d1;
            tetha3 = ((pow(r, 2) + pow(s, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3));
            D = cos(tetha3);
            tetha3_1 = atan2(D, pow(1 - pow(D, 2), 0.5));
            tetha3_2 = atan2(D, -1 * pow(1 - pow(D, 2), 0.5));
            // calculating tetha2
            h1 = atan2(r, s);
            h2 = atan2((a2 + a3 * cos(tetha3_1)), (a3 * sin(tetha3_1)));
            // h2_1=atan2((a2+a3*cos(tetha3_2)),(a3*sin(tetha3_2)));

            tetha2_1 = h1 - h2;
            //            tetha2_2 = h1 - h2_1;

            /////calculating H3-0 h3-0=h1-0*h2-1*h3-2
            u = tetha1;
            v1 = tetha2_1;
            // v2 = tetha2_2;
            w1 = tetha3_1;
            R3_0 = R0_3.inverse();
            R3_6 = R3_0 * R0_6;
            // w2 = tetha3_2;
            r33 = R3_6(2, 2);
            r13 = R3_6(0, 2);
            r23 = R3_6(1, 2);
            r31 = R3_6(2, 0);
            r32 = R3_6(2, 1);

            tetha5_1 = atan2(r33, pow(pow(r13, 2) + pow(r23, 2), .5));
            // tetha5_2 = atan2(r33,-1*pow(pow(r13,2)+pow(r23,2),.5));
            if (tetha5_1 > 0 && tetha5_1 < PI)
            {
                tetha4 = atan2(r13, r23);
                tetha6 = atan2(r31, r32);
            }
            else if (tetha5_1 > (-1 * PI) && tetha5_1 < 0)
            {
                tetha4 = atan2(-1 * r13, -1 * r23);
                tetha6 = atan2(r31, -1 * r32);
            }
            else
            {

                cout << "you are fucked "; // Type a number and press enter

                cout << tetha5_1; // Type a number and press enter
            }

                cout << "tetha1: "; // Type a number and press enter

                cout << tetha1; // Type a number and press enter
                
                cout << "tetha2 "; // Type a number and press enter

                cout << tetha2_1; // Type a number and press enter
                
                cout << "tetha3 "; // Type a number and press enter

                cout << tetha3_1; // Type a number and press enter
                
                cout << "tetha4:"; // Type a number and press enter

                cout << tetha4; // Type a number and press enter
                
                cout << "tetha5: "; // Type a number and press enter

                cout << tetha5_1; // Type a number and press enter
                
                cout << "tetha6"; // Type a number and press enter

                cout << tetha6; // Type a number and press enter
            ////////////////calculating H3-6,,,H3-6=(transpose (H0-3))*H(0-6)
        }
}