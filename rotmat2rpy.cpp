/*  http://planning.cs.uiuc.edu/node103.html
 
    |r11 r12 r13 |
    |r21 r22 r23 |
    |r31 r32 r33 |
 
    yaw: alpha=arctan(r21/r11)
    pitch: beta=arctan(-r31/sqrt( r32^2+r33^2 ) )
    roll: gamma=arctan(r32/r33)
*/
    double roll, pitch, yaw;
    roll=M_PI/3;
    pitch=M_PI/4;
    yaw=M_PI/6;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
 
    Eigen::Quaternion<double> q =  yawAngle*pitchAngle *rollAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();
 
 
 
    std::cout<<"roll is Pi/" <<M_PI/atan2( rotationMatrix(2,1),rotationMatrix(2,2) ) <<std::endl;
    std::cout<<"pitch: Pi/" <<M_PI/atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  ) <<std::endl;
    std::cout<<"yaw is Pi/" <<M_PI/atan2( rotationMatrix(1,0),rotationMatrix(0,0) ) <<std::endl;