#include <cmath>
// #include <iostream>
// using namespace std;

// Struct for 3-DOF results (two configurations: elbow-down and elbow-up)
struct ThreeDofResult {
    double theta1a, theta2a, theta3a;  // Elbow-down
    double theta1b, theta2b, theta3b;  // Elbow-up
    bool valid;  // True if reachable
};

// Struct for 4-DOF results
struct InverseKinematicsResult {
    double deg1, deg2, deg3, deg4;
    bool valid;  // True if reachable
};

// Function to calculate 3-DOF inverse kinematics for planar arm
ThreeDofResult three_dof(double x, double y, double l1, double l2, double l3) {
    ThreeDofResult result = {0, 0, 0, 0, 0, 0, false};
    double gamma = 0.0;  // End-effector orientation in radians (0 degrees)
    
    // Wrist position
    double x3 = x - l3 * cos(gamma);
    double y3 = y - l3 * sin(gamma);
    
    // Check reachability
    double d = sqrt(x3 * x3 + y3 * y3);
    if (d > l1 + l2 || d < fabs(l1 - l2)) {
        return result;  // Invalid
    }
    
    // Theta2
    double cos_theta2 = (x3 * x3 + y3 * y3 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (cos_theta2 > 1.0) cos_theta2 = 1.0;
    if (cos_theta2 < -1.0) cos_theta2 = -1.0;
    double theta2a = acos(cos_theta2);  // Elbow-down
    double theta2b = -theta2a;  // Elbow-up
    
    // Theta1
    double theta1a = atan2(y3, x3) - atan2(l2 * sin(theta2a), l1 + l2 * cos(theta2a));
    double theta1b = atan2(y3, x3) - atan2(l2 * sin(theta2b), l1 + l2 * cos(theta2b));
    
    // Theta3
    double theta3a = gamma - theta1a - theta2a;
    double theta3b = gamma - theta1b - theta2b;
    
    // Convert to degrees
    result.theta1a = theta1a * 180.0 / M_PI;
    result.theta2a = theta2a * 180.0 / M_PI;
    result.theta3a = theta3a * 180.0 / M_PI;
    result.theta1b = theta1b * 180.0 / M_PI;
    result.theta2b = theta2b * 180.0 / M_PI;
    result.theta3b = theta3b * 180.0 / M_PI;
    result.valid = true;
    
    return result;
}

// Function to calculate 4-DOF inverse kinematics (with base rotation)
InverseKinematicsResult inverse_kinematics(double x, double y, double z, double l1, double l2, double l3) {
    InverseKinematicsResult result = {0, 0, 0, 0, false};
    double r = sqrt(x * x + y * y);
    double deg1 = atan2(y, x) * 180.0 / M_PI;
    ThreeDofResult three_result = three_dof(r, z, l1, l2, l3);
    if (!three_result.valid) {
        return result;  // Invalid
    }
    // Choose elbow-up (b)
    result.deg1 = deg1;
    result.deg2 = three_result.theta1b;
    result.deg3 = three_result.theta2b;
    result.deg4 = three_result.theta3b;
    result.valid = true;
    return result;
}

// int main() {
//     InverseKinematicsResult res = inverse_kinematics(2.0, 1.0, 1.5, 1.0, 1.0, 1.0);
//     if (res.valid) {
//         cout << "deg1: " << res.deg1 << ", deg2: " << res.deg2 
//                   << ", deg3: " << res.deg3 << ", deg4: " << res.deg4 << endl;
//     } else {
//         cout << "Target not reachable." << endl;
//     }
//     return 0;
// }