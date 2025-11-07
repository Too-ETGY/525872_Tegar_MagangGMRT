#include <iostream>
#include <cmath>
#include <tuple>
#include <optional>

std::optional<std::tuple<double, double, double>> three_dof(double x, double y, double l1, double l2, double l3) {
    double psi = 0.0; // radians, end-effector orientation assumed 0 degrees

    double wrist_x = x - l3 * std::cos(psi);
    double wrist_y = y - l3 * std::sin(psi);

    double d = std::sqrt(wrist_x * wrist_x + wrist_y * wrist_y);

    if (d > l1 + l2 || d < std::abs(l1 - l2)) {
        return std::nullopt;
    }

    double cos_theta2 = (wrist_x * wrist_x + wrist_y * wrist_y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (cos_theta2 > 1) cos_theta2 = 1;
    if (cos_theta2 < -1) cos_theta2 = -1;
    double theta2 = std::acos(cos_theta2);

    double theta1 = std::atan2(wrist_y, wrist_x) - std::atan2(l2 * std::sin(theta2), l1 + l2 * std::cos(theta2));

    double theta3 = psi - theta1 - theta2;

    double theta1_deg = theta1 * 180.0 / M_PI;
    double theta2_deg = theta2 * 180.0 / M_PI;
    double theta3_deg = theta3 * 180.0 / M_PI;

    return std::make_tuple(theta1_deg, theta2_deg, theta3_deg);
}

std::optional<std::tuple<double, double, double, double>> inverse_kinematics(double x, double y, double z, double l0, double l1, double l2, double l3) {
    double r = std::sqrt(x * x + y * y);
    double deg1 = std::atan2(y, x) * 180.0 / M_PI;
    auto result = three_dof(r, z - l0, l1, l2, l3);
    if (!result.has_value()) {
        return std::nullopt;
    }
    auto [deg2, deg3, deg4] = result.value();
    return std::make_tuple(deg1, deg2, deg3, deg4);
}

int main() {
    double l0 = 2;
    double x = 10, y = 8, z = 8;
    double l1 = 7, l2 = 10, l3 = 3;

    auto result = inverse_kinematics(x, y, z, l0, l1, l2, l3);
    if (result.has_value()) {
        auto [deg1, deg2, deg3, deg4] = result.value();
        std::cout << "Deg1 (Base): " << deg1 << "°, Deg2: " << deg2 << "°, Deg3: " << deg3 << "°, Deg4: " << deg4 << "°\n";
    } else {
        std::cout << "Target unreachable\n";
    }

    return 0;
}