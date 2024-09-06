#include "ekf.hpp"

//sys函数
Eigen::Vector3d model(const Eigen::Vector3d xt_1, const Eigen::Vector2d u){
    Eigen::Vector3d xtt_1;
    //double newheading = xt_1(2)+u(1);//theta+deltatheta
    xtt_1(0) = xt_1(0) + u(0) * cos(xt_1(2)+u(1));
    xtt_1(1) = xt_1(1) + u(0) * sin(xt_1(2)+u(1));
    xtt_1(2) = xt_1(2) + u(1);
    return xtt_1;
}

Eigen::Matrix3d F_jac(const Eigen::Vector3d& xtt_1, const Eigen::Vector2d& u) {
    Eigen::MatrixXd result(3, 3);
    double newheading = xtt_1(2) + u(1);
    result << 1, 0, -u(0) * sin(newheading),
              0, 1,  u(0) * cos(newheading),
              0, 0,  1;
    return result;
}

Eigen::Vector3d h(const Eigen::Vector3d& x){
    Eigen::Vector3d z=x;
    return z;
}

Eigen::Matrix3d H_jac(const Eigen::Vector3d& xtt_1) {
    Eigen::Matrix3d H;
    H << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return H;
}
