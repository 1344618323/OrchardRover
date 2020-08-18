#include <opencv-3.3.1-dev/opencv/cv.hpp>
#include "PinholeCamera.h"

PinholeCamera::Parameters::Parameters()
        : m_k1(0.0),
          m_k2(0.0),
          m_p1(0.0),
          m_p2(0.0),
          m_fx(0.0),
          m_fy(0.0),
          m_cx(0.0),
          m_cy(0.0) {}


double PinholeCamera::Parameters::k1(void) const { return m_k1; }

double PinholeCamera::Parameters::k2(void) const { return m_k2; }

double PinholeCamera::Parameters::p1(void) const { return m_p1; }

double PinholeCamera::Parameters::p2(void) const { return m_p2; }

double PinholeCamera::Parameters::fx(void) const { return m_fx; }

double PinholeCamera::Parameters::fy(void) const { return m_fy; }

double PinholeCamera::Parameters::cx(void) const { return m_cx; }

double PinholeCamera::Parameters::cy(void) const { return m_cy; }

bool PinholeCamera::Parameters::readFromYamlFile(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        return false;
    }

    if (!fs["model_type"].isNone()) {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("PINHOLE") != 0) {
            return false;
        }
    }

    cv::FileNode n = fs["distortion_parameters"];
    m_k1 = static_cast<double>(n["k1"]);
    m_k2 = static_cast<double>(n["k2"]);
    m_p1 = static_cast<double>(n["p1"]);
    m_p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    m_fx = static_cast<double>(n["fx"]);
    m_fy = static_cast<double>(n["fy"]);
    m_cx = static_cast<double>(n["cx"]);
    m_cy = static_cast<double>(n["cy"]);
    return true;
}

PinholeCamera::PinholeCamera(const PinholeCamera::Parameters &params) {
    mParameters = params;

    if ((mParameters.k1() == 0.0) && (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) && (mParameters.p2() == 0.0)) {
        m_noDistortion = true;
    } else {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

/**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 * 已知 某个像素在像素坐标系下坐标p，注意这个坐标是畸变后得到的，
 * 即 X -> 内参矩阵 归一化平面坐标P -> 畸变Pd -> p
 * 我们希望得到 P
 * 使用的算法是 Recursive distortion model，还没找到出处，不过实验上是对的
 */
void PinholeCamera::liftProjective(const Eigen::Vector2d &p,
                                   Eigen::Vector3d &P) const {
    double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    // double lambda;

    // Lift points to normalised plane
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;

    if (m_noDistortion) {
        mx_u = mx_d;
        my_u = my_d;
    } else {
        if (0) {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();

            // Apply inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d * mx_d;
            my2_d = my_d * my_d;
            mxy_d = mx_d * my_d;
            rho2_d = mx2_d + my2_d;
            rho4_d = rho2_d * rho2_d;
            radDist_d = k1 * rho2_d + k2 * rho4_d;
            Dx_d = mx_d * radDist_d + p2 * (rho2_d + 2 * mx2_d) + 2 * p1 * mxy_d;
            Dy_d = my_d * radDist_d + p1 * (rho2_d + 2 * my2_d) + 2 * p2 * mxy_d;
            inv_denom_d = 1 / (1 + 4 * k1 * rho2_d + 6 * k2 * rho4_d + 8 * p1 * my_d +
                               8 * p2 * mx_d);

            mx_u = mx_d - inv_denom_d * Dx_d;
            my_u = my_d - inv_denom_d * Dy_d;
        } else {
            // Recursive distortion model
            int n = 8;
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            for (int i = 1; i < n; ++i) {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }
        }
    }

    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}

/**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 * 已知P，求p
 * 即 P -> 内参矩阵 归一化平面坐标X -> 畸变Xd -> p
 */
void PinholeCamera::spaceToPlane(const Eigen::Vector3d &P,
                                 Eigen::Vector2d &p, bool fix) const {
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    p_u << P(0) / P(2), P(1) / P(2);

    if (m_noDistortion || fix) {
        p_d = p_u;
    } else {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
            mParameters.fy() * p_d(1) + mParameters.cy();
}

/**
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 * 已知p_u，求p
 * 即 P -> 内参矩阵 归一化平面坐标p_u -> 畸变p_d -> p
 */
void PinholeCamera::undistToPlane(const Eigen::Vector2d &p_u,
                                  Eigen::Vector2d &p) const {
    Eigen::Vector2d p_d;

    if (m_noDistortion) {
        p_d = p_u;
    } else {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
            mParameters.fy() * p_d(1) + mParameters.cy();
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void PinholeCamera::distortion(const Eigen::Vector2d &p_u,
                               Eigen::Vector2d &d_u) const {
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);                         //x^2
    my2_u = p_u(1) * p_u(1);                         //y^2
    mxy_u = p_u(0) * p_u(1);                         //xy
    rho2_u = mx2_u + my2_u;                          //r^2=x^2+y^2
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u; //k1*r^2 + K2* r^4
    //dux = x*(k1*r^2 + K2* r^4) + 2* p1* xy + p2*(r^2 + 2*x^2)
    //duy =y*(k1*r^2 + K2* r^4) + 2* p2* xy + p1*(r^2 + 2*y^2)
    // xd = x+dux
    // yd = y+duy
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
            p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate Jacobian
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void PinholeCamera::distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u,
                               Eigen::Matrix2d &J) const {
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
            p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);

    //xd = x*(1+ k1*r^2 + K2* r^4) + 2* p1* xy + p2*(r^2 + 2*x^2)
    //yd = y*(1 +k1*r^2 + K2* r^4) + 2* p2* xy + p1*(r^2 + 2*y^2)
    //求 [xd yd]对 [x,y]的导数
    double dxdmx = 1.0 + rad_dist_u + k1 * 2.0 * mx2_u +
                   k2 * rho2_u * 4.0 * mx2_u + 2.0 * p1 * p_u(1) +
                   6.0 * p2 * p_u(0);
    double dydmx = k1 * 2.0 * p_u(0) * p_u(1) +
                   k2 * 4.0 * rho2_u * p_u(0) * p_u(1) + p1 * 2.0 * p_u(0) +
                   2.0 * p2 * p_u(1);
    double dxdmy = dydmx;
    double dydmy = 1.0 + rad_dist_u + k1 * 2.0 * my2_u +
                   k2 * rho2_u * 4.0 * my2_u + 6.0 * p1 * p_u(1) +
                   2.0 * p2 * p_u(0);

    J << dxdmx, dxdmy, dydmx, dydmy;
}

const PinholeCamera::Parameters &PinholeCamera::getParameters(void) const {
    return mParameters;
}

void PinholeCamera::projectPoints(const std::vector<cv::Point3f> &objectPoints,
                                  const cv::Mat &rvec, const cv::Mat &tvec,
                                  std::vector<cv::Point2f> &imagePoints, bool fix) const {
    // project 3D object points to the image plane
    imagePoints.reserve(objectPoints.size());

    // double
    cv::Mat R0;
    cv::Rodrigues(rvec, R0);

    Eigen::MatrixXd R(3, 3);
    R << R0.at<double>(0, 0), R0.at<double>(0, 1), R0.at<double>(0, 2),
            R0.at<double>(1, 0), R0.at<double>(1, 1), R0.at<double>(1, 2),
            R0.at<double>(2, 0), R0.at<double>(2, 1), R0.at<double>(2, 2);

    Eigen::Vector3d t;
    t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        const cv::Point3f &objectPoint = objectPoints.at(i);

        // Rotate and translate
        Eigen::Vector3d P;
        P << objectPoint.x, objectPoint.y, objectPoint.z;

        P = R * P + t;

        Eigen::Vector2d p;
        spaceToPlane(P, p, fix);

        imagePoints.push_back(cv::Point2f(p(0), p(1)));
    }
}