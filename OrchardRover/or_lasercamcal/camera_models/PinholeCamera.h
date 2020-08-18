#ifndef PINHOLECAMERA_H
#define PINHOLECAMERA_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include <Eigen/Core>
#include <ceres/rotation.h>

class PinholeCamera {
public:
    class Parameters {
    public:
        Parameters();

        double k1(void) const;

        double k2(void) const;

        double p1(void) const;

        double p2(void) const;

        double fx(void) const;

        double fy(void) const;

        double cx(void) const;

        double cy(void) const;

        bool readFromYamlFile(const std::string &filename);

    private:
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;
    };

    PinholeCamera(const Parameters &params);

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p, bool fix = false) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p,
                      Eigen::Matrix<double, 2, 3> &J) const;
    //%output p
    //%output J

    void undistToPlane(const Eigen::Vector2d &p_u, Eigen::Vector2d &p) const;
    //%output p


    void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const;

    void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u,
                    Eigen::Matrix2d &J) const;

    void initUndistortMap(cv::Mat &map1, cv::Mat &map2,
                          double fScale = 1.0) const;

    cv::Mat initUndistortRectifyMap(
            cv::Mat &map1, cv::Mat &map2, float fx = -1.0f, float fy = -1.0f,
            cv::Size imageSize = cv::Size(0, 0), float cx = -1.0f, float cy = -1.0f,
            cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;


    const Parameters &getParameters(void) const;

    void projectPoints(const std::vector<cv::Point3f> &objectPoints,
                       const cv::Mat &rvec, const cv::Mat &tvec,
                       std::vector<cv::Point2f> &imagePoints, bool fix = false) const;

private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

typedef std::shared_ptr<PinholeCamera> CameraPtr;

#endif
