//
// Created by cxn on 2020/6/24.
//

#ifndef MAKING_MAPS_ICPFUN_H
#define MAKING_MAPS_ICPFUN_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <vector>

using namespace std;

namespace icp_function {
    Eigen::Vector2d IcpMean(vector<Eigen::Vector2d> &src, vector<Eigen::Vector2d> &q_src) {
        q_src = src;
        Eigen::Vector2d val;
        for (auto &i:src) {
            val += i;
        }
        val = val / src.size();
        for (auto &i:q_src) {
            i -= val;
        }
        return val;
    }

    double ICP2D(std::vector<Eigen::Vector2d> &src, vector<Eigen::Vector2d> &dst) {
        vector<Eigen::Vector2d> q_src;
        vector<Eigen::Vector2d> q_dst;
        Eigen::Vector2d q_mean_src = icp_function::IcpMean(src, q_src);
        Eigen::Vector2d q_mean_dst = icp_function::IcpMean(dst, q_dst);

        Eigen::Matrix2d W;
        W.setZero();
        for (int i = 0; i < q_src.size(); i++) {
            W += q_src[i] * q_dst[i].transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix2d V = svd.matrixV(), U = svd.matrixU();
        Eigen::Matrix2d R = U * (V.transpose());
        Eigen::Vector2d t = q_mean_src - R * q_mean_dst;

        double e = 0;
        for (int i = 0; i < q_src.size(); i++) {
            e += (src[i] - R * dst[i] - t).norm();
        }
        double error = e / double(src.size());
        cout << error << endl;
        return error;
    }

    void IcpTest() {
        vector<Eigen::Vector2d> src;
        vector<Eigen::Vector2d> dst;
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 4; j++) {
                src.emplace_back(i * 3 + 6.5, j * 3 + 5);
            }
        }
        dst.emplace_back(6.51557, 4.72049);
        dst.emplace_back(6.48971, 7.71136);
        dst.emplace_back(6.54979, 10.7426);
        dst.emplace_back(6.54465, 13.7616);
        dst.emplace_back(9.42711, 4.58894);
        dst.emplace_back(9.5004, 7.6019);
        dst.emplace_back(9.49539, 10.6225);
        dst.emplace_back(9.56029, 13.6675);
        dst.emplace_back(12.3852, 4.6121);
        dst.emplace_back(12.4341, 7.63565);
        dst.emplace_back(12.3126, 10.6401);
        dst.emplace_back(12.5647, 13.6694);
        dst.emplace_back(15.3754, 4.55501);
        dst.emplace_back(15.4511, 7.56979);
        dst.emplace_back(15.5436, 10.6915);
        dst.emplace_back(15.6002, 13.7607);
        dst.emplace_back(18.3913, 4.63267);
        dst.emplace_back(18.4197, 7.64599);
        dst.emplace_back(18.5938, 10.938);
        dst.emplace_back(18.5292, 13.9812);
        dst.emplace_back(21.3952, 4.63682);
        dst.emplace_back(21.3932, 7.66796);
        dst.emplace_back(21.5391, 10.7096);
        dst.emplace_back(21.7621, 13.7351);
        dst.emplace_back(24.3903, 4.60888);
        dst.emplace_back(24.4003, 7.6429);
        dst.emplace_back(24.5718, 10.6763);
        dst.emplace_back(24.6453, 13.7544);
        ICP2D(src, dst);
    }
};


#endif //MAKING_MAPS_ICPFUN_H
