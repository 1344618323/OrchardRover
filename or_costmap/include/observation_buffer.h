//
// Created by cxn on 2020/7/2.
//

#ifndef OR_COSTMAP_OBSERVATIONBUFFER_H
#define OR_COSTMAP_OBSERVATIONBUFFER_H

#include <vector>
#include <list>
#include <string>
#include <mutex>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "observation.h"

namespace or_costmap {
    /**
 * @class ObservationBuffer
 * @brief Takes in point clouds from sensors, transforms them to the desired frame, and stores them
 */
    class ObservationBuffer {
    public:
        /**
         * @brief  Constructs an observation buffer
         * @param  topic_name The topic of the observations, used as an identifier for error and warning messages
         * @param  observation_keep_time Defines the persistence of observations in seconds, 0 means only keep the latest
         * @param  expected_update_rate How often this buffer is expected to be updated, 0 means there is no limit
         * @param  min_obstacle_height The minimum height of a hitpoint to be considered legal
         * @param  max_obstacle_height The minimum height of a hitpoint to be considered legal
         * @param  obstacle_range The range to which the sensor should be trusted for inserting obstacles
         * @param  raytrace_range The range to which the sensor should be trusted for raytracing to clear out space
         * @param  tf A reference to a TransformListener
         * @param  global_frame The frame to transform PointClouds into
         * @param  sensor_frame The frame of the origin of the sensor, can be left blank to be read from the messages
         * @param  tf_tolerance The amount of time to wait for a transform to be available when setting a new global frame
         */
        ObservationBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate,
                          double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                          double raytrace_range, tf::TransformListener &tf, std::string global_frame,
                          std::string sensor_frame, double tf_tolerance);

        /**
         * @brief  Destructor... cleans up
         */
        ~ObservationBuffer();

        /**
         * @brief Sets the global frame of an observation buffer. This will
         * transform all the currently cached observations to the new global
         * frame
         * @param new_global_frame The name of the new global frame.
         * @return True if the operation succeeds, false otherwise
         * 更换global_frame_，相应的观测点的坐标也要变换，有 Tnew_old×Pold
         */
        bool SetGlobalFrame(const std::string new_global_frame);

        /**
         * @brief  Transforms a PointCloud to the global frame and buffers it
         * @param  cloud The cloud to be buffered
         * 将ros格式的点云加入观测列表observation_list_
         */
        void BufferCloud(const sensor_msgs::PointCloud2 &cloud);

        /**
         * @brief  Transforms a PointCloud to the global frame and buffers it
         * <b>Note: The burden is on the user to make sure the transform is available... ie they should use a MessageNotifier</b>
         * @param  cloud The cloud to be buffered
         * 将pcl格式的点云加入观测列表
         */
        void BufferCloud(const pcl::PointCloud <pcl::PointXYZ> &cloud);

        /**
         * @brief  Pushes copies of all current observations onto the end of the vector passed in
         * @param  observations The vector to be filled
         * 获取观测列表中的所有点云信息
         */
        void GetObservations(std::vector <Observation> &observations);

        /**
         * @brief  Check if the observation buffer is being update at its expected rate
         * @return True if it is being updated at the expected rate, false otherwise
         * 检测观测列表是不是在更新
         */
        bool IsCurrent() const;

        /**
         * @brief  Lock the observation buffer
         */
        inline void Lock() {
            lock_.lock();
        }

        /**
         * @brief  Lock the observation buffer
         */
        inline void Unlock() {
            lock_.unlock();
        }

        /**
         * @brief Reset last updated timestamp
         */
        void ResetLastUpdated();

    private:
        /**
         * @brief  Removes any stale observations from the buffer list
         * 随着算法的运行，会将过时的观测信息剔除
         */
        void PurgeStaleObservations();

        tf::TransformListener &tf_;
        const ros::Duration observation_keep_time_;
        const ros::Duration expected_update_rate_;
        ros::Time last_updated_;
        std::string global_frame_;
        std::string sensor_frame_;
        std::list <Observation> observation_list_;
        std::string topic_name_;
        double min_obstacle_height_, max_obstacle_height_;
        std::recursive_mutex lock_;
        double obstacle_range_, raytrace_range_;
        double tf_tolerance_;
    };
}
#endif //OR_COSTMAP_OBSERVATIONBUFFER_H