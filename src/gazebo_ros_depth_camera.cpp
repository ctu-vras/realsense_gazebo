// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2013 Open Source Robotics Foundation
// SPDX-FileCopyrightText: Czech Technical University in Prague
//
// Original file from https://github.com/ros-simulation/gazebo_ros_pkgs edited by Martin Pecka:
// - rewrote the file to only handle depth data

/**
 * \file
 * \brief gazebo_ros plugin handling depth cameras so that only their depth data are published to ROS.
 * \author Martin Pecka
 */

#include <limits>
#include <string>

#include <gazebo/plugins/DepthCameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <sensor_msgs/image_encodings.h>

namespace cras
{

class GazeboRosDepthCamera : public gazebo::DepthCameraPlugin, gazebo::GazeboRosCameraUtils
{
public:
    GazeboRosDepthCamera() = default;

    ~GazeboRosDepthCamera() override
    {
        ROS_DEBUG_STREAM_NAMED("depth_camera", "Unloaded");
    }

    void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf) override
    {
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM_NAMED("depth_camera", "A ROS node for Gazebo has not been initialized, unable to load "
                "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
            return;
        }

        gazebo::DepthCameraPlugin::Load(_parent, _sdf);

        this->parentSensor_ = this->parentSensor;
        this->width_ = this->width;
        this->height_ = this->height;
        this->depth_ = this->depth;
        this->format_ = "L16";  // just to workaround GazeboRosCameraUtils::Init() unaware of FLOAT32
        this->camera_ = this->depthCamera;

        if (!_sdf->HasElement("useDepth16UC1Format"))
            this->use_depth_image_16UC1_format_ = false;
        else
            this->use_depth_image_16UC1_format_ = _sdf->GetElement("useDepth16UC1Format")->Get<bool>();

        gazebo::GazeboRosCameraUtils::Load(_parent, _sdf);

        ROS_INFO_NAMED("depth_camera", "Camera Plugin (ns = %s) uses %s depth format.",
                       gazebo::GetRobotNamespace(_parent, _sdf, "Camera").c_str(),
                       this->use_depth_image_16UC1_format_ ? "16UC1" : "32FC1");
    }

protected:
    void PutDepthCameraData(const float* _src, gazebo::common::Time& last_update_time)
    {
        this->sensor_update_time_ = last_update_time;
        this->PutDepthCameraData(_src);
    }

    void PutDepthCameraData(const float* _src)
    {
        if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
            return;

        /// don't bother if there are no subscribers
        if ((*this->image_connect_count_) > 0)
        {
            boost::mutex::scoped_lock lock(this->lock_);

            // copy data into image
            this->image_msg_.header.frame_id = this->frame_name_;
            this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
            this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

            // copy from src to image_msg_
            this->FillDepthImageHelper(this->image_msg_, this->height_, this->width_, _src);

            // publish to ros
            this->image_pub_.publish(this->image_msg_);
        }
    }

    void FillDepthImageHelper(sensor_msgs::Image& image_msg, uint32_t rows, uint32_t cols, const float* data) const
    {
        image_msg.height = rows;
        image_msg.width = cols;
        image_msg.is_bigendian = 0;

        if (!this->use_depth_image_16UC1_format_)
        {
            image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            image_msg.step = sizeof(float) * cols;
            image_msg.data.resize(rows * image_msg.step);
            memcpy(&image_msg.data[0], data, image_msg.step * rows);
            return;
        }

        image_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image_msg.step = sizeof(uint16_t) * cols;
        image_msg.data.resize(rows * image_msg.step);
        auto dest = reinterpret_cast<uint16_t*>(&image_msg.data[0]);

        int index = 0;

        // convert float depth to 16UC1
        for (uint32_t j = 0; j < rows; j++)
        {
            for (uint32_t i = 0; i < cols; i++)
            {
                float depth_mm = data[index++] * 1000.0f;

                uint16_t depth_int;
                if (ROS_UNLIKELY((std::isnan(depth_mm) || depth_mm < 0)))
                    depth_int = 0;
                else if (ROS_UNLIKELY((depth_mm > std::numeric_limits<uint16_t>::max())))
                    depth_int = std::numeric_limits<uint16_t>::max();
                else
                    depth_int = static_cast<uint16_t>(depth_mm);

                dest[i + j * cols] = depth_int;
            }
        }
    }

    void OnNewDepthFrame(const float* _image, unsigned int /*_width*/, unsigned int /*_height*/,
                         unsigned int /*_depth*/, const std::string& /*_format*/) override
    {
        auto sensor_update_time = this->parentSensor_->LastMeasurementTime();

        if (!this->parentSensor->IsActive())
        {
            if ((*this->image_connect_count_) > 0)
                // do this first so there's chance for sensor to run once after activated
                this->parentSensor->SetActive(true);
            return;
        }

        if ((*this->image_connect_count_) == 0)
            return;

        if (sensor_update_time < this->last_update_time_)
        {
            ROS_WARN_NAMED("depth_camera", "Negative sensor update time difference detected.");
            this->last_update_time_ = sensor_update_time;
        }

        if (sensor_update_time - this->last_update_time_ < this->update_period_)
            return;

        this->PutDepthCameraData(_image, sensor_update_time);
        this->PublishCameraInfo(sensor_update_time);
        this->last_update_time_ = sensor_update_time;
    }

    bool use_depth_image_16UC1_format_ {true};
};

}

GZ_REGISTER_SENSOR_PLUGIN(cras::GazeboRosDepthCamera)
