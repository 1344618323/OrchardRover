//
// Created by cxn on 19-2-19.
//
#include "serial_comm_node.h"

namespace leonard_serial_common
{
SerialComNode::SerialComNode(std::string name)
{
    CHECK(Init()) << "Module " << name << " initialized failed!";
}

bool SerialComNode::Init()
{
    int baud_rate = 115200;
    std::string serial_port;
    nh_.param<std::string>("or_base/serial_port", serial_port, "/dev/or_7523_serial");
    hardware_device_ = std::make_shared<SerialDevice>(serial_port, baud_rate);

    if (!hardware_device_->Init())
    {
        LOG_ERROR << "Can not open device. ";
        return false;
    }

    is_open_ = true;
    stop_receive_ = false;
    stop_send_ = false;
    total_length_ = 0;
    free_length_ = UART_BUFF_SIZE;

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_link";

    receive_loop_thread_ = new std::thread(boost::bind(&SerialComNode::ReceivePackLoop, this));
    send_loop_thread_ = new std::thread(boost::bind(&SerialComNode::SendPackLoop, this));

    //ros publisher
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 30);

    //ros subscriber
    sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &SerialComNode::ChassisSpeedCtrlCallback, this);

    return true;
}

SerialComNode::~SerialComNode()
{
    if (receive_loop_thread_ != nullptr)
    {
        stop_receive_ = true;
        receive_loop_thread_->join();
        delete receive_loop_thread_;
    }
    if (send_loop_thread_ != nullptr)
    {
        stop_send_ = true;
        send_loop_thread_->join();
        delete send_loop_thread_;
    }
    is_open_ = false;
}

/*************************** Call back ****************************/
void SerialComNode::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel)
{
    cmd_chassis_speed chassis_speed;

    chassis_speed.vx = vel->linear.x * 1000;
    chassis_speed.vy = vel->linear.y * 1000;
    chassis_speed.vw = vel->angular.z * 1800.0 / M_PI;

    if (!SendData((uint8_t *)&chassis_speed, sizeof(cmd_chassis_speed), CMD_SET_CHASSIS_SPEED))
    {
        LOG_WARNING << "Overflow in Chassis CB";
    }
}

/*************************** Receive DATA ****************************/
void SerialComNode::ReceivePackLoop()
{
    while (is_open_ && !stop_receive_ && ros::ok())
    {
        usleep(1);
        int32_t read_buff_index = 0;
        int32_t read_len = hardware_device_->Read(rx_buf_, UART_BUFF_SIZE);

        //            if(read_len>0) {
        //                while (read_len--) {
        //                    uint8_t read_byte = rx_buf_[read_buff_index++];
        //                    std::cout<<read_byte<<" ";
        //                }
        //                std::cout<<std::endl;
        //            }

        if (read_len > 0)
        {
            while (read_len--)
            {
                uint8_t read_byte = rx_buf_[read_buff_index++];
                switch (unpack_step_e_)
                {
                case STEP_HEADER_SOF:
                {
                    if (read_byte == SOF)
                    {
                        protocol_packet_[index_++] = read_byte;
                        unpack_step_e_ = STEP_LENGTH_LOW;
                    }
                    else
                    {
                        index_ = 0;
                    }
                }
                break;
                case STEP_LENGTH_LOW:
                {
                    header_.pack_len = read_byte;
                    protocol_packet_[index_++] = read_byte;
                    unpack_step_e_ = STEP_LENGTH_HIGH;
                }
                break;
                case STEP_LENGTH_HIGH:
                {
                    header_.pack_len |= (read_byte << 8);
                    protocol_packet_[index_++] = read_byte;
                    if (header_.pack_len < (PROTOCAL_FRAME_MAX_SIZE))
                    {
                        unpack_step_e_ = STEP_DATA_CRC16;
                    }
                    else
                    {
                        unpack_step_e_ = STEP_HEADER_SOF;
                        index_ = 0;
                    }
                }
                break;

                case STEP_DATA_CRC16:
                {
                    if (index_ < header_.pack_len)
                    {
                        protocol_packet_[index_++] = read_byte;
                    }
                    else if (index_ > header_.pack_len)
                    {
                        unpack_step_e_ = STEP_HEADER_SOF;
                        index_ = 0;
                    }
                    if (index_ == header_.pack_len)
                    {
                        unpack_step_e_ = STEP_HEADER_SOF;
                        index_ = 0;
                        if (CRCTailCheck(protocol_packet_, header_.pack_len))
                        {
                            DataHandle();
                        }
                    }
                }
                break;
                default:
                {
                    unpack_step_e_ = STEP_HEADER_SOF;
                    index_ = 0;
                }
                break;
                }
            }
        }
    }
}

void SerialComNode::DataHandle()
{
    std::lock_guard<std::mutex> guard(mutex_receive_);
    auto *p_header = (FrameHeader *)protocol_packet_;
    uint16_t data_length = p_header->pack_len - HEADER_LEN - CMD_LEN - CRC_DATA_LEN;
    uint8_t cmd_id = *(uint16_t *)(protocol_packet_ + HEADER_LEN);
    uint8_t *data_addr = protocol_packet_ + HEADER_LEN + CMD_LEN;

    switch (cmd_id)
    {
    case CMD_PUSH_CHASSIS_INFO:
    {
        ros::Time current_time = ros::Time::now();

        memcpy(&chassis_info_, data_addr, data_length);
        odom_msg_.header.stamp = current_time;
        odom_msg_.pose.pose.position.x = chassis_info_.position_x_mm / 1000.;
        odom_msg_.pose.pose.position.y = chassis_info_.position_y_mm / 1000.;
        odom_msg_.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info_.gyro_angle / 1800.0 * M_PI);
        odom_msg_.pose.pose.orientation = q;
        // odom_msg_.twist.twist.linear.x = chassis_info_.v_x_mm / 1000.0;
        // odom_msg_.twist.twist.linear.y = chassis_info_.v_y_mm / 1000.0;
        // odom_msg_.twist.twist.angular.z = chassis_info_.gyro_rate / 1800.0 * M_PI;
        odom_pub_.publish(odom_msg_);

        odom_tf_.header.stamp = current_time;
        odom_tf_.transform.translation.x = chassis_info_.position_x_mm / 1000.;
        odom_tf_.transform.translation.y = chassis_info_.position_y_mm / 1000.;
        odom_tf_.transform.translation.z = 0.0;
        odom_tf_.transform.rotation = q;
        tf_broadcaster_.sendTransform(odom_tf_);

        // std::cout << "x:" << chassis_info_.position_x_mm << std::endl;
        // std::cout << "y:" << chassis_info_.position_y_mm << std::endl;
        // std::cout << "z:" << chassis_info_.gyro_angle << std::endl;
    }
    break;

    default:
        break;
    }
}

/*************************** Send DATA ****************************/
void SerialComNode::SendPackLoop()
{
    while (is_open_ && !stop_send_ && ros::ok())
    {
        if (total_length_ > 0)
        {
            mutex_send_.lock();

            hardware_device_->Write(tx_buf_, total_length_);
            total_length_ = 0;
            free_length_ = UART_BUFF_SIZE;

            mutex_send_.unlock();
        }
        else
        {
            usleep(100);
        }
    }
}

bool SerialComNode::SendData(uint8_t *data, uint16_t len, uint8_t cmd_id)
{
    std::unique_lock<std::mutex> lock(mutex_pack_);
    uint8_t pack[PACK_MAX_SIZE];
    uint16_t pack_length = HEADER_LEN + CMD_LEN + len + CRC_DATA_LEN;
    ProtocolFillPack(data, pack, len, cmd_id);
    if (pack_length <= free_length_)
    {
        memcpy(tx_buf_ + total_length_, pack, pack_length);
        free_length_ -= pack_length;
        total_length_ += pack_length;
        return true;
    }
    else
    {
        return false;
    }
}

void SerialComNode::ProtocolFillPack(uint8_t *topack_data,
                                   uint8_t *packed_data,
                                   uint16_t len,
                                   uint8_t cmd_id)
{
    FrameHeader *p_header = (FrameHeader *)packed_data;
    uint16_t pack_length = HEADER_LEN + CMD_LEN + len + CRC_DATA_LEN;

    p_header->sof = SOF;
    p_header->pack_len = pack_length;

    memcpy(packed_data + HEADER_LEN, (uint8_t *)&cmd_id, CMD_LEN);
    memcpy(packed_data + HEADER_LEN + CMD_LEN, topack_data, len);

    uint32_t crc_data = CRC32Calc(packed_data, pack_length - CRC_DATA_LEN);
    memcpy(packed_data + len + HEADER_LEN + CMD_LEN, &crc_data, CRC_DATA_LEN);
}

/*************************** CRC Calculationns ****************************/
uint16_t SerialComNode::CRC16Update(uint16_t crc, uint8_t ch)
{
    uint16_t tmp;
    uint16_t msg;

    msg = 0x00ff & static_cast<uint16_t>(ch);
    tmp = crc ^ msg;
    crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

    return crc;
}

uint32_t SerialComNode::CRC32Update(uint32_t crc, uint8_t ch)
{
    uint32_t tmp;
    uint32_t msg;

    msg = 0x000000ffL & static_cast<uint32_t>(ch);
    tmp = crc ^ msg;
    crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
    return crc;
}

uint16_t SerialComNode::CRC16Calc(const uint8_t *data_ptr, size_t length)
{
    size_t i;
    uint16_t crc = CRC_INIT;

    for (i = 0; i < length; i++)
    {
        crc = CRC16Update(crc, data_ptr[i]);
    }

    return crc;
}

uint32_t SerialComNode::CRC32Calc(const uint8_t *data_ptr, size_t length)
{
    size_t i;
    uint32_t crc = CRC_INIT;

    for (i = 0; i < length; i++)
    {
        crc = CRC32Update(crc, data_ptr[i]);
    }

    return crc;
}

bool SerialComNode::CRCHeadCheck(uint8_t *data_ptr, size_t length)
{
    if (CRC16Calc(data_ptr, length) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool SerialComNode::CRCTailCheck(uint8_t *data_ptr, size_t length)
{
    if (CRC32Calc(data_ptr, length) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

} // namespace leonard_serial_common

int main(int argc, char **argv)
{
    leonard_serial_common::GLogWrapper gLogWrapper(argv[0]);
    ros::init(argc, argv, "or_base_node");
    leonard_serial_common::SerialComNode serial_common_node("or_base_node");
    ros::spin();
    ros::waitForShutdown();
    return 0;
}
