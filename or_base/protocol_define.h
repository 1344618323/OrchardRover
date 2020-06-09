//
// Created by cxn on 19-2-20.
//

#ifndef PROJECT_PROTOCOL_DEFINE_H
#define PROJECT_PROTOCOL_DEFINE_H

namespace leonard_serial_common {
/*************传输协议*************/
//  帧头(0xA5) -> pack_len -> data_id -> data_content -> CRC32(总数据包的校验)
//  底盘信息 0x01
//  底盘命令 0x03

#define SOF 0xA5
#define HEADER_LEN sizeof(FrameHeader)
#define CMD_LEN 1
#define CRC_DATA_LEN 4
#define PROTOCAL_FRAME_MAX_SIZE 1024
#define UART_BUFF_SIZE 1024
#define PACK_MAX_SIZE 200

#define CMD_PUSH_CHASSIS_INFO (0X01u)
#define CMD_SET_CHASSIS_SPEED (0X03u)
#define CMD_SET_CAM_ANGLE (0x05u)
#define CMD_PUSH_GPS_INFO (0X07u)

    typedef enum {
        STEP_HEADER_SOF = 0,
        STEP_LENGTH_LOW = 1,
        STEP_LENGTH_HIGH = 2,
        STEP_DATA_CRC16 = 3,
    } UnpackStep;

#pragma pack(push, 1)
// 对齐系数为1

    typedef struct {
        uint8_t sof;
        uint16_t pack_len;
    } FrameHeader;

    typedef struct {
        int16_t gyro_angle;
        int32_t position_x_mm;
        int32_t position_y_mm;
        // int16_t gyro_rate;
        // int16_t angle_deg;
        // int16_t v_x_mm;
        // int16_t v_y_mm;
    } cmd_chassis_info;

    typedef struct {
        int16_t vx;
        int16_t vy;
        int16_t vw;
    } cmd_chassis_speed;

    typedef struct {
        int32_t latitude;                //纬度 分扩大100000倍,实际要除以100000（我改成了1000000）
        int32_t longitude;                //经度 分扩大100000倍,实际要除以100000（我改成了1000000）
        int16_t altitude;                //海拔高度,放大了10倍,实际除以10.单位:0.1m
        uint8_t fixmode;                    //定位类型:0,没有定位; 1,有定位
        uint16_t pdop;                    //位置精度因子 0~500,对应实际值0~50.0
    } cmd_gps_info;

#pragma pack(pop)
// 取消 #pragma pack(push, 1) 的作用

} // namespace leonard_serial_common

#endif //PROJECT_PROTOCOL_DEFINE_H
