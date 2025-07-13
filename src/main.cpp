/**
 * @file main
 * @author mzy (mzy8329@163.com)
 * @brief 通信包，用来与stm32通信，接口为两个topic
 * @version 0.1
 *
 * @copyright Copyright (c) 2022
 *
 */


#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>


#include "motor_serial/motor_data.h"
#include "motor_serial/motor_ctrl.h"


#define HEAD_LENGTH 2
#define PAYLOAD_LENGTH 13
#define BAG_LENGTH (HEAD_LENGTH + PAYLOAD_LENGTH)

uint8_t HEADER[2] = { 0x44, 0x22 };

/**
 * @brief 接受包，接受motor反馈信息
 *
 */
typedef union
{
    uint8_t data[BAG_LENGTH];
    struct
    {
        uint8_t header[HEAD_LENGTH];
        union
        {
            uint8_t payload[PAYLOAD_LENGTH];
            struct
            {
                uint8_t id;
                float angle_fdb;
                float rpm_fdb;
                float torque_fdb;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) RECV_Bag_u;

/**
 * @brief 发送包，发送控制信息
 *
 */
typedef union
{
    uint8_t data[BAG_LENGTH];
    struct
    {
        uint8_t header[HEAD_LENGTH];
        union
        {
            uint8_t payload[PAYLOAD_LENGTH];
            struct
            {
                uint8_t id;
                float angle_ref;
                float rpm_ref;
                float current_ref;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) SEND_Bag_u;



motor_serial::motor_data  motorData;
ros::Publisher motorData_pub;
serial::Serial serialPort;



/**
 * @brief 初始化串口Sp
 * @param sp
 * @param portName 串口名，默认为"/dev/ttyUSB0"
 * @param baudrate 波特率，默认为115200
 * @param timeOut  传输延时，默认为2
 * @return 成功返回1,否则返回0
 */
int SerialPort_INIT(serial::Serial* sp, char* portName = "/dev/ttyUSB1", int baudrate = 115200, int timeOut = 2) {
    serial::Timeout to = serial::Timeout::simpleTimeout(timeOut);
    sp->setPort(portName);
    sp->setBaudrate(baudrate);
    sp->setTimeout(to);
    try {
        sp->open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return 0;
    }

    if (sp->isOpen()) {
        ROS_INFO_STREAM("Port is opened.");
    }
    else {
        return 0;
    }
    return 1;
}


/**
 * @brief 收到控制信号后，将其传给下位机
 * @param ctrl_msg
 */
void ctrlDataCallback(const motor_serial::motor_ctrl::ConstPtr& ctrl_msg) {
    SEND_Bag_u tempBag;
    tempBag.header[0] = HEADER[0];
    tempBag.header[1] = HEADER[1];

    tempBag.id = ctrl_msg->id;
    tempBag.angle_ref = ctrl_msg->angle_ref;
    tempBag.rpm_ref = ctrl_msg->rpm_ref;
    tempBag.current_ref = ctrl_msg->current_ref;

    serialPort.write(tempBag.data, BAG_LENGTH);
    serialPort.write(tempBag.data, BAG_LENGTH);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_serial");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);

    ros::Subscriber ctrlData_sub = nh.subscribe("ctrl_data", 10, ctrlDataCallback);
    motorData_pub = nh.advertise<motor_serial::motor_data>("motor_data", 10);


    if (!SerialPort_INIT(&serialPort)) {
        std::cout << 1 << std::endl;
        return  -1;
    }

    while (ros::ok()) {
        //获取缓冲区内的字节数，直至其长度大于等于两个包的长度
        size_t recv_len = 0;
        for (recv_len = 0; recv_len < 2 * BAG_LENGTH; recv_len = serialPort.available());

        //读取缓存区中数据
        uint8_t RxBuffer[2 * BAG_LENGTH];
        recv_len = serialPort.read(RxBuffer, 2 * BAG_LENGTH);

        //将电机反馈值发送出去
        for (int i = 0; i < 2 * BAG_LENGTH; ++i) {
            RECV_Bag_u tempBag = *(RECV_Bag_u*)(void*)(&(RxBuffer[i]));
            if (tempBag.header[0] == HEADER[0] && tempBag.header[1] == HEADER[1]) {
                motorData.id = tempBag.id;
                motorData.angle_fdb = tempBag.angle_fdb;
                motorData.rpm_fdb = tempBag.rpm_fdb;
                motorData.torque_fdb = tempBag.torque_fdb;
                motorData_pub.publish(motorData);
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    //关闭串口
    serialPort.close();

    return 0;
}