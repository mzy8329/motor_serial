# motor_serial
> [下位机代码地址: M2006_Drive](https://github.com/mzy8329/M2006_Drive "M2006_Drive")

该仓库为大疆A板(STM32F427)和大疆C板(STM32F407)的上位机程序，支持上位机通过串口通信，将其转为Can信号并发送给电机，实现对电机的控制。

当前上位机使用ROS1进行通信，提供了两个ros topic用于读取电机数据和发布控制信息，分别为“motor_serial/motor_data”和“motor_serial/ctrl_data"

控制优先级: 位置 > 速度 > 电流。 如需使用速度控制，需要将期望位置设为“-1”（这确实并不严谨） ，如需使用电流控制，需将期望位置和期望速度设为“-1”