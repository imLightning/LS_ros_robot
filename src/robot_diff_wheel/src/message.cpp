/*
 * 通信协议
 * 消息格式
 *     消息头 数据长度 数据 控制位 校验位 消息尾
 *     55 aa length 00 00 00 00 ctrl crc8 0d 0a
 *         控制位
 *         无 无 左电机方向(10 正 01 反 00 停) 右电机方向
 *         00000000
 *
 */

#include "../include/robot_diff_wheel/message.h"
#include "serial/serial.h"

serial::Serial ros_ser;

const unsigned char HEADER[2] = {0x55, 0xaa};
const unsigned char ENDER[2] = {0x0d, 0x0a};

MessageData LeftTargetSpeed, RightTargetSpeed, LeftMotorSpeed, RightMotorSpeed, Angle;
unsigned char sendBuffer[SENDBUFFERSIZE], tempBuffer[MAXBUFFERSIZE];

// 计算八位循环冗余校验
unsigned char CRC8(unsigned char *arr, unsigned short len)
{
	unsigned char crc = 0, i;
	while (len--)
	{
		crc ^= *arr++;
		for (i = 0; i < 8; i++)
		{
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}

void Send_ChassisData(double leftTargetSpeed, double rightTargetSpeed, unsigned char control)
{

	int i = 0, len = 0x07;

	LeftTargetSpeed.val = (short)leftTargetSpeed;
	RightTargetSpeed.val = (short)rightTargetSpeed;

	// 设置消息
	sendBuffer[0] = HEADER[0];
	sendBuffer[1] = HEADER[1];
	sendBuffer[2] = len;
	for (i = 0; i < 2; i++)
	{
		sendBuffer[i + 3] = LeftTargetSpeed.data[i];
		sendBuffer[i + 5] = RightTargetSpeed.data[i];
		sendBuffer[i + 7] = 0;
	}
	sendBuffer[9] = control;
	sendBuffer[10] = CRC8(sendBuffer, 3 + len);
	sendBuffer[11] = ENDER[0];
	sendBuffer[12] = ENDER[1];

	ros_ser.write(sendBuffer, SENDBUFFERSIZE);
}

bool Receive_ChassisData(double *leftMotorSpeed, double *rightMotorSpeed, double *angle, unsigned char *control)
{

	// 读取串口数据
	int cnt = ros_ser.available();
	ros_ser.read(tempBuffer, cnt);

	// 定位消息头
	for (int i = 0; i < cnt; i++)
	{
		// 消息校验
		if (tempBuffer[i] == HEADER[0] && tempBuffer[i + 1] == HEADER[1])
		{
			short len = tempBuffer[i + 2];
			unsigned char check = CRC8(&tempBuffer[i], len + 3);
			if (check == tempBuffer[i + len + 3])
			{
				// 读取数据
				for (int j = 0; j < 2; j++)
				{
					LeftMotorSpeed.data[j] = tempBuffer[i + j + 3];
					RightMotorSpeed.data[j] = tempBuffer[i + j + 5];
					Angle.data[j] = tempBuffer[i + j + 7];
				}
				*control = tempBuffer[i + 9];
				*leftMotorSpeed = (double)LeftMotorSpeed.val;
				*rightMotorSpeed = (double)RightMotorSpeed.val;
				*angle = (double)(Angle.val / 88);
				return true;
			}
		}
	}
	return false;
}

bool Serial_Init()
{
	std::string port_name = "/dev/ttyUSB1";
	int baud_rate_ = 115200;

	/**打开设备**/
	try
	{
		ros_ser.setPort(port_name);
		ros_ser.setBaudrate(baud_rate_);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ros_ser.setTimeout(to);
		ros_ser.open();
	}
	catch (serial::IOException &e)
	{
		printf("Serial init error\n");
	}

	if (ros_ser.isOpen())
	{
		printf("Serial init success\n");
	}

	return true;
}