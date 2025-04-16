#ifndef ROBOT_MESSAGE_H_
#define ROBOT_MESSAGE_H_

#define MAXBUFFERSIZE 200
#define SENDBUFFERSIZE 13

typedef union {
	// 16位读取数据
	short val;
	// 8位发送数据
	unsigned char data[2];
} MessageData;

bool Serial_Init();
unsigned char CRC8(unsigned char*, unsigned short);
bool Receive_ChassisData(double *, double *, double *, unsigned char *);
void Send_ChassisData(double, double, unsigned char);

#endif /* ROBOT_MESSAGE_H_ */