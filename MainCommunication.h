#pragma once
#pragma pack(1)

#ifndef MAINCOMMUNICATION___H

#define MAINCOMMUNICATION___H

#include "SerialPort.h"

#define SEND_PERIOD_MS 200

#define RECEIVE_PERIOD_MS 50

#define RECEIVE_TRY_MS 10

#define PACKET_HANGER_HEAD_BYTE 0x53

#define PACKET_HANGER_NEXT_BYTE 0x47

#define PACKET_HANGER_FRAME_HEAD_BYTE 0x01

#define PACKET_HANGER_LENGTH 0x13

#define PACKET_HANGER_MODE_MANUAL 0x01

#define PACKET_HANGER_MODE_NAVIGATION 0x02

#define PACKET_HANGER_MODE_LOCK 0x03

#define PACKET_HANGER_MODE_BIRDSEYE 0x04

#define PACKET_HANGER_MODE_FAVORITE 0x05

#define PACKET_HANGER_MODE_SCAN 0x07

#define PACKET_HANGER_SET_DRIFTCOMPENSATION 0x32

#define PACKET_HANGER_SET_RECORD 0x33

#define PACKET_HANGER_SET_POWER  0x34

#define PACKET_HANGER_SET_SENSIBILITY 0x35

#define PACKET_HANGER_SET_SENSOR 0x37

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_AZIMUTH_PLUS 0x01

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_AZIMUTH_MINUS 0x02

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_PITCH_PLUS 0x03

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_PITCH_MINUS 0x04

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_NONE 0x01

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_AUTOMATIC 0x02

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_MANUAL 0x03

#define PACKET_HANGER_SET_DRIFTCOMPENSATION_MEMORY 0x04

#define PACKET_HANGER_SET_RECORD_START 0x01

#define PACKET_HANGER_SET_RECORD_STOP 0x02

#define PACKET_HANGER_SET_POWER_ON 0x01

#define PACKET_HANGER_SET_POWER_OFF 0x02

#define PACKET_HANGER_SET_SENSIBILITY_SLOW 0x20

#define PACKET_HANGER_SET_SENSIBILITY_MIDDLE 0x80

#define PACKET_HANGER_SET_SENSIBILITY_FAST 0xFF

#define PACKET_HANGER_SET_SENSOR_CAMCORDER 0x01 

#define PACKET_HANGER_SET_SENSOR_MIDDLEWAVE 0x02

#define PACKET_HANGER_SET_SENSOR_SHORTWAVE 0x03

#define PACKET_HANGER_SET_SENSOR_CAMERA 0x04

#define PACKET_HANGER_SET_SENSOR_ULTRAVIOLET 0x05

#define PACKET_PREAMBLE_BYTE 0xFA

#define PACKET_BID_BYTE 0xFF

#define MID_FLIGHT_MODE 0x70

#define MID_FLIGHT_STATE 0x80

#define MID_OBJECT_INFORMATION 0x64 //100

#define MID_TRACK_INFORMATION 0x65 //101

#define MID_MODE_ACKNOWLEDGE 0x71

typedef void(*QueryObjectFunction)(float* pPositionX, float* pPositionY, float* pVelocityX, float* pVelocityY, unsigned char* pState);
typedef void(*QueryTrackFunction)(float* pCenterX, float* pCenterY, float* pOrientationX, float* pOrientationY, float* pOrientationZ, unsigned char* pState);
typedef void(*AssignModeFunction)(unsigned char mode);
typedef void(*AssignStateFunction)(float altitude, float attitudeX, float attitudeY, float attitudeZ);

typedef void(*AcquireHangerInformationFunction)(unsigned char mode, unsigned char state, float azimuth, float pitch, unsigned char light);

class MainCommunication
{
    private:
		bool mIsQueryAndSendThreadRunning;
		bool mIsReceiveAndAssignThreadRunning;
		SerialPort mSerialPort;
		CRITICAL_SECTION mCriticalSectionWrite;
		QueryObjectFunction mQueryObjectFunction;
		QueryTrackFunction mQueryTrackFunction;
		AssignModeFunction mAssignModeFunction;
		AssignStateFunction mAssignStateFunction;
		AcquireHangerInformationFunction mAcquireHangerInformationFunction;
		unsigned long writeBuffer(const unsigned char* pData, unsigned long dataLength);
		unsigned long sendPackage(unsigned char* pBuffer, unsigned int size);
	//	unsigned char computeCheckSum(unsigned char* inBuffer, int start, int end);
		unsigned long readBuffer(unsigned char* pData, unsigned long dataLength);



   
    public:
		MainCommunication();
		~MainCommunication();
		bool initializePipe();
		bool isPipeOpen();
		void finishPipe();
		unsigned long sendHangerMessage(unsigned char mode, unsigned short azimuth, unsigned short pitch, unsigned char set0, unsigned char parameter00, unsigned char parameter01, unsigned char parameter02, unsigned char parameter03, unsigned char set1, unsigned char parameter10, unsigned char parameter11, unsigned char parameter12, unsigned char parameter13);
		/*bool*/ unsigned long sendObjectInformation(float positionX, float positionY, float velocityX, float velocityY, unsigned char state);
		unsigned long sendTrackInformation(float centerX, float centerY, float orientationX, float orientationY, float orientationZ, unsigned char state);
		unsigned long sendModeAcknowledge();
		void setQueryObjectFunction(QueryObjectFunction pQueryObjectFunction);
		QueryObjectFunction getQueryObjectFunction();
		void setQueryTrackFunction(QueryTrackFunction pQueryTrackFunction);
		QueryTrackFunction getQueryTrackFunction();
		void startQueryAndSendThread();
		void stopQueryAndSendThread();
		bool isQueryAndSendThreadRunning();
		void setAssignModeFunction(AssignModeFunction pAssignModeFunction);
		AssignModeFunction getAssignModeFunction();
		void setAssignStateFunction(AssignStateFunction pAssignStateFunction);
		AssignStateFunction getAssignStateFunction();
		void setAcquireHangerInformationFunction(AcquireHangerInformationFunction pAcquireHangerInformationFunction);
		AcquireHangerInformationFunction getAcquireHangerInformationFunction();
		void startReceiveAndAssignThread();
		void stopReceiveAndAssignThread();
		bool isReceiveAndAssignThreadRunning();
		unsigned long readOneByteWithRetry(unsigned char * pByte);
	//	unsigned long writeBuffer(const unsigned char* pData, unsigned long dataLength);


};


#endif
