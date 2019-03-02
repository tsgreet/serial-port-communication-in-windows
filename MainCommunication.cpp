#include "stdafx.h"
#include "MainCommunication.h"

struct HangerMessage {
	unsigned char preamble : 8;
	unsigned char next : 8;
	unsigned char len : 8;
	unsigned char mode : 8;
	unsigned short azimuth : 16;
	unsigned short pitch : 16;
	unsigned char set0 : 8;
	unsigned char parameter00 : 8;
	unsigned char parameter01 : 8;
	unsigned char parameter02 : 8;
	unsigned char parameter03 : 8;
	unsigned char set1 : 8;
	unsigned char parameter10 : 8;
	unsigned char parameter11 : 8;
	unsigned char parameter12 : 8;
	unsigned char parameter13 : 8;
	unsigned char checkSum : 8;
};

struct HangerInformationMessage {
	unsigned char preamble : 8;
	unsigned char next : 8;
	unsigned char len : 8;
	unsigned char frameToken : 8;
	unsigned char mode : 8;
	unsigned char state : 8;
	float azimuth;
	float pitch;
	unsigned char light : 8;
	unsigned char reserved[21];
	unsigned char synchronized : 8;
};


struct ObjectMessage {
//	ObjectMessage() : preamble(0xFA), bid(0xFF), mid(100), len(24), reserved({ 0,0,0,0,0,0 }) {}
	unsigned char preamble : 8;
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	float positionX;
	float positionY;
	float velocityX;
	float velocityY;
	unsigned char state : 8;
	unsigned char reserved[7];
	unsigned char checkSum;
};

struct TrackMessage {
	//	ObjectMessage() : preamble(0xFA), bid(0xFF), mid(100), len(24), reserved({ 0,0,0,0,0,0 }) {}
	unsigned char preamble : 8;
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	float centerX;
	float centerY;
	float orientationX;
	float orientationY;
	float orientationZ;
	unsigned char state : 8;
	unsigned char reserved[3];
	unsigned char checkSum;
};

struct ModeAcknowledgeMessage {
	//	ObjectMessage() : preamble(0xFA), bid(0xFF), mid(100), len(24), reserved({ 0,0,0,0,0,0 }) {}
	unsigned char preamble : 8;
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	unsigned char checkSum;
};

struct ModeMessage {
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	unsigned char mode : 8;
};

struct StateMessage {
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	float altitude;
	float attitudeX;
	float attitudeY;
	float attitudeZ;
};


template<typename T> T convertEndian(T in) {
	unsigned char* buffer = reinterpret_cast<unsigned char*>(&in);
	unsigned char length = sizeof(T);
	unsigned char store;
	for (int index = 0;index < (length / 2);index++) {
		store = *(buffer + index);
		*(buffer + index) = *(buffer + length - 1 - index);
		*(buffer + length - 1 - index) = store;
	}
	return in;
}

unsigned char computeByteSum(unsigned char * inBuffer, int start, int end)
{
	unsigned int sum = 0;
	unsigned char result;
	for (int index = start; index <= end; index++) {
		sum += *(inBuffer + index);
		sum = sum & (0xFF);
	}
	result = (unsigned char)sum;
	return result;
}


unsigned char /*MainCommunication::*/computeCheckSum(unsigned char * inBuffer, int start, int end)
{
	unsigned char result;
	/*
	unsigned int sum = 0;
	for (int index = start; index <= end; index++) {
		sum += *(inBuffer + index);
		sum = sum & (0xFF);
	}
	result = (unsigned char)sum;
	*/
	result = computeByteSum(inBuffer, start, end);
	result = (~result) + 1;
	return result;
}

DWORD CALLBACK ReceiveThreadProc(PVOID pvoid)
{
	unsigned char byte;
	int phase = 0;
	int stage = 0;
	int count = 0;
	unsigned char checkSum;
	unsigned char mid;
	unsigned char length;
	unsigned char* pPacket = NULL;
	AssignModeFunction pAssignMode = NULL;
	AssignStateFunction pAssignState = NULL;
	AcquireHangerInformationFunction pAcquireHangerInformationFunction = NULL;
	struct ModeMessage* pMode = NULL;
	struct StateMessage* pState = NULL;
	struct HangerInformationMessage* pHangerInformationMessage = NULL;
	MainCommunication* pThis = (MainCommunication*)pvoid;
	while (pThis->isReceiveAndAssignThreadRunning()) {
		if (pThis->isPipeOpen()) {
			if (pThis->readOneByteWithRetry(&byte)>0) {
				switch (phase) {
				    case 11:
						if (byte == PACKET_PREAMBLE_BYTE) {
							phase = 1;
						}
						else
						if (byte == PACKET_HANGER_HEAD_BYTE) {
							phase = 11;
						}
						else
						if (byte == PACKET_HANGER_NEXT_BYTE) {
							phase = 12;
							stage = 0;
						}
						else {
							phase = 0;
						}
						break;
					case 12:
						switch (stage) {
							case 0:
								length = byte;
								pPacket = new unsigned char[length-1];
								*pPacket = PACKET_HANGER_HEAD_BYTE;
								*(pPacket + 1) = PACKET_HANGER_NEXT_BYTE;
								count = 3;
								stage = 1;
								break;
							case 1:
								*(pPacket + (count++)) = byte;
								if (count == length) {
									stage = 2;
									checkSum = computeByteSum(pPacket, 0, length - 2);
								}
								break;
							case 2:
								if (checkSum == byte) {
									//package is correct
									if (*(pPacket + 3) == PACKET_HANGER_FRAME_HEAD_BYTE) {
										pAcquireHangerInformationFunction = pThis->getAcquireHangerInformationFunction();
										if (pAcquireHangerInformationFunction != NULL) {
											pHangerInformationMessage = reinterpret_cast<struct HangerInformationMessage*>(pPacket);
											pAcquireHangerInformationFunction(pHangerInformationMessage->mode, pHangerInformationMessage->state, pHangerInformationMessage->azimuth, pHangerInformationMessage->pitch, pHangerInformationMessage->light);
										}
									}
								}
								if (pPacket != NULL) {
									delete pPacket;
									pPacket = NULL;
								}
								phase = 0;
								stage = 0;
								break;
						}
						break;

				    case 0:
						if (byte == PACKET_HANGER_HEAD_BYTE) {
							phase = 11;
						}
						if (byte == PACKET_PREAMBLE_BYTE) {
							phase = 1;
						}
						break;
					case 1:
						if (byte == PACKET_HANGER_HEAD_BYTE) {
							phase = 11;
						}
						else
						if (byte == PACKET_PREAMBLE_BYTE) {
							phase = 1;
						}
						else if (byte == PACKET_BID_BYTE) {
							phase = 2;
						}
						else {
							phase = 0;
						}
						break;
					case 2:
						if (byte == PACKET_HANGER_HEAD_BYTE) {
							phase = 11;
						}
						else
						if (byte == PACKET_PREAMBLE_BYTE) {
							phase = 1;
						}
						else if ((byte == MID_FLIGHT_MODE) || (byte == MID_FLIGHT_STATE)) {
							phase = 3;
							mid = byte;
							stage = 0;
						}
						else {
							phase = 0;
						}
						break;
					case 3:
						switch (stage) {
						    case 0:
								length = byte;
								pPacket = new unsigned char[length+3];
								*pPacket = PACKET_BID_BYTE;
								*(pPacket + 1) = mid;
								*(pPacket + 2) = length;
								count = 0;
								stage = 1;
								break;

							case 1:
								*(pPacket + 3 + (count++)) = byte;
								if (count == length) {
									stage = 2;
									checkSum = computeCheckSum(pPacket, 0, length + 2);
								}
								break;

							case 2:
								if (checkSum == byte) {
									//package is correct
									//to do callback
									switch (mid) {
									    case MID_FLIGHT_MODE:
											pAssignMode = pThis->getAssignModeFunction();
											if (pAssignMode != NULL) {
												pMode = reinterpret_cast<struct ModeMessage*>(pPacket);
												pAssignMode(pMode->mode);
											}
											pThis->sendModeAcknowledge();
										    break;

										case MID_FLIGHT_STATE:
											pAssignState = pThis->getAssignStateFunction();
											if (pAssignState != NULL) {
												pState = reinterpret_cast<struct StateMessage*>(pPacket);
												pAssignState(pState->altitude,pState->attitudeX,pState->attitudeY,pState->attitudeZ);
											}
											break;
									}

								}
								if (pPacket != NULL) {
									delete pPacket;
									pPacket = NULL;
								}
								phase = 0;
								stage = 0;
								break;
						}
						break;
				}

			}
			else {
				break;
			}
		}
		else {
			break;
		}
		Sleep(RECEIVE_PERIOD_MS); //
	}
	
	pThis->stopReceiveAndAssignThread();
	return 0;
}

DWORD CALLBACK SendThreadProc(PVOID pvoid)
{
	MainCommunication* pThis = (MainCommunication*) pvoid;
	QueryObjectFunction fQueryObject=NULL;
	QueryTrackFunction fQueryTrack = NULL;
	while (pThis->isQueryAndSendThreadRunning()) {
		fQueryObject = pThis->getQueryObjectFunction();
		if (fQueryObject != NULL) {
			float positionX, positionY, velocityX, velocityY;
			unsigned char state;
			fQueryObject(&positionX, &positionY, &velocityX, &velocityY, &state);
			printf("thread fqueryobject positionX=%f, positionY=%f \n", positionX,positionY);
			pThis->sendObjectInformation(positionX, positionY, velocityX, velocityY, state);
		}
		fQueryTrack = pThis->getQueryTrackFunction();
		if (fQueryTrack != NULL) {
			float centerX, centerY, orientationX, orientationY, orientationZ;
			unsigned char state;
			fQueryTrack(&centerX, &centerY, &orientationX, &orientationY, &orientationZ, &state);
			printf("thread fqueryobject centerX=%f, centerY=%f \n", centerX, centerY);
			pThis->sendTrackInformation(centerX, centerY, orientationX, orientationY, orientationZ, state);
		}
		Sleep(SEND_PERIOD_MS);
	}
	return 0;
}



unsigned long MainCommunication::writeBuffer(const unsigned char * pData, unsigned long dataLength)
{
	unsigned long sentSize = 0;
	EnterCriticalSection(&mCriticalSectionWrite);
	sentSize = mSerialPort.writeBuffer(pData, dataLength);
	LeaveCriticalSection(&mCriticalSectionWrite);
	//	sentSize = sizeof(struct ObjectMessage);
	return sentSize;
}

unsigned long MainCommunication::sendPackage(unsigned char *pBuffer,unsigned int size) {
	unsigned long sentSize = 0;
	unsigned char checkSum = computeCheckSum(pBuffer, 1, size - 2);
	*(pBuffer+size-1) = checkSum;
	sentSize = writeBuffer(pBuffer, sizeof(struct ObjectMessage));
	return sentSize;
}



unsigned long MainCommunication::readBuffer(unsigned char * pData, unsigned long dataLength)
{
	unsigned long readSize = 0;
//	EnterCriticalSection(&mCriticalSectionWrite);
	readSize = mSerialPort.readBuffer(pData, dataLength);
//	LeaveCriticalSection(&mCriticalSectionWrite);
	//	sentSize = sizeof(struct ObjectMessage);
	return readSize;
	
}

MainCommunication::MainCommunication()
	: mIsQueryAndSendThreadRunning(false),mIsReceiveAndAssignThreadRunning(false),
	mQueryObjectFunction(NULL),mQueryTrackFunction(NULL),
	mAssignModeFunction(NULL), mAssignStateFunction(NULL)
{
//	mQueryObjectFunction = NULL;
	InitializeCriticalSection(&mCriticalSectionWrite);

}

MainCommunication::~MainCommunication()
{
	if (mSerialPort.isPortOpen()) {
		finishPipe();
	}
	mIsQueryAndSendThreadRunning = false;
	mIsReceiveAndAssignThreadRunning = false;
	mQueryObjectFunction = NULL;
	mQueryTrackFunction = NULL;
	mAssignModeFunction = NULL;
	mAssignStateFunction = NULL;
	DeleteCriticalSection(&mCriticalSectionWrite);
}

bool MainCommunication::initializePipe()
{
	return mSerialPort.openPort();
//	return false;
}

bool MainCommunication::isPipeOpen()
{
	return mSerialPort.isPortOpen();
}

void MainCommunication::finishPipe()
{
	EnterCriticalSection(&mCriticalSectionWrite); //prevents port closing if WriteBuffer is not complete
	mSerialPort.closePort();
	LeaveCriticalSection(&mCriticalSectionWrite);
}

unsigned long MainCommunication::sendHangerMessage(
	unsigned char mode, unsigned short azimuth, unsigned short pitch,
	unsigned char set0, unsigned char parameter00, unsigned char parameter01, unsigned char parameter02, unsigned char parameter03,
	unsigned char set1, unsigned char parameter10, unsigned char parameter11, unsigned char parameter12, unsigned char parameter13
)
{
	bool flag = true;
	unsigned long sentSize = 0;
	struct HangerMessage hangerMessage = {
		PACKET_HANGER_HEAD_BYTE, PACKET_HANGER_NEXT_BYTE, PACKET_HANGER_LENGTH, 
		mode,azimuth,pitch,
		set0,parameter00,parameter01,parameter02,parameter03,
		set1,parameter10,parameter11,parameter12,parameter13,
		0
	};

	unsigned char *p = reinterpret_cast<unsigned char *>(&hangerMessage);

	hangerMessage.checkSum = computeByteSum(p,0,sizeof(struct HangerMessage)-2);

	sentSize = writeBuffer(p, sizeof(struct HangerMessage));

	if (sentSize == 0) {
		flag = false;
	}
	return sentSize;
	//return flag;
}


/*bool*/ unsigned long MainCommunication::sendObjectInformation(float positionX, float positionY, float velocityX, float velocityY, unsigned char state)
{
	bool flag = true;
	unsigned long sentSize = 0;
	struct ObjectMessage objectMessage = {
		PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_OBJECT_INFORMATION, 24, convertEndian<float>(positionX),positionY,velocityX,velocityY,state,{ 0,0,0,0,0,0,0 },0
	};
	
	//	objectMessage.positionX = positionX;
	//	objectMessage.positionY = positionY;
	//	objectMessage.velocityX = velocityX;
	//	objectMessage.velocityY = velocityY;
	//	objectMessage.state = state;

	unsigned char *p = reinterpret_cast<unsigned char *>(&objectMessage);
	sentSize = sendPackage(p, sizeof(struct ObjectMessage));

	//	unsigned char checkSum = computeCheckSum(p, 1, sizeof(struct ObjectMessage)-2);
	//	objectMessage.checkSum = checkSum;
	//	sentSize = writeBuffer(p,sizeof(struct ObjectMessage));
	if (sentSize == 0) {
		flag = false;
	}
	return sentSize;
	//return flag;
}

/*bool*/ unsigned long MainCommunication::sendTrackInformation(float centerX, float centerY, float orientationX, float orientationY, float orientationZ, unsigned char state)
{
	bool flag = true;
	unsigned long sentSize = 0;
	struct TrackMessage trackMessage = {
		PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_TRACK_INFORMATION, 24, centerX,centerY,orientationX,orientationY,orientationZ,state,{ 0,0,0 },0
	};


	unsigned char *p = reinterpret_cast<unsigned char *>(&trackMessage);
	sentSize = sendPackage(p, sizeof(struct TrackMessage));

	
	if (sentSize == 0) {
		flag = false;
	}
	return sentSize;
	//return flag;
}

unsigned long MainCommunication::sendModeAcknowledge()
{
	struct ModeAcknowledgeMessage modeAcknowledgeMessage = {
		PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_MODE_ACKNOWLEDGE, 0,0
	};
	unsigned char *p = reinterpret_cast<unsigned char *>(&modeAcknowledgeMessage);
	unsigned long sentSize = sendPackage(p, sizeof(struct ModeAcknowledgeMessage));

	return sentSize;
}

void MainCommunication::setQueryObjectFunction(QueryObjectFunction pQueryObjectFunction)
{
	mQueryObjectFunction = pQueryObjectFunction;
}

QueryObjectFunction MainCommunication::getQueryObjectFunction()
{
	return mQueryObjectFunction;
}

void MainCommunication::setQueryTrackFunction(QueryTrackFunction pQueryTrackFunction)
{
	mQueryTrackFunction = pQueryTrackFunction;
}

QueryTrackFunction MainCommunication::getQueryTrackFunction()
{
	return mQueryTrackFunction;
}

void MainCommunication::startQueryAndSendThread()
{
	mIsQueryAndSendThreadRunning = true;
	DWORD dwThreadId;
	//	HANDLE hThread = CreateThread(NULL, 0, ThreadProc,  /* cannot far call TimerFunction TimerFunction */ TimeProc, 0, &dwThreadId);
	//	MMRESULT nIDTimerEvent = timeSetEvent(1000, 0, MMTimeProc, 0, (UINT)TIME_PERIODIC);
	HANDLE hThread = CreateThread(NULL, 0, SendThreadProc, this, 0, &dwThreadId);
	CloseHandle(hThread);
}

void MainCommunication::stopQueryAndSendThread()
{
	mIsQueryAndSendThreadRunning = false;
}

bool MainCommunication::isQueryAndSendThreadRunning()
{
	return mIsQueryAndSendThreadRunning;
}

void MainCommunication::setAssignModeFunction(AssignModeFunction pAssignModeFunction)
{
	mAssignModeFunction = pAssignModeFunction;
}

AssignModeFunction MainCommunication::getAssignModeFunction()
{
	return mAssignModeFunction;
}

void MainCommunication::setAssignStateFunction(AssignStateFunction pAssignStateFunction)
{
	mAssignStateFunction = pAssignStateFunction;
}

AssignStateFunction MainCommunication::getAssignStateFunction()
{
	return mAssignStateFunction;
}

void MainCommunication::setAcquireHangerInformationFunction(AcquireHangerInformationFunction pAcquireHangerInformationFunction)
{
	mAcquireHangerInformationFunction = pAcquireHangerInformationFunction;
}

AcquireHangerInformationFunction MainCommunication::getAcquireHangerInformationFunction()
{
	return mAcquireHangerInformationFunction;
}

void MainCommunication::startReceiveAndAssignThread()
{
	mIsReceiveAndAssignThreadRunning = true;
	DWORD dwThreadId;
	HANDLE hThread = CreateThread(NULL, 0, ReceiveThreadProc, this, 0, &dwThreadId);
	CloseHandle(hThread);
}

void MainCommunication::stopReceiveAndAssignThread()
{
	mIsReceiveAndAssignThreadRunning = false;
}

bool MainCommunication::isReceiveAndAssignThreadRunning()
{
	return mIsReceiveAndAssignThreadRunning;
}

unsigned long MainCommunication::readOneByteWithRetry(unsigned char* pByte) {
	unsigned long readCount=0;
	//	EnterCriticalSection(&mCriticalSectionWrite);
	while ( isReceiveAndAssignThreadRunning()) {
		readCount=readBuffer(pByte,1);
		if (readCount == 0) {
			Sleep(RECEIVE_TRY_MS);
		}
		else {
			break;
		}
	}
	return readCount;
	// LeaveCriticalSection(&mCriticalSectionWrite);
}
