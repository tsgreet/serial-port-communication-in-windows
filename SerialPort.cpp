#include "stdafx.h"
#include "SerialPort.h"
#include <stdio.h>

//#include <stdlib.h>
//#define SERIALPORT_INTERNAL__TIMEOUT 1

SerialPort::SerialPort()
{
	mSerialPortHandle = INVALID_HANDLE_VALUE;
	//	InitializeCriticalSection(&mCriticalSectionRead);
	//	InitializeCriticalSection(&mCriticalSectionWrite);
	
}

SerialPort::~SerialPort()
{
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
		closePort();
	}

//	DeleteCriticalSection(&mCriticalSectionRead);
//	DeleteCriticalSection(&mCriticalSectionWrite);
}

char* SerialPort::parseString(const char* start, char* output) {
	int wordLength;
	const char* end = NULL;
	char* lap = NULL;
	//com port
	end = strchr(start, '\n');
	if (end == NULL) {
		end = start + strlen(start);
	}
	wordLength = end - start;
	if (wordLength > 0) {
		
		lap=(char*)strchr(start, '=');
		if ((lap == NULL) || (lap >= end)) {
			lap = (char *)start;
		}
		else {
			lap = lap + 1;
			
		}
		//delete space
		while ((*lap) == ' ') {
			lap++;
		}
		wordLength = end - lap;
		//delete '\r' in window style
		if (*(end - 1) == '\r') {
			wordLength--;
		}
		//delete space
		while (*(lap + wordLength - 1) == ' ') {
			wordLength--;
		}
	//	memcpy(output, start, sizeof(char)*wordLength);
		memcpy(output, lap, sizeof(char)*wordLength);
		*(output + wordLength) = 0;
	}
	return (char*)end;
}

void SerialPort::configureSerialPort(char** arrayParameter,int arraySize) {
	HANDLE hConfig;
	DWORD bytesRead = 0;
	char lpBuffer[TEXT_LENGTH] = "235";//�ļ���ȡ������ 
	hConfig = CreateFileA("Config\\serial.txt",//"\\\\.\\COM3",            //port name 

		GENERIC_READ | GENERIC_WRITE, //Read/Write   				 

		0,            // No Sharing                               

		NULL,         // No Security                              
					  //	CREATE_ALWAYS, //
		OPEN_EXISTING,// Open existing port only                     

		0,            // Non Overlapped I/O                           

		NULL);        // Null for Comm Devices

	if (hConfig == INVALID_HANDLE_VALUE)
	{
	//	printf("Error in opening configuration");
	}
	else {
		int fileOpen = ReadFile// WriteFile//
		(
			hConfig,
			lpBuffer,
			TEXT_LENGTH,//��ȡ�ļ��ж������� 
			&bytesRead,
			NULL
		);

		if (fileOpen == 0)
		{
		//	printf("Error in reading configuration");
		}
		else {
		//	printf("bytesRead=%d\n", bytesRead);
			char* start = lpBuffer;
			char* tail;
			for (int index = 0;index <arraySize;index++) {
				tail = parseString((const char*)start,arrayParameter[index]);
				if (tail != (start + strlen(start))) {
					start = tail + 1;
				}
				else {
					break;
				}
			}

		}
	};
	CloseHandle(hConfig);

}

HANDLE SerialPort::openSerialPort() {
	HANDLE hComm = NULL;

	char comPort[6] = "COM3";
	char sizeInputBuffer[8] = "1024";//"64";//"1024";
	char sizeOutputBuffer[8] = "1024";//"64";//"1024";
	char baudRate[7] = "9600";
	char bitValid[3] = "8";
	char checkOption[11] = "NOPARITY";
	char stopOption[13] = "TWOSTOPBITS";

	char readIntervalTimeout[8] = "0";//"100";
	char readTotalTimeoutMultiplier[8] = "0";//"50";
	char readTotalTimeoutConstant[8] = "0";//"5000";
	char writeTotalTimeoutMultiplier[8] = "0";
	char writeTotalTimeoutConstant[8] = "0";

	char* arrayParameter[] = { comPort,sizeInputBuffer,sizeOutputBuffer,baudRate,bitValid,checkOption,stopOption,
		readIntervalTimeout,readTotalTimeoutMultiplier,readTotalTimeoutConstant,writeTotalTimeoutMultiplier,writeTotalTimeoutConstant
	};

	configureSerialPort(arrayParameter,sizeof(arrayParameter)/sizeof(char*));

	hComm = CreateFileA(comPort,//"\\\\.\\COM3",            //port name 

		GENERIC_READ | GENERIC_WRITE, //Read/Write   				 

		0,            // No Sharing                               

		NULL,         // No Security                              

		OPEN_EXISTING,// Open existing port only                     

		0,            // Non Overlapped I/O                           

		NULL);        // Null for Comm Devices



	if (hComm == INVALID_HANDLE_VALUE) {
		//	printf("Error in opening Serial port")

	}
	else {

		//	printf("\nopening serial port successfully\n")
	}




	//configure
	SetupComm(hComm, atoi(sizeInputBuffer), atoi(sizeOutputBuffer)); //���뻺����������������Ĵ�С����1024

	COMMTIMEOUTS TimeOuts;
	//�趨����ʱ
	TimeOuts.ReadIntervalTimeout = atoi(readIntervalTimeout);// 90; // 1000;
	TimeOuts.ReadTotalTimeoutMultiplier = atoi(readTotalTimeoutMultiplier);//50; // 500;
	TimeOuts.ReadTotalTimeoutConstant = atoi(readTotalTimeoutConstant);//5000;
	//�趨д��ʱ
	TimeOuts.WriteTotalTimeoutMultiplier = atoi(writeTotalTimeoutMultiplier);//0;// -1; // 500;
	TimeOuts.WriteTotalTimeoutConstant = atoi(writeTotalTimeoutConstant);//0;//-1; //2000;
	SetCommTimeouts(hComm, &TimeOuts); //���ó�ʱ

	DCB dcb;
	GetCommState(hComm, &dcb);
	dcb.BaudRate = atoi(baudRate); //������Ϊ9600
	dcb.ByteSize = atoi(bitValid); //ÿ���ֽ���8λ
	if (!strcmp(checkOption, "NOPARITY")) {
		dcb.Parity = NOPARITY; //����żУ��λ
	}
	else if (!strcmp(checkOption, "ODDPARITY")) {
		dcb.Parity = ODDPARITY; //��У��λ
	}
	else if (!strcmp(checkOption, "EVENPARITY")) {
		dcb.Parity = EVENPARITY; //żУ��λ
	}
	else if (!strcmp(checkOption, "MARKPARITY")) {
		dcb.Parity = MARKPARITY; //��У��λ
	}

	if (!strcmp(stopOption, "ONESTOPBIT")) {
		dcb.StopBits = ONESTOPBIT; //һ��ֹͣλ
	}
	else if (!strcmp(stopOption, "TWOSTOPBITS")) {
		dcb.StopBits = TWOSTOPBITS; //����ֹͣλ
	}
	else if (!strcmp(stopOption, "ONE5STOPBITS")) {
		dcb.StopBits = ONE5STOPBITS; //һ����ֹͣλ
	}
	SetCommState(hComm, &dcb);

	PurgeComm(hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);

	return hComm;
}

bool SerialPort::openPort()
{
	bool isOpen = false;
	if (mSerialPortHandle == INVALID_HANDLE_VALUE) {
		mSerialPortHandle = openSerialPort();
	}
	if (mSerialPortHandle != INVALID_HANDLE_VALUE) {
		isOpen = true;
	}
	return isOpen;
}

void SerialPort::closePort()
{
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
		CloseHandle(mSerialPortHandle);
		mSerialPortHandle = INVALID_HANDLE_VALUE;
	}
}

bool SerialPort::isPortOpen() {
	bool isOpen = false;
	if (mSerialPortHandle != INVALID_HANDLE_VALUE) {
		isOpen = true;
	}
	return isOpen;
}

unsigned long SerialPort::writeBuffer(const unsigned char* pData, unsigned long dataLength)
{
	
//	unsigned long result = 0;
	DWORD bytesWritten = 0;
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
	//	WriteComm();
	//	OpenComm(
		printf("serial port write begin time:%ld\n", GetTickCount());
		if (!WriteFile(mSerialPortHandle, pData, dataLength, &bytesWritten, NULL))
		{
			bytesWritten = 0;
		}
		printf("serial port write end time:%ld\n", GetTickCount());
	}
	return bytesWritten;

}

unsigned long SerialPort::readBuffer(unsigned char* pData, unsigned long dataLength)
{
	//	unsigned long result = 0;
	DWORD bytesRead = 0;
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
		//	WriteComm();
		//	OpenComm(
		printf("serial port read begin time:%ld\n", GetTickCount());
		if (!ReadFile(mSerialPortHandle, pData, dataLength, &bytesRead, NULL))
		{
			bytesRead = 0;
		}
		printf("serial port read end time:%ld\n", GetTickCount());
	}
	return bytesRead;

}

