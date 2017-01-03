#define WINVER 0x0A00
#define _WIN32_WINNT 0x0A00
#include <Windows.h>
#include <Synchapi.h>
#include <iostream>
#include <stdio.h>
#include <tchar.h>
#include <stdint.h>
#include <fstream>
#include <string>
#include <sstream>

#include "mavlink/include/mavlink/v1_0/common/mavlink.h"

using namespace std;

mavlink_heartbeat_t mavHeartbeat;
mavlink_rc_channels_raw_t rc_channels_raw;
mavlink_servo_output_raw_t servo_output_raw;
mavlink_message_t readMessage;
mavlink_status_t status;
mavlink_system_t mavlink_system;
mavlink_command_long_t comm = { 0 };
mavlink_message_t writeMessage;
mavlink_message_t heartbeatMessage;
mavlink_message_t rcOverrideMessage;
mavlink_rc_channels_override_t rcChannelsOverride;
char rcOverrideBuff[MAVLINK_MAX_PACKET_LEN];
unsigned packetLen = 0;
uint16_t heartbeatCount = 0;
char heartbeatBuff[MAVLINK_MAX_PACKET_LEN];
int testCount = 0;
int rcOverrideCount = 0;
bool isQuit = false;
uint16_t chan4Val;
uint16_t chan2Val;
uint16_t chan1Val;
//
// const parameters
//
const uint16_t day = 57600;
const LONG period = 500;
const uint16_t chan4High = 1650;
const uint16_t chan4Low = 1350;
const uint16_t chan2High = 1650;
const uint16_t chan2Low = 1350;
const uint16_t chan1High = 1650;
const uint16_t chan1Low = 1350;

ofstream logFile;

const DWORD waitTimeOut = 2000;
HANDLE mutex;
CONDITION_VARIABLE sendHeartbeat;
CONDITION_VARIABLE moveServos;
CRITICAL_SECTION heartbeatAccess;
CRITICAL_SECTION rcOverrideAccess;

/*****************************************************************************
 *
 * A basic waitable timer thread function
 *
*****************************************************************************/
DWORD WINAPI waitableTimerThread(LPVOID lpParameter)
{
    HANDLE hTimer = NULL;
    LARGE_INTEGER liDueTime;
    //
    // put in some value to get it stared...
    //
    liDueTime.QuadPart = -1000000LL;
    //
    // create an unnamed waitable timer.
    //
    hTimer = CreateWaitableTimer(NULL,
                                 FALSE,
                                 NULL);
    if (NULL == hTimer)
    {
        cout << "CreateWaitableTimer failed..." << endl;
        return 1;
    }
    //
    // set a timer to wait for 2 seconds.
    //
    if (!SetWaitableTimer(hTimer,
                          &liDueTime,
                          period,
                          NULL,
                          NULL,
                          FALSE))
    {
        cout << "SetWaitableTimer failed..." << endl;
        return 2;
    }
    //
    // Wait for the timer.
    //
    while(!isQuit){
        if (WaitForSingleObject(hTimer, INFINITE) != WAIT_OBJECT_0)
            cout << "WaitForSingleObject (timer) failed..." << endl;
        else {
            WaitForSingleObject(mutex, INFINITE);
            //
            // wake thread to send heartbeat
            //
            WakeConditionVariable(&sendHeartbeat);
            Sleep(100);
            ++heartbeatCount;
            ++rcOverrideCount;
            ReleaseMutex(mutex);
            if (rcOverrideCount == 3) {
                WaitForSingleObject(mutex, INFINITE);
                rcOverrideCount = 0;
                ReleaseMutex(mutex);
            }
            if (rcOverrideCount == 1 || rcOverrideCount == 2) {
                //
                // wake thread to send rc override command
                //
                WakeConditionVariable(&moveServos);
            }
            if(heartbeatCount == day) {
                WaitForSingleObject(mutex, INFINITE);
                cout << "closing threads, hit <Enter> to quit..." << endl;
                isQuit = true;
                ReleaseMutex(mutex);
            }
        }
    }
    CloseHandle(hTimer);

    return 0;
}

/*****************************************************************************
 *
 * mavlink heartbeat thread function
 *
*****************************************************************************/
DWORD WINAPI heartbeatThread(LPVOID lpParameter)
{
    DWORD bytesWritten = 0;
    HANDLE hSerial = (HANDLE)lpParameter;
    //
    // start HIL loop
    //
    while (!isQuit) {
        //
        // wait for wake up signal from timer thread
        //
        SleepConditionVariableCS(&sendHeartbeat, &heartbeatAccess, waitTimeOut);
        WaitForSingleObject(mutex, INFINITE);
        logFile << "hb\n";
        //
        // send heartbeat message
        //
        //
        // Translate message to buffer
        //
        packetLen = mavlink_msg_to_send_buffer((uint8_t*)heartbeatBuff, &heartbeatMessage);
        //
        // Write buffer to serial port, locks port while writing
        //
        if(!WriteFile(hSerial, heartbeatBuff, packetLen, &bytesWritten, NULL))
        {
            cout << "write failed..." << endl;
            return 1;
        }
        cout << bytesWritten << " bytes" << endl;
        bytesWritten = 0;
        ReleaseMutex(mutex);
    }
    return 0;
}

/*****************************************************************************
 *
 * mavlink heartbeat rc override thread function
 *
*****************************************************************************/
DWORD WINAPI rcOverrideThread(LPVOID lpParameter)
{
    DWORD bytesWritten = 0;
    HANDLE hSerial = (HANDLE)lpParameter;
    //
    // start HIL loop
    //
    while (!isQuit) {
        //
        // wait for wake up signal from timer thread
        //
        SleepConditionVariableCS(&moveServos, &rcOverrideAccess, waitTimeOut);
        WaitForSingleObject(mutex, INFINITE);
        SYSTEMTIME commandTime;
        GetLocalTime(&commandTime);
        stringstream commandTimeSS;
        string commandTimeStr;
        commandTimeSS << commandTime.wHour << ':' << commandTime.wMinute
                      << ':' << commandTime.wSecond;
        commandTimeSS >> commandTimeStr;
        logFile << commandTimeStr;
        logFile << "\n";
        stringstream chan1SS;
        stringstream chan2SS;
        stringstream chan4SS;
        chan1SS << chan1Val;
        string chan1Str;
        chan1SS >> chan1Str;
        chan2SS << chan2Val;
        string chan2Str;
        chan2SS >> chan2Str;
        chan4SS << chan4Val;
        string chan4Str;
        chan4SS >> chan4Str;
        string command = "rc 1 ";
        command.append(chan1Str);
        command.append(", rc 2 ");
        command.append(chan2Str);
        command.append(", rc 4 ");
        command.append(chan4Str);
        logFile << command;
        logFile << "\n";
        //
        // send rc override message
        //
        rcChannelsOverride.chan1_raw = chan1Val;
        rcChannelsOverride.chan2_raw = chan2Val;
        rcChannelsOverride.chan4_raw = chan4Val;
        //
        // Encode
        //
        mavlink_msg_rc_channels_override_encode(mavlink_system.sysid,
                                                mavlink_system.compid,
                                                &rcOverrideMessage,
                                                &rcChannelsOverride);
        //
        // Translate message to buffer
        //
        packetLen = mavlink_msg_to_send_buffer((uint8_t*)rcOverrideBuff, &rcOverrideMessage);
        //
        // Write buffer to serial port, locks port while writing
        //
        if(!WriteFile(hSerial, rcOverrideBuff, packetLen, &bytesWritten, NULL)) {
            cout << "write failed..." << endl;
            CloseHandle(hSerial);
            return 1;
        }
        cout << bytesWritten << " bytes" << endl;
        bytesWritten = 0;
        if (chan4Val < chan4High) {
            chan4Val = chan4High;
            chan2Val = chan2Low;
            chan1Val = chan1Low;
        }
        else {
            chan4Val = chan4Low;
            chan2Val = chan2High;
            chan1Val = chan1High;
        }
        ReleaseMutex(mutex);
    }
    return 0;
}


int main(void)
{
    //
    // initialize servo settings...
    //
    chan4Val = chan4High;
    chan2Val = chan2Low;
    chan1Val = chan1Low;
    //
    // get local date and time
    //
    SYSTEMTIME startTime;
    GetLocalTime(&startTime);
    cout << " local time: "
         << endl
         << "month: " << startTime.wMonth << endl
         << "day: " << startTime.wDay << endl
         << "year: " << startTime.wYear << endl
         << "hour: " << startTime.wHour << endl
         << "minute: " << startTime.wMinute << endl
         << "seconds: " << startTime.wSecond << endl
         << endl;
    //
    // declare variables to hold date/time string
    //
    stringstream currentDateSS;
    stringstream currentTimeSS;
    string currentDate;
    string currentTime;
    string currentDateAndTime;
    //
    // initialize date/time string
    //
    currentDateSS << startTime.wMonth << '_' << startTime.wDay << '_'
                      << startTime.wYear;
    currentTimeSS << startTime.wHour << '-' << startTime.wMinute
                << '-' << startTime.wSecond;
    currentDateSS >> currentDate;
    currentTimeSS >> currentTime;
    currentDateAndTime.append(currentDate);
    currentDateAndTime.append("__");
    currentDateAndTime.append(currentTime);
    //
    // open log file for writing...
    //
    string logFileName = "C:\\hilTestingLogs\\logFile";
    logFileName.append("_");
    logFileName.append(currentDateAndTime);
    logFileName.append(".txt");
    cout << "log file name: " << logFileName << endl;
    //
    // open the log file for writing
    //
    logFile.open(logFileName);
    if(logFile.is_open()) {
        cout << "log file opened..." << endl;
    }
    else {
        cout << "could not open log file..." << endl;
        return 1;
    }
    //
    // replace "-" with ":". and "__" with "  "
    //
    currentDateAndTime.replace(currentDateAndTime.find("-"), 1, ":");
    currentDateAndTime.replace(currentDateAndTime.find("-"), 1, ":");
    currentDateAndTime.replace(currentDateAndTime.find("__"), 2, "  ");
    cout << "starting date and time: " << currentDateAndTime << endl;
    logFile << "HIL Log File Opened:  " << currentDateAndTime;
    logFile << "\n";
    //
    // create the mutex to protect access to data
    //
    mutex = CreateMutexA(0, 0, 0);
    //
    // initialize the condition variables used to synchronize threads
    //
    InitializeConditionVariable(&sendHeartbeat);
    InitializeConditionVariable(&moveServos);
    //
    // initialize the critical section variable
    //
    InitializeCriticalSection(&heartbeatAccess);
    InitializeCriticalSection(&rcOverrideAccess);
    //
    // Declare serial port variables and structures
    //
    HANDLE hSerial;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    LPCWSTR commPort = TEXT("COM4");
    //
    // Open the serial port
    //
    cout << "Opening serial port..." << endl;
    hSerial = CreateFile(commPort,
                         GENERIC_READ|GENERIC_WRITE,
                         0,
                         NULL,
                         OPEN_EXISTING,
                         FILE_ATTRIBUTE_NORMAL,
                         NULL );
    if (hSerial == INVALID_HANDLE_VALUE)
    {
            cout << "Failed to open serial port..." << endl;
            return 1;
    }
    else fprintf(stderr, "OK\n");
    //
    // Set device parameters (115200 baud, 1 start bit,
    // 1 stop bit, no parity)
    //
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0)
    {
        cout << "Error getting serial port device state..." << endl;
        CloseHandle(hSerial);
        return 1;
    }
    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if(SetCommState(hSerial, &dcbSerialParams) == 0)
    {
        cout << "Error setting serial port device parameters..." << endl;
        CloseHandle(hSerial);
        return 1;
    }
    //
    // Set COM port timeout settings
    //
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if(SetCommTimeouts(hSerial, &timeouts) == 0)
    {
        cout << "Error setting timeouts..." << endl;
        CloseHandle(hSerial);
        return 1;
    }
    //
    // Intialize heartbeat message
    //
    mavlink_system.sysid = 255;
    mavlink_system.compid = 1;
    mavlink_msg_heartbeat_encode(mavlink_system.sysid,
                                 mavlink_system.compid,
                                 &heartbeatMessage,
                                 &mavHeartbeat);
    //
    // Initialze rc override message
    //
    rcChannelsOverride.chan1_raw = 0;
    rcChannelsOverride.chan2_raw = chan2Val;
    rcChannelsOverride.chan3_raw = 0;
    rcChannelsOverride.chan4_raw = chan4Val;
    rcChannelsOverride.chan5_raw = 0;
    rcChannelsOverride.chan6_raw = 0;
    rcChannelsOverride.chan7_raw = 0;
    rcChannelsOverride.chan8_raw = 0;
    rcChannelsOverride.target_component = MAV_AUTOPILOT_PX4;
    rcChannelsOverride.target_system = MAV_TYPE_FIXED_WING;
    //
    // create and start waitable timer thread
    //
    DWORD waitableTimerThreadID;
    HANDLE waitableTimerThreadHandle = CreateThread(
                    0, 0, waitableTimerThread, 0, 0, &waitableTimerThreadID);
    //
    // create and start the HIL thread
    //
    DWORD heartbeatThreadID;
    HANDLE heartbeatThreadHandle = CreateThread(
                    0, 0, heartbeatThread, hSerial, 0, &heartbeatThreadID);

    //
    // create and start the HIL thread
    //
    DWORD rcOverrideThreadID;
    HANDLE rcOverrideThreadHandle = CreateThread(
                    0, 0, rcOverrideThread, hSerial, 0, &rcOverrideThreadID);
    //
    // wait for keyboard input
    //
    while (!isQuit) {
        cin.get();
        WaitForSingleObject(mutex, INFINITE);
        isQuit = true;
        ReleaseMutex(mutex);
    }
    //
    // wait for threads to exit
    //
    WaitForSingleObject(rcOverrideThreadHandle, INFINITE);
    WaitForSingleObject(heartbeatThreadHandle, INFINITE);
    WaitForSingleObject(waitableTimerThreadHandle, INFINITE);
    //
    // close the serial port
    //
    cout << "Closing serial port..." << endl;
    if (CloseHandle(hSerial) == 0)
    {
        cout << "Error closing serial port..." << endl;
        return 1;
    }
    fprintf(stderr, "OK\n");
    //
    // declare and intialize the ending date/time string
    //
    SYSTEMTIME endTime;
    GetLocalTime(&endTime);
    stringstream endDateSS;
    stringstream endTimeSS;
    string endDate;
    string endTimeStr;
    endDateSS << endTime.wMonth << '_' << endTime.wDay << '_'
                      << endTime.wYear;
    endTimeSS << endTime.wHour << ':' << endTime.wMinute
                << ':' << endTime.wSecond;
    endDateSS >> endDate;
    endDate.append(" ");
    endTimeSS >> endTimeStr;
    endDate.append(endTimeStr);
    cout << "ending date and time: " << endDate << endl;
    logFile << "HIL Log File Closed:  ";
    logFile << endDate;
    logFile << "\n";
    //
    // close the log file
    //
    logFile.close();
    //
    // exit normally
    //
    return 0;
}
