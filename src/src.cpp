#include<pybind11/pybind11.h>
#include<string>
#include<iostream>
#include<thread>
#include<chrono>
#include"cmdline.h"
#include"ViewLink.h"


bool g_bConnected = false;
int VLK_ConnStatusCallback(int iConnStatus, const char* szMessage, int iMsgLen, void* pUserParam)
{
    using namespace std;
    if (VLK_CONN_STATUS_TCP_CONNECTED == iConnStatus)
    {
        cout << "TCP Gimbal connected !!!" << endl;
        g_bConnected = true;
    }
    else if (VLK_CONN_STATUS_TCP_DISCONNECTED == iConnStatus)
    {
        cout << "TCP Gimbal disconnected !!!" << endl;
        g_bConnected = false;
    }
    else if (VLK_CONN_STATUS_SERIAL_PORT_CONNECTED == iConnStatus)
    {
        cout << "serial port connected !!!" << endl;
        g_bConnected = true;
    }
    else if (VLK_CONN_STATUS_SERIAL_PORT_DISCONNECTED == iConnStatus)
    {
        cout << "serial port disconnected !!!" << endl;
        g_bConnected = false;
    }
    else
    {
        cout << "unknown connection stauts: " << iConnStatus << endl;
        g_bConnected = false;
    }

    return 0;
}

int VLK_DevStatusCallback(int iType, const char* szBuffer, int iBufLen, void* pUserParam)
{
    using namespace std;
    if (VLK_DEV_STATUS_TYPE_MODEL == iType)
    {
        VLK_DEV_MODEL* pModel = (VLK_DEV_MODEL*)szBuffer;
        cout << "model code: " << pModel->cModelCode << ", model name: " << pModel->szModelName << endl;
    }
    else if (VLK_DEV_STATUS_TYPE_CONFIG == iType)
    {
        VLK_DEV_CONFIG* pDevConfig = (VLK_DEV_CONFIG*)szBuffer;
        cout << "VersionNO: " << pDevConfig->cVersionNO << ", DeviceID: " << pDevConfig->cDeviceID << ", SerialNO: " << pDevConfig->cSerialNO << endl;
    }
    else if (VLK_DEV_STATUS_TYPE_TELEMETRY == iType)
    {
        /*
         * once device is connected, telemetry information will keep updating,
         * in order to avoid disturbing user input, comment out printing telemetry information
         */
        // VLK_DEV_TELEMETRY* pTelemetry = (VLK_DEV_TELEMETRY*)szBuffer;
        // cout << "Yaw: " << pTelemetry->dYaw << ", Pitch: " << pTelemetry->dPitch << ", sensor type: " << pTelemetry->emSensorType << ", Zoom mag times: " << pTelemetry->sZoomMagTimes << endl;
    }
    else {
        cout << "error: unknown status type: " << iType << endl;
    }
    return 0;
}




int Init()
{
    using namespace std;
    // parse cmd line
    cmdline::parser a;
    a.add<string>("type", 't', "connection type", true, "tcp", cmdline::oneof<string>("serial", "tcp"));
    a.add<string>("ip", 'i', "gimbal tcp ip", false, "192.168.2.119");
    a.add<int>("port", 'p', "gimbal tcp port", false, 2000);
    // a.parse_check(argc, argv);

    int isSuccess = 1;
    // initialize sdk
    int iRet = VLK_Init();
    if (VLK_ERROR_NO_ERROR != iRet) {
       cout << "VLK_Init failed, error: " << iRet << endl;
       isSuccess = 0;
    }
    
    cout << "ViewLink SDK版本: " << GetSDKVersion() << endl;// print sdk version

    VLK_RegisterDevStatusCB(VLK_DevStatusCallback, NULL);// register device status callback

    // connect device
    VLK_CONN_PARAM param;
    memset(&param, 0, sizeof(param));
    param.emType = VLK_CONN_TYPE_TCP;
    strncpy(param.ConnParam.IPAddr.szIPV4, a.get<string>("ip").c_str(), sizeof(param.ConnParam.IPAddr.szIPV4) - 1);
    param.ConnParam.IPAddr.iPort = a.get<int>("port");

    cout << "connecting gimbal ip: " << a.get<string>("ip") << ", port: " << a.get<int>("port") << "..." << endl;
    iRet = VLK_Connect(&param, VLK_ConnStatusCallback, NULL);
    if (VLK_ERROR_NO_ERROR != iRet) {
        cout << "VLK_Connect failed, error: " << iRet << endl;
        isSuccess = 0;
    }
    cout << "wait device connected..." << endl;
    while (1)  {
        if (g_bConnected) {
           break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return isSuccess;
}



/**
 * @brief 向左转动10度
 * 
 */
void TurnToRight() 
{
    double yaw = 10.0;
    double pitch = 0;
    std::cout << "TurnToRight !!!!!";
    VLK_TurnTo(yaw, pitch);
}


/**
 * @brief 向右转动10度
 * 
 */
void TurnToLeft() 
{
    double yaw = -10.0;
    double pitch = 0;
    std::cout << "TurnToLeft !!!!!";
    VLK_TurnTo(yaw, pitch);
}



/**
 * @brief 归为到（0, 0）
 * 
 */
void TurnToHome() 
{
    double yaw = 0.0;
    double pitch = 0.0;
    std::cout << "TurnToHome !!!!!";
    VLK_TurnTo(yaw, pitch);
}



void PrintInfo()
{
    std::cout << "INFO:\n";
    std::cout << "press \'g\' turn to \n";
    std::cout << "press \'s\' move down \n";
    std::cout << "press \'a\' move left \n";
    std::cout << "press \'d\' move right \n";
    std::cout << "press \'h\' move to home posiion \n";
    std::cout << "press \'1\' zoom in, \'2\' zoom out\n";
    std::cout << "press \'c\' exit\n";
}


PYBIND11_MODULE(vlkpy, m) {
    m.def("VLK_init",[](){return Init()==0;});
    m.def("VLK_move",&VLK_Move);
    m.def("VLK_Stop",&VLK_Stop);
    m.def("VLK_TurnTo",&VLK_TurnTo);
    m.def("turnToRight", [](){TurnToRight();});
    m.def("turnToLeft", [](){TurnToLeft();});
    m.def("turnToHome", [](){TurnToHome();});
    m.def("PrintInfo", &PrintInfo);
}