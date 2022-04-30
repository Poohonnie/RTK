#pragma once

#include "lib.h"

class Client
{
private:
    CONFIG config{};

public:
    void SetConfig();  // 设置配置表
    
    void Run();  // 启动函数
    int FileSPP();  // 读取文件数据进行单点定位
    int ServerSPP();  // 接收服务器数据进行单点定位
    int FileRTK();  // 读取文件数据进行实时动态定位
    int ServerRTK();  // 接收服务器数据进行实时动态定位
};

