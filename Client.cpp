#include "lib.h"
#include "SPP.h"
#include "RTK.h"
#include "Client.h"
#include "CDecode.h"
#include "Detect.h"
#include <iostream>
#include <windows.h>
#include <queue>
#include <cmath>

void Client::SetConfig()
{
    config.posMode = 1;  // 0:SPP 1:RTK
    config.iptStream = 1;  // 0:file 1:server
    config.elmin = 10 * constant::D2R;  // 高度角阈值 10°
    config.ratioThres = 3.0;  // ratio值阈值 3.0
    strcpy(config.iptFileName[0], R"(C:\Users\Zing\Desktop\Junior2\SNAP2\oem719-202202131000-2.bin)");  // 流动站文件
    strcpy(config.iptFileName[1], R"(C:\Users\Zing\Desktop\Junior2\SNAP2\oem719-202202131000-1.bin)");  // 基站文件
//    strcpy(config.iptFileName[0], R"(C:\Users\Zing\Desktop\Junior2\SNAP2\Novatel0304.bin)");  // 流动站文件
//    strcpy(config.iptFileName[1], R"(C:\Users\Zing\Desktop\Junior2\SNAP2\3.4-basedata-novatel.bin)");  // 基站文件
    
    strcpy(config.iptIP[0], "47.114.134.129");  // 流动站IP地址
    strcpy(config.iptIP[1], "47.114.134.129");  // 基站IP地址
    
    config.port[0] = 7180;  // 流动站
    config.port[1] = 7190;  // 基站
}

void Client::Run()
{
    SetConfig();
    if (config.posMode == 0)
        switch (config.iptStream)
        {
            case 0:
                FileSPP();
                break;
            case 1:
                ServerSPP();
                break;
            default:
                break;
        }
    else if (config.posMode == 1)
        switch (config.iptStream)
        {
            case 0:
                FileRTK();
                break;
            case 1:
                ServerRTK();
                break;
            default:
                break;
        }
}

int Client::FileSPP()
{
    CFileDecode fileDecode{};
    CDetectOutlier detectOutlier{};
    SPP spp{};  // 单点定位类
    char fileName[200]{};
    strcpy(fileName, R"(C:\Users\Zing\Desktop\Junior1\SNAP1\OEM719-1126\202010261820.oem719)");
//    strcpy(fileName, R"(C:\Users\Zing\Desktop\Junior2\SNAP2\oem719-202202131000-1.bin)");
    fileDecode.FileRead(fileName);
    int flag{};
    XYZ refXyz = {-2267794.937, 5009345.236, 3220980.312};  // 真实位置
    
    //FILE *outFp = fopen("202010261820.oem719.pos", "w");
    while (true)
    {
        flag = fileDecode.DecodeOem719Msg();
        if (flag == -114514)
            break;
        if (flag != 43)
            // CRC校验失败
            continue;
        detectOutlier.DetectOutlier(fileDecode.raw);
        spp.StdPntPos(fileDecode.raw, detectOutlier.curEpk, config);
        spp.StdPntVel(fileDecode.raw, detectOutlier.curEpk, config);
        double dNEU[3]{};
        CalDNEU(refXyz, spp.sttnXyz, dNEU);
        printf("%4d %9.3f %12.4f %12.4f %12.4f %11.8f %11.8f %7.3f %6.3f %6.3f %6.3f %7.4f %7.4f %7.4f %5.3f %5.3f %5.3f %3d %3d %3d\n",
               spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z,
               spp.sttnBlh.B * constant::R2D, spp.sttnBlh.L * constant::R2D, spp.sttnBlh.H,
               dNEU[0], dNEU[1], dNEU[2], spp.sttnV[0], spp.sttnV[1], spp.sttnV[2],
               spp.PDOP, spp.sigmaP, spp.sigmaV, spp.sysNum[0], spp.sysNum[1], spp.sysNum[0] + spp.sysNum[1]);
        /*fprintf(outFp, "%4d %9.3f %12.4f %12.4f %12.4f %11.8f %11.8f %7.3f %6.3f %6.3f %6.3f %7.4f %7.4f %7.4f %5.3f %5.3f %5.3f %3d %3d %3d\n",
               spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z,
               spp.sttnBlh.B * constant::R2D, spp.sttnBlh.L * constant::R2D, spp.sttnBlh.H,
               dNEU[0], dNEU[1], dNEU[2], spp.sttnV[0], spp.sttnV[1], spp.sttnV[2],
               spp.PDOP, spp.sigmaP, spp.sigmaV, spp.sysNum[0], spp.sysNum[1], spp.sysNum[0] + spp.sysNum[1]);*/
    }
    //fclose(outFp);
    return 0;
}

int Client::ServerSPP()
{
    int lenRem{};  // 上次解码余下字节
    int curLen{};  // 此次接收到的报文总长度
    auto *buf = new unsigned char[204800];
    memset(buf, 0, 204800);
    auto *socketDecode = new CSocketDecode;  // 解码类
    CDetectOutlier detectOutlier;  // 粗差探测类
    SOCKET sock;  // 套接字
    SPP spp;  // 单点定位类
    unsigned short port = 7180;  // 端口
    int val;  // 解码返回值
    XYZ refXyz = {-2267804.5263, 5009342.3723, 3220991.8632};
    
    if (!CSocketDecode::OpenSocket(sock, config.iptIP[0], port))
    {
        //网络通信失败
        printf("Cannot open socket.\n");
        return -114514;
    }
    //FILE *outFp = fopen("202111250936.oem719.pos", "a+");
    double epoch{};
    while (epoch < 8 * 3600)
    {
        if (lenRem < 51200)
            Sleep(960);
        //防止运算速度追不上接收信息速度
        if (lenRem > 204800 || lenRem < 0)
        {
            //防止卡死
            lenRem = 0;
            memset(buf, 0, 204800);
        }
        curLen = recv(sock, (char *) buf + lenRem, 204800 - lenRem, 0);  // 在余留的缓冲区基础上接收报文
        if (curLen < 0)
        {
            printf("Please check out the network.\n");
            delete[] buf;
            delete socketDecode;
            return -114514;
        }
        val = socketDecode->DecodeOem719Msg(buf, curLen, lenRem);  // 网络接收到的报文解码
        if (val == 43)
        {
            detectOutlier.DetectOutlier(socketDecode->raw);
            int flag = spp.StdPntPos(socketDecode->raw, detectOutlier.curEpk, config);
            if (flag == -114514)  // SPP定位失败
                continue;
            spp.StdPntVel(socketDecode->raw, detectOutlier.curEpk, config);
            
            double dNEU[3]{};
            CalDNEU(refXyz, spp.sttnXyz, dNEU);
            printf("%4d %9.3f  %11.4f  %11.4f  %11.4f  %11.8f  %11.8f %10.3f %6.3f %6.3f %6.3f %10.4f %10.4f %10.4f  %5.3f %5.3f %5.3f %d %d %d\n",
                   spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z,
                   spp.sttnBlh.B * constant::R2D, spp.sttnBlh.L * constant::R2D, spp.sttnBlh.H,
                   dNEU[0], dNEU[1], dNEU[2], spp.sttnV[0], spp.sttnV[1], spp.sttnV[2],
                   spp.PDOP, spp.sigmaP, spp.sigmaV, spp.sysNum[0], spp.sysNum[1], spp.sysNum[0] + spp.sysNum[1]);
            /*fprintf(outFp, "%4d %9.3f %11.4f %11.4f %11.4f %11.8f %11.8f %7.3f %6.3f %6.3f %5.3f %5.3f %d %d %d\n",
                    spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * constant::R2D,
                    spp.sttnBlh.L * constant::R2D, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP,
                    spp.sysNum[0], spp.sysNum[1], spp.sysNum[0] + spp.sysNum[1]);*/
            
        }
    }
    //fclose(outFp);
    return 0;
}

int Client::FileRTK()
{
    //始终认为 0为流动站rover 1为基准站base
    CFileDecode fileDecode[2]{};
    CDetectOutlier detectOutlier[2]{};
    RTK rtk{};
    
    fileDecode[0].FileRead(config.iptFileName[0]);  // 流动站rover数据文件读取
    fileDecode[1].FileRead(config.iptFileName[1]);  // 基准站base数据文件读取
    
    CFileDecode lastBase{};  // 上一历元的基准站数据
    int flag[2]{};
    XYZ refXyz = {-2267804.5263, 5009342.3723, 3220991.8632};
    XYZ totalXyz{};
//    FILE *outFp = fopen(R"(C:\Users\Zing\Desktop\Junior2\SNAP2\oem719-202202131000-2.pos)", "w");
//    FILE *outFp = fopen(R"(C:\Users\Zing\Desktop\Junior2\SNAP2\Novatel0304.pos)", "w");
    double dtime{};
    int epoch{};
    while (true)
    {
        flag[0] = fileDecode[0].DecodeOem719Msg();  // 先解码流动站, 以流动站时间为基准
        if (flag[0] == -114514)  // 读到文件结尾，退出程序
            break;
        if (flag[0] != 43)  // CRC校验失败, 该历元不进行解算
            continue;
        // 流动站文件解码成功
        dtime = fileDecode[0].raw.epkObs.t - lastBase.raw.epkObs.t;  // 流动站数据和基准站数据之间的时间差
        
        while (dtime > 0.5)  // 一直读到超前于流动站当前历元的数据 或者时间差在0.5秒之内时停下(0.5秒内停下就是同历元)
        {
            flag[1] = fileDecode[1].DecodeOem719Msg();  // 解码基准站数据
            if (flag[1] == -114514)  // 读到文件结尾, 退出循环
                break;
            if (flag[1] != 43)  // CRC校验失败
                continue;
            dtime = fileDecode[0].raw.epkObs.t - fileDecode[1].raw.epkObs.t;  // 流动站数据减基准站数据 GPS时减法已重载
        }
        if (flag[1] == -114514)  // 读到文件结尾，退出程序
            break;
        if (flag[1] != 43)  // CRC校验失败
            continue;
        if (dtime < 0.5)
            lastBase = fileDecode[1];  // 将该历元基准站数据保存起来, 作为下一历元的lastBase
        if (dtime < 0)
            continue;  // 读到来自未来的基准站数据
        // 星历复制
        for (int i = 0; i < MAXGPSNUM; ++i)
            fileDecode[0].raw.gpsEphem[i] = fileDecode[1].raw.gpsEphem[i];
        for (int i = 0; i < MAXBDSNUM; ++i)
            fileDecode[0].raw.bdsEphem[i] = fileDecode[1].raw.bdsEphem[i];
        // RTK定位解算
        // 观测值粗差探测
        detectOutlier[0].DetectOutlier(fileDecode[0].raw);
        detectOutlier[1].DetectOutlier(fileDecode[1].raw);
        
        rtk.CalFixedSolution(fileDecode[0].raw, fileDecode[1].raw,
                             detectOutlier[0].curEpk, detectOutlier[1].curEpk, config);
        if (!rtk.valid)  // 定位结果异常
            continue;
        
        // 输出解算结果
        XYZ dXyz = rtk.pos - refXyz;
        double line = sqrt(dXyz.x * dXyz.x + dXyz.y * dXyz.y + dXyz.z * dXyz.z);
        double dNEU[3]{};
        CalDNEU(refXyz, rtk.pos, dNEU);
        char sol[50]{};  // 解的类型
        switch (rtk.sol)
        {
            case 0:  // 单点解
                strcpy(sol, "Single");
                break;
            case 1:  // 浮点解
                strcpy(sol, "Float");
                break;
            case 2:  // 固定解
                strcpy(sol, "Fixed");
                break;
            default:
                break;
        }
//        if(rtk.sol == 2)  // 本历元成功固定
//        {
//            ++epoch;
//            totalXyz = totalXyz + rtk.pos;  // 坐标值综合, 方便下面求平均
//            refXyz.x = totalXyz.x / epoch;  // 固定解平均值作为参考坐标
//            refXyz.y = totalXyz.y / epoch;
//            refXyz.z = totalXyz.z / epoch;
//        }
//        if(epoch < 1)  // 还没有固定解出现
//            continue;
        printf("%4d %10.3f %7.3f %13.4f %13.4f %13.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %2d %2d %7.2f  %s\n",
               rtk.t.week, rtk.t.secOfWeek, dtime, rtk.pos.x, rtk.pos.y, rtk.pos.z,
               dXyz.x, dXyz.y, dXyz.z, line, dNEU[0], dNEU[1], dNEU[2],
               rtk.ddObs.sysNum[0], rtk.ddObs.sysNum[1], rtk.ratio, &sol);
/*
        fprintf(outFp, "%4d %10.3f %7.3f %13.4f %13.4f %13.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %2d %2d %7.2f %2d\n",
                rtk.t.week, rtk.t.secOfWeek, dtime, rtk.pos.x, rtk.pos.y, rtk.pos.z,
                dXyz.x, dXyz.y, dXyz.z, line, dNEU[0], dNEU[1], dNEU[2],
                rtk.ddObs.sysNum[0], rtk.ddObs.sysNum[1], rtk.ratio, rtk.sol);
*/
    }

//    fclose(outFp);
    return 0;
}

int Client::ServerRTK()
{
    int lenRem[2]{};  // 上次解码余下字节
    int curLen[2]{};  // 此次接收到的报文总长度
    unsigned char buf[2][204800]{};
    CSocketDecode socketDecode[2]{};  // 解码类
    CSocketDecode lastBase{};  // 上一历元结果
    CDetectOutlier detectOutlier[2]{};  // 粗差探测类
    SOCKET sock[2]{};  // 套接字
    RTK rtk{};  // RTK类
    int val[2]{};  // 解码返回值
    XYZ refXyz = {-2267804.5263, 5009342.3723, 3220991.8632};
    XYZ totalXyz{};
    
    FILE *outFp = fopen(R"(C:\Users\Zing\Desktop\Junior2\SNAP2\202204302144.oem719.rtk.pos)", "w");
    
    if (!CSocketDecode::OpenSocket(sock[0], config.iptIP[0], config.port[0]))
    {
        // 流动站通信失败
        printf("Cannot open socket 0.\n");
        return -114514;
    }
    if (!CSocketDecode::OpenSocket(sock[1], config.iptIP[1], config.port[1]))
    {
        // 流动站通信失败
        printf("Cannot open socket 1.\n");
        return -114514;
    }
    double dtime{};
    int epoch{};
    while (epoch < 10 * 3600)
    {
        if (lenRem[0] < 51200)
            Sleep(980);
        if (lenRem[1] > 204800)
        {
            memset(&buf[1], 0, 204800 * sizeof(unsigned char));
            lenRem[1] = 0;
        }
        curLen[0] = recv(sock[0], (char *) buf[0] + lenRem[0], 204800 - lenRem[0], 0);  // 在余留的缓冲区基础上接收报文
        
        if (curLen[0] < 0)
        {
            printf("Please check out the network.\n");
            return -114514;
        }
        val[0] = socketDecode[0].DecodeOem719Msg(buf[0], curLen[0], lenRem[0]);  // 网络接收到的报文解码
        dtime = socketDecode[0].t - lastBase.t;  // 看看上一历元数据是不是来自未来的, 来自未来这一历元就不读基准站了
        int iter{};
        while (iter < 3 && dtime > 0.5)
        {
            curLen[1] = recv(sock[1], (char *) buf[1] + lenRem[1], 204800 - lenRem[1], 0);
            val[1] = socketDecode[1].DecodeOem719Msg(buf[1], curLen[1], lenRem[1]);  // 网络接收到的报文解码
            dtime = socketDecode[0].t - socketDecode[1].t;  // 时间间隔
            ++iter;
        }
        if (dtime < 0.5)
            lastBase = socketDecode[1];  // 将该历元基准站数据保存起来, 作为下一历元的lastBase
        if (val[0] == 43 && val[1] == 43)
        {
            // 星历复制
            for (int i = 0; i < MAXGPSNUM; ++i)
                socketDecode[0].raw.gpsEphem[i] = socketDecode[1].raw.gpsEphem[i];
            for (int i = 0; i < MAXBDSNUM; ++i)
                socketDecode[0].raw.bdsEphem[i] = socketDecode[1].raw.bdsEphem[i];
            // RTK定位解算
            // 观测值粗差探测
            detectOutlier[0].DetectOutlier(socketDecode[0].raw);
            detectOutlier[1].DetectOutlier(socketDecode[1].raw);
            
            rtk.CalFixedSolution(socketDecode[0].raw, socketDecode[1].raw,
                                 detectOutlier[0].curEpk, detectOutlier[1].curEpk, config);
            if (!rtk.valid)  // 定位结果异常
                continue;
            if (fabs(dtime) > 10.0)
            {
                // 基准站数据与流动站时间差太多
                rtk.pos = rtk.spp[0].sttnXyz;
            }
            // 输出解算结果
            XYZ dXyz = rtk.pos - refXyz;
            double dNEU[3]{};
            CalDNEU(refXyz, rtk.pos, dNEU);
            char sol[50]{};  // 解的类型
            switch (rtk.sol)
            {
                case 0:  // 单点解
                    strcpy(sol, "Single");
                    break;
                case 1:  // 浮点解
                    strcpy(sol, "Float");
                    break;
                case 2:  // 固定解
                    strcpy(sol, "Fixed");
                    break;
                default:
                    break;
            }
//                if(rtk.sol == 2)  // 本历元成功固定
//                {
//                    ++epoch;
//                    totalXyz = totalXyz + rtk.pos;  // 坐标值综合, 方便下面求平均
//                    refXyz.x = totalXyz.x / epoch;  // 固定解平均值作为参考坐标
//                    refXyz.y = totalXyz.y / epoch;
//                    refXyz.z = totalXyz.z / epoch;
//                }
//                if(epoch < 1)  // 还没有固定解出现
//                    continue;
            printf("%4d %10.3f %7.3f %13.4f %13.4f %13.4f %7.4f %7.4f %7.4f %2d %2d %7.2f  %s\n",
                   rtk.t.week, rtk.t.secOfWeek, dtime, rtk.pos.x, rtk.pos.y, rtk.pos.z,
                   dNEU[0], dNEU[1], dNEU[2],
                   rtk.ddObs.sysNum[0], rtk.ddObs.sysNum[1], rtk.ratio, &sol);
            fprintf(outFp, "%4d %10.3f %7.3f %13.4f %13.4f %13.4f %7.4f %7.4f %7.4f %2d %2d %7.2f %2d\n",
                    rtk.t.week, rtk.t.secOfWeek, dtime, rtk.pos.x, rtk.pos.y, rtk.pos.z,
                    dNEU[0], dNEU[1], dNEU[2],
                    rtk.ddObs.sysNum[0], rtk.ddObs.sysNum[1], rtk.ratio, rtk.sol);
        }
    }
    
    fclose(outFp);
    return 0;
}

