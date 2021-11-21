#include "Client.h"

void Client::Run()
{
	int lenRem = 0;//上次解码余下字节
	int curLen = 0;//此次接收到的报文总长度
	unsigned char* buf = new unsigned char[204800];
	memset(buf, 0, 204800);
	CSocketDecode* socketDecode = new CSocketDecode;//解码类
	CDetectOutlier detectOutlier;//粗差探测类
	SOCKET sock;//套接字
	SPP spp;//单点定位类
	char IP[20] = "47.114.134.129";//IP地址
	unsigned short port = 7190;//端口
	int val;//解码返回值
	if (socketDecode->OpenSocket(sock, IP, port) == false)
	{
		//网络通信失败
		printf("Cannot open socket.\n");
		return;
	}
	FILE* fp = fopen("202111211136.oem719.pos", "a+");

	while(1)
	{
		if (lenRem < 10240)
			Sleep(1000);
		//防止运算速度追不上接收信息速度
		if (lenRem > 180000 || lenRem < 0)
		{
			lenRem = 0;
			memset(buf, 0, 204800);
		}
		//防止卡死
		try 
		{
			curLen = recv(sock, (char*)buf + lenRem, 204800 - lenRem, 0);//在余留的缓冲区基础上接收报文
		}
		catch(int e)
		{
			e = 0;
			continue;
		}
		val = socketDecode->DecodeOem719Msg(buf, curLen, lenRem);//网络接收到的报文解码
		if (val == 43)
		{
			detectOutlier.DetectOutlier(socketDecode->raw);
			spp.SglPntPos(socketDecode->raw, detectOutlier.curEpk);
			printf("SPP: %4d %9.3f X:%11.4f Y:%11.4f Z:%11.4f B:%11.8f L:%11.8f H:%10.3f GClk:%6.3f BClk:%6.3f PDOP:%5.3f SigmaP:%5.3f GS:%d BS:%d n:%d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
			fprintf(fp, "%4d %9.3f %11.4f %11.4f %11.4f %11.8f %11.8f %7.3f %6.3f %6.3f %5.3f %5.3f %d %d %d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
		}

	}
	delete[] buf;
	fclose(fp);
	return;
}

void Client::FileSpp()
{
	CFileDecode* fileDecode = new CFileDecode;
	SatPositioning satPositioning;
	CDetectOutlier detectOutlier;
	SPP spp;
	char fileName[50] = "202010261820.oem719";
	FILE* inFp = fileDecode->FileRead(fileName);
	int flag = 0;
	//FILE* outFp = fopen("file.oem719.pos", "w");

	while (1)
	{
		flag = fileDecode->DecodeOem719Msg(inFp);
		if (flag == -114514)
			break;
		else if (flag == 43)
		{
			detectOutlier.DetectOutlier(fileDecode->raw);
			spp.SglPntPos(fileDecode->raw, detectOutlier.curEpk);
			printf("SPP: %4d %9.3f X:%11.4f Y:%11.4f Z:%11.4f B:%11.8f L:%11.8f H:%10.3f GClk:%6.3f BClk:%6.3f PDOP:%5.3f SigmaP:%5.3f GS:%d BS:%d n:%d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
			//fprintf(outFp, "%4d %9.3f %11.4f %11.4f %11.4f %11.8f %11.8f %7.3f %6.3f %6.3f %5.3f %5.3f %d %d %d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
		}
	}
	fclose(inFp);
	//fclose(outFp);
}
