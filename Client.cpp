#include "Client.h"

void Client::Run()
{
	int lenRem = 0;//�ϴν��������ֽ�
	int curLen = 0;//�˴ν��յ��ı����ܳ���
	unsigned char* buf = new unsigned char[204800];
	memset(buf, 0, 204800);
	CSocketDecode* socketDecode = new CSocketDecode;//������
	CDetectOutlier detectOutlier;//�ֲ�̽����
	SOCKET sock;//�׽���
	SPP spp;//���㶨λ��
	char IP[20] = "47.114.134.129";//IP��ַ
	unsigned short port = 7190;//�˿�
	int val;//���뷵��ֵ
	if (socketDecode->OpenSocket(sock, IP, port) == false)
	{
		//����ͨ��ʧ��
		printf("Cannot open socket.\n");
		return;
	}
	//FILE* outfp = fopen("202111211136.oem719.pos", "a+");

	while(1)
	{
		if (lenRem < 10240)
			Sleep(1000);
		//��ֹ�����ٶ�׷���Ͻ�����Ϣ�ٶ�
		if (lenRem > 204800 || lenRem < 0)
		{
			//��ֹ����
			lenRem = 0;
			memset(buf, 0, 204800);
		}
		curLen = recv(sock, (char*)buf + lenRem, 204800 - lenRem, 0);//�������Ļ����������Ͻ��ձ���
		val = socketDecode->DecodeOem719Msg(buf, curLen, lenRem);//������յ��ı��Ľ���
		if (val == 43)
		{
			detectOutlier.DetectOutlier(socketDecode->raw);
			spp.StdPntPos(socketDecode->raw, detectOutlier.curEpk);
			spp.StdPntVel(socketDecode->raw, detectOutlier.curEpk);
			printf("SPP: %4d %9.3f X:%11.4f Y:%11.4f Z:%11.4f B:%11.8f L:%11.8f H:%10.3f Vx:%10.4f Vy:%10.4f Vz:%10.4f GClk:%6.3f BClk:%6.3f PDOP:%5.3f SigmaP:%5.3f SigmaV:%5.3f GS:%d BS:%d n:%d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnV[0], spp.sttnV[1], spp.sttnV[2], spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.sigmaV, spp.gNum, spp.bNum, spp.bNum + spp.gNum);

			//printf("SPP: %4d %9.3f X:%11.4f Y:%11.4f Z:%11.4f B:%11.8f L:%11.8f H:%10.3f GClk:%6.3f BClk:%6.3f PDOP:%5.3f SigmaP:%5.3f GS:%d BS:%d n:%d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
			//fprintf(outfp, "%4d %9.3f %11.4f %11.4f %11.4f %11.8f %11.8f %7.3f %6.3f %6.3f %5.3f %5.3f %d %d %d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
		}

	}
	delete[] buf;
	delete socketDecode;

	//fclose(outfp);
	return;
}

void Client::FileSpp()
{
	CFileDecode* fileDecode = new CFileDecode;
	SatPositioning satPositioning;
	CDetectOutlier detectOutlier;
	SPP spp;//���㶨λ��
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
			spp.StdPntPos(fileDecode->raw, detectOutlier.curEpk);
			spp.StdPntVel(fileDecode->raw, detectOutlier.curEpk);
			printf("SPP: %4d %9.3f X:%11.4f Y:%11.4f Z:%11.4f B:%11.8f L:%11.8f H:%10.3f Vx:%10.4f Vy:%10.4f Vz:%10.4f GClk:%6.3f BClk:%6.3f PDOP:%5.3f SigmaP:%5.3f SigmaV:%5.3f GS:%d BS:%d n:%d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnV[0], spp.sttnV[1], spp.sttnV[2], spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.sigmaV, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
			//fprintf(outfp, "%4d %9.3f %11.4f %11.4f %11.4f %11.8f %11.8f %7.3f %6.3f %6.3f %5.3f %5.3f %d %d %d\n", spp.t.week, spp.t.secOfWeek, spp.sttnXyz.x, spp.sttnXyz.y, spp.sttnXyz.z, spp.sttnBlh.B * 180.0 / constant::pi, spp.sttnBlh.L * 180.0 / constant::pi, spp.sttnBlh.H, spp.sttnClkG, spp.sttnClkB, spp.PDOP, spp.sigmaP, spp.gNum, spp.bNum, spp.bNum + spp.gNum);
		}
	}
	fclose(inFp);
	//fclose(outFp);
}
