#include "CDecode.h"

void SATOBS::check()
{
	if (fabs(P[0]) < 1.0e-5 || fabs(P[1]) < 1.0e-5 || fabs(L[0]) < 1.0e-5 || fabs(L[1]) < 1.0e-5 || fabs(D[0]) < 1.0e-5 || fabs(D[1]) < 1.0e-5 )
		valid = false;
}

int EPKOBS::FindSatObsIndex(const int prn, const GNSS sys)
{
	for (int i = 0; i < MAXCHANNELNUM; i++)
	{
		if ((satObs[i].prn == prn && satObs[i].sys == sys) || satObs[i].prn == 0)
			return i;//�������ҵ���satObs�����±� �� ���ؿ�satObs�±�
	}
	return 114514;
}

//bool BDSEPHEM::isGeo() const
//{
//	if ((this->satId > 0 && this->satId <= 5 )|| (this->satId >= 59 && this->satId <= 61))
//		return true;
//	else
//		return false;
//}

bool EPHEMERIS::isGeo() const
{
    if ((prn > 0 && prn <= 5 )|| (prn >= 59 && prn <= 61))
        return true;
    else
        return false;
}


int CFileDecode::DecodeOem719Msg(FILE* fp)
{
	unsigned char buf[10240];//������
	memset(buf, 0, 10240);//��ʼ��
	int msgId, lenth;

	while (true)
	{
		//����ʼ��� AA 44 12
		while (true)//����������ѭ�����ڲ�
		{
			if (fread(buf + 2, sizeof(unsigned char), 1, fp) < 1)
				return -114514;
			if (buf[0] == 0xAA && buf[1] == 0x44 && buf[2] == 0x12)
				//�ҵ���ʼ�����ֽ�������ѭ��
				break;
			else
			{
				buf[0] = buf[1];
				buf[1] = buf[2];
			}
		}
		//�ҵ���ʼ�ֽں��������ȡ25bytes һֱ����buf + 27
		if (fread(buf + 3, sizeof(unsigned char), 25, fp) < 25)
			return -114514;

		lenth = U2(buf + 8);//Message lenth
		msgId = U2(buf + 4);//Message ID
		this->t.week = U2(buf + 14);
		this->t.secOfWeek = U4(buf + 16) * 1e-3;

		if (fread(buf + 28/*�����Ŀ�ʼ��*/, sizeof(unsigned char), lenth + 4/*�����Լ�32λCRC��*/, fp) < lenth + 4)
			return -114514;
		//���ݶ�ȡ����

		if (crc32(buf, lenth + 28) != U4(buf + 28 + lenth))
		{
			printf_s("CRCУ��δͨ��\n");
			return 0;
		}
		//crc32У��ͨ�������msgID������һ������
		switch (msgId)
		{
		case 43:
			//range
			DecodeOem719Obs(buf);
			return 43;
		case 42:
			//PSRPOS
			DecodeOem719Bestpos(buf);
			break;
		case 7:
			//GPSEPHEM
			DecodeOem719GpsEphem(buf);
			break;
		case 1696:
			//BDSEPHEMERIS
			DecodeOem719BdsEphem(buf);
			break;
		default:
			break;
		}
	}
}

void CDecode::DecodeOem719Obs(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	memset(&this->raw.epkObs, 0, sizeof(EPKOBS));
	double obsNum = U4(H);
	unsigned char* p = H + 4;//ÿ�ι۲�ֵ����ʼָ�� H+44����p+40

	int chTrStatus;//��12�� ch-tr-status
	int sigType;//Signal Type
	int sysn;//Satellite System
	GNSS sys;//Satellite System�ľ�������
	unsigned short prn;//����prn��
	int freq;//SATOBSƵ���±� 0��1
	int n;//satObs���±�
	double wl;//wavelength

	for (int i = 0; i < obsNum; i++, p += 44)
	{
		prn = U2(p);
		chTrStatus = U4(p + 40);
		int lockedFlag{};
		lockedFlag = chTrStatus >> 12 & 0x01;
		if (lockedFlag == 0)
			continue;//�����Ԫ������δ�����������ɿ���ֱ������

		sigType = chTrStatus >> 21 & 0x1F;
		sysn = chTrStatus >> 16 & 0x07;
		if (sysn == 0)
		{
			sys = GNSS::GPS;
			if (sigType == 0)
			{
				freq = 0;
				wl = constant::c / 1575.42e+6;
			}
			else if (sigType == 9)
			{
				freq = 1;
				wl = constant::c / 1227.60e+6;
			}
			else continue;//signal type��������������
		}
		else if (sysn == 4)
		{
			sys = GNSS::BDS;
			if (sigType == 0 || sigType == 4)
			{
				freq = 0;
				wl = constant::c / 1561.098e+6;
			}
			else if (sigType == 2 || sigType == 6)
			{
				freq = 1;
				wl = constant::c / 1268.52e+6;
			}
			else
				continue;//signal type��������������
		}
		else
			continue;//�������Ͳ�ΪGPS��BDS�е�����һ��

		n = raw.epkObs.FindSatObsIndex(prn, sys);//�ҵ��۲�������staObs�����еĴ洢λ��
		raw.epkObs.satObs[n].prn = prn;
		raw.epkObs.satObs[n].sys = sys;

		raw.epkObs.satObs[n].P[freq] = R8(p + 4);

		raw.epkObs.satObs[n].L[freq] = -wl * R8(p + 16);//Ҫ��-1����Ȼ����ͨ����

		raw.epkObs.satObs[n].D[freq] = R4(p + 28);
		raw.epkObs.satObs[n].valid = true;
	}
	raw.epkObs.t = t;//�۲�����ʱ��
	raw.epkObs.satNum = raw.epkObs.FindSatObsIndex(-1, GNSS::GPS);//ȷ��epkObs��������������Ŀ
}

void CDecode::DecodeOem719GpsEphem(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	unsigned long prn = U4(H);
    
    raw.gpsEphem[prn - 1].satSys = GNSS::GPS;
    raw.gpsEphem[prn - 1].prn = U4(H);
	raw.gpsEphem[prn - 1].health = U4(H + 12);
	raw.gpsEphem[prn - 1].toeG.week = U4(H + 24);
	raw.gpsEphem[prn - 1].toeG.secOfWeek = R8(H + 32);
	raw.gpsEphem[prn - 1].A = R8(H + 40);
	raw.gpsEphem[prn - 1].deltaN = R8(H + 48);
	raw.gpsEphem[prn - 1].m0 = R8(H + 56);
	raw.gpsEphem[prn - 1].ecc = R8(H + 64);
	raw.gpsEphem[prn - 1].omega = R8(H + 72);
	raw.gpsEphem[prn - 1].cuc = R8(H + 80);
	raw.gpsEphem[prn - 1].cus = R8(H + 88);
	raw.gpsEphem[prn - 1].crc = R8(H + 96);
	raw.gpsEphem[prn - 1].crs = R8(H + 104);
	raw.gpsEphem[prn - 1].cic = R8(H + 112);
	raw.gpsEphem[prn - 1].cis = R8(H + 120);
	raw.gpsEphem[prn - 1].i0 = R8(H + 128);
	raw.gpsEphem[prn - 1].iDot = R8(H + 136);
	raw.gpsEphem[prn - 1].omega0 = R8(H + 144);
	raw.gpsEphem[prn - 1].omegaDot = R8(H + 152);

	raw.gpsEphem[prn - 1].toc = R8(H + 164);

	raw.gpsEphem[prn - 1].tgd[0] = R8(H + 172);
	memcpy(raw.gpsEphem[prn - 1].af, H + 180, 24);
}

void CDecode::DecodeOem719BdsEphem(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	unsigned long prn = U4(H);
    
    raw.bdsEphem[prn - 1].satSys = GNSS::BDS;
	raw.bdsEphem[prn - 1].prn = U4(H);
	raw.bdsEphem[prn - 1].toeB.week = U4(H + 4);
	raw.bdsEphem[prn - 1].health = U4(H + 16);
	memcpy(raw.bdsEphem[prn - 1].tgd, H + 20, 16);
	raw.bdsEphem[prn - 1].toc = U4(H + 40);
	memcpy(raw.bdsEphem[prn - 1].af, H + 44, 24);
	raw.bdsEphem[prn - 1].toeB.secOfWeek = U4(H + 72);
	double rootA = R8(H + 76);
	raw.bdsEphem[prn - 1].A = rootA * rootA;
	raw.bdsEphem[prn - 1].ecc = R8(H + 84);
	raw.bdsEphem[prn - 1].omega = R8(H + 92);
	raw.bdsEphem[prn - 1].deltaN = R8(H + 100);
	raw.bdsEphem[prn - 1].m0 = R8(H + 108);
	raw.bdsEphem[prn - 1].omega0 = R8(H + 116);
	raw.bdsEphem[prn - 1].omegaDot = R8(H + 124);
	raw.bdsEphem[prn - 1].i0 = R8(H + 132);
	raw.bdsEphem[prn - 1].iDot = R8(H + 140);
	raw.bdsEphem[prn - 1].cuc = R8(H + 148);
	raw.bdsEphem[prn - 1].cus = R8(H + 156);
	raw.bdsEphem[prn - 1].crc = R8(H + 164);
	raw.bdsEphem[prn - 1].crs = R8(H + 172);
	raw.bdsEphem[prn - 1].cic = R8(H + 180);
	raw.bdsEphem[prn - 1].cis = R8(H + 188);
}

void CDecode::DecodeOem719Bestpos(unsigned char* buf)
{
	//�۲�����ʱ��
	raw.bestPos.time = t;

	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	raw.bestPos.blh.B = R8(H + 8);
	raw.bestPos.blh.L = R8(H + 16);
	raw.bestPos.blh.H = R8(H + 24);
}


unsigned int CDecode::crc32(const unsigned char* buf, int lenth)
{
	unsigned int crc = 0;

	for (int i = 0; i < lenth; i++)
	{
		crc ^= buf[i];
		for (int j = 0; j < 8; j++)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ POLYCRC32;
			else
				crc >>= 1;
		}
	}
	return crc;
}


unsigned short CDecode::U2(unsigned char* buf)
{
	unsigned short val = 0;
	memcpy(&val, buf, 2);
	return val;
}

unsigned int CDecode::U4(unsigned char* buf)
{
	unsigned int val = 0;
	memcpy(&val, buf, 4);
	return val;
}

float CDecode::R4(unsigned char* buf)
{
	float val = 0.0;
	memcpy(&val, buf, 4);
	return val;
}

double CDecode::R8(unsigned char* buf)
{
	double val = 0.0;
	memcpy(&val, buf, 8);
	return val;
}

FILE* CFileDecode::FileRead(const char* fileName)
{
	FILE* fp;
	fopen_s(&fp, fileName, "rb");
	if (fp == nullptr)
	{
		printf("Cannot open OEM719 file. \n");
		std::abort();
	}
	else
	{
		return fp;
	}
}

int CSocketDecode::DecodeOem719Msg(unsigned char* buf, int curLen, int& lenRem)
{
	int i = 0, val = 0;
	int len = curLen + lenRem;//�ɶ������ܳ���
	unsigned char Buff[10240];//������Ϣ������
	int msgId, msgLen;

	while (true)
	{
		for (; i < len - 3; i++)
		{
			if (buf[i] == 0xAA && buf[i + 1] == 0x44 && buf[i + 2] == 0x12)
			{
				break;   //�ҵ����������к���Ķ�ȡ��
			}
			//����ѭ���ҵ�AA4412Ϊֹ 178 68 18
		}
		if (i + 28 <= len)
		{
			//˵���˶α���ʣ�µ����ݴ�С���ڵ���һ��ͷ�ļ�
			msgLen = U2(buf + i + 8);
		}
		else
		{
			lenRem = len - i;//�ɶ�����������������ǰ��λ��
			memcpy(buf, buf + i, lenRem);
			memset(buf + lenRem, 0, 204800 - lenRem);
			break;//�˳��ôα��Ľ���
		}

		if (i + msgLen + 28 + 4 <= len)
		{
			//�˶α���ʣ�����ݴ��ڵ���һ��������Ϣ
			memcpy(Buff, buf + i, msgLen + 28 + 4);//������Ϣ������׼������
			i += msgLen + 28 + 4;
		}
		else
		{
			lenRem = len - i;//ʣ�����ݴ�����;
			memcpy(buf, buf + i, lenRem);
			break;
		}
		//CRCУ��
		if (crc32(Buff, msgLen + 28) != U4(Buff + msgLen + 28))
		{
			lenRem = len - i;
			memcpy(buf, buf + i, len - i);
			return -114514;
		}

		this->t.week = U2(Buff + 14);
		this->t.secOfWeek = U4(Buff + 16) * 1e-3;

		msgId = U2(Buff + 4);

		//crc32У��ͨ�������msgID������һ������
		switch (msgId)
		{
		case 43:
		//range
		{
			DecodeOem719Obs(Buff);
			lenRem = len - i;
			memcpy(buf, buf + i, lenRem);
			return 43;
		}
		case 42:
		//BESTPOS
		{
			DecodeOem719Bestpos(Buff);
			break;
		}
		case 7:
		//GPSEPHEM
		{
			DecodeOem719GpsEphem(Buff);
			break;
		}
		case 1696:
		//BDSEPHEMERIS
		{
			DecodeOem719BdsEphem(Buff);
			break;
		}
		default:
			break;
		}
	}

	return val;
}

bool CSocketDecode::OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{
	WSADATA wsaData;
	SOCKADDR_IN addrSrv;

	if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
	{
		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET)
		{
			addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(Port);
			connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
			return true;
		}
	}
	return false;
}

void CSocketDecode::CloseSocket(SOCKET& sock)
{
	closesocket(sock);
	WSACleanup();
}

