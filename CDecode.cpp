#include "CDecode.h"

void SATOBS::check()
{
	if (fabs(this->P[0]) < 1.0e-5 || fabs(this->P[1]) < 1.0e-5 || fabs(this->L[0]) < 1.0e-5 || fabs(this->L[1]) < 1.0e-5 || fabs(this->D[0]) < 1.0e-5 || fabs(this->D[1]) < 1.0e-5 )
		this->valid = false;
}

void CDecode::Reset()
{
	memset(this, 0, sizeof(CDecode));
}

int EPKOBS::FindSatObsIndex(const int prn, const GNSS sys)
{
	for (int i = 0; i < MAXCHANNELNUM; i++)
	{
		if (this->satObs[i].prn == prn && this->satObs[i].sys == sys)
			return i;//�������ҵ���satObs�����±�
		else if (this->satObs[i].prn == 0)
			return i;//���ؿ�satObs�±�
	}
	return 114514;
}

bool BDSEPHEM::isGeo() const
{
	if ((this->satId > 0 && this->satId <= 5 )|| (this->satId >= 59 && this->satId <= 61))
		return true;
	else
		return false;
	return false;
}

int CFileDecode::DecodeOem719Msg(FILE* fp)
{
	unsigned char buf[10240];//������
	memset(buf, 0, 10240);//��ʼ��
	int msgId, lenth;

	while (1)
	{
		//����ʼ��� AA 44 12
		while (1)//����������ѭ�����ڲ�
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
		{
			DecodeOem719Obs(buf);
			return 43;
		}
		case 42:
			//PSRPOS
		{
			DecodeOem719Bestpos(buf);
			break;
		}
		case 7:
			//GPSEPHEM
		{
			DecodeOem719GpsEphem(buf);
			break;
		}
		case 1696:
			//BDSEPHEMERIS
		{
			DecodeOem719BdsEphem(buf);
			break;
		}
		default:
			break;
		}
	}
	return 0;
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
	int prn;//����prn��
	int freq;//SATOBSƵ���±� 0��1
	int n;//satObs���±�
	double wl;//wavelength

	for (int i = 0; i < obsNum; i++, p += 44)
	{
		prn = U2(p);
		chTrStatus = U4(p + 40);
		int lockedFlag = 1;
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

		n = this->raw.epkObs.FindSatObsIndex(prn, sys);//�ҵ��۲�������staObs�����еĴ洢λ��
		this->raw.epkObs.satObs[n].prn = prn;
		this->raw.epkObs.satObs[n].sys = sys;

		this->raw.epkObs.satObs[n].P[freq] = R8(p + 4);
		this->raw.epkObs.satObs[n].psrSigma[freq] = R4(p + 12);

		this->raw.epkObs.satObs[n].L[freq] = -wl * R8(p + 16);//Ҫ��-1����Ȼ����ͨ����
		this->raw.epkObs.satObs[n].cpSigma[freq] = R4(p + 24);

		this->raw.epkObs.satObs[n].D[freq] = R4(p + 28);
		this->raw.epkObs.satObs[n].cnr = R4(p + 32);
		this->raw.epkObs.satObs[n].valid = true;
	}
	this->raw.epkObs.t = this->t;//�۲�����ʱ��
	this->raw.epkObs.satNum = this->raw.epkObs.FindSatObsIndex(-1, GNSS::GPS);//ȷ��epkObs��������������Ŀ
}

void CDecode::DecodeOem719GpsEphem(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	int prn = U4(H);
	this->raw.gpsEphem[prn - 1].prn = U4(H);
	this->raw.gpsEphem[prn - 1].tow = R8(H + 4);
	this->raw.gpsEphem[prn - 1].health = U4(H + 12);
	memcpy(this->raw.gpsEphem[prn - 1].iode, H + 16, 8);
	this->raw.gpsEphem[prn - 1].toe.week = U4(H + 24);
	//z week ��ȥ
	this->raw.gpsEphem[prn - 1].toe.secOfWeek = R8(H + 32);
	this->raw.gpsEphem[prn - 1].A = R8(H + 40);
	this->raw.gpsEphem[prn - 1].deltaN = R8(H + 48);
	this->raw.gpsEphem[prn - 1].m0 = R8(H + 56);
	this->raw.gpsEphem[prn - 1].ecc = R8(H + 64);
	this->raw.gpsEphem[prn - 1].omega = R8(H + 72);
	this->raw.gpsEphem[prn - 1].cuc = R8(H + 80);
	this->raw.gpsEphem[prn - 1].cus = R8(H + 88);
	this->raw.gpsEphem[prn - 1].crc = R8(H + 96);
	this->raw.gpsEphem[prn - 1].crs = R8(H + 104);
	this->raw.gpsEphem[prn - 1].cic = R8(H + 112);
	this->raw.gpsEphem[prn - 1].cis = R8(H + 120);
	this->raw.gpsEphem[prn - 1].i0 = R8(H + 128);
	this->raw.gpsEphem[prn - 1].iDot = R8(H + 136);
	this->raw.gpsEphem[prn - 1].omega0 = R8(H + 144);
	this->raw.gpsEphem[prn - 1].omegaDot = R8(H + 152);
	this->raw.gpsEphem[prn - 1].iodc = U4(H + 160);

	this->raw.gpsEphem[prn - 1].toc.week = this->raw.gpsEphem[prn - 1].toe.week;//����
	this->raw.gpsEphem[prn - 1].toc.secOfWeek = R8(H + 164);

	this->raw.gpsEphem[prn - 1].tgd = R8(H + 172);
	memcpy(this->raw.gpsEphem[prn - 1].af, H + 180, 24);
	this->raw.gpsEphem[prn - 1].as = U4(H + 204);
	this->raw.gpsEphem[prn - 1].N = R8(H + 208);
	this->raw.gpsEphem[prn - 1].ura = R8(H + 216);
}

void CDecode::DecodeOem719BdsEphem(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	int satId = U4(H);
	this->raw.bdsEphem[satId - 1].satId = U4(H);
	this->raw.bdsEphem[satId - 1].toe.week = U4(H + 4);
	this->raw.bdsEphem[satId - 1].ura = R8(H + 8);
	this->raw.bdsEphem[satId - 1].health = U4(H + 16);
	memcpy(this->raw.bdsEphem[satId - 1].tgd, H + 20, 16);
	this->raw.bdsEphem[satId - 1].aodc = U4(H + 36);
	this->raw.bdsEphem[satId - 1].toc = U4(H + 40);
	memcpy(this->raw.bdsEphem[satId - 1].a, H + 44, 24);
	this->raw.bdsEphem[satId - 1].aode = U4(H + 68);
	this->raw.bdsEphem[satId - 1].toe.secOfWeek = U4(H + 72);
	this->raw.bdsEphem[satId - 1].rootA = R8(H + 76);
	this->raw.bdsEphem[satId - 1].A = this->raw.bdsEphem[satId - 1].rootA * this->raw.bdsEphem[satId - 1].rootA;
	this->raw.bdsEphem[satId - 1].ecc = R8(H + 84);
	this->raw.bdsEphem[satId - 1].omega = R8(H + 92);
	this->raw.bdsEphem[satId - 1].deltaN = R8(H + 100);
	this->raw.bdsEphem[satId - 1].m0 = R8(H + 108);
	this->raw.bdsEphem[satId - 1].omega0 = R8(H + 116);
	this->raw.bdsEphem[satId - 1].omegaDot = R8(H + 124);
	this->raw.bdsEphem[satId - 1].i0 = R8(H + 132);
	this->raw.bdsEphem[satId - 1].iDot = R8(H + 140);
	this->raw.bdsEphem[satId - 1].cuc = R8(H + 148);
	this->raw.bdsEphem[satId - 1].cus = R8(H + 156);
	this->raw.bdsEphem[satId - 1].crc = R8(H + 164);
	this->raw.bdsEphem[satId - 1].crs = R8(H + 172);
	this->raw.bdsEphem[satId - 1].cic = R8(H + 180);
	this->raw.bdsEphem[satId - 1].cis = R8(H + 188);
}

void CDecode::DecodeOem719Bestpos(unsigned char* buf)
{
	//�۲�����ʱ��
	this->raw.bestPos.time = this->t;

	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	this->raw.bestPos.blh.B = R8(H + 8);
	this->raw.bestPos.blh.L = R8(H + 16);
	this->raw.bestPos.blh.H = R8(H + 24);
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

RAWDATA CDecode::Raw()
{
	return this->raw;
}

FILE* CFileDecode::FileRead(const char* fileName)
{
	FILE* fp;
	fopen_s(&fp, fileName, "rb");
	if (fp == NULL)
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

	while (1)
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