#include "CDecode.h"

int CDecode::DecodeOem719Msg(FILE* fp)
{
	unsigned char buf[10240];//������
	int msgId, lenth;

	//����ʼ��� AA 44 12
	while(1)//����������ѭ�����ڲ�
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

	if (fread(buf + 28/*�����Ŀ�ʼ��*/, sizeof(unsigned char), lenth + 4/*�����Լ�32λCRC��*/, fp) < lenth + 4)
		return -114514;
	//���ݶ�ȡ����

	if (crc32(buf, lenth + 28) != U4(buf + 28 + lenth))
	{
		printf_s("CRCУ��δͨ��");
		return 0;
	}
	//crc32У��ͨ�������msgID������һ������
	switch (msgId)
	{
	case 43:
		//range
		DecodeOem719Obs(buf);
		break;
	case 47:
		//PSRPOS
		DecodeOem719Psrpos(buf);
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

	return 0;
}

void CDecode::DecodeOem719Obs(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	memset(&this->epcObs, 0, sizeof(EPCOBS));
	this->epcObs.satNum = U4(H);
	unsigned char* p = H + 4;//ÿ�ι۲�ֵ����ʼָ�� H+44����p+40

	int chTrStatus;//��12�� ch-tr-status
	int sigType;//Signal Type
	int sysn;//Satellite System
	GNSS sys;//Satellite System�ľ�������
	int prn;//����prn��
	int freq;//SATOBSƵ���±� 0��1
	int n;//satObs���±�

	for (int i = 0; i < this->epcObs.satNum; i++, p += 44)
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
				freq = 0;
			else if (sigType == 9)
				freq = 1;
			else continue;//signal type��������������
		}
		else if (sysn == 4)
		{
			sys = GNSS::BDS;
			if (sigType == 0 || sigType == 4)
				freq = 0;
			else if (sigType == 2 || sigType == 6)
				freq = 1;
			else
				continue;//signal type��������������
		}
		else
			continue;//�������Ͳ�ΪGPS��BDS�е�����һ��

		n = FindSatObsIndex(prn, sys);//�ҵ��۲�������staObs�����еĴ洢λ��
		this->epcObs.satObs[n].prn = prn;
		this->epcObs.satObs[n].sys = sys;

		this->epcObs.satObs[n].P[freq] = R8(p + 4);
		this->epcObs.satObs[n].psrSigma[freq] = U4(p + 12);

		this->epcObs.satObs[n].L[freq] = R8(p + 16);
		this->epcObs.satObs[n].cpSigma[freq] = U4(p + 24);

		this->epcObs.satObs[n].D[freq] = U4(p + 28);
		this->epcObs.satObs[n].cnr = U4(p + 32);
	}
}

void CDecode::DecodeOem719GpsEphem(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	this->gpsEphem.prn = U4(H);
	this->gpsEphem.tow = R8(H + 4);
	this->gpsEphem.health = U4(H + 12);
	memcpy(this->gpsEphem.iode, H + 16, 8);
	this->gpsEphem.toe.week = U4(H + 24);
	//z week ��ȥ
	this->gpsEphem.toe.secOfWeek = R8(H + 32);
	this->gpsEphem.A = R8(H + 40);
	this->gpsEphem.deltaN = R8(H + 48);
	this->gpsEphem.m0 = R8(H + 56);
	this->gpsEphem.ecc = R8(H + 64);
	this->gpsEphem.omega = R8(H + 72);
	this->gpsEphem.cuc = R8(H + 80);
	this->gpsEphem.cus = R8(H + 88);
	this->gpsEphem.crc = R8(H + 96);
	this->gpsEphem.crs = R8(H + 104);
	this->gpsEphem.cic = R8(H + 112);
	this->gpsEphem.cis = R8(H + 120);
	this->gpsEphem.i0 = R8(H + 128);
	this->gpsEphem.iDot = R8(H + 136);
	this->gpsEphem.omega0 = R8(H + 144);
	this->gpsEphem.omegaDot = R8(H + 152);
	this->gpsEphem.iodc = U4(H + 160);
	this->gpsEphem.toc = R8(H + 164);
	this->gpsEphem.tgd = R8(H + 172);
	memcpy(this->gpsEphem.af, H + 180, 24);
	this->gpsEphem.as = U4(H + 204);
	this->gpsEphem.N = R8(H + 208);
	this->gpsEphem.ura = R8(H + 216);
}

void CDecode::DecodeOem719BdsEphem(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	this->bdsEphem.satId = U4(H);
	this->bdsEphem.toe.week = U4(H + 4);
	this->bdsEphem.ura = R8(H + 8);
	this->bdsEphem.health = U4(H + 16);
	memcpy(this->bdsEphem.tgd, H + 20, 16);
	this->bdsEphem.aodc = U4(H + 36);
	this->bdsEphem.toc = U4(H + 40);
	memcpy(this->bdsEphem.a, H + 44, 24);
	this->bdsEphem.aode = U4(H + 68);
	this->bdsEphem.toe.secOfWeek = U4(H + 72);
	this->bdsEphem.rootA = R8(H + 76);
	this->bdsEphem.ecc = R8(H + 84);
	this->bdsEphem.omega = R8(H + 92);
	this->bdsEphem.deltaN = R8(H + 100);
	this->bdsEphem.m0 = R8(H + 108);
	this->bdsEphem.omega0 = R8(H + 116);
	this->bdsEphem.omegaDot = R8(H + 124);
	this->bdsEphem.i0 = R8(H + 132);
	this->bdsEphem.iDot = R8(H + 140);
	this->bdsEphem.cuc = R8(H + 148);
	this->bdsEphem.cus = R8(H + 156);
	this->bdsEphem.crc = R8(H + 164);
	this->bdsEphem.crs = R8(H + 172);
	this->bdsEphem.cic = R8(H + 180);
	this->bdsEphem.cis = R8(H + 188);
}

void CDecode::DecodeOem719Psrpos(unsigned char* buf)
{
	unsigned char* H = buf + 28;//ͷ�ļ�������������ʼ
	this->psrPos.blh.B = R8(H + 8);
	this->psrPos.blh.L = R8(H + 16);
	this->psrPos.blh.H = R8(H + 24);
}

int CDecode::FindSatObsIndex(const int prn, const GNSS sys)
{
	for (int i = 0; i < MAXCHANNELNUM; i++)
	{
		if (this->epcObs.satObs[i].prn == prn && this->epcObs.satObs[i].sys == sys)
			return i;//�������ҵ���satObs�����±�
		else if (this->epcObs.satObs[i].prn == 0)
			return i;//���ؿ�satObs�±�
	}
	return -114514;
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

unsigned long CDecode::CRC32Value(int i)
{
	/*************************************
	*		�α��ϵĴ��룬ֱ�ӳ�
	*	�����һ����CRC�������ʹ�õ�CRC��
	*************************************/
	unsigned long ulCRC = i;
	for (int j = 0; j < 8; j++)
	{
		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}

	return ulCRC;
}

unsigned long CDecode::CalculateBloackCRC32(unsigned long ulCount, unsigned char* ucBuffer)
{
	/******************************
	*	   �α��ϵĴ��룬ֱ�ӳ�
	*	��Ҳ��֪����������Ǹ����
	******************************/
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while (ulCount-- > 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xFF);
		ulCRC = ulTemp1 ^ ulTemp2;
	}

	return ulCRC;
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

double CDecode::R8(unsigned char* buf)
{
	double val = 0.0;
	memcpy(&val, buf, 8);
	return val;
}

EPCOBS CDecode::EpcObs()
{
	return this->epcObs;
}

GPSEPHEM CDecode::GpsEphem()
{
	return this->gpsEphem;
}

BDSEPHEM CDecode::BdsEphem()
{
	return this->bdsEphem;
}

PSRPOS CDecode::PsrPos()
{
	return this->psrPos;
}

FILE* CFileDecode::FileRead(const char* fileName)
{
	FILE* fp;
	fopen_s(&fp, fileName, "rb");
	if (fp == NULL)
	{
		printf("Cannot open GPS obs file. \n");
		std::abort();
	}
	else
	{
		return fp;
	}
}

