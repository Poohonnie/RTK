#include <iostream>
#include <iomanip>
#include <fstream>
#include "CMatrix.h"
#include "TimeConversion.h"
#include "CoordinateConversion.hpp"
#include "Constant.h"
#include "CDecode.h"


int main()
{
	CFileDecode fileDecode;
	char fileName[50] = "OEM719-1126\\202010261820.oem719";
	FILE* fp = fileDecode.FileRead(fileName);
	while (1)
	{
		fileDecode.DecodeOem719Msg(fp);
	}

	return 0;
}

