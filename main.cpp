#include <iostream>
#include <iomanip>
#include <fstream>
#include "CMatrix.h"
#include "TimeConversion.h"
#include "CoordinateConversion.hpp"
#include "Constant.h"
#include "CDecode.h"
#include "SatPositioning.hpp"
#include "CDetectOutlier.h"
#include "Client.h"

int main()
{
	Client client;
	client.FileSpp();

	return 0;
}

