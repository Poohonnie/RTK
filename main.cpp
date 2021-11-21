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
	client.Run();
	//double a[9] = { 1.9, 2.3, 3.4, 4, 4.13, 6, 7, 7.9, 8 };
	//double b[3] = { 3.14, 2.74, 9.81 };
	//CMatrix A(a, 3, 3);
	//CMatrix B(b, 3, 1);
	//CMatrix pro = A * B;
	//pro.Show();

	return 0;
}

