
#include <fstream>

#include "io.h"
#include "common.h"
#include <errno.h>

enum  SensorNum { LeftSensor, FrontSensor, RightSensor, AllSensors };

void senorInFile(SensorNum num, int* size, int* offset){

	switch (num)
	{
	case LeftSensor:
		*offset = 4;
		*size = 2;
		break;
	case FrontSensor:
		*offset = 6;
		*size = 2;
		break;
	case RightSensor:
		*offset = 8;
		*size = 2;
		break;
	case AllSensors:
		*offset = 4;
		*size = 6;
		break;
	default:
		break;
	}
}

void client::readScene(std::string file, std::vector<double>& points){
	
	/*Read File*/
	//setlocale(LC_ALL, "RUS");

	std::fstream f;
	f.open(file, std::ios::in);
	if(!f.is_open())
		throw client::RDAException(strerror(errno));
	
	std::string buf;
	getline(f, buf);

	int lineNumber = 0;
	int readFromLineNum = 0;
	int size = 2;		// количество считываемых элементов
	int offset = 4; // сдвиг относительно 0-ого
	
	points.push_back(0); // reserv element for size

	std::vector<std::string> a;
	client::split(buf, " ", a);

	if (strcmp(a[2].c_str(), "0.") == 0){
		readFromLineNum = 1;
	}
	if (strcmp(a[2].c_str(), "1.") == 0){
		readFromLineNum = 12;
	}
	if (strcmp(a[2].c_str(), "2.") == 0){
		readFromLineNum = 27;		
		senorInFile(SensorNum::FrontSensor, &size, &offset);		
	}

	while (!f.eof())
	{
		if (lineNumber++ >= readFromLineNum)
		{
			std::vector<std::string> a;
			client::split(buf, " ", a);

			for (int i = 0; i < size; i++)
				if (a[offset + i][0] != 'n'){				
					points.push_back(atof(a[offset + i].c_str()));
				if((i & 1) == 1){
					points.push_back(1.0); // z;
					points.push_back(2.0);
					points.push_back(3.0);
				}
			}			
		}
		getline(f, buf);
	}

	points[0] = points.size();

	f.close();

}

void client::writeScene(std::string file, double** cloud, int size, int point_size){
	std::ofstream f;
	f.open(file);
	
	for(int i = 0; i < size; i++){
		
		for(int j = 1; j < cloud[i][0]-1; j+= point_size){			
			for(int k = 0; k < point_size; k++){
				f << cloud[i][j+k] << " ";
			}
			f << std::endl;
		}
		f << std::endl;
	}

	f.close();
	
}