/********************************************
 * Class: advancingFronReconstruct
 * Author: Dayu
 * Site: WHU
 * Date: 2020514
 * email: dayuyu@whu.edu.cn
********************************************/
#ifndef advancingFronReconstruct_H
#define advancingFronReconstruct_H

#include <string>

#include "PCH.h"

class surfaceReconstruct
{
public:
	surfaceReconstruct(std::string pointPath);
	~surfaceReconstruct();
	
	void doReconstruct();

private:

	std::string _pointPath;
};


#endif