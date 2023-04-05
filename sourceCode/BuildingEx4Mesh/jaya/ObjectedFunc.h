#ifndef OBJECTEDFUNCBASE
#define OBJECTEDFUNCBASE

#include <vector>
#include <stdlib.h>
#include <time.h>

//Ŀ�꺯���ĳ������
//�̳д�Ŀ�꺯��
class ObjectedFuncBase
{
public:
	ObjectedFuncBase(){}
	~ObjectedFuncBase(){}

	virtual double getValue(std::vector<double> solution) = 0;
};

class HimmelblauFunc : public ObjectedFuncBase
{
public:
	HimmelblauFunc(){}
	~HimmelblauFunc(){}

	double getValue(std::vector<double> solution)
	{
		return pow(solution[0] * solution[0] + solution[1] - 11, 2) + pow(solution[0] + solution[1] * solution[1] - 7, 2);
	}

private:

};

#endif // OBJECTEDFUNCBASE
