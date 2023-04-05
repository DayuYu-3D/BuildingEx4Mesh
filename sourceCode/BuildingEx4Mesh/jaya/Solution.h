#ifndef SOLUTION
#define SOLUTION

#include <vector>
#include <stdlib.h>
#include <time.h>

#include "ObjectedFunc.h"

class Variable
{
public:
	Variable(double bottom, double upper)
		:_bottom(bottom),
		_upper(upper),
		_type(1)
	{}
	~Variable() {}

	void setType(int type)
	{
		_type = type;
	}

	//����һ��doubleֵ���ڸ����ķ�Χ��
	double get() const 
	{
		double randNum = (double)rand() / (double)(RAND_MAX);
		if (_type == 1) //double
		{
			return (_upper - _bottom) * randNum + _bottom;
		}
		else if (_type == 2) //int
		{
			return (int)((_upper - _bottom) * randNum + _bottom);
		}
	}

	//��һ������ת��ָ����Χ��
	double convert(double item) const
	{
		if (_type == 1) //double
		{
			double temp = item > _upper ? _upper : item;
			return temp < _bottom ? _bottom : temp;
		}
		else if (_type == 2) //int
		{
			double temp = item > _upper ? _upper : item;
			int value = temp < _bottom ? _bottom : temp;
			return value;
		}
	}

private:
	double _bottom; //�½�
	double _upper;  //�Ͻ�

	int _type; //���ͣ�1Ϊdouble�� 2Ϊint��Ĭ��Ϊdouble
};


class Solution
{
public:
	//vValRanges: ���������½�
	Solution(std::vector<Variable> vVriable,ObjectedFuncBase* func)
		:
		_numVars(vVriable.size()),
		_vVriable(vVriable),
		_func(func)
	{

	}
	~Solution()
	{}

	void generate()
	{
		_solution.resize(_numVars);
		for (size_t i = 0; i < _solution.size(); ++i)
		{
			_solution[i] = _vVriable[i].get();
		}
		_value = this->evaluate();
	}

	//�����solution�ĺ���ֵ
	double evaluate()
	{
		return _func->getValue(_solution);
	}

	void setSolution(std::vector<double> solution)
	{
		_solution = solution;
		_value = this->evaluate();
	}

public:
	double _value; //��solution�ĺ���ֵ

	std::vector<double> _solution; //һ��solution	

private:
	int _numVars; //��������
	std::vector<Variable> _vVriable; //�����б�ÿ�����������������½�

	ObjectedFuncBase* _func; //���Ż���Ŀ�꺯��
};

#endif // SOLUTION
