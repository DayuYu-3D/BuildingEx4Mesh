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

	//返回一个double值，在给定的范围内
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

	//将一个数字转到指定范围内
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
	double _bottom; //下界
	double _upper;  //上界

	int _type; //类型，1为double， 2为int，默认为double
};


class Solution
{
public:
	//vValRanges: 参数的上下界
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

	//计算该solution的函数值
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
	double _value; //该solution的函数值

	std::vector<double> _solution; //一个solution	

private:
	int _numVars; //变量个数
	std::vector<Variable> _vVriable; //变量列表，每个变量都有它的上下界

	ObjectedFuncBase* _func; //待优化的目标函数
};

#endif // SOLUTION
