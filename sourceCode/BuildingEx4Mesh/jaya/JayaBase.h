#ifndef JAYABASE
#define JAYABASE

#include <vector>

#include "Population.h"

/**
Jaya相关算法的基类
*/
class JayaBase
{
public:
	JayaBase(int numSolutions, std::vector<Variable> vriables, bool bMinOrMax, ObjectedFuncBase* func)
		:
		_numSolutions(numSolutions),
		_vriables(vriables),
		_numVars(vriables.size()),
		_bMinOrMax(bMinOrMax),
		_func(func)
	{
		_populaiton = this->generatePopulation();
	}

	~JayaBase() {}

	//产生population
	Population generatePopulation()
	{
		Population population;
		population.setObjectedFuncBase(_func);
		population.setMaximize(_bMinOrMax);
		population.generate(_numSolutions, _vriables);
		return population;
	}

	void setPopulation(Population population)
	{
		_populaiton = population;
	}

	//产生随机数
	//用于随机生成没有限制的变量，先舍弃不用
	std::vector<std::vector < std::vector<double> >> generate_rn(int numIterations)
	{		
		//time_t t;//输入seed
		//srand((unsigned)time(&t));

		std::vector<std::vector < std::vector<double> >> rn(numIterations);
		for (int i = 0; i < numIterations; ++i)
		{
			rn[i] = std::vector < std::vector<double> >(_numVars);
			for (int j = 0; j < _numVars; ++j)
			{
				//取0-1之间的随机数
				double randNum = (double)rand() / (double)(RAND_MAX);
				rn[i][j].push_back(randNum);
				double randNum2 = (double)rand() / (double)(RAND_MAX);
				rn[i][j].push_back(randNum2);
			}
		}
		return rn;
	}

	void toMaximize()
	{
		this->_bMinOrMax = true;
		this->_populaiton.toMaximize();
	}

	BestAndWorst getBestAndWorst()
	{
		return this->_populaiton.getBestAndWorstValue();
	}

	virtual Population run(int numIterations) = 0;

public:
	//TODO:目标函数
	int _numSolutions; //population个数.
	std::vector<Variable> _vriables; //变量列表，每个变量都有它的上下界
	int _numVars; //变量的个数
	bool _bMinOrMax; //Objective function to be (false: minimized or true: maximized)

	Population _populaiton; 

	ObjectedFuncBase* _func; //待优化的目标函数
};


#endif // JAYABASE
