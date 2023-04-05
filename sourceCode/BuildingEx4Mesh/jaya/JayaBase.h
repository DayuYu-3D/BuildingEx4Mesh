#ifndef JAYABASE
#define JAYABASE

#include <vector>

#include "Population.h"

/**
Jaya����㷨�Ļ���
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

	//����population
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

	//���������
	//�����������û�����Ƶı���������������
	std::vector<std::vector < std::vector<double> >> generate_rn(int numIterations)
	{		
		//time_t t;//����seed
		//srand((unsigned)time(&t));

		std::vector<std::vector < std::vector<double> >> rn(numIterations);
		for (int i = 0; i < numIterations; ++i)
		{
			rn[i] = std::vector < std::vector<double> >(_numVars);
			for (int j = 0; j < _numVars; ++j)
			{
				//ȡ0-1֮��������
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
	//TODO:Ŀ�꺯��
	int _numSolutions; //population����.
	std::vector<Variable> _vriables; //�����б�ÿ�����������������½�
	int _numVars; //�����ĸ���
	bool _bMinOrMax; //Objective function to be (false: minimized or true: maximized)

	Population _populaiton; 

	ObjectedFuncBase* _func; //���Ż���Ŀ�꺯��
};


#endif // JAYABASE
