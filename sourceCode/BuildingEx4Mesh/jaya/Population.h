#ifndef POPULATION
#define POPULATION

#include <vector>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include "Solution.h"

struct BestAndWorst
{
	double bestV = 0.;
	double worstV = 0.;
	Solution bestSolution;
	Solution worstSolution;
};

class Population
{
public:
	Population(){}
	~Population(){}

	void setMaximize(bool minOrmax)
	{
		_bMinOrMax = minOrmax;
	}
	void setObjectedFuncBase(ObjectedFuncBase* func)
	{
		_func = func;
	}

	//numSolutions (int): Number of solutions.
	//vVriable (list): Range list.
	//space(bool) : Spaced numbers over a specified interval.
	void generate(int numSolutions, std::vector<Variable> vVriable)
	{
		for (int i = 0; i < numSolutions; ++i)
		{
			Solution solution = Solution(vVriable, _func);
			solution.generate();
			_solutions.push_back(solution);
		}
	}

	void toMaximize()
	{
		this->_bMinOrMax = true;
	}

	//����������������solutions
	std::vector<Solution> sorted()
	{
		std::vector<Solution> sortedSolutions = _solutions;
		using namespace std;
		std::sort(
			sortedSolutions.begin(), sortedSolutions.end(),
			[=](const Solution& u, const Solution& v)
			{
				//trueΪ����falseΪ����
				return _bMinOrMax ? (u._value > v._value) : (u._value < v._value);
			}
		);
		return sortedSolutions;
	}

	//���أ�Best value, worst value, best solution and worst solution.
	BestAndWorst getBestAndWorstValue()
	{		
		std::vector<Solution> sortedSolutions = this->sorted();
		if (_bMinOrMax)
		{
			size_t n = sortedSolutions.size();
			BestAndWorst bestandworst = { sortedSolutions[n - 1]._value, sortedSolutions[0]._value,
									      sortedSolutions[n - 1], sortedSolutions[0] };
			return bestandworst;
		}
		else
		{
			size_t n = sortedSolutions.size();
			BestAndWorst bestandworst = { sortedSolutions[0]._value, sortedSolutions[n - 1]._value,
										  sortedSolutions[0], sortedSolutions[n - 1] };
			return bestandworst;
		}
	}

	//���أ��˿�����
	int size()
	{
		return _solutions.size();
	}

public:
	std::vector<Solution> _solutions; //��population��һ���
	bool _bMinOrMax; //Objective function to be (false: minimized or true: maximized)

	ObjectedFuncBase* _func; //���Ż���Ŀ�꺯��
};

#endif // POPULATION
