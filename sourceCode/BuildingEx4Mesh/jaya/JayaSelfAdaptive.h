#ifndef JAYASELFADAPTIVE
#define JAYASELFADAPTIVE

#include "JayaBase.h"
#include "JayaClassic.h"

/**
自适应Jaya算法，消除了population数量参数，
*/
class JayaSelfAdaptive : public JayaBase
{
public:
	JayaSelfAdaptive(std::vector<Variable> vriables, bool bMinOrMax,ObjectedFuncBase* func)
		:JayaBase(vriables.size()*10, vriables, bMinOrMax, func)
	{

	}

	void nextPopulation()
	{
		int numOldSolutions = _populaiton._solutions.size();
		double r = 1 + ((double)rand() / (double)(RAND_MAX) - 0.5);

		int numNewSolutions = std::round(numOldSolutions * r);

		if (numNewSolutions == numOldSolutions)
		{}
		else
		{
			Population newPopulation;
			newPopulation.setMaximize(_bMinOrMax);  
			newPopulation.setObjectedFuncBase(_func);
			if (numNewSolutions < numOldSolutions)
			{
				if (numNewSolutions < this->_numVars)
				{
					numNewSolutions = this->_numVars;
				}
				if (this->_bMinOrMax)
				{
					auto sortedPolu = _populaiton.sorted();
					for (int i = sortedPolu.size() - numNewSolutions; i < sortedPolu.size(); ++i)
						newPopulation._solutions.push_back(sortedPolu[i]);
				}
				else {
					auto sortedPolu = _populaiton.sorted();
					for (int i = 0; i < numNewSolutions; ++i)
						newPopulation._solutions.push_back(sortedPolu[i]);
				}
			}
			else if(numNewSolutions > numOldSolutions)
			{
				for (auto solution : _populaiton._solutions)
					newPopulation._solutions.push_back(solution);
				if (this->_bMinOrMax)
				{
					int begin = numOldSolutions - numNewSolutions; //负的
					auto sortedPolu = _populaiton.sorted();
					for (int i = sortedPolu.size() - begin; i < sortedPolu.size(); ++i)
						newPopulation._solutions.push_back(sortedPolu[i]);
				}
				else {
					int limit = numNewSolutions - numOldSolutions; //正的
					//if (limit < 0)
					//	std::cerr << "numNewSolutions:　" << numNewSolutions  <<" "<< numOldSolutions << std::endl;
					//else
					//	std::cerr << "numNewSolutions:　" << numNewSolutions << " " << numOldSolutions << std::endl;
					auto sortedPolu = _populaiton.sorted();
					for (int i = 0; i < limit; ++i)
						newPopulation._solutions.push_back(sortedPolu[i]);
				}
			}
			this->_populaiton = newPopulation;
		}
	}

	Population run(int numIterations)
	{
		_rn = this->generate_rn(numIterations);
		for (int i = 0; i < numIterations; ++i)
		{
			if (i > 0)
			{
				this->nextPopulation();
			}
			BestAndWorst result = this->_populaiton.getBestAndWorstValue();
			for (Solution& solution : this->_populaiton._solutions)
			{
				std::vector<double> solt;
				std::vector<double> solution1 = solution._solution;
				for (int j = 0; j < solution1.size(); ++j)
				{
					double xxxx =
						_vriables[j].convert
						(
							solution1[j] + _rn[i][j][0] *
							(result.bestSolution._solution[j] - std::abs(solution1[j]))
							- _rn[i][j][1] *
							(result.worstSolution._solution[j] - std::abs(solution1[j]))
						);
					solt.push_back(xxxx);
				}
				Solution auxSolution = Solution(_vriables, _func);
				auxSolution.setSolution(solt);
				if (_bMinOrMax)
				{
					if (auxSolution._value > solution._value)
						solution.setSolution(auxSolution._solution);
				}
				else
				{
					if (auxSolution._value < solution._value)
						solution.setSolution(auxSolution._solution);
				}
			}
			std::cout << this->_populaiton.size() << std::endl;
			//BestAndWorst beworst1 = _populaiton.getBestAndWorstValue();
			//std::cout << "best ada: " << beworst1.bestV << std::endl;
			//std::cout << "worst ada: " << beworst1.worstV << std::endl;
			//std::cout << "bestSolution ada: " << beworst1.bestSolution._solution[0] << " " << beworst1.bestSolution._solution[1] << std::endl;
			//std::cout << "=========\n";

		}
		return _populaiton;
	}
private:
	std::vector<std::vector < std::vector<double> >> _rn;

};


#endif // JAYASELFADAPTIVE
