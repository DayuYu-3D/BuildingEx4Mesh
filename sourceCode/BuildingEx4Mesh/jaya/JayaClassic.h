#ifndef JAYACLASSIC
#define JAYACLASSIC

#include "JayaBase.h"

/**
JayaClassicÀ„∑®
*/
class JayaClassic : public JayaBase
{
public:
	JayaClassic(int numSolutions, std::vector<Variable> vriables, bool bMinOrMax, ObjectedFuncBase* func)
		:JayaBase(numSolutions, vriables, bMinOrMax, func)
	{}

	Population run(int numIterations)
	{
		_rn = this->generate_rn(numIterations);

		for (int i = 0; i < numIterations; ++i)
		{
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
							solution1[j] + _rn[i][j][0]*
							(result.bestSolution._solution[j]-std::abs(solution1[j]))
							- _rn[i][j][1]*
							(result.worstSolution._solution[j]-std::abs(solution1[j]))
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
			//std::cout << "best: " << result.bestV << std::endl;
			//std::cout << "worst: " << result.worstV << std::endl;
			//std::cout << "bestSolution: " << result.bestSolution._solution[0] << " " << result.bestSolution._solution[1] << std::endl;
			//std::cout << "=========\n";
		}
		return _populaiton;
	}
private:
	std::vector<std::vector < std::vector<double> >> _rn;
};


#endif // JAYACLASSIC
