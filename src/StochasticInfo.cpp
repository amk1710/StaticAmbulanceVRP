#include "StochasticInfo.h"

StochasticInfo::StochasticInfo()
{

}

void StochasticInfo::SetupGrid(double lx, double ux, double ly, double uy, int nbVertical, int nbHorizontal, double bernoulliCoefficient)
{
	this->lx = lx;
	this->ux = ux;
	this->ly = ly;
	this->uy = uy;

	this->nbVertical = nbVertical;
	this->nbHorizontal = nbHorizontal;

	bernoulliCoefficients.resize(nbHorizontal * nbVertical);
	for (int i = 0; i < nbHorizontal; i++)
	{
		for (int j = 0; j < nbVertical; j++)
		{
			bernoulliCoefficients[i * nbVertical + j] = bernoulliCoefficient;
		}
	}
}


void StochasticInfo::GenerateScenarios(const ProblemData& base, int n_scenarios, vector<ProblemData>& outScenarios)
{
	outScenarios.clear();
	outScenarios.reserve(n_scenarios);
	
	vector<Request> newRequests;
	vector<Destination> newDestinations;
	vector<Position> newPositions;

	newRequests.push_back(*base.GetRequestByIndex(0));
	newRequests[0].id = -1;

	newDestinations.push_back(*base.GetDestinationByIndex(0));
	newDestinations[0].id = -1;


	for (int iScenario = 0; iScenario < n_scenarios; iScenario++)
	{
		ProblemData scenario = base; //copy

		if (iScenario % 2 == 0)
		{
			scenario.SetScenario(newRequests, newDestinations);
		}


		outScenarios.push_back(scenario);
	}
}