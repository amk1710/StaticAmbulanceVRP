#pragma once


#include <vector>


#include "ProblemData.h"

using std::vector;

//class for apending problemData with scenario of future demand
class DemandScenario 
{
	vector<Position> positions;
	std::vector<Request> requests;
	std::vector<Destination> destinations;
};

class AggregatedProblem
{
	ProblemData baseDate;
	DemandScenario scenario;
};

/*

helper and storage class for handling projection of future demands

*/
class StochasticInfo
{
	//temp 
	vector<double> bernoulliCoefficients;

	//for now, just use some square grid
	double lx, ux, ly, uy;
	int nbVertical, nbHorizontal;

public:
	void SetupGrid(double lx, double ux, double ly, double uy, int nbVertical, int nbHorizontal, double bernoulliCoefficient);

	StochasticInfo();

	/*
	* generate n_scenarios scenarios using &base as a a starting point. 
	*/
	void GenerateScenarios(const ProblemData &base, int n_scenarios, vector<ProblemData>& outScenarios);

};