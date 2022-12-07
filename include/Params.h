#pragma once

#include <ctime>
#include "ProblemData.h"
//#include "StochasticInfo.h"

using std::string;



enum class PricingAlgorithm {DAG, bellman, spacedBellman, spacedBellman2, bellmanWSets, spacedBellmanWSets, PricerTester, hybrid};

/*

class centralizing model inputs and solver meta-parameters

*/
class Params
{
private:
	std::default_random_engine generator; // Random number generator, used for 'completing' missing info on some instances
	int seed;

	std::clock_t alg_start;

public:

	string descriptiveString;
	string outputDirectory;
	string outputSuffix;

/*
 Solver INPUTS:
*/

	bool solveRelaxedProblem = false; //if true, solve the linear relaxation
	int heuristic_run = 0;

	bool outputDuals; //should output final dual values to file?

/*
	META-PARAMETERS:
		- meta-parameters controlling the solving process, should not affect objective function
*/
	
	int newRoutesPerPricing;

	double initialDSF;
	double DSFDecrement;

	PricingAlgorithm pricingAlgorithm;


	double max_time; //max optimization time in seconds
	double max_memory; //max estimated memory usage in MBs. Actual value used may be greater than reported! Crucially, is *does not* account for pricing algorithm!
	double maxTimeSinglePricing; // mas time to be spent on any single pricing run
	double maxMemorySinglePricing; //max memory used in any single pricing run

	int nbRandomInitialRoutes;
	int route_gen_seed;

	bool useBranchingOnVehicles;
	bool useBranchingOnEdges;


	// false -> repeat pricing on vehicle until pricing fails
	// true -> always go to next vehicle
	bool alwaysLoopVehicles;

	int maxSolverIterations; //is this still used?

	//this is a very delicate parameter! setting this too low may cause non-convergence of the SPwCG procedure, and thus infinite looping!
	int maxNbRoutes;

	/**
		epsilon for objective cost and reduced cost related comparisons

		set to 1 because it correlates to a one second delay of the lowest priority level we're using (which is also 1.0)
	*/
	double RCEpsilon = 1.0;

	/** epsilon for comparison related to detection of fractional solutions*/
	double fractionalityEpsilon = 0.05;

	/*
	Status tracking:
	*/

	bool timeout; //did any computation time out OR ran out of memory? If so, SCIP might say that the solution is optimal when this isn't really the case!

	Params()
	{
		pricingAlgorithm = PricingAlgorithm::DAG;


		max_time = 3600;
		max_memory = 1000;
		maxTimeSinglePricing = max_time / 10.0;
		maxMemorySinglePricing = 200;

		useBranchingOnVehicles = false;
		useBranchingOnEdges = false;

		newRoutesPerPricing = 1;
		initialDSF = 0.9;
		DSFDecrement = 0.1;
		alwaysLoopVehicles = false;
		maxSolverIterations = 10000;
		maxNbRoutes = 50000;

		double lx = HUGE_VAL;
		double ux = -HUGE_VAL;
		double ly = HUGE_VAL;
		double uy = -HUGE_VAL;

		int seed = 0;
		generator.seed(seed);

		timeout = false;
	}
	Params(string pathToInstance, string instanceType = "pdptw")
	{
		pricingAlgorithm = PricingAlgorithm::DAG;

		max_time = 3600;
		max_memory = 1000;
		maxTimeSinglePricing = max_time / 10.0;
		maxMemorySinglePricing = 200;

		useBranchingOnVehicles = false;
		useBranchingOnEdges = false;

		newRoutesPerPricing = 1;
		initialDSF = 0.9;
		DSFDecrement = 0.1;
		alwaysLoopVehicles = false;
		maxSolverIterations = 10000;
		maxNbRoutes = 50000;

		double lx = HUGE_VAL;
		double ux = -HUGE_VAL;
		double ly = HUGE_VAL;
		double uy = -HUGE_VAL;

		int seed = 0;
		generator.seed(seed);

		timeout = false;

	}

	void StartTime()
	{
		alg_start = std::clock();
	}

	double GetElapsedTime()
	{
		std::clock_t alg_now = std::clock();
		double time = ((double)(alg_now - alg_start)) / CLOCKS_PER_SEC;
		return time;
	}

	bool Timeout()
	{
		double time = this->GetElapsedTime();
		//std::cout << time << "," << max_time << std::endl;
		return time > max_time;
	}

	bool AllowsPositiveRCElimination(WaitingStationPolicy wsp)
	{
		if(wsp == WaitingStationPolicy::mandatoryStopInFixedStation)
			return true;
		else return false;
	}
	
};