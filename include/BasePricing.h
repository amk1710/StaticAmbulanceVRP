#pragma once

#include <utility>
#include <set>
#include <map>

using std::pair;
using std::map;

#include "ProblemData.h"
#include "ProblemSolution.h"
#include "Params.h"

enum PricingReturnStatus
{
	OK, FAIL
};

struct PricingReturn
{
	PricingReturnStatus status;
	size_t labelsPriced;
	size_t labelsStored;
	size_t maxLabelsStoredSimultaneously;
	size_t labelsDeleted;
	size_t mostLabelsInRequest;
	size_t nbConsideredRequests;
	bool timeout;
	vector<double> reducedCostPerRoute;
};

class BasePricing
{
protected:
	Params *params;

	double max_time;
	int max_memory; // in MB
public:

	void SetMaxTime(double max_time){this->max_time = max_time;}
	void SetMaxMemory(int max_memory){this->max_memory = max_memory;}

	ProblemData* problemData;

	BasePricing()
	{
		max_time = 1.0e+20;
		max_memory = 1000;
	};

	//pure virtual function:
	// alpha duals are the duals related to constraints vehicleConstraints in the MIPSolver (nbVehicles total)
	// beta duals are the duals requestConstraints in MIP Solver (nbRequests total)
	// n_routes indicates how many new routes we want: the algorithm may return less routes if there aren't enough feasible negative reduced cost routes
	// returns true if routes with negative reduced cost were found
	virtual PricingReturn Price(int vehicle_id, int n_routes, vector<double>& alpha_duals, vector<double>& beta_duals, vector<Route>& outRoutes, vector<int> consideredRequests, set<pair<int, int>> forbiddenEdges, map<pair<int, int>, double> edgeDuals) = 0;

	PricingReturn Price(int vehicle_id, int n_routes, vector<double>& alpha_duals, vector<double>& beta_duals, vector<Route>& outRoutes)
	{
		vector<int> consideredRequests;
		consideredRequests.reserve(problemData->NbRequests());
		for (int i = 0; i < problemData->NbRequests(); i++)
		{
			consideredRequests.push_back(problemData->IndexToRequestId(i));
		}
		set<pair<int, int>> forbiddenEdges;
		map<pair<int, int>, double> edgeDuals;
		return Price(vehicle_id, n_routes, alpha_duals, beta_duals, outRoutes, consideredRequests, forbiddenEdges, edgeDuals);
	}


	virtual ~BasePricing() //base class used polymorphically must have virtual destructor or else we incurr on undefined behaviour
    {
	}    
};