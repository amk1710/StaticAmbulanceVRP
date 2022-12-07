#pragma once

#include <vector>
#include <memory>
#include <set>

#include <list>
#include <algorithm>

#include "ProblemData.h"
#include "ProblemSolution.h"
#include "BasePricing.h"

using std::vector;
using std::shared_ptr;
using std::multiset;

/*
	implements a pricing algorithm in a bellman-ford like manner, 
	but with several 'spaced' labels for each request, instead of just one
*/
class SpacedBellmanPricing final : public BasePricing
{

	class PricingLabel
	{
	public:
		int reqId;
		double reducedCost;
		double time;
		int lastWaitingStation; //-1 if no waiting station was visited between lastReqIndex and this
		IntermediateVertex* intermediatePosition;
		//double total_lateness;

		const PricingLabel* lastLabel;
		mutable bool referenced;
    	mutable bool alreadyExpanded;

		//bool alreadyExpanded;

	};

public:
	bool limitNbLabels;
	int maxLabels;
	bool useRepeatedSetVerification;

	void SetHeuristicPricing(bool value){ heuristicPricing = value;}

private:

	PricingReturn pricing_ret;

	static bool compGTime(const PricingLabel &l1, const PricingLabel &l2);
	static bool compLRC(const unique_ptr<PricingLabel> &l1, const unique_ptr<PricingLabel> &l2);
	static bool compLTime(const unique_ptr<PricingLabel> &l1, const unique_ptr<PricingLabel> &l2);
	static bool compLRCRef(const PricingLabel &l1, const PricingLabel &l2);
	static bool compLTimeRef(const PricingLabel &l1, const PricingLabel &l2);
	static bool compGRC(const PricingLabel &l1, const PricingLabel &l2);

	// each vector-per-request is ordered by *increasing time*
	//vector<std::multiset < std::unique_ptr<PricingLabel>, decltype(compLTime)* >> labels;
	vector< std::multiset < std::unique_ptr<PricingLabel> , decltype(compLTime)* > > labels; // has nbRequests labels

	//vector<std::multiset <std::unique_ptr<PricingLabel>, decltype(compLTime)*::iterator> ItrNextExpansion;

	bool heuristicPricing;
	

	vector<std::multiset <std::unique_ptr<PricingLabel>, decltype(compLTime)* >::iterator> ItrNextExpansion;
	vector<bool> ItrNextExpansion_IsValid; //is the iterator pointing to a valid location?

	vector<IntermediateVertex*> intermediatePositions;

	bool TryAddLabel(PricingLabel &newLabel, int j, bool initial);

	static bool comp(const PricingLabel& lhs, const PricingLabel& rhs);
	static bool comp2(const PricingLabel* lhs, const PricingLabel* rhs);
	bool comp3(const PricingLabel& lhs, const PricingLabel& rhs);

	void FilterLabels(int index, PricingLabel &just_added, std::multiset <unique_ptr<PricingLabel>, decltype(compLTime)*>::iterator just_added_itr);

	void Cleanup();



	//how many labels are currently stored across all vectors?
	size_t total_labels;

	//heap storing best labels, ordered such that worst label in on the head
	size_t n_desired_routes;
	bool TryAddToBestLabelsHeap(PricingLabel &label);
	
	// obs: bestLabelsHeap does NOT take ownership of labels!
	vector<PricingLabel> bestLabelsHeap;
	
public:
	
	SpacedBellmanPricing(Params *params, ProblemData* problemData);

	~SpacedBellmanPricing(){}
	

	// alpha duals are the duals related to constraints vehicleConstraints in the MIPSolver (nbVehicles total)
	// beta duals are the duals requestConstraints in MIP Solver (nbRequests total)
	// n_routes indicates how many new routes we want: the algorithm may return less routes if there aren't enough feasible negative reduced cost routes
	//returns true if routes with negative reduced cost were found
	PricingReturn Price(int vehicle_id, int n_routes, vector<double>& alpha_duals, vector<double>& beta_duals, vector<Route>& outRoutes, vector<int> consideredRequests, std::set <pair<int, int>> forbiddenEdges, map<pair<int, int>, double> edgeDuals);
};

