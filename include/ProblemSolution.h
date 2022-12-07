#pragma once

#include <vector>

#include "ProblemData.h"
#include "Params.h"
#include <assert.h>
#include <set>

#include <utility>
using std::pair;

#include <boost/container_hash/hash.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

using std::vector;
using std::set;
using std::endl;

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "?"
#endif

//ommited representation for route
class Route {

//private:
	//ProblemData *problemData;
public:
	int veh_index;

	/*
		note: intermediate vertices represent middle points between other vertices, and are used when rerouting occurs. 
			they were added after-the-fact in the project, and are this a little hacky in their implementation
			here, they appear with a negative -1 id, and any other information is stored in the separate intermediates vector
			each intermediateVertex in vertices is paired to one in intermediated only by its relative ordering!
	*/
	vector<IntermediateVertex> intermediates;
	vector<Vertex> vertices;

	/*
		the arrival time and departure times for each vertex at the route.
		watch out for special cases at the begining and end:
		for initial position, arrival_time = 0
		for last position, departure_time = infinity
	*/

	//time when arriving somewhere
	vector<double> arrival_times;

	/*
		time when leaving somewhere,
			but remember, this is the ommited representation! Destination vertices are implicit
			this means that the departure time paired with a request vertex is actually the time when leaving the destination vertex!
	*/
	vector<double> departure_times;
	
	bool has_cycles;
	double total_lateness;
	double end_time;

	void UpdateCost(ProblemData* problemData);
	
	//set arrival_times and departure_times vectors. 
	void SetArrivalsAndDepartures(ProblemData *problemData);

	Route() = default;
	Route ( const Route & ) = default;

	bool operator==(const Route& otherRoute) const
	{
		if(this->veh_index != otherRoute.veh_index) return false;
		if(this->vertices.size() != otherRoute.vertices.size()) return false;
		for(int i = 0; i < this->vertices.size(); i++)
		{
			if(this->vertices[i].id != otherRoute.vertices[i].id) return false;
		}
		
		return true;

	}

	size_t GetHash(ProblemData *problemData)
	{
		size_t hash = 0;

		for(auto& vertex : this->vertices)
		{
			boost::hash_combine(hash, vertex.id);
		}
		return hash;
	}

	int GetRequestCount(ProblemData* problemData, int req_id)
	{
		assert(problemData->IsRequest(req_id));
		int count = 0;	
		for(Vertex &vertex : this->vertices)
		{
			if(vertex.id == req_id) count++;
		}
		return count;
	}

	std::map<pair<int, int>, int> GetEdgeUsage(ProblemData* problemData);

	// struct HashFunction
	// {
	// 	size_t operator()(const Route& route) const
	// 	{
	// 		size_t hash = route.vertices.size();
	// 		for(auto& vertex : route.vertices)
	// 		{
	// 			boost::hash_combine(hash, vertex.id);
	// 		}
	// 		return hash;
	// 	}
	// };
	
};

struct ResponseSummary
{
	double meanResponseTime;
	double maxResponseTime;

	double meanWeightedResponseTime;
	double maxWeightedResponseTime;


	int nServiced;
	int nNotServiced;

	double meanNonServicePenalty;
	double maxNonServicePenalty;
};

class ProblemSolution
{
public:
	ProblemData* problemData;

	//one valid route for each vehicle
	//uses vertices ids. Ommits destination vertices (ie. includes only requests and waiting stations)
	vector<Route> routes;
	//coefficients for each route. Should be equal to 1.0 if binary problem was solved
	vector<double> coeffs; 

	bool relaxed;

	double cost;
	double route_cost;
	double penalty_cost;
	
	ProblemSolution(ProblemData* data);

	void AnySolution();
	
	//creates a simple solution that covers every request from its closest waiting station, if time allows
	static vector<Route> ClosestAvailableVehicleSolution(ProblemData *problemData, Params *params);
	static vector<Route> FastestArrivingVehicleSolution(ProblemData *problemData, Params *params);
	
	
	void SetToInitialSolution(ProblemData *problemData, Params *params, bool outputToFile = false, int heuristic = 2);
	
	ResponseSummary GetResponseSummary ();

	//creates a simple solution with single-request routes from every vehicle to every request
	static vector<Route> AllOneSizedRoutesSolution(ProblemData *problemData, Params *params);

	static void RandomRoutes(ProblemData *problemData, Params* params, int n_routes, uint32_t seed, vector<Route> &out_routes);


	void UpdateCost();
	void PrintSolution();
	void WriteSolution(std::string path);
	void WriteRequestsOutput(std::string path);
};