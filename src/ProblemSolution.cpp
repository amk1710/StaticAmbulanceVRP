#include <iostream>
#include <fstream> 
#include <random>
#include <algorithm>
#include <iterator>
#include <vector>

#include "ProblemSolution.h"
#include "RouteExpander.h"

#define RC_EPS 0.1

#include <set>

using std::vector;
using std::cout;
using std::endl;

void Route::UpdateCost(ProblemData* problemData)
{
	assert(vertices.size() > 0);
	assert(vertices.size() == arrival_times.size());
	assert(vertices.size() == departure_times.size());
	
	set<int> coveredVertices;

	double time = arrival_times[0];
	
	if(vertices[0].id != veh_index) throw std::runtime_error("invalid route");
	if(departure_times[0] > arrival_times[0]) throw std::runtime_error("invalid route");
	if(arrival_times[0] != problemData->getVehicle(veh_index)->timeAvailable) throw std::runtime_error("invalid route");

	total_lateness = 0.0;
	has_cycles = false;
	if (vertices.size() == 1) return; //empty route

	int iIntermediate = 0;

	for (int ivertex = 0; ivertex < vertices.size(); )
	{
		int vertex_id = vertices[ivertex].id;
		if(ivertex < vertices.size() - 1)
		{
			//trip i -> i + 1 sanity check:

			//coherent time check
			int next_id = vertices[ivertex + 1].id;
			double dist = 0.0;
			if(problemData->IsRequest(vertex_id)) 
			{
				const Destination* dest = problemData->GetDestination(problemData->GetRequest(vertex_id)->destination);
				if(problemData->IsIntermediateVertex(next_id)) 
				{	
					dist = problemData->geodesicDistance(dest->position, this->intermediates[iIntermediate].position);
				}
				else dist = problemData->Distance(dest->id, next_id);
			}
			else if(problemData->IsIntermediateVertex(vertex_id)) 
			{
				dist = problemData->geodesicDistance(this->intermediates[iIntermediate].position, problemData->GetVertex(next_id)->position);
			}
			else
			{
				if(problemData->IsIntermediateVertex(next_id)) 
				{	
					const Vertex *v = problemData->GetVertex(vertex_id);
					dist = problemData->geodesicDistance(v->position, this->intermediates[iIntermediate].position);
				}
				else dist = problemData->Distance(vertex_id, next_id);
			}

			bool b = (arrival_times[ivertex + 1] >= (departure_times[ivertex] + dist));
			if(!(arrival_times[ivertex + 1] >= (departure_times[ivertex] + dist))) 
			{
				throw std::runtime_error("invalid route");
			}

			//cant move i and j check
			if(problemData->IsWaitingStation(vertex_id) && problemData->IsWaitingStation(next_id)) throw std::runtime_error("invalid route: can't move between waiting stations");
			if(problemData->IsIntermediateVertex(vertex_id) && problemData->IsIntermediateVertex(next_id)) throw std::runtime_error("invalid route: can't move between intermediate vertices");
			
		}

		time = arrival_times[ivertex];
		
		if (problemData->IsRequest(vertex_id))
		{
			const Request* req = problemData->GetRequest(vertex_id);
			total_lateness += problemData->weighted_lateness(req, time);
			assert(time + RC_EPS > req->arrival_time);

			// check if has cycles
			if(has_cycles)
			{
				//dont need to keep track anymore
			}
			else if (coveredVertices.find(vertex_id) != coveredVertices.end())
			{
				has_cycles = true;
			}
			else
			{
				coveredVertices.insert(vertex_id);
			}
		}
		else if(problemData->IsIntermediateVertex(vertex_id))
		{
			iIntermediate++;
		}
		
		ivertex++;

	}
	end_time = time;
	return;
}

void Route::SetArrivalsAndDepartures(ProblemData *problemData)
{

	arrival_times.clear();
	departure_times.clear();

	arrival_times.resize(vertices.size());
	departure_times.resize(vertices.size());

	int iIntermediate = 0;


	arrival_times[0] = problemData->getVehicle(veh_index)->timeAvailable;
	departure_times[0]= arrival_times[0];
	const Request* lastRequest = NULL;
	for(int i = 1; i < vertices.size(); i++)
	{
		if(problemData->IsRequest(vertices[i].id))
		{
			const Request* req = problemData->GetRequest(vertices[i].id);

			if(problemData->IsWaitingStation(vertices[i - 1].id))
			{
				departure_times[i - 1] = std::max(departure_times[i - 1], req->arrival_time); //adjust non-anticipativity in last departure
			}
			else if(req->arrival_time > departure_times[i - 1] + RC_EPS)
			{
				if(problemData->IsIntermediateVertex(vertices[i - 1].id))
				{
					//intermediate vertices are exempt from this check right now because of the inconsistencies we get with calculated intermediate points
				}
				else throw std::runtime_error("invalid route : cant wait outside of waiting station");
			}

			lastRequest = req;

			if(problemData->IsRequest(vertices[i-1].id)){
				const Request *req = problemData->GetRequest(vertices[i-1].id);
				arrival_times[i] = departure_times[i - 1] + problemData->Distance(req->destination, vertices[i].id);
			}
			else if(problemData->IsWaitingStation(vertices[i-1].id) || problemData->IsInitialPosition(vertices[i-1].id) )
			{
				arrival_times[i] = departure_times[i - 1] + problemData->Distance(vertices[i-1].id, vertices[i].id);
			}
			else if(problemData->IsIntermediateVertex(vertices[i - 1].id))
			{
				arrival_times[i] = departure_times[i - 1] + ProblemData::geodesicDistance(intermediates[iIntermediate - 1].position, vertices[i].position);

				//intermediate position have numerical issues that may result in weird situations like this
				// we don't error in this case so we can still hold a comparison to Vincent's other codebase, 
				// but we still fix it manually like this:
				if(arrival_times[i] < req->arrival_time)
				{
					arrival_times[i] = req->arrival_time;
				}
			
			}
			else throw std::runtime_error("invalid route");

			assert(arrival_times[i] >= req->arrival_time);	

			
			departure_times[i] = arrival_times[i] 
									+ req->service_time 
									+ problemData->Distance(req->id, req->destination);

		}
		else if(problemData->IsWaitingStation(vertices[i].id))
		{
			if(problemData->IsWaitingStation(vertices[i - 1].id)) throw std::runtime_error("invalid route : cant move between waiting stations");
			
			if(problemData->IsRequest(vertices[i-1].id)) {
				const Request *req = problemData->GetRequest(vertices[i-1].id);
				arrival_times[i] = departure_times[i - 1] + problemData->Distance(req->destination, vertices[i].id);
			}
			else if(problemData->IsWaitingStation(vertices[i-1].id) || problemData->IsInitialPosition(vertices[i-1].id) ){
				arrival_times[i] = departure_times[i - 1] + problemData->Distance(vertices[i-1].id, vertices[i].id);
			}
			else throw std::runtime_error("invalid route");
			
			//setup earliest departure, but this may change in the next iteration due to waiting here
			departure_times[i] = arrival_times[i];
		 
			 
			
		}
		else if(problemData->IsIntermediateVertex(vertices[i].id))
		{
			if(!(vertices.size() > i)) throw std::runtime_error("invalid route"); // there is at least one more vertex
			if(!(problemData->IsRequest(vertices[i + 1].id) )) throw std::runtime_error("invalid route"); //next vertex is request (no rerouting between anything else)
			if(!(problemData->IsWaitingStation(intermediates[iIntermediate].ws_id))) throw std::runtime_error("invalid route");

			Position pos1;
			if(problemData->IsRequest(vertices[i-1].id))
			{
				const Request *req = problemData->GetRequest(vertices[i-1].id);	
				const Destination *dest = problemData->GetDestination(req->destination);
				double d = ProblemData::geodesicDistance(dest->position, intermediates[iIntermediate].position);
				
				//may fail due to numerical errors
				//assert(d < problemData->Distance(dest->id, intermediates[iIntermediate].ws_id) + RC_EPS);
				
				arrival_times[i] = departure_times[i - 1] + ProblemData::geodesicDistance(problemData->GetDestination(req->destination)->position, intermediates[iIntermediate].position);
			}
			else if(problemData->IsInitialPosition(vertices[i-1].id))
			{
				InitialPosition const *v = problemData->GetInitialPosition(vertices[i-1].id);
				double d = ProblemData::geodesicDistance(v->position, intermediates[iIntermediate].position);

				//may fail due to numerical errors
				//assert(d < problemData->Distance(v->id, intermediates[iIntermediate].ws_id) + RC_EPS);

				arrival_times[i] = departure_times[i - 1] + ProblemData::geodesicDistance(v->position, intermediates[iIntermediate].position);
			}
			else
			{
				throw std::runtime_error("invalid route");
			}
			
			departure_times[i] = arrival_times[i];
			
			
			//this equality is what i'd like, but it isn't happening because of numerical errors in intermediate point distance calculations. 
			//so I use the above, as it's at least still consistent with the way routes were built
			//problemData->GetRequest(vertices[i + 1].id)->arrival_time;
			//assert(std::abs(departure_times[i] - arrival_times[i]) < 0.1);

			iIntermediate++;
		}
		else throw std::runtime_error("invalid route");
	}

#ifdef _DEBUG
	for(int i = 1; i < vertices.size() - 1; i++)
	{
		assert(std::abs(departure_times[i] - (arrival_times[i+1] - problemData->Distance(vertices[i].id, vertices[i+1].id)) < RC_EPS);
		assert(departure_times[i] > arrival_times[i] - RC_EPS);
	}
	departure_times[departure_times.size() - 1] = std::numeric_limits<double>::infinity();
#endif
	
}

std::map<pair<int, int>, int> Route::GetEdgeUsage(ProblemData* problemData)
	{
		std::map<pair<int, int>, int> edgeUsage;
		int lastReq = -1;
		int currReq = -1;
		int edgeCount = 0;
		for(int i = 0; i < this->vertices.size(); i++)
		{
			if(problemData->IsRequest(vertices[i].id)) {
				lastReq = currReq;
				currReq = vertices[i].id;
			}
			else continue; // !!!!!

			if(currReq != -1 && lastReq != -1)
			{
				assert(currReq != lastReq);
				int ii = std::min(currReq, lastReq);
				int jj = std::max(currReq, lastReq);
				assert(ii < jj);

				pair<int, int> pair = std::make_pair(ii,jj);

				if(edgeUsage.find(pair) == edgeUsage.end())
				{
					edgeUsage[pair] = 1;
					edgeCount++;
				}
				else edgeUsage[pair] = edgeUsage[pair] + 1;
			}
		}
		return edgeUsage;
	}



/*
constructs a solution that covers all arriving requests with the closest available vehicle.
ps: this function 
*/
vector<Route> ProblemSolution::ClosestAvailableVehicleSolution(ProblemData *problemData, Params *params)
{
	RouteExpander expander(params);

	//construct routes for each vehicle in such a way that they, in conjunction, cover all requests

	vector<Route> routes;
	routes.clear();
	routes.resize(problemData->NbVehicles());

	vector<double> last_time(problemData->NbVehicles());
	vector<double> avail_at(problemData->NbVehicles());

	for(int i = 0; i <  problemData->NbVehicles(); i++)
	{
		routes[i].veh_index = i;
		routes[i].vertices.push_back((Vertex) *problemData->GetInitialPositionByIndex(i));
		last_time[i] = problemData->getVehicle(i)->timeAvailable;
		avail_at[i] = problemData->getVehicle(i)->timeAvailable;
	}


	std::vector<int> orderedRequests(problemData->NbRequests()) ; // vector with 100 ints.
	std::iota (std::begin(orderedRequests), std::end(orderedRequests), 0); // Fill with 0, 1, ..., 99.

	std::stable_sort (orderedRequests.begin(), orderedRequests.end(), 
		[problemData](const int & a, const int & b) -> bool
		{ 
			return problemData->GetRequestByIndex(a)->arrival_time < problemData->GetRequestByIndex(b)->arrival_time;
		});


	for(int i = 0; i < orderedRequests.size(); i++)
	{
		const Request *req = problemData->GetRequestByIndex(orderedRequests[i]);

		/*
			finds closest available vehicle
		*/
		int i_closest = -1;
		double min_arr_time = std::numeric_limits<double>::infinity();
		int ws_closest;
		bool useIntermediate_closest;
		IntermediateVertex bestIntermediateVertex;

		bool atLeastOneIsAvailable = false;

		for(int iveh = 0; iveh < problemData->NbVehicles(); iveh++)
		{
			const Vehicle *veh = problemData->getVehicle(iveh);
			Vertex current_pos = routes[iveh].vertices.back();
			double cur_time = last_time[iveh];
			double outTime;
			int waitingStation;
			bool useIntermediate;
			IntermediateVertex intermediateVertex;
			bool success = expander.checkRouteExpansion(problemData, iveh, req, &current_pos, cur_time, outTime, waitingStation, useIntermediate, intermediateVertex);

			bool isAvailableAtTime = avail_at[iveh] <= req->arrival_time;

			if(success && (!atLeastOneIsAvailable || isAvailableAtTime )) 
			{

				//if time is better OR this is the first available vehicle that we find, 
				if(outTime < min_arr_time || (!atLeastOneIsAvailable && isAvailableAtTime))
				{
					min_arr_time = outTime;
					i_closest = iveh;
					if(useIntermediate)
					{
						ws_closest = -1;
						bestIntermediateVertex = intermediateVertex;
						useIntermediate_closest = true;
					}
					else
					{
						ws_closest = waitingStation;
						useIntermediate_closest = false;
						bestIntermediateVertex = IntermediateVertex();
					}

					atLeastOneIsAvailable = atLeastOneIsAvailable || isAvailableAtTime;
				}

			}
		}

		//no more services are available because of time limit?
		if(i_closest == -1 && problemData->timeHorizon == std::numeric_limits<double>::infinity())
		{
			//in problems with problemData->timeHorizon == inf, every request should be serviceable
			throw std::runtime_error("Initial routes generation failed");
		}
		else if(i_closest == -1)
		{
			//we can't service this request, because of time limits, but the problem should still be valid and we'll incur in non-service penalty
			continue;
		}

		//expand selected route
		if(useIntermediate_closest)
		{
			assert(bestIntermediateVertex.id == -1);
			routes[i_closest].vertices.push_back((Vertex) bestIntermediateVertex);
			routes[i_closest].intermediates.push_back(bestIntermediateVertex);
		}
		if(ws_closest != -1)
		{
			Vertex v = (Vertex) *problemData->GetWaitingStation(ws_closest);
			routes[i_closest].vertices.push_back(v);
		}

		Vertex vet = (Vertex) *req; //copy
		routes[i_closest].vertices.push_back(vet);

		last_time[i_closest] = min_arr_time;
		avail_at[i_closest] = min_arr_time + problemData->Distance(req->id, req->destination);
		
	}

	for(int i = routes.size() - 1; i >= 0; i--)
	{
		if(routes[i].vertices.size() == 1) routes.erase(routes.begin() + i);
		else
		{
			routes[i].SetArrivalsAndDepartures(problemData);
			routes[i].UpdateCost(problemData);
		}

	}
	
	return routes;
}

/*
	constructs routes using always the fastest arriving vehicle to a given call.

	this is equivalent to vh's "forward" or something like that. This should always better or equal to the closest available heuristic
*/
vector<Route> ProblemSolution::FastestArrivingVehicleSolution(ProblemData *problemData, Params *params)
{
	RouteExpander expander(params);

	//construct routes for each vehicle in such a way that they, in conjunction, cover all requests

	vector<Route> routes;
	routes.clear();
	routes.resize(problemData->NbVehicles());

	vector<double> last_time(problemData->NbVehicles());

	for(int i = 0; i <  problemData->NbVehicles(); i++)
	{
		routes[i].veh_index = i;
		routes[i].vertices.push_back((Vertex) *problemData->GetInitialPositionByIndex(i));
		last_time[i] = problemData->getVehicle(i)->timeAvailable;
	}


	std::vector<int> orderedRequests(problemData->NbRequests()) ; // vector with 100 ints.
	std::iota (std::begin(orderedRequests), std::end(orderedRequests), 0); // Fill with 0, 1, ..., 99.

	std::stable_sort (orderedRequests.begin(), orderedRequests.end(), 
		[problemData](const int & a, const int & b) -> bool
		{ 
			return problemData->GetRequestByIndex(a)->arrival_time < problemData->GetRequestByIndex(b)->arrival_time;
		});


	for(int i = 0; i < orderedRequests.size(); i++)
	{
		const Request *req = problemData->GetRequestByIndex(orderedRequests[i]);

		/*
			finds "closest available vehicle", that is, which available vehicle would arrive the earliest?
		*/
		int i_closest = -1;
		double min_arr_time = std::numeric_limits<double>::infinity();
		int ws_closest;
		bool useIntermediate_closest;
		IntermediateVertex bestIntermediateVertex;

		for(int iveh = 0; iveh < problemData->NbVehicles(); iveh++)
		{
			const Vehicle *veh = problemData->getVehicle(iveh);
			Vertex current_pos = routes[iveh].vertices.back();
			double cur_time = last_time[iveh];
			double outTime;
			int waitingStation;
			bool useIntermediate;
			IntermediateVertex intermediateVertex;
			bool success = expander.checkRouteExpansion(problemData, iveh, req, &current_pos, cur_time, outTime, waitingStation, useIntermediate, intermediateVertex);

			//to-do: make tiebreak criteria considering vehicle type
			if(success && outTime < min_arr_time) 
			{
				min_arr_time = outTime;
				i_closest = iveh;
				if(useIntermediate)
				{
					ws_closest = -1;
					bestIntermediateVertex = intermediateVertex;
					useIntermediate_closest = true;
				}
				else
				{
					ws_closest = waitingStation;
					useIntermediate_closest = false;
					bestIntermediateVertex = IntermediateVertex();
				}
				
				
			}
		}

		//no more services are available because of time limit?
		if(i_closest == -1 && problemData->timeHorizon == std::numeric_limits<double>::infinity())
		{
			//in problems with problemData->timeHorizon == inf, every request should be serviceable
			throw std::runtime_error("Initial routes generation failed");
		}
		else if(i_closest == -1)
		{
			//we can't service this request, because of time limits, but the problem should still be valid and we'll incur in non-service penalty
			continue;
		}

		//expand selected route
		if(useIntermediate_closest)
		{
			assert(bestIntermediateVertex.id == -1);
			routes[i_closest].vertices.push_back((Vertex) bestIntermediateVertex);
			routes[i_closest].intermediates.push_back(bestIntermediateVertex);
		}
		if(ws_closest != -1)
		{
			Vertex v = (Vertex) *problemData->GetWaitingStation(ws_closest);
			routes[i_closest].vertices.push_back(v);
		}

		Vertex vet = (Vertex) *req; //copy
		routes[i_closest].vertices.push_back(vet);

		last_time[i_closest] = min_arr_time;
		
	}

	for(int i = routes.size() - 1; i >= 0; i--)
	{
		if(routes[i].vertices.size() == 1) routes.erase(routes.begin() + i);
		else
		{
			routes[i].SetArrivalsAndDepartures(problemData);
			routes[i].UpdateCost(problemData);
		}

	}
	
	return routes;
}

ResponseSummary ProblemSolution::GetResponseSummary()
{
	size_t count = 0;
	ResponseSummary ret;

	ret.meanResponseTime = 0.0;
	ret.maxResponseTime = 0.0;
	ret.meanWeightedResponseTime = 0.0;
	ret.maxWeightedResponseTime = 0.0;

	ret.nServiced = 0;
	ret.nNotServiced = 0;

	std::set<int> serviced_requests;

	ret.meanNonServicePenalty = 0.0;
	ret.maxNonServicePenalty = 0.0;

	for(Route &route : routes)
	{
		for(int iV = 0; iV < route.vertices.size(); iV++)
		{
			if(problemData->IsRequest(route.vertices[iV].id))
			{
				const Request* req = problemData->GetRequest(route.vertices[iV].id);

				double responseTime = route.arrival_times[iV] - req->arrival_time; // this is what terrible variable naming does to people
				double weightedResponseTime = ret.meanResponseTime * req->weight;

				ret.meanResponseTime += responseTime;
				ret.maxResponseTime = std::max(ret.maxResponseTime, responseTime);
				ret.meanWeightedResponseTime += weightedResponseTime;
				ret.maxWeightedResponseTime = std::max(ret.maxWeightedResponseTime, weightedResponseTime);

				count++;

				serviced_requests.insert(req->id);
			}
		}
	}

	ret.meanResponseTime = ret.meanResponseTime / count;
	ret.meanWeightedResponseTime = ret.meanWeightedResponseTime / count;

	ret.nServiced = serviced_requests.size();
	ret.nNotServiced = problemData->NbRequests() - ret.nServiced;

	for(int i = 0; i < problemData->NbRequests(); i++)
	{
		const Request *req = problemData->GetRequestByIndex(i);
		if(serviced_requests.find(req->id) == serviced_requests.end())
		{
			ret.meanNonServicePenalty += req->non_service_penalty;
			ret.maxNonServicePenalty = std::max(ret.maxNonServicePenalty, req->non_service_penalty);
		}
	}

	ret.meanNonServicePenalty = ret.meanNonServicePenalty / ret.nNotServiced;

	return ret;
	
}

vector<Route> ProblemSolution::AllOneSizedRoutesSolution(ProblemData *problemData, Params *params)
{
	RouteExpander expander(params);

	vector<Route> routes;
	routes.clear();
	routes.reserve(problemData->NbRequests() * problemData->NbVehicles());

	for(int i = 0; i < problemData->NbRequests(); i++)
	{
		for(int iveh = 0; iveh < problemData->NbVehicles(); iveh++)
		{
			int nveh = problemData->NbVehicles();
			Route route;
			const Request *req = problemData->GetRequestByIndex(i);
			const Vehicle *veh = problemData->getVehicle(iveh);
			const InitialPosition *initial_position = problemData->GetInitialPositionByIndex(iveh);

			int waitingStation;
			double outTime;
			
			bool ret = expander.checkRouteExpansion(problemData, iveh, req, initial_position, veh->timeAvailable, outTime, waitingStation);

			//problably incompatibility between vehicle and request
			if(!ret)
			{
				continue;
			}

			route.veh_index = iveh;
			//cover request using best vehicle found
			
			Vertex iv = (Vertex) *initial_position;
			route.vertices.push_back(iv);

			if(waitingStation != -1)
			{
				Vertex v = (Vertex) *problemData->GetWaitingStation(waitingStation);
				route.vertices.push_back(v);
			}

			Vertex vet = (Vertex) *req; //copy
			route.vertices.push_back(vet);

			route.SetArrivalsAndDepartures(problemData);
			route.UpdateCost(problemData);

			routes.push_back(route);
		}
	}
	
	return routes;
}

#include <random>
void ProblemSolution::RandomRoutes(ProblemData *problemData, Params* params, int n_routes, uint32_t seed, vector<Route> &out_routes)
{
	RouteExpander expander(params);

	//std::random_device dev;
    std::mt19937 rng;
    rng.seed(seed);
	std::uniform_int_distribution<std::mt19937::result_type> dist(0, problemData->NbRequests() - 1);
	std::uniform_int_distribution<std::mt19937::result_type> dist_veh(0, problemData->NbVehicles() - 1);
	

	//(...)


	for(int count = 0; count < n_routes; count++)
	{
		Route route;
		std::vector<int> random_requests;
		random_requests.reserve(problemData->NbRequests());

		
		for(int i = 0; i < problemData->NbRequests(); i++) random_requests.push_back(i);
		
		int n_requests = std::max(1, (int) dist(rng));
		int veh_index = dist_veh(rng);
		const Vehicle *veh = problemData->getVehicle(veh_index);
		
		std::shuffle(random_requests.begin(), random_requests.end(), rng);

		//reduce size down to n_requests
		random_requests.erase(random_requests.begin() + (random_requests.size() - n_requests), random_requests.end());

		//build route using the random requests, whenever possible
		route.veh_index = veh_index;
		route.vertices.push_back((Vertex) *problemData->GetInitialPositionByIndex(veh_index));
		double last_time = veh->timeAvailable;
		for(auto& req_index : random_requests)
		{
			const Request* req = problemData->GetRequestByIndex(req_index);
			double time = 0;
			int ws = -1;
			bool ret = expander.checkRouteExpansion(problemData, veh_index, req, &route.vertices.back(), last_time, time, ws);

			if(ret)
			{
				if(ws != -1) route.vertices.push_back((Vertex) *problemData->GetWaitingStation(ws));
				route.vertices.push_back((Vertex)*req);
			}
		}

		if(route.vertices.size() > 1)
		{
			route.SetArrivalsAndDepartures(problemData);
			route.UpdateCost(problemData);
			out_routes.push_back(route);
		}

	}

}

ProblemSolution::ProblemSolution(ProblemData* data) : cost(-1.0), problemData(data), routes(data->NbVehicles()), coeffs(data->NbVehicles())
{
	for (int iroute = 0; iroute < data->NbVehicles(); iroute++)
	{
		Route route;
		route.total_lateness = 0.0;
		route.veh_index = iroute;
		route.vertices = vector<Vertex>(); route.vertices.push_back(*problemData->GetInitialPosition(iroute));
		routes[iroute] = route;

		coeffs[iroute] = 1.0;
	}

	relaxed = false;
}

void ProblemSolution::UpdateCost()
{
	vector<double> is_serviced = vector<double>(problemData->NbRequests(), 0.0);

	vector<double> is_used = vector<double>(problemData->NbVehicles(), 0.0);

	route_cost = 0.0;
	for (int iroute = 0; iroute < routes.size(); iroute++)
	{
		routes[iroute].UpdateCost(problemData);
		
		//if(!relaxed && fabs(coeffs[iroute] - 1.0) > RC_EPS)
		//	throw std::runtime_error("invalid solution: non-relaxed solution with coeff != 1.0");

		route_cost += routes[iroute].total_lateness * coeffs[iroute];

		//is vehicle used more than once?
		is_used[routes[iroute].veh_index] += coeffs[iroute];
		if(is_used[routes[iroute].veh_index] > 1.0 + RC_EPS) 
			throw std::runtime_error("invalid solution: multiple use of vehicle");

		for (int ivertex = 0; ivertex < routes[iroute].vertices.size(); ivertex++)
		{
			int id = routes[iroute].vertices[ivertex].id;
			if (problemData->IsRequest(id))
			{
				int index = problemData->RequestIdToIndex(id);
				is_serviced[index] += coeffs[iroute];
			}
		}
	}

	for (int i = 0; i <  problemData->NbRequests(); i++)
	{
		if(is_serviced[i] > 1.0 + RC_EPS || is_serviced[i] < 0.0 - RC_EPS)
		{
			std::cout << is_serviced[i] << std::endl;
			throw std::runtime_error("invalid solution: request coeff invalid");

		}
			
	}

	penalty_cost = 0.0;
	for (int i = 0; i < problemData->NbRequests(); i++)
	{
		if(problemData->GetRequestByIndex(i)->non_service_penalty == std::numeric_limits<double>::infinity())
		{
			if(is_serviced[i] < 1.0 - RC_EPS)
			{
				throw std::runtime_error("invalid solution: request was mandatory but wasn't fully covered (" + std::to_string(is_serviced[i]));
			}
		}
		else
		{
			penalty_cost += problemData->GetRequestByIndex(i)->non_service_penalty * (1.0 - is_serviced[i]);
		}
		
	}

	cost = route_cost + penalty_cost;
}

void ProblemSolution::PrintSolution()
{
	std::set<int> servicedRequests;
	for (int i = 0; i < routes.size(); i++)
	{
		if (routes[i].vertices.size() == 0) cout << i << ": empty" << endl;

		for (int iver = 0; iver < routes[i].vertices.size(); iver++)
		{
			int id = routes[i].vertices[iver].id;
			char c = 'X';
			if (problemData->IsInitialPosition(id)) c = 'i';
			if (problemData->IsRequest(id)) c = 'r';
			if (problemData->IsWaitingStation(id)) c = 'w';

			servicedRequests.emplace(id);
			cout << id << "(" << c << ")";
		}

		cout << endl;

	}

	cout << "not serviced: ";
	for (int ireq = 0; ireq < problemData->NbRequests(); ireq++)
	{
		int idreq = problemData->GetRequestByIndex(ireq)->id;
		if (servicedRequests.count(idreq) == 0) {
			cout << idreq << ", ";
		}
	}
	cout << endl;

}

void ProblemSolution::WriteSolution(std::string path)
{
	std::ofstream out_file(path);

	//total cost, route cost, penalty cost
	
	out_file << cost << "," << route_cost << "," << penalty_cost << std::endl;
	out_file << routes.size() << std::endl;
	for(int iroute = 0; iroute < routes.size(); iroute++)
	{
		Route &route = routes[iroute];
		out_file << "(" << coeffs[iroute] << ")";
		int i = 0;
		for(Vertex &v : route.vertices)
		{
			out_file << v.id << " ";
			if(problemData->IsRequest(v.id))
			{
				const Request *req = problemData->GetRequest(v.id);
				const Destination *dest = problemData->GetDestination(req->destination);
				out_file << dest->id  << " ";
			}
			i++;
		} 
		
		out_file << std::endl;
	}

	out_file.close();
}


void ProblemSolution::WriteRequestsOutput(std::string path)
{
	//for each request in order, output:
	vector<double> timeServiced(problemData->NbRequests());
	vector<bool> isServiced(problemData->NbRequests());
	vector<int> ambulanceIndex(problemData->NbRequests());

	for(Route &route : routes)
	{
		for(int i = 0; i < route.vertices.size(); i++)
		{
			if(problemData->IsRequest(route.vertices[i].id))
			{				
				int reqIndex = problemData->RequestIdToIndex(route.vertices[i].id);
				if(isServiced[reqIndex])
				{
					std::cout << "warning: request serviced multiple times: " << reqIndex << std::endl;
				}
				timeServiced[reqIndex] = route.arrival_times[i];
				isServiced[reqIndex] = true;
				ambulanceIndex[reqIndex] = route.veh_index;
			}
		}
	}

	std::ofstream out_file(path);

	for(int i = 0; i < problemData->NbRequests(); i++)
	{
		const Request *req = problemData->GetRequestByIndex(i);


		double wait = 0.0; double weightedWait = 0.0;
		if(isServiced[i])
		{
			wait = timeServiced[i] - req->arrival_time;
			assert(wait + RC_EPS > 0.0);
			weightedWait = wait * req->weight;
		}

		out_file << i << "," << req->arrival_time << "," << req->weight << "," << isServiced[i] << "," << timeServiced[i] << "," << wait << "," << weightedWait << "," << ambulanceIndex[i] << std::endl;
	}

	out_file.close();

}

//sets this solution to any non-empty solution where each vehicle is occupied
void ProblemSolution::AnySolution()
{
	for (int iroute = 0; iroute < problemData->NbVehicles(); iroute++)
	{
		routes[iroute].vertices.clear();
		routes[iroute].vertices.push_back(*problemData->GetInitialPosition(iroute));
		if (iroute < problemData->NbRequests())
		{
			routes[iroute].vertices.push_back(*problemData->GetRequestByIndex(iroute)); //pick any request to insert here
		}
	}
}

/*

this heuristic that i did is (probably) similar to the 'best myopic' described in new heuristics, 
which is refered to 'forward' in VH's code

*/

void ProblemSolution::SetToInitialSolution(ProblemData *problemData, Params *params, bool outputToFile, int heuristic)
{
	if(heuristic == 1) this->routes = ProblemSolution::ClosestAvailableVehicleSolution(problemData, params);
	else if(heuristic == 2) this->routes = ProblemSolution::FastestArrivingVehicleSolution(problemData, params);
	else this->routes = ProblemSolution::FastestArrivingVehicleSolution(problemData, params); //default

	this->coeffs = vector<double>(routes.size(), 1.0);
	this->UpdateCost();

	//this does not really change any solutions, but I do it to have consistent output between heuristic and mip runs
	if(problemData->computeTimeHorizon)
   	{
		double max_time = 0.0;
		for(Route &route : this->routes)
		{
			assert(problemData->IsRequest(route.vertices.back().id));
			max_time = std::max(max_time, route.arrival_times.back());
		}
		assert(max_time > 0);
		problemData->timeHorizon = max_time * 1.5; //1.5 ?
		std::cout << "time horizon computed as " << problemData->timeHorizon << std::endl;
	}

	if(!outputToFile) return;

	fs::path dir (params->outputDirectory);
	fs::path out_path = dir / fs::path(problemData->name + params->outputSuffix + ".out");
	std::ofstream myfile;
	myfile.open(out_path.string(), std::ios_base::app);


	ResponseSummary responseSummary = this->GetResponseSummary();

	std::string commit_hash = GIT_COMMIT_HASH;

		//some solution stats:
	double avg_requests_per_route = 0;
	int max_req_in_route = 0;
	bool has_cycles = false;
	for(int i = 0; i < this->routes.size(); i++)
	{
		Route* route = &this->routes[i];
		has_cycles = has_cycles || route->has_cycles;
		int nreq = 0;
		for(int iV = 0; iV < route->vertices.size(); iV++)
		{
			if(problemData->IsRequest(route->vertices[iV].id)) nreq++;
		}
		avg_requests_per_route += nreq;
		max_req_in_route = std::max(max_req_in_route, nreq);
	}
	avg_requests_per_route = avg_requests_per_route / this->routes.size();

	std::scientific(myfile);
	myfile.precision(std::numeric_limits<double>::max_digits10);

	//allow rerouting?

	myfile  << problemData->name << "," << params->descriptiveString << "," << problemData->NbRequests() << "," << problemData->NbVehicles() << "," << problemData->timeHorizon << "," << (int) problemData->waitingStationPolicy << ","
			<< this->cost << "," << this->route_cost << "," << this->penalty_cost << "," << this->routes.size() << "," << avg_requests_per_route << "," << max_req_in_route << "," << has_cycles << ","
			<< responseSummary.nServiced << "," << responseSummary.nNotServiced << ","
            << responseSummary.meanResponseTime << "," << responseSummary.maxResponseTime << "," << responseSummary.meanWeightedResponseTime << "," << responseSummary.maxWeightedResponseTime << "," 
            << responseSummary.meanNonServicePenalty << "," << responseSummary.maxNonServicePenalty << ","
			<< params->heuristic_run << ","
            << commit_hash
	<< endl;


}