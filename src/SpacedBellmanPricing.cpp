#include "SpacedBellmanPricing.h"
#include "RouteExpander.h"

#include <iterator>
#include <ctime>
#include <vector>
#include <set>
#include <iostream>
#include <algorithm>

SpacedBellmanPricing::SpacedBellmanPricing(Params *params, ProblemData* problemData) : labels()
{
	this->problemData = problemData;
	this->params = params;

	limitNbLabels = false;
	maxLabels = 10;
	useRepeatedSetVerification = false;

	heuristicPricing = false;
	
}

bool SpacedBellmanPricing::comp(const PricingLabel& lhs, const PricingLabel& rhs)
{
	return lhs.reducedCost < rhs.reducedCost;
}

bool SpacedBellmanPricing::comp2(const PricingLabel* lhs, const PricingLabel* rhs)
{
	return lhs->reducedCost > rhs->reducedCost;
}

bool SpacedBellmanPricing::comp3(const PricingLabel& lhs, const PricingLabel& rhs)
{
	return lhs.time < rhs.time;
}

bool SpacedBellmanPricing::TryAddToBestLabelsHeap(PricingLabel &label)
{
	bool ret = false;
	if(label.reducedCost > -params->RCEpsilon) return false;

	if(bestLabelsHeap.size() < n_desired_routes)
	{
		bestLabelsHeap.push_back(label);
		push_heap(bestLabelsHeap.begin(), bestLabelsHeap.end(), compLRCRef);
		ret =  true;
	} 
	// bestLabelsHeap.front() is the worst label in bestLabelsHeap
	//if label is better, replace it
	else if(bestLabelsHeap.front().reducedCost - params->RCEpsilon > label.reducedCost)
	{
		pop_heap(bestLabelsHeap.begin(), bestLabelsHeap.end(), compLRCRef);
    	bestLabelsHeap.pop_back();

		bestLabelsHeap.push_back(label);
		push_heap(bestLabelsHeap.begin(), bestLabelsHeap.end(), compLRCRef);

		ret =  true;
	}

	return ret;

}

bool SpacedBellmanPricing::TryAddLabel(PricingLabel& newLabel, int j, bool initial = false)
{
	/* suppose that labels[j] is already:
		- ordered by increasing time
		- no labels dominate other labels

		supposing this, I'll possibly add the new label and then filter the vector to keep this properties 
	*/

	//unique_ptr<PricingLabel> newLabel = std::make_unique<PricingLabel>(originalLabel);

	assert( initial || (j >= 0 && j < labels.size()) );

	if(params->AllowsPositiveRCElimination(problemData->waitingStationPolicy) && newLabel.reducedCost > -params->RCEpsilon) return false;

	if(initial)
	{
		labels.push_back(std::multiset <std::unique_ptr<PricingLabel>, decltype(compLTime)*>(compLTime));


		bool best = TryAddToBestLabelsHeap(newLabel);
		//newLabel.lastLabel->referenced = true;
		unique_ptr label_uptr = std::make_unique<PricingLabel>(newLabel);
		labels.back().insert(std::move(label_uptr));

		ItrNextExpansion.push_back(labels.back().begin());
		ItrNextExpansion_IsValid.push_back(true);

		total_labels++;
		pricing_ret.labelsStored++;
		pricing_ret.maxLabelsStoredSimultaneously = std::max(pricing_ret.maxLabelsStoredSimultaneously, total_labels);

		return true;
	}

	assert(labels[j].begin()->get()->reqId == newLabel.reqId);

	bool added = false;


	auto insert_position = std::upper_bound(labels[j].begin(), labels[j].end(), std::make_unique<PricingLabel>(newLabel), compLTime);
	//assert that i'm not inserting before 'fixed labels' position

	//auto prev = std::prev(insert_position);
	//assert(insert_position == labels[j].begin() || insert_position == labels[j].end() || (*prev).time < (*insert_position).time);


	bool dominated = true;
	bool bestRCedLabel = false;
	if(labels[j].size() == 0)
	{
		dominated = false;
		bestRCedLabel = true;
	}
	else if(insert_position == labels[j].end())
	{
		//rbegin points to last element in the set.
		dominated = newLabel.reducedCost + params->RCEpsilon > (*labels[j].rbegin())->reducedCost;
		bestRCedLabel = newLabel.reducedCost + params->RCEpsilon < (*labels[j].rbegin())->reducedCost;
	}
	else if(insert_position == labels[j].begin())
	{
		assert(newLabel.time < (*labels[j].begin())->time);
		dominated = false;
		bestRCedLabel = false;
	}
	else
	{
		insert_position--; //lower_bounds points to the first not less than newLabel. I want the label before that so I can compare. edge cases are treated above
		assert((*insert_position)->time <= newLabel.time); // upper_bound and -- makes us points to exact ties too!
		dominated = newLabel.reducedCost + params->RCEpsilon > (*insert_position)->reducedCost;
		bestRCedLabel = false;
	}

	assert(bestRCedLabel ? !dominated : true);

	bool best = TryAddToBestLabelsHeap(newLabel);
	if(best)
	{
		newLabel.referenced = true;
		newLabel.lastLabel->referenced = true; //mark last label as 'referenced' so it isnt deleted
	}
		
	if (dominated && !best)
	{	
		return false;
	}

	if(!best && !bestRCedLabel && heuristicPricing)
	{
		return false;
	}

	//start to actually insert, given that it isn't dominated
	newLabel.lastLabel->referenced = true; //mark last label as 'referenced' so it isnt deleted

	unique_ptr label_uptr = std::make_unique<PricingLabel>(newLabel);	
	

	auto just_inserted_itr = labels[j].insert(std::move(label_uptr)); //labels[j] assumes ownership of newLabel
	// if(!ret.second)
	// {
	// 	//elements have the same time. But, we already know it isn't dominated
	// 	//so, I remove the old one and add the new

	// 	//ps: std::move granted ownership to labels[j], but labels[j] chose not save it because of repeated key value, and thus label_uptr has already been deallocated!
		
	// 	if(ItrNextExpansion_IsValid[j] && ItrNextExpansion[j] == ret.first)
	// 	{
	// 		ItrNextExpansion_IsValid[j] = false;
	// 	}
		
	// 	if(!ret.first->referenced) labels[j].erase(ret.first);
	// 	auto ret2 = labels[j].insert(std::make_unique<PricingLabel>(newLabel));
	// 	assert(ret2.second);
	// 	just_inserted_itr = ret2.first;
	// 	pricing_ret.labelsDeleted++;
	// 	total_labels--;
	// }

	total_labels++;
	pricing_ret.labelsStored++;
	pricing_ret.maxLabelsStoredSimultaneously = std::max(pricing_ret.maxLabelsStoredSimultaneously, total_labels);

	//update ItrNextExpansion if necessary
	/*
		notice that if the current ItrNextExpansion will be deleted by the filter, than it must be true that just_added has smaller time than it
		This ensures that ItrNextExpansion will never point to a deleted element!

	*/
	
	if(!ItrNextExpansion_IsValid[j] || newLabel.time < (*ItrNextExpansion[j])->time)
	{
		ItrNextExpansion[j] = just_inserted_itr; //ret.first is an iterator to the just inserted element
		ItrNextExpansion_IsValid[j] = true;
	}

	FilterLabels(j, newLabel, just_inserted_itr);

	assert(std::is_sorted(labels[j].begin(), labels[j].end(), compLTime));
	//assert(std::is_sorted(labels[j].begin(), labels[j].end(), compGRC));
	
	return true;
}

//assumes the labels are already ordered by best reduced cost!
//start filtering at starting pos
void SpacedBellmanPricing::FilterLabels(int index, PricingLabel &just_added, std::multiset <unique_ptr<PricingLabel>, decltype(compLTime)*>::iterator just_added_itr)
{
	return;
	//assert(starting_pos != labels[index].begin());

	//PricingLabel just_added = (*std::prev(starting_pos));

	//std::sort(labels[index].begin(), labels[index].end(), comp);
	if(labels[index].size() <= 1) return;
	//if(starting_pos == labels[index].end()) 
	//	return;

	//auto prev = std::prev(starting_pos);
	//auto it = starting_pos;

	if(heuristicPricing)
	{
		double bestRC = (*labels[index].rbegin())->reducedCost;
		//remove if label is worse than bestRC
		auto remove_test_function = [&just_added, &bestRC](const unique_ptr<PricingLabel> &label)
		{ 
			//assert(label.time + RC_EPS > just_added.time);
			return !label->referenced && label->reducedCost > bestRC;
		};

		size_t size_before = labels[index].size();

		/*
		weirdly enough, erase_if for std::multiset  does not support providing a starting point for erasing, which I already know it's just_added_itr
		but its complexity is still logn when iterating and deleting would be nlogn...
		unless I can be confident that just_added_itr is very often in the end of the set, which would make iterating fast
		either way, have to test
		*/
		std::erase_if(labels[index], remove_test_function);
		
		//labels[index].erase(std::remove_if(labels[index].begin(), labels[index].end(), remove_test_function), labels[index].end());
		size_t size_after = labels[index].size();

		int labels_removed = size_before - size_after;
		assert(labels_removed >= 0);
		pricing_ret.labelsDeleted += labels_removed;
		total_labels -= labels_removed;

		return;

	}
	else
	{

		//remove if label is dominated by just_added
		auto remove_test_function = [&just_added](const unique_ptr<PricingLabel> &label)
		{ 
			//assert(label.time + RC_EPS > just_added.time);
			return !label->referenced && label->reducedCost > just_added.reducedCost && label->time > just_added.time; //RC_EPS?
		};

		size_t size_before = labels[index].size();

		/*
		weirdly enough, erase_if for std::multiset  does not support providing a starting point for erasing, which I already know it's just_added_itr
		but its complexity is still logn when iterating and deleting would be nlogn...
		unless I can be confident that just_added_itr is very often in the end of the set, which would make iterating fast
		either way, have to test
		*/
		std::erase_if(labels[index], remove_test_function);
		
		//labels[index].erase(std::remove_if(labels[index].begin(), labels[index].end(), remove_test_function), labels[index].end());
		size_t size_after = labels[index].size();

		int labels_removed = size_before - size_after;
		assert(labels_removed >= 0);
		pricing_ret.labelsDeleted += labels_removed;
		total_labels -= labels_removed;

		return;
	}
}

bool SpacedBellmanPricing::compGTime(const PricingLabel &l1, const PricingLabel &l2)
{
	return l1.time > l2.time;
}

bool SpacedBellmanPricing::compGRC(const PricingLabel &l1, const PricingLabel &l2)
{
	return l1.reducedCost > l2.reducedCost;
}

bool SpacedBellmanPricing::compLTime(const unique_ptr<PricingLabel> &l1, const unique_ptr<PricingLabel> &l2)
{
	return l1->time < l2->time;
}
bool SpacedBellmanPricing::compLTimeRef(const PricingLabel &l1, const PricingLabel &l2)
{
	return l1.time < l2.time;
}

bool SpacedBellmanPricing::compLRC(const unique_ptr<PricingLabel> &l1, const unique_ptr<PricingLabel> &l2)
{
	return l1->reducedCost < l2->reducedCost;
}
bool SpacedBellmanPricing::compLRCRef(const PricingLabel &l1, const PricingLabel &l2)
{
	return l1.reducedCost < l2.reducedCost;
}


void SpacedBellmanPricing::Cleanup()
{
	labels.clear();

	//free intermediate positions:
	for(int i = 0; i < intermediatePositions.size(); i++)
	{
		delete intermediatePositions[i];
	}
	intermediatePositions.clear();

	// best labels heap does NOT have ownership of labels!
	bestLabelsHeap.clear();

}

PricingReturn SpacedBellmanPricing::Price(int vehicle_id, int n_routes, vector<double>& alpha_duals, vector<double>& beta_duals, vector<Route>& outRoutes, vector<int> consideredRequests, set<pair<int, int>> forbiddenEdges, map<pair<int, int>, double> edgeDuals)
{
	assert(alpha_duals.size() == problemData->NbVehicles());
	assert(beta_duals.size() == problemData->NbRequests());
	
	//reset return struct
	pricing_ret = PricingReturn();

	
	this->n_desired_routes = n_routes;
	int preferredWaitingStation = problemData->GetWaitingStationByIndex(0)->id;
	const Vehicle* vehicle = problemData->getVehicle(vehicle_id);

	assert(consideredRequests.size() > 0);

	RouteExpander routeExpander(params);

	std::clock_t alg_start = std::clock();
	bool timeout = false;

	pricing_ret = PricingReturn();
	pricing_ret.nbConsideredRequests = consideredRequests.size();
	// int mostLabelsInRequest;

	//initialize first labels
	labels.clear();
	labels.reserve(consideredRequests.size());
	int index = 0;
	total_labels = 0;

	bestLabelsHeap.clear();

	ItrNextExpansion.clear();
	ItrNextExpansion_IsValid.clear();

	//annoying manual memory allocation, but not too crazy
	//vector<IntermediateVertex*> intermediatePositions;
	
	labels = vector<std::multiset <unique_ptr<PricingLabel>, decltype(compLTime)*> >();
	
	for (int i = 0; i < consideredRequests.size(); i++)
	{
		const Request* nextReq = problemData->GetRequest(consideredRequests[i]);

		int reqIndex = problemData->RequestIdToIndex(nextReq->id);
				
		double newTime;
		int waitingStation;
		bool useIntermediate;
		IntermediateVertex intermediateVertex;
		bool ret = routeExpander.checkRouteExpansion(problemData, vehicle_id, nextReq, problemData->GetInitialPosition(vehicle_id), vehicle->timeAvailable, newTime, waitingStation, useIntermediate, intermediateVertex);
		pricing_ret.labelsPriced++;

		if (!ret || newTime > problemData->timeHorizon)
		{
			continue;
		}			
		double lateness = problemData->weighted_lateness(nextReq, newTime);

		PricingLabel label;

		label.reqId = nextReq->id;
		label.reducedCost = lateness -alpha_duals[vehicle_id] - beta_duals[reqIndex];
		label.time = newTime;
		if(useIntermediate)
		{
			label.lastWaitingStation = -1;
			IntermediateVertex *ptr = new IntermediateVertex(intermediateVertex);
			intermediatePositions.push_back(ptr);
			label.intermediatePosition = ptr;
		}
		else
		{
			label.lastWaitingStation = waitingStation;
			label.intermediatePosition = NULL;
		}
		
		label.alreadyExpanded = false;
		
		label.lastLabel = NULL;
		label.referenced = false;

		bool ret2 = TryAddLabel(label, i, true);
	}


	/*
		do not consider requests if:
		 - vehicle is not compatible
		 - its beta reduced cost is negative (since no RC gain can be achieved by visiting it!), 
		 		unless the vehicles alpha RC is positive, which would still allow a route using the request!

		this is already checked during route construction, so we just borrow from the above initialization
	*/ 
	std::vector<int> new_consideredRequests; new_consideredRequests.reserve(labels.size());
	for(int i = 0; i < labels.size(); i++)
	{
		assert(ItrNextExpansion_IsValid[i]);
		assert(labels[i].begin() == ItrNextExpansion[i]);
		new_consideredRequests.push_back((*labels[i].begin())->reqId);
	}
	consideredRequests = new_consideredRequests;

	if(consideredRequests.size() == 0)
	{
		Cleanup();
		pricing_ret.status = PricingReturnStatus::FAIL;
		return pricing_ret;
	}
		
	assert(labels.size() == consideredRequests.size());

	/*
	since the vectors are ordered by increasing time, there is a certain position before which:
		- all labels have been expanded
	we keep track of this position for each request, and work up from there, one request at a time, until no expansions are possible anymore
	*/

	//initialized above!

	//bellman-ford like
	int iteration = 0;
	bool anySuccess = true;
	while (anySuccess && !timeout)
	{
		anySuccess = false;
		for(int i = 0; i < consideredRequests.size() && !timeout; i++)
		{
			const Request* req = problemData->GetRequest(consideredRequests[i]);
			int iLabel;

			if(labels[i].size() == 0) continue;
			if(!ItrNextExpansion_IsValid[i]) continue;

			for(	auto itr = heuristicPricing ? --labels[i].end() : ItrNextExpansion[i];
					itr != labels[i].end() && !timeout; 
					itr++
			)
			{
				const PricingLabel* label = itr->get();
				assert(req->id == label->reqId);
				assert(labels[i].size() > 0);

				if(label->alreadyExpanded) continue;

				for(int j = 0; j < consideredRequests.size() && !timeout; j++)
				{
					//try expading from i to j
					const Request* nextReq = problemData->GetRequest(consideredRequests[j]);
					assert(labels[j].size() == 0 || nextReq->id == (*labels[j].begin())->reqId);

					if(req->id == nextReq->id) continue;

					//if branching rule forbids this request-to-request connection, skip it
					pair<int, int> pair1 = std::make_pair(req->id, nextReq->id);
					pair<int, int> pair2 = std::make_pair(nextReq->id, req->id);

					if(forbiddenEdges.find(pair1) != forbiddenEdges.end() || forbiddenEdges.find(pair2) != forbiddenEdges.end()) continue;
					
					
					int iNextReq = problemData->RequestIdToIndex(nextReq->id);

					//manually checking for cycles! 
					//if !useRepeatedSetVerification, check is skipped
					// if (!useRepeatedSetVerification && label->coveredRequests.find(nextReq->id) != label->coveredRequests.end()) {
					// 	continue;
					// }

					double newTime = 0.0;
					int waitingStation = -1;
					bool useIntermediate = false;
					IntermediateVertex intermediateVertex = IntermediateVertex();
					bool ret = routeExpander.checkRouteExpansion(problemData, vehicle_id, nextReq, req, label->time, newTime, waitingStation, useIntermediate, intermediateVertex);
					pricing_ret.labelsPriced++;

					//if we've priced enough labels to exceed 95% of max_memory,
					if((((double)total_labels * sizeof(PricingLabel)) / 1000000) > (double) max_memory * 0.95)
					{
						//abort
						//to-do: maybe clean up and continue?
						std::cout << "memory out (" << total_labels << " labels)" << std::endl;
						timeout = true;
						break;
					}

					if (!ret) continue; //block routes

					double lateness = problemData->weighted_lateness(nextReq, newTime);
					assert(lateness > params->RCEpsilon);

					double newReducedCost = label->reducedCost + lateness - beta_duals[iNextReq];

					//add edge duals:
					int a = std::min(req->id, nextReq->id);
					int b = std::max(req->id, nextReq->id);
					assert(a != b);
					
					std::pair edge = std::make_pair(a, b);
					if(edgeDuals.contains(edge))
					{
						newReducedCost = newReducedCost - edgeDuals[edge];
					}
					//double newLateness = label->total_lateness + lateness;

					PricingLabel newLabel;
					newLabel.reqId = nextReq->id;
					newLabel.reducedCost = newReducedCost;
					newLabel.time = newTime;

					if(useIntermediate)
					{
						newLabel.lastWaitingStation = -1;
						IntermediateVertex *ptr = new IntermediateVertex(intermediateVertex);
						intermediatePositions.push_back(ptr);
						newLabel.intermediatePosition = ptr;
					}
					else
					{
						newLabel.lastWaitingStation = waitingStation;
						newLabel.intermediatePosition = NULL;
					}
					
					newLabel.alreadyExpanded = false;
					
					newLabel.lastLabel = label;
					newLabel.referenced = false;
					//newLabel->total_lateness = newLateness;

					//newLabel->arrTimeLastLabel = label->time;

					//newLabel->alreadyExpanded = false;

					// if(useRepeatedSetVerification)
					// {
					// 	newLabel->coveredRequests = label->coveredRequests; //copy
					// 	newLabel->coveredRequests.insert(nextReq->id);
					// }

					bool ret2 = TryAddLabel(newLabel, j);
					
					anySuccess = anySuccess || ret2;

					if(pricing_ret.labelsPriced % 1000 == 0)
					{
						std::clock_t alg_now = std::clock();
						double time = ((double)(alg_now - alg_start)) / CLOCKS_PER_SEC;
						if(time > max_time || params->Timeout())
						{
							timeout = true;
							break;
						}
					}
				}

				label->alreadyExpanded = true;
				
			}
			
			ItrNextExpansion[i] = labels[i].end(); //unfortunatelly .end() may move when we add elements to the set, so testing for end later won't work. Thus we need a bool to tell if the iterator is valid!
			ItrNextExpansion_IsValid[i] = false;
		
		}
		
		iteration++;
	}

	//assert all labels have been expanded
	#ifndef NDEBUG
		if(!heuristicPricing && !timeout)
		{
			for(int i = 0; i < labels.size(); i++)
			{
				for(auto &label : labels[i])
				{
					assert(label->alreadyExpanded);
				}
			}
		}
	#endif

	pricing_ret.mostLabelsInRequest = 0;

	if(bestLabelsHeap.size() == 0)
	{
		//no routes found with RC < 0
		Cleanup();
		pricing_ret.status = PricingReturnStatus::FAIL;
		return pricing_ret;
	}

	outRoutes.clear();

	pricing_ret.timeout = timeout;

	double bestRC = HUGE_VAL;
	int i = 0;
	for (int i = 0; i < bestLabelsHeap.size(); i++)
	{
		Route route;
		const PricingLabel* label = &bestLabelsHeap[i];

		//if label->reducedCost is positive or not very negative, dont return it
		if (label->reducedCost > -params->RCEpsilon) continue;

		double firstLabelRC = label->reducedCost;

		route.veh_index = vehicle_id;

		//first, add it all in reverse:
		while (true) 
		{
			if(label == NULL) break;

			//first, add it all in reverse:
			const Request* req = problemData->GetRequest(label->reqId);
			route.vertices.push_back(*req);
			//outRoutes[i].arrival_times.push_back(label->time);

			assert(!(label->intermediatePosition != NULL && label->lastWaitingStation != -1));

			if(label->intermediatePosition != NULL)
			{
				assert(label->intermediatePosition->id == -1);
				route.intermediates.push_back(*label->intermediatePosition);
				route.vertices.push_back((Vertex) *label->intermediatePosition);
			}
			if (label->lastWaitingStation != -1) { //if this transition stops at a waiting station...
				const WaitingStation* ws = problemData->GetWaitingStation(label->lastWaitingStation);
				route.vertices.push_back(*ws); //add waiting station
			}

			//get next label:

			assert(label->lastLabel == NULL || label->lastLabel->referenced);
			label = label->lastLabel;
		
		}
		
		//insert source vertex
		route.vertices.push_back(*problemData->GetInitialPosition(vehicle_id));
		route.veh_index = vehicle_id;

		//reverse entire route
		std::reverse(route.vertices.begin(), route.vertices.end());
		std::reverse(route.intermediates.begin(), route.intermediates.end());


		//tratar corretamente desvios nessas funcoes auxiliares
		route.SetArrivalsAndDepartures(problemData);

		route.UpdateCost(problemData);
		outRoutes.push_back(route);
		pricing_ret.reducedCostPerRoute.push_back(firstLabelRC);
		
	}

	#ifndef NDEBUG
	//before returning, check if pricing found repeated routes
	vector<size_t> hashVec;
	for(Route &route : outRoutes)
	{
		size_t hash = route.GetHash(problemData);
		if(std::find(hashVec.begin(), hashVec.end(), hash) != hashVec.end())
		{
			assert(false);
		}
		else hashVec.push_back(hash);
	}
	#endif

	Cleanup();
	pricing_ret.status = PricingReturnStatus::OK;
	return pricing_ret;


}

