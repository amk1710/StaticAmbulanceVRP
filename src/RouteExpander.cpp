
#include "RouteExpander.h"
#include "ProblemData.h"

#define RC_EPS 0.1


bool RouteExpander::checkRouteExpansion(ProblemData *problemData, int vehicle_id, const Request* nextRequest, const Vertex* lastVertex, double lastVertexArrivalTime, double& outTime, int& outWaitingStationId, bool &outUseIntermediateIntermediateVertex,  IntermediateVertex &outIntermediateVertex)
{
    assert(problemData != NULL);
    assert(problemData->IsInitialPosition(lastVertex->id) || problemData->IsRequest(lastVertex->id));

    WaitingStationPolicy wsPolicy = problemData->waitingStationPolicy;
    bool allowRerouting = problemData->allowRerouting;

    const Vehicle* vehicle = problemData->getVehicle(vehicle_id);

    bool mandatoryStop = wsPolicy == WaitingStationPolicy::mandatoryStopInFixedStation;
    WhichStation whichStation;
    if(wsPolicy == WaitingStationPolicy::mandatoryStopInFixedStation || wsPolicy == WaitingStationPolicy::optionalStopInFixedStation)
    {
        whichStation = WhichStation::preferred;
    }
    else if(wsPolicy == WaitingStationPolicy::optionalStopInClosestWaitingStation)
    {
        whichStation = WhichStation::closest;
    }
    else if(wsPolicy == WaitingStationPolicy::bestOptionalStop)
    {
        whichStation = WhichStation::best;
    }

    //rerouting from waiting stations to requests. No req-to-req rerouting admitted
    outUseIntermediateIntermediateVertex = false;

    double startingTime = std::max(lastVertexArrivalTime, vehicle->timeAvailable);

    if (!problemData->IsCompatible(nextRequest, vehicle_id)) return false; //block if request is not compatible with vehicle

    //time when vehicle is ready to leave its (startVertex) location
    double firstAvailable = startingTime;
    const Vertex* startVertex = lastVertex;

    //calculate correct firstAvailable, trying to NOT stop at intermediate ws
    if(problemData->IsInitialPosition(lastVertex->id))
    {
        // firstAvailable is already correct
        // startVertex is alreadyCorrect
    }
    else if(problemData->IsRequest(lastVertex->id))
    {
        const Request* lastRequest = problemData->GetRequest(lastVertex->id);
        firstAvailable += 
            lastRequest->service_time + 
            problemData->Distance(lastRequest->id, lastRequest->destination);
        startVertex = problemData->GetDestination(lastRequest->destination);
    }
    else std::invalid_argument("unhandled lastVertex case");

    //is it possible to not stop?
    if( !mandatoryStop && firstAvailable >= nextRequest->arrival_time ) //possible
    {
        
        double newTime = firstAvailable + problemData->Distance(startVertex->id, nextRequest->id);
        assert(newTime > nextRequest->arrival_time);

        if(newTime > problemData->timeHorizon) return false;

        //return:
        outTime = newTime;
        outWaitingStationId = -1;
        return true;
    }
    else //must go towards ws
    {

        double bestTime = HUGE_VAL;
        int bestWS = -1;
        IntermediateVertex bestIntermediateVertex;
        bool bestUseIntermediate;

        // iterate all and find best one:
        // if we don't really need to iterate, this for is degenerate, uses < 1 and ignores the counter
        for(int ws_i = 0; ws_i < (whichStation == WhichStation::best ? problemData->NbWaitingStations() : 1); ws_i++)
        {
            int ws_id = -1;
            if(whichStation == WhichStation::best)
            {
                ws_id = problemData->GetWaitingStationByIndex(ws_i)->id;
            }
            else if(whichStation == WhichStation::closest)
            {
                ws_id = lastVertex->closestWaitingStation;
            }
            else if(whichStation == WhichStation::preferred)
            {
                ws_id = vehicle->preferredWaitingStation;
            }

            assert(ws_id != -1);
            const WaitingStation* ws = problemData->GetWaitingStation(ws_id);
            
            double arriveAtWS = firstAvailable
                                + problemData->Distance(startVertex->id, ws->id);

            double firstLeaveWS = arriveAtWS;

            double newTime = 0.0;
            bool hasRerouted = false;
            IntermediateVertex intermediateVertex = IntermediateVertex();
            if(allowRerouting && arriveAtWS > nextRequest->arrival_time)
            {
                assert(firstLeaveWS == arriveAtWS);

                //this rerouting strategy is supposing that everything is geodesic. To-do: generalize this

                //sanity checks on intermediate positions:
                //both should be the halfway between points...
                double dist = problemData->Distance(startVertex->id, ws->id);
                Position pos1 = ProblemData::GetIntermediatePosition(startVertex->position, ws->position, firstAvailable, firstAvailable + problemData->Distance(startVertex->id, ws->id) / 2.0);
                Position pos2 = ProblemData::GetIntermediatePosition2(startVertex->position, ws->position, problemData->Distance(startVertex->id, ws->id) / 2.0);

                //
                Position intermediatePos = ProblemData::GetIntermediatePosition(startVertex->position, ws->position, firstAvailable, nextRequest->arrival_time);


                //
                Position intermediatePos2 = ProblemData::GetIntermediatePosition2(startVertex->position, ws->position, nextRequest->arrival_time - firstAvailable);
                
                //std::cout << "rerouting: " << intermediatePos.x << " , " << intermediatePos.y << std::endl;
                
                double t1 = ProblemData::geodesicDistance(startVertex->position, intermediatePos); //from start to intermediate pos
                double t2 = ProblemData::geodesicDistance(intermediatePos, nextRequest->position); //from intermediate pos to req
                double t3 = ProblemData::geodesicDistance(intermediatePos, ws->position);
                
                
                //std::cout << t1 << " , " << t2 << std::endl;

                assert(t1 + t2 + RC_EPS > ProblemData::geodesicDistance(startVertex->position, nextRequest->position)); 
                assert(t1 + t2 + RC_EPS > problemData->Distance(startVertex->id, nextRequest->id)); 
                assert(firstAvailable + t1  + RC_EPS > nextRequest->arrival_time);
                
                newTime = firstAvailable + t1 + t2;
                if(newTime < nextRequest->arrival_time) //this may happen dure to numerical errors. We fix it to not violated antecipativity
                {
                    newTime = nextRequest->arrival_time + t2;
                }
                
                assert(newTime >= nextRequest->arrival_time);

                hasRerouted = true;
                
                intermediateVertex.id = -1;
                intermediateVertex.ws_id = ws->id;
                intermediateVertex.position = intermediatePos;

                //rerouting was actually better than stopping at ws?
                
                //unfortunately, both of these reasonable assumptions may fail, apparently because of precision errors in coordinate and trigonometric operations!
                double alt = std::max(firstLeaveWS, nextRequest->arrival_time) + problemData->Distance(ws->id, nextRequest->id);
                assert(abs((firstAvailable + t1) - nextRequest->arrival_time) < RC_EPS);
                assert(newTime < alt + RC_EPS);

            }
            else
            {
                newTime = std::max(firstLeaveWS, nextRequest->arrival_time) //adjust for non-antecipativity
                            + problemData->Distance(ws->id, nextRequest->id);
                hasRerouted = false;
            }

            //std::cout << hasRerouted << " , " << newTime << " , " << nextRequest->arrival_time << std::endl;
            assert(newTime > nextRequest->arrival_time);

            if(newTime < bestTime + 0.1)
            {
                bestTime = newTime;
                bestWS = ws->id;

                if(hasRerouted)
                {
                    bestUseIntermediate = true;
                    bestIntermediateVertex = intermediateVertex;
                    bestWS = -1;
                }
                else 
                {
                    bestUseIntermediate = false;
                    bestIntermediateVertex = IntermediateVertex(); //empty
                }
            }

        }
        
        assert(bestWS != -1 || bestUseIntermediate == true);

        if(bestTime > problemData->timeHorizon) return false;

        //return:
        outTime = bestTime;
        outWaitingStationId = bestWS;
        outUseIntermediateIntermediateVertex = bestUseIntermediate;
        outIntermediateVertex = bestIntermediateVertex;
        return true;
    }

    return false;
   
}