
#include "Params.h"

enum WhichStation { closest, best, preferred };

class RouteExpander
{
    Params *params;

private:
    

public:
    explicit RouteExpander(Params* params) : params(params)
    {

    }

    /*
		checks and evaluates cost of expanding a route 
		returns true if successful, false otherwise
	*/

    /*
        actually, the best way to have written this would be to have split this function in two: 
            one for constructing initial routes from the vehicle's initial position, 
            and other for expanding routes from request to request

        this one handles both cases and this introduces unecessary complexity
        maybe even splitting it into different functions for differents wsp would be good too

    */

    bool checkRouteExpansion(ProblemData *problemData, int vehicle_id, const Request* nextRequest, const Vertex* lastVertex, double lastVertexArrivalTime, double& outTime, int& outWaitingStationId, bool &outUseIntermediateIntermediateVertex,  IntermediateVertex &outIntermediateVertex);

    bool checkRouteExpansion(ProblemData *problemData, int vehicle_id, const Request* nextRequest, const Vertex* lastVertex, double lastVertexArrivalTime, double& outTime, int& outWaitingStationId)
    {
        IntermediateVertex intermediateVertex = IntermediateVertex();
        bool useIntermediate = false;
        bool ret = checkRouteExpansion(problemData, vehicle_id, nextRequest, lastVertex, lastVertexArrivalTime, outTime, outWaitingStationId, useIntermediate, intermediateVertex);
        return ret;
    }
};