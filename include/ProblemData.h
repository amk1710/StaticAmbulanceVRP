#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <random>
#include <assert.h>
#include <climits>
#include <map>

using std::unique_ptr;
using std::vector;
using std::string;

enum class DistanceType{
	euclidian, 
	geodesic, 
	osrm
};

//waiting station policy: how to handle waiting stations when routing vehicles?
enum class WaitingStationPolicy { 
	mandatoryStopInFixedStation, // forces ambulance to always stop at the same predetermined ws
	optionalStopInFixedStation, // ambulance may stop or not, always at predetermined station
	optionalStopInClosestWaitingStation, // ambulance may stop or not, always at closest station
	bestOptionalStop //ambulance may or may not stop. If stop, it may choose which ws to go to, picking the one with the best resulting time
};

struct Vehicle
{
	int type;
	double timeAvailable; //the time when this vehicle is first available
	int preferredWaitingStation; //id of waiting station to return to by default
};

struct Position
{
	double x;
	double y;
};

struct Vertex
{
	int identifier; //this number is arbitrary and used to uniquely identify a request in sets, vectors etc. 
	int id; //this number is the vertex id, and must match the order in which vertices appear in the distance matrix
	Position position;

	//useful for for std::count
	bool operator==(const Vertex& otherVertex) const
	{
		return this->id == otherVertex.id;

	}

	int closestWaitingStation; //precomputed closest ws, for speed

};

struct IntermediateVertex : public Vertex
{
	int ws_id; //id of ws where vehicle was going before it was redirected
};

struct Destination : public Vertex
{
	bool projected; //is this an actual Destination, of a projected future Destination?
};

struct InitialPosition : public Vertex
{
	
};

struct Request : public Vertex
{
public:
	
	int destination; // index of destination
	int type;
	double weight;
	double non_service_penalty;
	double arrival_time;
	double service_time; //time servicing node, *not* including movement

	bool projected; //is this an actual request, or a projected future request?

}; 

struct WaitingStation : public Vertex
{

	int capacity;
};

//should be copyable
class ProblemData
{
public:
	int NbVertices() const { return 2*NbRequests() + NbVehicles() + NbWaitingStations(); }
	int NbRequests() const { return requests.size() /*+ projected_requests.size()*/ ; }
	int NbVehicles() const { return vehicles.size(); }
	int NbWaitingStations() const { return waitingStations.size(); }

	int GetClosestDestination(int vertexId) const;

	string name;

	double timeHorizon;
	bool computeTimeHorizon = false;
	bool allowRerouting = false;
	WaitingStationPolicy waitingStationPolicy = WaitingStationPolicy::optionalStopInClosestWaitingStation;
	bool useTargetWaitTimeObjective = false;
	
	// ???
	std::map<double, double> target_times_per_weight = {{1, 30*60}, {2, 15*60}, {4, 10*60}};

	DistanceType distanceType;
	
private:

	std::vector<Vehicle> vehicles;

	std::vector<InitialPosition> initialPositions;
	std::vector<Request> requests;
	std::vector<Destination> destinations;
	std::vector<WaitingStation> waitingStations;

	// for computing this matrix, vertices are ordered by arbitrary id, in order
	//vertices are ordered: initial_positions, requests, destination, waiting_stations, (projected_requests, projected_destination)
	vector<vector<double>> distances; //distaces[i][j] -> distance from i to j
	
	static void CalculateDistanceMatrix(vector<Vertex*> &vertices, vector<vector<double>> &outMatrix, DistanceType distanceType);

public:	
	//calculates distance between points considering instance characteristics
	static double calculateDistance(const Position& pos1, const Position& pos2, DistanceType distanceType);
	static double euclidianDistance(const Position& pos1, const Position& pos2);
	static double geodesicDistance(const Position& pos1, const Position& pos2);

	double calculateDistance(const Position& pos1, const Position& pos2)
	{
		return calculateDistance(pos1, pos2, this->distanceType);
	}

	static Position GetIntermediatePosition2(Position p1, Position p2, double timeFromStart);
	static double VehicleSpeed() { return (60.0 / 3.6); } //60 km/h in m/s

	void exampleInstance();
	void readPDPTWInstance(string path);
	void readSDVRPTWInstance(string path);

	//reads instance number instance_index from one of Vincent files, if possible
	static bool readVincentInstance(string requests_path, string hospitals_path, string waiting_stations_path, string cleaning_stations_path, int instance_index, ProblemData &outInstance, bool tenColumns, std::string osmPath, bool useTimeHorizon = false, double timeHorizon = 0.0, int overwriteNbVehicles = -1); 

	//when using these getter functions, the usage (or not) of scenarios is transparent. 

	const InitialPosition* GetInitialPosition(int id) const;
	const InitialPosition* GetInitialPositionByIndex(int index) const;
	const Request* GetRequest(int id) const;
	const Request* GetRequestByIndex(int index) const;
	const Destination* GetDestination(int id) const;
	const Destination* GetDestinationByIndex(int index) const;
	const WaitingStation* GetWaitingStation(int id) const;
	const WaitingStation* GetWaitingStationByIndex(int id) const;

	const Vertex* GetVertex(int id) const;
	
	//these ones convert between id and index _when considering only (requests/destination/etc)
	//this is slightly weird but is useful for some pricing vectors
	int RequestIdToIndex(int id) const;
	int IndexToRequestId(int index) const;

	// changes number of vehicles in the instance to the new number of vehicles, 
	// by sequentially distributing vehicles amongts existing initial positions/waiting stations 
	void ResetNumberOfVehicles(int newNbVehicles);
	
private:
	//these ones follow the same idea but arent useful outside of class
	int InitialPositionIdToIndex(int id) const;
	int InitialPositionIndexToId(int index) const;
	int DestinationIdToIndex(int id) const;
	int IndexToDestinationId(int index) const;
	int WaitingStationIdToIndex(int id) const;
	int IndexToWaitingStationId(int index) const;

	void PrecomputeClosestWSs();
	int GetClosestWaitingStation(int vertexId) const;

public:

	bool IsRequest(int id) const;
	bool IsDestination(int id) const;
	bool IsInitialPosition(int id) const;
	bool IsWaitingStation(int id) const;
	bool IsIntermediateVertex(int id) const;

	bool IsCompatible(const Request* request, int vehicle_index);

	// get distance for the ommited representation of a route
	double OmmitedDistance(int i, int j);

	double Distance(int i, int j);

	const Vehicle* getVehicle(int id) { return &vehicles[id]; }

	double weighted_lateness(int req_id, double time);
	double weighted_lateness(const Request *request, double time);

	void SetVehicleAvailability(vector<double> vehicleAvailability);


private:
	//vector<Request> projected_requests;
	//vector<Destination> projected_destinations;
public:
	//void SetScenario(const vector<Request>& newRequests, const vector<Destination>& newDestinations);

	void SetVehiclePositions(vector<Position> vehiclePositions);

	ProblemData(); // no path -> example instance, hardcoded
	ProblemData(string pathToInstance, string instanceType = "pdptw");
	~ProblemData() {};

	static Position GetIntermediatePosition(Position source, Position destination, double t0, double t);

private:
	void Validate();
};