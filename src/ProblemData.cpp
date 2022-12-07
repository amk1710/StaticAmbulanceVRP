
#include <assert.h>
//#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>  
#include <chrono>
#include <random>
//#include <float.h>
#include <limits.h>

#include "ProblemData.h"
#include "OSRMHelper.h"

using std::unique_ptr;
using std::make_unique;
using std::move;

#define RC_EPS 0.1


ProblemData::ProblemData()
{
	this->exampleInstance();
#ifdef _DEBUG
	this->Validate();
#endif

}

ProblemData::ProblemData(string pathToInstance, string instance_type)
{
	name = pathToInstance.substr(pathToInstance.find_last_of("/\\") + 1);
	std::string::size_type const p(name.find_last_of('.'));
	name = name.substr(0, p);
	if (pathToInstance == "") {
		std::cout << "using example instance " << std::endl;
		exampleInstance();
	}
	else if (instance_type == "pdptw") readPDPTWInstance(pathToInstance);
	else if (instance_type == "sdvrptw") readSDVRPTWInstance(pathToInstance);
	else throw std::invalid_argument("unsuported instance type");
#ifdef _DEBUG
	this->Validate();
#endif

}

void ProblemData::Validate()
{
	assert(NbRequests() > 0);
	assert(NbVehicles() > 0);
	assert(requests.size() == destinations.size());
	int i = 0;
	for (; i < initialPositions.size(); i++) assert(initialPositions[i].id == i);
	for (int ii = 0; ii < requests.size(); ii++)
	{
		assert(requests[ii].id == i);
		i++;
	}
	for (int ii = 0; ii < requests.size(); ii++)
	{
		assert(destinations[ii].id == i);
		i++;
	}
	for (int ii = 0; ii < waitingStations.size(); ii++)
	{
		assert(waitingStations[ii].id == i);
		i++;
	}
	for (int iVeh = 0; iVeh < vehicles.size(); iVeh++)
	{
		assert(IsWaitingStation(vehicles[iVeh].preferredWaitingStation));
	}

	int nbVertices = NbVertices();
	assert(nbVertices == distances.size());
	for(int i = 0; i < distances.size(); i++)
	{
		assert(distances[i].size() == nbVertices);
	}
}

void ProblemData::SetVehicleAvailability(vector<double> vehicleAvailability)
{
	assert(vehicleAvailability.size() == vehicles.size());

	for (int i = 0; i < vehicles.size(); i++)
	{
		if(vehicleAvailability[0] < 0.0) throw std::invalid_argument("negative value not allowed");
		vehicles[i].timeAvailable = vehicleAvailability[i];
	}
}

void ProblemData::exampleInstance()
{
	int nbRequests = 5;

	timeHorizon = 1000;

	//one vehicle
	Vehicle vehicle;
	vehicle.type = 0; vehicle.timeAvailable = 0.0;
	vehicles.push_back(vehicle);
	
	Position ipos; ipos.x = 0; ipos.y = 0;
	InitialPosition initial_position;
	initial_position.id = 0;
	initial_position.position = ipos;
	initialPositions.push_back(initial_position);

	int offset = 1;
	requests.reserve(nbRequests);
	destinations.reserve(nbRequests);
	for(int i = 0; i < nbRequests; i++)
	{
		Request request; Position rpos;
		request.id = offset + i; rpos.x = -5; rpos.y = i*10;
		request.position = rpos;
		request.weight = 1.0; request.non_service_penalty = 1000.0; 
		request.arrival_time = 0.0; request.service_time = 1.0;
		request.type = 0;
		request.destination = request.id + nbRequests;
		request.projected = false;

		Destination destination; Position dpos;
		destination.id = request.id + nbRequests;
		dpos.x = 5; dpos.y = 0;
		destination.position = dpos;
		destination.projected = false;
		
		requests.push_back(request);
		destinations.push_back(destination);

	}

	WaitingStation ws; Position wpos;
	ws.id = 1 + nbRequests * 2;
	wpos.x = 0.0; wpos.y = 0.0;
	ws.capacity = 0;
	ws.position = wpos;
	waitingStations.push_back(ws);

	//vertices are ordered: initial_positions, requests, destination, waiting_stations
	vector<Vertex*> vertices;
	vertices.resize(NbVertices());
	for (int i = 0; i < initialPositions.size(); i++) vertices[i] = &initialPositions[i];
	for (int i = 0; i < requests.size(); i++) vertices[NbVehicles() + i] = &requests[i];
	for (int i = 0; i < destinations.size(); i++) vertices[NbVehicles() + nbRequests + i] = &destinations[i];
	for (int i = 0; i < waitingStations.size(); i++) vertices[NbVehicles() + 2*nbRequests + i] = &waitingStations[i];

	distanceType = DistanceType::euclidian;
	distances = vector<vector<double>>();
	CalculateDistanceMatrix(vertices, distances, DistanceType::euclidian);
	
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i]->identifier = vertices[i]->id;
	}

	for (int i = 0; i < vehicles.size(); i++)
	{
		vehicles[i].preferredWaitingStation = GetClosestWaitingStation(i);
	}

	return;

}

void ProblemData::readPDPTWInstance(string path)
{
	timeHorizon = 1.0;

	std::default_random_engine generator;				// Random number generator, used for 'completing' missing info on some instances
	int seed = 0;
	generator.seed(0);

	//random nb of vehicles one vehicle
	std::uniform_int_distribution<> distr(25, 50); // Defining the distribution
	int nbVehicles = distr(generator);
	Vehicle vehicle;
	vehicle.type = 0;
	vehicle.timeAvailable = 0.0;
	for(int i = 0; i < nbVehicles; i++)	vehicles.push_back(vehicle);
	
	std::ifstream infile(path);
	std::string line;

	int iline = 0;

	if(!infile.good()) throw std::invalid_argument("file not found");

	vector<Position> positions; //this is a silly 'shuffle' of positions, but I want to keep the instanced the same way they were 
	

	while (std::getline(infile, line))
	{
		if (iline == 0)	{ //skip first line
			iline++; continue;
		}
		std::istringstream iss(line);
		int id, posx, posy, demand, earliest_pickup, latest_pickup, service_time, pickup_i, delivery_i;
		if (!(iss >> id >> posx >> posy >> demand >> earliest_pickup >> latest_pickup >> service_time >> pickup_i >> delivery_i)) {
			throw std::invalid_argument("unexpected file format");
		} // error

		if (id == 0) {//is depot
			InitialPosition ipos; ipos.id = id;
			Position pos; pos.x = posx; pos.y = posy;
			ipos.position = pos;
			for (int i = 0; i < nbVehicles; i++) {
				ipos.id = i;
				initialPositions.push_back(ipos);
				positions.push_back(pos);
			}
			

		}
		else if (demand > 0) {
			Position pos; pos.x = posx; pos.y = posy;
			Request req; req.id = id;
			req.position = pos;
			req.arrival_time = earliest_pickup;
			req.destination = delivery_i; //we dont know this yet
			req.non_service_penalty = demand * 20000;
			req.service_time = service_time;
			req.type = 0;
			req.weight = demand;
			req.projected = false;

			requests.push_back(req);
			positions.push_back(pos);

			if (timeHorizon < req.arrival_time) timeHorizon = req.arrival_time;
		}
		else if (demand < 0)
		{
			Position pos; pos.x = posx; pos.y = posy;
			Destination dest;
			dest.id = id;
			dest.position = pos;
			dest.projected = false;
			destinations.push_back(dest);
			positions.push_back(pos);
		}
		
		iline++;
	}

	timeHorizon = 2.0 * timeHorizon;


	WaitingStation ws; Position wpos;
	ws.id = nbVehicles + requests.size() * 2;
	wpos.x = initialPositions[0].position.x; wpos.y = initialPositions[0].position.y;
	ws.position = wpos;
	ws.capacity = 0;
	waitingStations.push_back(ws);
	positions.push_back(wpos);

	//fix indices
	//for now, i jumble requests-destination pairings
	//this also does a silly 'shuffle' of positions, but I want to keep the instanced the same way they were 
	int i = 0;
	for (; i < nbVehicles; i++)
	{
		initialPositions[i].id = i;
		initialPositions[i].position = positions[i];
	}
	for (int ii = 0; ii < requests.size(); ii++)
	{
		requests[ii].id = i;
		requests[ii].destination = i + requests.size();
		requests[ii].position = positions[i];
		i++;
	}
	for (int ii = 0; ii < requests.size(); ii++)
	{
		destinations[ii].id = i;
		destinations[ii].position = positions[i];
		i++;
	}
	for (int ii = 0; ii < waitingStations.size(); ii++)
	{
		waitingStations[ii].id = i;
		waitingStations[ii].position = positions[i];
		i++;
	}

	//vertices are ordered: initial_positions, requests, destination, waiting_stations
	vector<Vertex*> vertices;
	vertices.resize(NbVertices());
	for (int i = 0; i < initialPositions.size(); i++) vertices[i] = &initialPositions[i];
	for (int i = 0; i < requests.size(); i++) vertices[NbVehicles() + i] = &requests[i];
	for (int i = 0; i < destinations.size(); i++) vertices[NbVehicles() + NbRequests() + i] = &destinations[i];
	for (int i = 0; i < waitingStations.size(); i++) vertices[NbVehicles() + 2 * NbRequests() + i] = &waitingStations[i];


#ifdef DEBUG
	for (int i = 0; i < nbVertices.size(); i++) assert(vertices[i]->id == i);
#endif // DEBUG

	distanceType = DistanceType::euclidian;
	distances = vector<vector<double>>();
	CalculateDistanceMatrix(vertices, distances, DistanceType::euclidian);

	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i]->identifier = vertices[i]->id;
	}

	for (int i = 0; i < vehicles.size(); i++)
	{
		vehicles[i].preferredWaitingStation = GetClosestWaitingStation(i);
	}

	return;

}

void ProblemData::CalculateDistanceMatrix(vector<Vertex*> &vertices, vector<vector<double>> &outMatrix, DistanceType distanceType)
{
	//to-do: osrm
	outMatrix = vector<vector<double>>();
	outMatrix.reserve(vertices.size());
	for (int i = 0; i < vertices.size(); i++)
	{
		outMatrix.push_back(vector<double>(vertices.size()));
		for (int j = 0; j < vertices.size(); j++)
		{
			if (i == j) outMatrix[i][j] = 0.0;
			else outMatrix[i][j] = calculateDistance(vertices[i]->position, vertices[j]->position, distanceType);
		}
	}
}

/*/
for adapting the sdvrp instances, I use depots both as destinations and as waiting stations
*/
void ProblemData::readSDVRPTWInstance(string path)
{
	std::ifstream infile(path);
	std::string line;

	if (!infile.good()) throw std::invalid_argument("file not found");

	//first line has '7' for instance type, nbVehicles, nbRequests and number of types of vehicles
	std::getline(infile, line); std::istringstream iss(line);
	int type, m, n, n_vehicle_types;
	if (!(iss >> type >> m >> n >> n_vehicle_types)) {
		throw std::invalid_argument("unexpected file format");
	} // error
	if(type != 7) throw std::invalid_argument("unexpected file format");
	
	int nbVehicles = m;
	for (int i = 0; i < nbVehicles; i++)
	{
		Vehicle veh; veh.type = 0; veh.timeAvailable = 0.0;
		vehicles.push_back(veh);
	}
	int nbRequests = n;
	int nbWaitingStations = nbVehicles;
	//positions.resize(nbVehicles + nbRequests * 2 + nbWaitingStations);

	timeHorizon = 0.0;
	for (int i = 0; i < n_vehicle_types; i++)
	{
		std::getline(infile, line); std::istringstream iss(line);
		int max_duration, max_load;
		if (!(iss >> max_duration >> max_load)) {
			throw std::invalid_argument("unexpected file format");
		} // error
		timeHorizon = std::max(timeHorizon, (double)max_duration);
	}


	while (std::getline(infile, line))
	{
		std::istringstream iss(line);
		int id, service_time, demand, freq, nbCombinations, earliest_pickup, latest_pickup;
		double posx, posy;
		int trash;
		if (!(iss >> id >> posx >> posy >> service_time >> demand >> freq >> nbCombinations)) throw std::invalid_argument("unexpected file format");
		for (int i = 0; i < nbCombinations; i++) iss >> trash; //discard combination lists, for now
		if (!(iss >> earliest_pickup >> latest_pickup)) throw std::invalid_argument("unexpected file format");

		if (id == 0) {//is depot
			//reproduce depot/waiting stations multiple times
			Position pos; pos.x = posx; pos.y = posy;
			InitialPosition ipos; ipos.id = -1;
			ipos.position = pos;
			WaitingStation ws; ws.id = -1;
			ws.position = pos;
			
			for (int i = 0; i < nbVehicles; i++) {
				ipos.id = i; 
				initialPositions.push_back(ipos); //positions[i] = pos;
				
				int w_id = nbVehicles + 2 * nbRequests + i;
				ws.id = w_id; waitingStations.push_back(ws);
				//positions[w_id] = pos;

			}
			
		}
		else {
			Position pos; pos.x = posx; pos.y = posy;
			Request req; req.id = nbVehicles + id - 1;
			req.position = pos;
			req.arrival_time = earliest_pickup;
			req.destination = req.id + nbRequests; //we dont know this yet
			req.non_service_penalty = demand * 10000;
			req.service_time = service_time;
			std::uniform_int_distribution<> distr(0, nbVehicles); // Defining the distribution
			//req.type = distr(generator);
			req.type = 0;
			req.weight = demand;
			req.projected = false;

			requests.push_back(req);
			
			//positions[req.id] = pos;

			Destination dest;
			dest.id = req.id + nbRequests;
			dest.position = pos;
			dest.projected = false;
			destinations.push_back(dest);
			//positions[dest.id] = pos;
		}

	}

	int nbVertices = nbVehicles + nbRequests * 2 + nbWaitingStations;
	//vertices are ordered: initial_positions, requests, destination, waiting_stations
	vector<Vertex*> vertices;
	vertices.resize(nbVertices);
	for (int i = 0; i < initialPositions.size(); i++) vertices[i] = &initialPositions[i];
	for (int i = 0; i < requests.size(); i++) vertices[nbVehicles + i] = &requests[i];
	for (int i = 0; i < destinations.size(); i++) vertices[nbVehicles + nbRequests + i] = &destinations[i];
	for (int i = 0; i < waitingStations.size(); i++) vertices[nbVehicles + 2 * nbRequests + i] = &waitingStations[i];


#ifdef DEBUG
	for (int i = 0; i < nbVertices.size(); i++) assert(vertices[i]->id == i);
#endif // DEBUG

	distanceType = DistanceType::euclidian;
	distances = vector<vector<double>>();
	CalculateDistanceMatrix(vertices, distances, DistanceType::euclidian);

	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i]->identifier = vertices[i]->id;
	}

	for (int i = 0; i < vehicles.size(); i++)
	{
		vehicles[i].preferredWaitingStation = GetClosestWaitingStation(i);
	}

	return;

}

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
bool ProblemData::readVincentInstance(string requests_path, string hospitals_path, string waiting_stations_path, string cleaning_stations_path, int instance_index, ProblemData &outInstance, bool tenColumns, std::string osmPath, bool useTimeHorizon, double timeHorizon, int overwriteNbVehicles)
{
	bool useOSM = false;
	if(osmPath != "") useOSM = true;

	vector<InitialPosition> initialPositions;
	vector<WaitingStation> waitingStations;
	vector<Vehicle> vehicles;

	vector<Position> hospitalPositions;
	
	//file 1: bases.txt
	/* 
		assume each base only has one ambulance in it and capacity for one ambulance
		also assume that this ambulance has the base as its home base
	*/
	{
		std::ifstream infile(waiting_stations_path);
		std::string line;

		bool useOverwriteNbVehicles = overwriteNbVehicles > 0;
		
		if (!infile.good()) throw std::invalid_argument("file " + waiting_stations_path + " not found");

		double posx, posy;

		int veh_type = 0;

		while (std::getline(infile, line))
		{
			if(line == "END") break;

			if(useOverwriteNbVehicles && vehicles.size() >= overwriteNbVehicles) break;

			std::istringstream iss(line);
			//(lat, long) order -> (posy, posx)
			if (!(iss >> posy >> posx)) {
				throw std::invalid_argument("unexpected file format");
			}

			Position pos; pos.x = posx; pos.y = posy;
			InitialPosition ipos; ipos.id = initialPositions.size(); ipos.identifier = initialPositions.size();
			ipos.position = pos;
			initialPositions.push_back(ipos);

			WaitingStation ws; ws.id = -1;
			ws.position = pos;
			//positions[i] = pos;
			waitingStations.push_back(ws);


			//alterna tipos de veiculo, do 0 ate 2
			Vehicle veh; veh.type = veh_type; veh.timeAvailable = 0.0;
			veh_type = (veh_type + 1) % 3;
			veh.preferredWaitingStation = -1;
			vehicles.push_back(veh);

		}

		if(useOverwriteNbVehicles)
		{
			//extend up to overWriteNbVehicles
			int origNb = vehicles.size();
			for(int i = 0; vehicles.size() < overwriteNbVehicles; i++)
			{
				//create copy from original veh
				int origVeh = i % origNb;
				Position pos; pos.x = initialPositions[origVeh].position.x; pos.y = initialPositions[origVeh].position.y;
				InitialPosition ipos; ipos.id = initialPositions.size(); ipos.identifier = initialPositions.size();
				ipos.position = pos;
				initialPositions.push_back(ipos);

				//alterna tipos de veiculo, do 0 ate 2
				Vehicle veh; veh.type = veh_type; veh.timeAvailable = 0.0;
				veh_type = (veh_type + 1) % 3;
				veh.preferredWaitingStation = -1;
				vehicles.push_back(veh);

			}
		}

	}

	//file 2: hospitals.txt
	/* 
		read hospitals locations, later to be used as destinations
	*/
	{
		std::ifstream infile(hospitals_path);
		std::string line;

		if (!infile.good()) throw std::invalid_argument("file " + hospitals_path + " not found");
		double posx, posy;

		while (std::getline(infile, line))
		{
			if(line == "END") break;
			std::istringstream iss(line);
			//(lat, long) order -> (posy, posx)
			if (!(iss >> posy >> posx)) {
				throw std::invalid_argument("unexpected file format");
			}

			Position pos; pos.x = posx; pos.y = posy;
			hospitalPositions.push_back(pos);
		}

	}

	vector<Position> cleaningBases;
	vector<int> reqToCleaningBaseIndex;

	//aux file: cleaning bases file
	/* 
		read cleaning base locations, later to be used as destinations
	*/
	{
		std::ifstream infile(cleaning_stations_path);
		std::string line;

		if (!infile.good()) throw std::invalid_argument("file " + cleaning_stations_path + " not found");
		double posx, posy;

		//first line is nb of stations:
		std::getline(infile, line);
		int nbCleaning;
		std::istringstream iss(line);
		if (!(iss >> nbCleaning)) {
			throw std::invalid_argument("unexpected file format: " + cleaning_stations_path);
		}

		while (std::getline(infile, line))
		{
			if(line == "END") break;
			std::istringstream iss(line);
			//(lat, long) order -> (posy, posx)
			if (!(iss >> posy >> posx)) {
				break;
				//throw std::invalid_argument("unexpected file format");
			}

			Position pos; pos.x = posx; pos.y = posy;
			cleaningBases.push_back(pos);
		}

		if(nbCleaning != cleaningBases.size())
			throw std::invalid_argument("unexpected file format: " + cleaning_stations_path);

	}

	//file 3: calls file
	/* 
		file has several scenarios/instance. Iterate until found the one with the correct index
	*/
    std::mt19937 gen(0);
	{
		std::ifstream infile(requests_path);
		std::string line;
		if (!infile.good()) throw std::invalid_argument("file not found : " + requests_path);

		double time, time_on_scene, lat, lon, time_at_hospital, time_cleaning_base;
		double region_index, priority, day, cleaning_needed, hospital_needed, index_hospital, index_cleaning; //(int, but double for correct parsing)

		int nbRequests;
		
		int instance_i = 0;
		//move to correct instance_index
		while(instance_i < instance_index && std::getline(infile, line))
		{
			std::istringstream iss(line);
			if (!(iss >> nbRequests)) {
				throw std::invalid_argument("unexpected file format:" + requests_path);
			}

			for(int i = 0; i < nbRequests; i++)
			{
				if(!std::getline(infile, line) ) throw std::invalid_argument("unexpected file format" + requests_path);
			}

			instance_i++;
		}

		if(instance_i != instance_index) throw std::invalid_argument(std::to_string(instance_index) + " exceeds number of instances in file " + requests_path);

		while(std::getline(infile, line))
		{	
			std::istringstream iss(line);
			if (!(iss >> nbRequests)) {
					throw std::invalid_argument("unexpected file format");
			}

			outInstance = ProblemData();
			outInstance.name = "V-" + std::to_string(instance_index) + "-" + std::to_string(nbRequests);

			if(useTimeHorizon)
			{	
				outInstance.timeHorizon = timeHorizon;
			}
			else 
			{
				outInstance.timeHorizon = std::numeric_limits<double>::infinity();
			}

			outInstance.vehicles = vehicles;
			outInstance.initialPositions = initialPositions;
			outInstance.waitingStations = waitingStations;
			for(int i = 0; i < outInstance.waitingStations.size(); i++)
			{
				int w_id = outInstance.vehicles.size() + 2 * nbRequests + i;
				outInstance.waitingStations[i].id = w_id;
				outInstance.waitingStations[i].identifier = w_id;
				outInstance.vehicles[i].preferredWaitingStation = w_id;
			}
			
			outInstance.requests.clear();
			outInstance.destinations.clear();
			outInstance.requests.reserve(nbRequests);
			outInstance.destinations.reserve(nbRequests);
			for(int i = 0; i < nbRequests; i++)
			{
				if(!std::getline(infile, line) ) throw std::invalid_argument("unexpected file format : missing line");
				std::istringstream iss(line);
				
				
				if(!tenColumns)
				{
					//line order
					//Time - Region index - Priority - Day - Time on scene - Lat - Long - Time at hospital - TimeCleaningBase - Cleaning needed - Hospital needed - index hospital - index_cleaning

					/*
						index_cleaning has been added recently, it represents the cleaning location where the ambulance should be sent
						however, I have not implemented this
					*/

					if (!(iss >> time >> region_index >> priority >> day >> time_on_scene >> lat >> lon >> time_at_hospital >> time_cleaning_base >> cleaning_needed >> hospital_needed >> index_hospital >> index_cleaning)) {
						throw std::invalid_argument("unexpected file format : misread line data");
					}
				}
				else
				{
					//Time Lat Long Priority Cleaning_needed Cleaning_time Index hospital Time_on_scene Hospital needed Time at hospital
					if (!(iss >> time >> lon >> lat >> priority >> cleaning_needed >> time_cleaning_base >> index_hospital >> time_on_scene >> hospital_needed >> time_at_hospital)) {
						throw std::invalid_argument("unexpected file format : misread line data");
					}
				}
				
				

				Position pos; 

				pos.x = lon; 
				pos.y = lat;

				Request req; req.id = outInstance.initialPositions.size() + i; req.identifier = req.id;
				req.position = pos;
				req.arrival_time = time; //will be multiplied by 3600 later
				req.destination = req.id + nbRequests; //temporarily store hospital index here, will be fixed later

				//
				//*3600 because file is in fractions of hours
				req.service_time = time_on_scene * 3600 + time_at_hospital * 3600;

				//time_on_scene and index_hospital
				
				req.type = priority;
				if(priority == 0)
				{
					req.weight = 4;
				}
				else if(priority == 1)
				{
					req.weight = 2;
				}
				else if(priority == 2)
				{
					req.weight = 1;
				}
				else throw std::invalid_argument("unexpected file format : out of bounds priority");

				assert(req.weight > 0.0);

				req.projected = false;

				if(useTimeHorizon)
				{
					//not servicing worse than servicing at the last viable time
					req.non_service_penalty = (outInstance.timeHorizon + 1.0) * req.weight; // ??????????
				}
				else
				{
					req.non_service_penalty = std::numeric_limits<double>::infinity(); //all requests must be serviced!
				}

				// cheating out cleaning requirements in this model:
				// 1st cleaning time is added to service time
				// see below for distance adjustments 
				if(cleaning_needed)
				{
					req.service_time += time_cleaning_base * 3600;
					reqToCleaningBaseIndex.push_back(index_cleaning);
				}
				else reqToCleaningBaseIndex.push_back(-1);
				
				outInstance.requests.push_back(req);
				
				//positions[req.id] = pos;

				//if hospital isn't needed, to do (set destination to same as request, set times to zero)
				assert(hospital_needed);

				Destination dest;
				dest.id = req.id + nbRequests; dest.identifier = dest.id;
				dest.position = hospitalPositions[(int) index_hospital];
				dest.projected = false;

				outInstance.destinations.push_back(dest);

			}
			assert(outInstance.requests.size() == nbRequests && outInstance.destinations.size() == nbRequests);

			int nbVehicles = outInstance.vehicles.size();
			int nbVertices = outInstance.vehicles.size() + nbRequests * 2 + outInstance.waitingStations.size();
			//vertices are ordered: initial_positions, requests, destination, waiting_stations
			vector<Vertex*> vertices;
			vertices.resize(nbVertices);
			for (int i = 0; i < initialPositions.size(); i++) vertices[i] = &outInstance.initialPositions[i];
			for (int i = 0; i < outInstance.requests.size(); i++) vertices[nbVehicles + i] = &outInstance.requests[i];
			for (int i = 0; i < outInstance.destinations.size(); i++) vertices[nbVehicles + nbRequests + i] = &outInstance.destinations[i];
			for (int i = 0; i < waitingStations.size(); i++) vertices[nbVehicles + 2 * nbRequests + i] = &outInstance.waitingStations[i];

			outInstance.distanceType = useOSM ? DistanceType::osrm : DistanceType::geodesic;

			// 1st step: fill cleaning cases in distance matrix
			/* For each request demanding cleaning:
				- set its distance to destination to original distance + distance from destination to cleaning base
				- change destination's coordinates to cleaning base

			after this is done, calculate other coordinates
			*/

			vector<vector<double>> distances = vector<vector<double>>();
			distances.reserve(nbVertices);
			//initialize empty:
			for(int i = 0; i < vertices.size(); i++)
			{
				vector<double> tVec(vertices.size(), -1.0);
				distances.push_back(tVec);
			}
			
			//first, do all req->dest pairs, because of cleaning bases cheat
			for(int iReq = 0; iReq < outInstance.NbRequests(); iReq++)
			{
				const Request *req = outInstance.GetRequestByIndex(iReq);
				Destination *dest = &outInstance.destinations[outInstance.DestinationIdToIndex(req->destination)];
				if(reqToCleaningBaseIndex[iReq] != -1)
				{
					//this request requires cleaning. Adjust distances and coordinates accordingly
					int cleaningIndex = reqToCleaningBaseIndex[iReq];
					assert(cleaningIndex >= 0 && cleaningIndex < cleaningBases.size());
					distances[req->id][req->destination] = calculateDistance(req->position, dest->position, outInstance.distanceType) + calculateDistance(dest->position, cleaningBases[cleaningIndex], outInstance.distanceType);
					
					// the destination's position is changed to the cleaning base, since this is where the service actually ends
					dest->position = cleaningBases[cleaningIndex];

					//unfortunately we're losing data here, but these positions won't be necessary anyway
				}
				else
				{
					distances[req->id][req->destination] = calculateDistance(req->position, dest->position, outInstance.distanceType);
				}

				// 2nd step: calculate temp matrix of real distances
				// (by using this intermediate matrix we're calculating some useless distances, but calculating them individually is bad in the OSRM case and elimating the repeated pairs before calculations is a nightmare)
				vector<vector<double>> temp_matrix;
				CalculateDistanceMatrix(vertices, temp_matrix, outInstance.distanceType);

				//3rd step: fill out everyone else and don't alter req->dest pairs
				for (int i = 0; i < nbVertices; i++)
				{
					for (int j = 0; j < nbVertices; j++)
					{
						if (i == j) 
						{
							distances[i][j] = 0.0;
							continue;
						}
						else if(outInstance.IsRequest(i) && outInstance.IsDestination(j))
						{
							const Request *req = outInstance.GetRequest(i);
							const Destination *dest = outInstance.GetDestination(j);
							if(req->destination == dest->id) //if this is a req - dest pair
							{
								//already calculated above, dont change it!
								continue;
							}
						}

						distances[i][j] = temp_matrix[i][j]; 
					}
				}

				outInstance.distances = distances;
			}

			#ifdef _DEBUG
			//verify if all values have been initialized
			for(int i = 0; i < vertices.size(); i++)
			{
				for(int j = 0; j < vertices.size(); j++)
				{
					assert(outInstance.distances[i][j] < 0);
				}
			}
			#endif

			//fix request arrival times:
			// get initial time over all requests , then use floor
			double initialTime = HUGE_VAL;
			for(int i = 0; i < outInstance.requests.size(); i++)
			{
				initialTime = std::min(initialTime, outInstance.requests[i].arrival_time);
			}
			initialTime = std::floor(initialTime);
			for(int i = 0; i < outInstance.requests.size(); i++)
			{
				outInstance.requests[i].arrival_time = (outInstance.requests[i].arrival_time - initialTime) * 3600; // fractions of hour * 3600 = seconds
			}

			outInstance.PrecomputeClosestWSs();

			outInstance.Validate();

			return true;
		}

	}

	return false;

}

const InitialPosition* ProblemData::GetInitialPositionByIndex(int index) const
{
	assert(index >= 0 && index < initialPositions.size());
	return &initialPositions[index];
}
const InitialPosition* ProblemData::GetInitialPosition(int id) const
{
	int index = InitialPositionIdToIndex(id);
	return &initialPositions[index];
}

const Request* ProblemData::GetRequestByIndex(int index) const
{
	assert(index >= 0 && index < NbRequests());
	assert(!requests[index].projected);
	return &requests[index];

	// if (index < requests.size()) 
	// {
	// 	assert(!requests[index].projected);
	// 	return &requests[index];
	// }
	// else
	// {
	// 	assert(projected_requests[index - requests.size()].projected);
	// 	return &projected_requests[index - requests.size()];
	// }
}

const Request* ProblemData::GetRequest(int id) const
{
	int index = RequestIdToIndex(id);
	return GetRequestByIndex(index);
}

const WaitingStation* ProblemData::GetWaitingStationByIndex(int index) const
{
	assert(index >= 0 && index < waitingStations.size());
	return &waitingStations[index];
}
const WaitingStation* ProblemData::GetWaitingStation(int id) const
{
	assert(IsWaitingStation(id));
	int index = WaitingStationIdToIndex(id);
	return GetWaitingStationByIndex(index);
}

const Destination* ProblemData::GetDestinationByIndex(int index) const
{
	assert(index >= 0 && index < NbRequests());
	assert(!destinations[index].projected);
	return &destinations[index];
	
	// if (index < destinations.size())
	// {
	// 	assert(!destinations[index].projected);
	// 	return &destinations[index];
	// }
	// else
	// {
	// 	assert(projected_destinations[index - destinations.size()].projected);
	// 	return &projected_destinations[index - destinations.size()];
	// }
}
const Destination* ProblemData::GetDestination(int id) const
{
	assert(IsDestination(id));
	int index = DestinationIdToIndex(id);
	return GetDestinationByIndex(index);
}

bool ProblemData::IsIntermediateVertex(int id) const 
{
	assert(id >= -1 && id < NbVertices());
	return id == -1;
}

bool ProblemData::IsInitialPosition(int id) const
{
	assert(id >= -1 && id < NbVertices());
	return id >= 0 && id < NbVehicles();
}

bool ProblemData::IsRequest(int id) const
{
	assert(id >= -1 && id < NbVertices());
	return id >= vehicles.size() && id < vehicles.size() + requests.size();
		//|| id >= vehicles.size() + 2*requests.size() + waitingStations.size() && id < vehicles.size() + 2 * requests.size() + waitingStations.size() + projected_requests.size();
}

bool ProblemData::IsDestination(int id) const
{
	assert(id >= -1 && id < NbVertices());
	return (id >= vehicles.size() + requests.size() && id < vehicles.size() + 2 * requests.size());
		//|| (id >= vehicles.size() + 2 * requests.size() + waitingStations.size() + projected_requests.size() && id < vehicles.size() + 2 * requests.size() + waitingStations.size() + 2*projected_requests.size());
}

bool ProblemData::IsWaitingStation(int id) const
{
	assert(id >= -1 && id < NbVertices());
	return id >= vehicles.size() + 2 * requests.size() && id < vehicles.size() + 2 * requests.size() + waitingStations.size();
}

int ProblemData::RequestIdToIndex(int id) const
{
	assert(id >= -1 && id < NbVertices());
	assert(IsRequest(id));

	return id - vehicles.size();

	// if (id >= vehicles.size() && id < vehicles.size() + requests.size())
	// {
	// 	return id - vehicles.size();
	// }
	// else if (id >= vehicles.size() + 2 * requests.size() + waitingStations.size() && id < vehicles.size() + 2 * requests.size() + waitingStations.size() + projected_requests.size())
	// {
	// 	return id - (vehicles.size() + 2 * requests.size() + waitingStations.size());
	// }
	// else return -1;
}

int ProblemData::InitialPositionIdToIndex(int id) const
{
	assert(id >= -1 && id < NbVehicles());
	return id;
}
int ProblemData::InitialPositionIndexToId(int index) const
{
	assert(index >= 0 && index < NbVehicles());
	return index;
}

int ProblemData::IndexToRequestId(int index) const
{
	assert(index >= 0 && index < NbRequests());
	return index + vehicles.size();

	// if (index < requests.size())
	// {
	// 	return index + vehicles.size();
	// }
	// else
	// {
	// 	return index + (vehicles.size() + 2 * requests.size() + waitingStations.size());
	// }
	
}

int ProblemData::DestinationIdToIndex(int id) const
{
	assert(id >= 0 && id < NbVertices());
	assert(IsDestination(id));

	return id - (vehicles.size() + requests.size());
	
	// if (id >= vehicles.size() + requests.size() && id < vehicles.size() + 2 * requests.size())
	// {
	// 	return id - (vehicles.size() + requests.size());
	// }
	// else if ((id >= vehicles.size() + 2 * requests.size() + waitingStations.size() + projected_requests.size() && id < vehicles.size() + 2 * requests.size() + waitingStations.size() + 2 * projected_requests.size()))
	// {
	// 	return id - (vehicles.size() + 2 * requests.size() + waitingStations.size() + projected_requests.size());
	// }
	// else return -1;
}
int ProblemData::IndexToDestinationId(int index) const
{
	assert(index >= 0 && index < NbRequests());

	return index + vehicles.size() + requests.size();

	// if (index < destinations.size())
	// {
	// 	return index + vehicles.size() + requests.size();
	// }
	// else
	// {
	// 	return index + (vehicles.size() + 2 * requests.size() + waitingStations.size() + projected_requests.size());
	// }
}


int ProblemData::WaitingStationIdToIndex(int id) const
{
	assert(IsWaitingStation(id));
	return id - (vehicles.size() + 2 * requests.size());
}
int ProblemData::IndexToWaitingStationId(int index) const
{
	assert(index >= 0 && index < waitingStations.size());
	return index + (vehicles.size() + 2 * requests.size());
}

const Vertex* ProblemData::GetVertex(int id) const
{
	if (IsInitialPosition(id)) return GetInitialPosition(id);
	else if (IsRequest(id)) return GetRequest(id);
	else if (IsDestination(id)) return GetDestination(id);
	else if (IsWaitingStation(id)) return GetWaitingStation(id);
	else
	{
		throw std::invalid_argument("received id value out of range");
	}
}

void ProblemData::PrecomputeClosestWSs()
{
	for(int i = 0; i < initialPositions.size(); i++)
	{
		initialPositions[i].closestWaitingStation = GetClosestWaitingStation(initialPositions[i].id);
	}
	for(int i = 0; i < requests.size(); i++)
	{
		requests[i].closestWaitingStation = GetClosestWaitingStation(requests[i].id);
	}
	for(int i = 0; i < destinations.size(); i++)
	{
		destinations[i].closestWaitingStation = GetClosestWaitingStation(destinations[i].id);
	}
	for(int i = 0; i < waitingStations.size(); i++)
	{
		waitingStations[i].closestWaitingStation = initialPositions[i].id;
	}
}
int ProblemData::GetClosestWaitingStation(int vertexId) const
{
	assert(vertexId >= 0 && vertexId < NbVertices());
	//assert(!IsWaitingStation(vertexId));

	if(IsWaitingStation(vertexId)) return vertexId;

	double minDist = HUGE_VAL;
	int ws_id = -1;

	//vertices are ordered: initial_positions, requests, destination, waiting_stations
	int first = NbVehicles() + 2*NbRequests();
	int last = first + NbWaitingStations();
	for(int i = first; i < last; i++)
	{
		if(ws_id == -1 || minDist > distances[vertexId][i])
		{
			minDist = distances[vertexId][i];
			ws_id = i;
		}
	}

	assert(IsWaitingStation(ws_id));
	return ws_id;
}

int ProblemData::GetClosestDestination(int vertexId) const
{
	assert(vertexId >= 0 && vertexId < NbVertices());
	assert(!IsDestination(vertexId));

	double minDist = HUGE_VAL;
	int dest_id = -1;

	//vertices are ordered: initial_positions, requests, destination, waiting_stations
	int first = NbVehicles() + NbRequests();
	int last = first + NbRequests();
	for(int i = first; i < last; i++)
	{
		if(dest_id == -1 || minDist > distances[vertexId][i])
		{
			minDist = distances[vertexId][i];
			dest_id = i;
		}
	}

	assert(IsDestination(dest_id));
	return dest_id;
}

// void ProblemData::UpdateDistanceMatrix()
// {
// 	std::cout << "UpdateDistanceMatrix" << std::endl;

// 	//update distance matrix:
// 	int nbVehicles = NbVehicles();
// 	int nbRequests = NbRequests();
// 	int nbWaitingStations = NbWaitingStations();
// 	vector<const Vertex*> vertices;

// 	vertices.resize(nbVehicles + nbRequests * 2 + nbWaitingStations);
// 	for (int i = 0; i < initialPositions.size(); i++) vertices[i] = &initialPositions[i];
// 	for (int i = 0; i < requests.size(); i++) vertices[nbVehicles + i] = &requests[i];
// 	for (int i = 0; i < destinations.size(); i++) vertices[nbVehicles + nbRequests + i] = &destinations[i];
// 	for (int i = 0; i < waitingStations.size(); i++) vertices[nbVehicles + 2 * nbRequests + i] = &waitingStations[i];
// 	//for (int i = 0; i < projected_requests.size(); i++) vertices[nbVehicles + 2 * nbRequests + nbWaitingStations + i] = &projected_requests[i];
// 	//for (int i = 0; i < projected_destinations.size(); i++) vertices[nbVehicles + 2 * nbRequests + nbWaitingStations + projected_requests.size() + i] = &projected_destinations[i];

// 	std::vector<double> longitudes;
//     std::vector<double> latitudes;

//     longitudes.reserve(vertices.size());
//     latitudes.reserve(vertices.size());
//     for( int i = 0; i < vertices.size(); i++)
//     {
//         longitudes.push_back(vertices[i]->position.x);
//         latitudes.push_back(vertices[i]->position.y);
//     }

// 	distances.clear();
// 	for (int i = 0; i < vertices.size(); i++)
// 	{
// 		distances.push_back(vector<double>(vertices.size()));
// 		for (int j = 0; j < vertices.size(); j++)
// 		{
// 			if (i == j) distances[i][j] = 0.0;
// 			else distances[i][j] = dist(vertices[i]->position, vertices[j]->position);
// 		}
// 	}
// }

// void ProblemData::UpdateDistanceMatrix(std::string osmPath)
// {
// 	//update distance matrix:
// 	int nbVehicles = NbVehicles();
// 	int nbRequests = NbRequests();
// 	int nbWaitingStations = NbWaitingStations();
// 	vector<const Vertex*> vertices;

// 	vertices.resize(nbVehicles + nbRequests * 2 + nbWaitingStations);
// 	for (int i = 0; i < initialPositions.size(); i++) vertices[i] = &initialPositions[i];
// 	for (int i = 0; i < requests.size(); i++) vertices[nbVehicles + i] = &requests[i];
// 	for (int i = 0; i < destinations.size(); i++) vertices[nbVehicles + nbRequests + i] = &destinations[i];
// 	for (int i = 0; i < waitingStations.size(); i++) vertices[nbVehicles + 2 * nbRequests + i] = &waitingStations[i];
// 	//for (int i = 0; i < projected_requests.size(); i++) vertices[nbVehicles + 2 * nbRequests + nbWaitingStations + i] = &projected_requests[i];
// 	//for (int i = 0; i < projected_destinations.size(); i++) vertices[nbVehicles + 2 * nbRequests + nbWaitingStations + projected_requests.size() + i] = &projected_destinations[i];

// 	std::vector<double> longitudes; //.reserve(vertices.size());
//     std::vector<double> latitudes; //.reserve(vertices.size());
//     for( int i = 0; i < vertices.size(); i++)
//     {
//         longitudes.push_back(vertices[i]->position.x);
//         latitudes.push_back(vertices[i]->position.y);
//     }

// 	distances.clear();
// 	OSRMHelper osrmHelper(osmPath);
// 	distances = osrmHelper.TableRequest(vertices);
// }


bool ProblemData::IsCompatible(const Request* request, int vehicle_index)
{
	assert(vehicle_index >= 0 && vehicle_index < vehicles.size());
	return vehicles[vehicle_index].type <= request->type;
}


double ProblemData::weighted_lateness(int req_id, double time)
{
	const Request* req = GetRequest(req_id);
	return weighted_lateness(req, time); 
}

double ProblemData::weighted_lateness(const Request* request, double time)
{
	assert(time + RC_EPS > request->arrival_time);
	if(useTargetWaitTimeObjective)
	{
		double waitTime = time - request->arrival_time;
		auto itr = lower_bound(target_times_per_weight.begin(), target_times_per_weight.end(), std::make_pair(request->weight, 0.0), 
			[](const std::pair<const double, double> a, std::pair<const double, double> b){return a.first < b.first;}
		);
		if(itr == target_times_per_weight.end()) throw std::runtime_error("priority out of map");
		if(waitTime > (*itr).second) //if wait time > target wait time for this weight
		{
			return request->weight;
		}
		else return 0.0;
	}
	return (time - request->arrival_time) * request->weight;
}

//maybe this function cant work with more flexible routing conditions????
double ProblemData::OmmitedDistance(int i, int j)
{
#ifdef _DEBUG
	if (i < 0 || i > NbVertices() || j < 0 /*|| j > nbVertices*/)
	{
		throw std::invalid_argument("received id value out of range");
	}
	if (IsDestination(i))
	{
		throw std::invalid_argument("destination vertices should be ommited!");
	}
#endif
	if (IsRequest(i) && j > NbVertices())
	{
		//moving from request to sink: that is servicing the request + 0.0
		const Request* req = GetRequest(i);
		int destination = req->destination;
		return req->service_time + distances[i][destination];
	}
	else if (IsRequest(i))
	{
		//in the ommited representation, leaving a request vertex means servicing it and then moving on
		//thus: service_time + move time from i to destination + move time from destination to j
		const Request* req = GetRequest(i);
		int destination = req->destination;
		return req->service_time + distances[i][destination] + distances[destination][j];
	}
	else if (j > NbVertices())
	{
		return 0.0;
	}
	else return distances[i][j];
}

double ProblemData::Distance(int i, int j)
{
	assert(i >= 0 && j >= 0 && i < NbVertices() && j < NbVertices());
	return distances[i][j];
}

double ProblemData::calculateDistance(const Position& pos1, const Position& pos2, DistanceType distanceType)
{
	if(distanceType == DistanceType::euclidian)
	{
		return ProblemData::euclidianDistance(pos1, pos2);
	}
	else if(distanceType == DistanceType::geodesic)
	{
		return ProblemData::geodesicDistance(pos1, pos2);
	}
	//to-do: osrm
	else throw std::runtime_error("unexpected enumeration value"); 
}
double ProblemData::geodesicDistance(const Position& pos11, const Position& pos22)
{
	
	//invert it to keep it the same as victor hugo
	Position pos1, pos2;
	pos1.x = pos11.y; pos1.y = pos11.x;
	pos2.x = pos22.y; pos2.y = pos22.x;
	
	
	if(pos1.x == pos2.x && pos1.y == pos2.y) return 0.0;

	const double R = 6371; //km
	const double radian = M_PI/180;

	double p11 = R*cos(radian*pos1.x)*cos(radian*pos1.y);
	double p12 = R*cos(radian*pos1.x)*sin(radian*pos1.y);
	double p13 = R*sin(radian*pos1.x);

	double p21 = R*cos(radian*pos2.x)*cos(radian*pos2.y);
	double p22 = R*cos(radian*pos2.x)*sin(radian*pos2.y);
	double p23 = R*sin(radian*pos2.x);

	double d = sqrt(pow(p11 - p21, 2) + pow(p12 - p22, 2) + pow(p13 - p23, 2)); //km

	double ret = 2*R*asin(d/(2*R)); // in km

	ret = ret * 1000; //in meters
	ret = ret / ProblemData::VehicleSpeed(); //for ease of usage, return computed TIME not distance (in seconds)
	assert(ret > -RC_EPS);
	return ret;
}

double ProblemData::euclidianDistance(const Position& pos1, const Position& pos2)
{
	if (pos1.x == pos2.x && pos1.y == pos2.y) return 0;
	else return std::sqrt((pos1.x - pos2.x) * (pos1.x - pos2.x) + (pos1.y - pos2.y) * (pos1.y - pos2.y));
}

void ProblemData::SetVehiclePositions(vector<Position> vehiclePositions)
{
	assert(vehiclePositions.size() == vehicles.size());

	for (int i = 0; i < vehicles.size(); i++)
	{
		initialPositions[i].position = vehiclePositions[i];
	}
}


static double norm(Position& a, Position& b){
	
    double x1,y1,x2,y2;
    x1 = a.x; y1 = a.y;
    x2 = b.x; y2 = b.y;
    return sqrt(pow(x2-x1,2) + pow(y2-y1,2));

}

Position ProblemData::GetIntermediatePosition(Position source, Position destination, double t0, double t)
{
    const double pi = M_PI;
    const double R = 6371; // kms !!!
    const double radian = pi/180;
    const double v = 60; // km/h

    double p11 = R*cos(radian*source.y)*cos(radian*source.x);
    double p12 = R*cos(radian*source.y)*sin(radian*source.x);
    double p13 = R*sin(radian*source.y);

    double p21 = R*cos(radian*destination.y)*cos(radian*destination.x);
    double p22 = R*cos(radian*destination.y)*sin(radian*destination.x);
    double p23 = R*sin(radian*destination.y);

    double d = sqrt(pow(p11 - p21, 2) + pow(p12 - p22, 2) + pow(p13 - p23, 2));
    double alpha = 2*asin(d/(2*R));
    double dearth = R*alpha;

    double ttravel = dearth/v; //travel time in h
    ttravel = ttravel * 3600; // in seconds

    Position position = source;
    if(t < t0 + ttravel){

        //terrible for numerical errors. Reformulate:
		//double alpha0 = ((t-t0) / 3600) * v/R;
        
		double alpha0 = (t - t0) * ((v/3.6) / (R*1000));

		//double alpha0 = alpha0_meters / 1000;

		double sinA = sin(alpha);

		double beta = sin(alpha-alpha0)/sin(alpha);
        double gamma = cos(alpha-alpha0)-sin(alpha-alpha0)*cos(alpha)/sin(alpha);
        double curr_p1 = beta*p11 + gamma*p21;
        double curr_p2 = beta*p12 + gamma*p22;
        double curr_p3 = beta*p13 + gamma*p23;
        position.y = (asin(curr_p3/R)/M_PI)*180;
        if(curr_p2 > -0.05){
            position.x = (acos(curr_p1/sqrt(pow(R,2)-pow(curr_p3,2)))/M_PI)*180;
        }else{
            position.x = -(acos(curr_p1/sqrt(pow(R,2)-pow(curr_p3,2)))/M_PI)*180;
        }



        return position;
    }else{
        return destination;
    }

}

Position ProblemData::GetIntermediatePosition2(Position p1, Position p2, double timeFromStart) //dist -> dist from p1 in km
{

	double dist = timeFromStart * ProblemData::VehicleSpeed(); // meters 
	
	dist = dist / 1000.0; // kilometers

	const double pi = 3.14159265;
	double constant = pi / 180;
	double angular = dist / 6371;
	double a = sin(0 * angular) / sin(angular);
	double b = sin(1 * angular) / sin(angular);
	double x = a * cos(p1.y* constant) * cos(p1.x* constant) + 
				b * cos(p2.y* constant) * cos(p2.x* constant);
	double y = a * cos(p1.y* constant) * sin(p1.x* constant) + 
				b * cos(p2.y* constant) * sin(p2.x* constant);
	double z = a * sin(p1.y* constant) + b * sin(p2.y* constant);
	double lat3 = atan2(z, sqrt(x * x + y * y));
	double lon3 = atan2(y, x);
	
	Position newPos;
	newPos.x = lon3 / constant;
	newPos.y = lat3 / constant;
	return newPos;
}


