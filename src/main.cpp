#include <iostream>
#include <chrono>
#include <exception>
#include <typeinfo>
#include <stdexcept>

#ifndef _DEBUG
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

using namespace boost::filesystem;
#endif



#include "ProblemData.h"
#include "ProblemSolution.h"
#include "Solver.h"




#define RC_EPS 1.0e-6

using std::cout;
using std::endl;

#ifndef _DEBUG
int run_tests()
{
    string dirPath = "D:\\Documents\\NaoPSR\\MasterThesis\\instances\\pdptw\\";
    string dirPath2 = "D:\\Documents\\NaoPSR\\MasterThesis\\instances\\sdvrptw\\";

    std::ofstream myfile;
    myfile.open("./output.csv");

    //print csv header
    myfile << "instanceName,nbRequests,nbVehicles,descriptiveString,optimum,routeCost,penaltyCost,total_time,n_pricings,total_pricing_time,totalLabelsInPricing,n_pricings_FAST,total_pricing_time_FAST,n_cplex_calls,total_cplex_time,routePoolSize,newRoutesPerPricing,nbUsedRoutes,initialDSF,DSFDecrement,dualStabilizationFactor,maxNbRoutes,maxSolverIterations,converges" << endl;

    bool useAlwaysLoopVehicles = true;
    string descriptiveString = "exactSolving-SBP(no-limits-forward-list)";
    double stabilizationFactor = 0.00;


    for (int nRoutes = 20; nRoutes <= 20; nRoutes+= 10)
    {
        for (double stabDecrement = 0.15; stabDecrement <= 0.15; stabDecrement += 0.05)
        {

            cout << "doing " << nRoutes << ", " << stabDecrement << endl;

            if (is_directory(dirPath)) {
                //pdptw
                for (auto& entry : boost::make_iterator_range(directory_iterator(dirPath), {})) {
            
                    std::cout << " running instance: " << entry.path().string() << endl;
                    Params params(entry.path().string(), "pdptw");
                    ProblemSolution solution(&params.currentState);

                    Solver solver(&params, &params.currentState);
                    params.newRoutesPerPricing = nRoutes;
                    params.initialDSF = stabilizationFactor;
                    params.DSFDecrement = stabDecrement;
                    params.descriptiveString = descriptiveString;
                    params.alwaysLoopVehicles = useAlwaysLoopVehicles;

                    try {
                        solver.solve(solution, myfile);
                    }
                    catch (std::exception& e) {
                        std::cerr << e.what() << std::endl;
                    }
                    catch (IloException& e) {
                        cerr << "Concert Exception: " << e << endl;
                    }
                    catch (...) {
                        std::cerr << "???" << std::endl;
                        //std::exception_ptr p = std::current_exception();
                        //std::clog << (p ? p.__cxa_exception_type()->name() : "null") << std::endl;
                    }
                }
                // sdvrptw
                for (auto& entry : boost::make_iterator_range(directory_iterator(dirPath2), {})) {

                    std::cout << " running instance: " << entry.path().string() << endl;
                    Params params(entry.path().string(), "sdvrptw");
                    ProblemSolution solution(&params.currentState);

                    Solver solver(&params, &params.currentState);
                    params.newRoutesPerPricing = nRoutes;
                    params.initialDSF = stabilizationFactor;
                    params.DSFDecrement = stabDecrement;
                    params.descriptiveString = descriptiveString;
                    params.alwaysLoopVehicles = useAlwaysLoopVehicles;

                    try {
                        solver.solve(solution, myfile);
                    }
                    catch (std::exception& e) {
                        std::cerr << e.what() << std::endl;
                    }
                    catch (IloException& e) {
                        cerr << "Concert Exception: " << e << endl;
                    }
                    catch (...) {
                        std::cerr << "???" << std::endl;
                        //std::exception_ptr p = std::current_exception();
                        //std::clog << (p ? p.__cxa_exception_type()->name() : "null") << std::endl;
                    }
                }
            
            }
        }
    }
    return 0;
}
#endif

/*
    * 'solves' the problem, without foreknowledge, in 60s intervals
*/
void simulateOperation(Params* params)
{
    ProblemData& instance = params->currentState;

    double discretization_time = 60; //60 seconds
    int iReq = 0;

    vector<Position> vehiclePositions(instance.NbVehicles());
    for (int i = 0; i < instance.NbVehicles(); i++)
    {
        const InitialPosition *original = instance.GetInitialPositionByIndex(i);
        vehiclePositions[i] = original->position;
    }
    

    set<int> alreadyServiced;
    vector<double> vehicleAvailability(instance.NbVehicles());
    for (int i = 0; i < instance.NbVehicles(); i++)
    {
        vehicleAvailability[i] = instance.getVehicle(i)->timeAvailable;
    }
    
    
    double total_cost = 0;
    //main loop
    for (double time = 0; time <= instance.timeHorizon; time += discretization_time)
    {
        // create a problem instance :
        //		 - considering only requests up to this minute
        //		 - not considering requests that were previously serviced
        //		 - excluding vehicles that are busy
        vector<int> wantedIndices;
        for (int i = 0; i < instance.NbRequests(); i++)
        {
            const Request* req = instance.GetRequestByIndex(i);
            if (req->arrival_time <= time && alreadyServiced.count(req->id) == 0) wantedIndices.push_back(i);
        }

        if (wantedIndices.size() == 0) continue;

        ProblemData partialInstance = instance;
        partialInstance.FilterRequests(wantedIndices);
        partialInstance.SetVehicleAvailability(vehicleAvailability);
        partialInstance.SetVehiclePositions(vehiclePositions);

        //solve it and add its cost
        Solver newSolver(params, &partialInstance);
        ProblemSolution partialSolution(&partialInstance);

        newSolver.solveExactly(partialSolution);
        partialSolution.UpdateCost();
        //consider only route costs:
        for (int i = 0; i < partialSolution.routes.size(); i++)
        {
            total_cost += partialSolution.routes[i].total_lateness;
            for (int iV = 0; iV < partialSolution.routes[i].vertices.size(); iV++)
            {
                alreadyServiced.insert(partialSolution.routes[i].vertices[iV].identifier);
            }
        }
        for (int i = 0; i < partialSolution.routes.size(); i++)
        {
            int iVeh = partialSolution.routes[i].veh_index;
            const Vehicle* vehicle = partialInstance.getVehicle(iVeh);
            vehicleAvailability[iVeh] = partialSolution.routes[i].end_time + partialInstance.OmmitedDistance(partialSolution.routes[i].vertices.back().id, vehicle->preferredWaitingStation);
            vehiclePositions[i] = partialInstance.GetVertex(vehicle->preferredWaitingStation)->position;
        }

    }

    //add non-service cost:
    for (int i = 0; i < instance.NbRequests(); i++)
    {
        const Request* req = instance.GetRequestByIndex(i);
        if (alreadyServiced.count(req->identifier) == 0) total_cost += req->non_service_penalty;
    }


    std::cout << total_cost << std::endl;
    //while (iReq < instance.NbRequests())
    //{
    //	/*
    //		
    //	*/
    //	ProblemData copy = instance;



    //}
}

int main(int argc, char** argv) {

    //return run_tests();
    
    if (argc < 2) {
        cout << "Usage: main.exe instance_path" << endl;
        exit(1);
    }
    string instancePath = std::string(argv[1]); // "D:\\NAOPSR\\MasterThesis\\PDPTW\\pdptw1000\\LC1_10_1.txt";
    string instanceType = "sdvrptw";
    if (argc >= 3) instanceType = std::string(argv[2]);

    Params params(instancePath, instanceType);
    //Params params("", "");

    ProblemSolution solution(&params.currentState);

    int n_scenarios = 100;
    vector<ProblemData> scenarios;
    params.stochasticInfo.GenerateScenarios(params.currentState, n_scenarios, scenarios);



    Solver solver(&params, &params.currentState);
 
    /*
    if (argc >= 4) params.newRoutesPerPricing = stoi(std::string(argv[3]));
    else params.newRoutesPerPricing = 10;
    if (argc >= 5) params.initialDSF = stod(std::string(argv[4]));
    else params.initialDSF = 0.0;
    */

    params.initialDSF = 0.00;
    params.newRoutesPerPricing = 20;
    params.alwaysLoopVehicles = true;
    params.DSFDecrement = 0.15;

    params.descriptiveString = "test";

    try {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        solver.solve(solution);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        //std::cout << "Time difference (sec) = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000000.0 << std::endl;

        //simulateOperation(&params);

        solution.UpdateCost();
        //solution.PrintSolution();
    }catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    catch (IloException& e) {
        cerr << "Concert Exception: " << e << endl;
    }
    catch (...) {
        std::cerr << "???" << std::endl;
        //std::exception_ptr p = std::current_exception();
        //std::clog << (p ? p.__cxa_exception_type()->name() : "null") << std::endl;
    }

    return 0;
}

