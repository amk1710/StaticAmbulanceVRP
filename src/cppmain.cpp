#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/program_options.hpp>

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <string>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/multi_array.hpp>
#include <cassert>
#include <cmath>
#include <sstream>
#include <climits>
#include <sys/wait.h>


#include "ProblemData.h"
#include "ProblemSolution.h"
#include "Params.h"
#include "SCIPSolver.h"

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using std::cout;
using std::endl;
using std::string;



//supposes the correct directory structure in this path
// instance_index: valid range [0, 5039]

/*
   about timeHorizonUsage:
   timeHorizonUsage == "" || "default" --> use predetermined default value per instance type
   timeHorizonUsage == "infinite" --> consider "infinite" time horizon, all requests must be serviced
   timeHorizonUsage == "value" --> use value of timeHorizon param
*/
bool FindVInstanceByIndex(string dirPath, int instance_index, string osmPath, ProblemData &problemData,  string timeHorizonUsage, double timeHorizon, int setNbVehicles)
{
   fs::path dir (dirPath);
   
   std::vector<fs::path> paths; std::vector<bool> tenColumns; std::vector<std::string> prefixes; std::vector<bool> OSM_able;
   std::vector<fs::path> hosp_paths;
   std::vector<fs::path> ws_paths;
   std::vector<int> n_entries; //number of entries in input file
   std::vector<double> default_THs;

   fs::path cleaning_path = dir / "cleaning.txt";

   paths.push_back(dir / "Queues_Of_Calls/simualtedQueues.txt"); tenColumns.push_back(false);
   prefixes.push_back("queues_"); n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   paths.push_back(dir / "Real_Data_Continuous_Scenarios/scenarios_corrected.txt"); tenColumns.push_back(false);
   prefixes.push_back("realDataContinuous_"); n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   paths.push_back(dir / "Simulated_Data_Continuous_Scenarios/Closest_Hospital/scenarios_corrected_40.txt"); tenColumns.push_back(false);
   prefixes.push_back("simulatedDataClosestHospital_");  n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");
   
   paths.push_back(dir / "Simulated_Data_Continuous_Scenarios/Closest_Hospital/scenarios_corrected_200.txt"); tenColumns.push_back(false);
   prefixes.push_back("simulatedDataClosestHospital_"); n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");
   
   paths.push_back(dir / "Simulated_Data_Continuous_Scenarios/Closest_Hospital/scenarios_corrected_600.txt"); tenColumns.push_back(false);
   prefixes.push_back("simulatedDataClosestHospital_"); n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   paths.push_back(dir / "Simulated_Data_Continuous_Scenarios/Random_Hospital/scenarios_corrected_40.txt"); tenColumns.push_back(false);
   prefixes.push_back("simulatedDataRandomHospital_"); n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");
   
   paths.push_back(dir / "Simulated_Data_Continuous_Scenarios/Random_Hospital/scenarios_corrected_200.txt"); tenColumns.push_back(false);
   prefixes.push_back("simulatedDataRandomHospital_"); n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   paths.push_back(dir / "Simulated_Data_Continuous_Scenarios/Random_Hospital/scenarios_corrected_600.txt"); tenColumns.push_back(false);
   prefixes.push_back("simulatedDataRandomHospital_"); n_entries.push_back(500); OSM_able.push_back(true); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   paths.push_back(dir / "Simulated_Data_Rectangle/simualtedRectanglePoisson.txt"); tenColumns.push_back(true);
   prefixes.push_back("rectanglePoisson_"); n_entries.push_back(500); OSM_able.push_back(false); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   paths.push_back(dir / "Simulated_Data_Rectangle/simualtedRectangleUniform.txt"); tenColumns.push_back(true);
   prefixes.push_back("rectangleUniform_"); n_entries.push_back(500); OSM_able.push_back(false); default_THs.push_back(7200);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");



   // ---------
   //instances requested by Thibaut ocupy indices 5000 - 5129:

   //t2:
   paths.push_back(dir / "Andre/scenarios_n10_t2_r11.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t2_r11"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(2*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases11.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t2_r38.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t2_r38"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(2*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases38.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t2_r76.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t2_r76"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(2*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   //t4:
   paths.push_back(dir / "Andre/scenarios_n10_t4_r11.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t4_r11"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(4*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases11.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t4_r38.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t4_r38"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(4*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases38.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t4_r76.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t4_r76"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(4*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   //t6:
   paths.push_back(dir / "Andre/scenarios_n10_t6_r11.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t6_r11"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(6*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases11.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t6_r38.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t6_r38"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(6*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases38.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t6_r76.txt"); tenColumns.push_back(false); //5080
   prefixes.push_back("RJ_t6_r76"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(6*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   //t8:
   paths.push_back(dir / "Andre/scenarios_n10_t8_r11.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t8_r11"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(8*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases11.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t8_r38.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t8_r38"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(8*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases38.txt");

   paths.push_back(dir / "Andre/scenarios_n10_t8_r76.txt"); tenColumns.push_back(false);
   prefixes.push_back("RJ_t8_r76"); n_entries.push_back(10); OSM_able.push_back(true);
   default_THs.push_back(8*3600);
   hosp_paths.push_back(dir / "hospitals.txt"); ws_paths.push_back( dir / "bases.txt");

   assert(paths.size() == hosp_paths.size() && hosp_paths.size() == ws_paths.size());

   int i_file = 0;
   int i_position = 0;
   int i_total = 0;
   bool found = false;
   while(i_total <= instance_index && i_file < n_entries.size())
   {
      if(i_total + n_entries[i_file] <= instance_index) //next file
      {
         i_total += n_entries[i_file];
         i_file++;
      }
      else 
      {
         i_position = instance_index - i_total;
      
         bool useTimeHorizon = false;
         double THValue = 0.0;
         if(timeHorizonUsage == "" || timeHorizonUsage == "default")
         {
            useTimeHorizon = true;
            THValue = default_THs[i_file];
         }
         else if(timeHorizonUsage == "value")
         {
            useTimeHorizon = true;
            THValue = timeHorizon;
         }
         else if(timeHorizonUsage == "infinite")
         {
            useTimeHorizon = false;
            THValue = 0.0;
         }
         else if(timeHorizonUsage == "compute")
         {
            //set to default, will be overwritten during execution
            useTimeHorizon = true;
            THValue = default_THs[i_file];
         }
         else
         {
            throw std::invalid_argument("Invalid timeHorizonUsage argument.");
         }


         bool ret = ProblemData::readVincentInstance(paths[i_file].string(), hosp_paths[i_file].string(), ws_paths[i_file].string(), cleaning_path.string(), i_position, problemData, tenColumns[i_file], OSM_able[i_file] ? osmPath : "", useTimeHorizon, THValue, setNbVehicles);
         if(ret)
         {
            problemData.name = prefixes[i_file] + problemData.name;
            return true;
         }
         return false;
      }
   }

   throw std::invalid_argument("V instance of id  " + std::to_string(instance_index) + " not found");
   return false;


}

bool ParseCommandLine(int argc, char ** argv, Params &params, ProblemData &problemData)
{
   string type;
   string path;
   string v_index;
   string requests_path;
   string osmPath;
   string output_dir;
   string suffix;
   string descriptiveString;
   string timeHorizonUsage;

    // Declare the supported options.
   po::options_description desc("Allowed options");
   desc.add_options()
      ("help", "produce help message")

      //input
      ("type,t", po::value<std::string>(&type), "type of instance. Options: PDPTW, SDVRPTW, V, V10")
      ("path,p", po::value<std::string>(&path), "path of instance")
      ("requests_path", po::value<std::string>(&requests_path), "path of requests file. Only used with V instances")
      ("v_index", po::value<std::string>(&v_index), "Index of V instance in file. Only used with V instances")
      ("osmPath", po::value<std::string>(&osmPath)->default_value(""), "Optional path to Open Street Map data")
      ("instance_index", po::value<int>(), "select instance by index considering fixed order on the usual input files")
      ("time_horizon_usage", po::value<string>(&timeHorizonUsage)->default_value("default"), "Should a time horizon be used? How? ('default') use predetermined default value per instance type, ('infinite') no time horizon, ('value') use value of time_horizon arg, ('compute') compute minimum required time horizon ")
      ("time_horizon", po::value<double>(), "use a time horizon of this many seconds")
      ("waitingStationPolicy", po::value<int>()->default_value(0), "which station policy to use? (0) mandatory stop at fixed station, (1) optional stop at fixed station, (2) optionalStopInClosestWaitingStation, (3) bestOptionalStop")
      ("allow_rerouting", po::value<int>()->default_value(0), "allow rerouting from ws to req? (0) No, (1) Yes")      
      ("use_target_objective", po::value<int>()->default_value(0), "Use target wait time objective? (0) No, (1) Yes")
      ("set_nb_vehicles", po::value<int>()->default_value(0), "Overwrite number of vehicles in instance to _ ")
      ("branch_on_vehicles", po::value<int>()->default_value(0), "should branching at vehicles be used? (0) No, (1) Yes")
      ("branch_on_edges", po::value<int>()->default_value(0), "should branching on edges be used? (0) No, (1) Yes")
      ("output_dir", po::value<string>(&output_dir)->default_value("./"), "where to output log and solution files.")
      ("output_suffix", po::value<string>(&suffix)->default_value(""), "add this suffix to output files.")
      ("outputDuals", po::value<int>()->default_value(0), "output dual values to file? 0 no, 1 yes")
      
      ("relaxed", po::value<int>()->default_value(0), "0 for integer problem, 1 for relaxation.")

      ("heuristic_run", po::value<int>()->default_value(0), "Should run  heuristic instead of MIP? (0) No, (1) Closest Available, (2) Earliest Arrival")
      
      //meta-parameters
      ("max_time", po::value<double>()->default_value(1.0e+20), "max optimization time in seconds.")
      ("max_pricing_time", po::value<double>()->default_value(1.0e+20), "max time to spend on a single call to the pricing algorithm")
      ("max_memory", po::value<double>()->default_value(10000.0), "max memory (used by SCIP alone) in MBs.")
      ("max_pricing_memory", po::value<double>()->default_value(10000.0), "max memory used in single pricing run in MBs")
      ("pricing_alg", po::value<int>()->default_value(2), "set pricing algorithm. (0) DAG, (1) bellman, (2) SpacedBellman, (3) SpacedBellman2, (4) bellmanWSets, (5) spacedBellmanWSets, (6) PricerTester, (7) Hybrid")
      ("new_routes_per_pricing", po::value<int>()->default_value(10), "How many routes to add per pricing round?")
      ("n_random_initial_routes", po::value<int>()->default_value(0), "Number of random routes to be added to initial solution")
      ("route_gen_seed", po::value<int>()->default_value(0), "Seed for the generation of random initial routes.")
      
      //logging:
      ("descriptiveString", po::value<std::string>(&descriptiveString)->default_value(""), "Optional string, appended to output. Useful for differentiating runs")
   ;

   po::variables_map vm;
   po::store(po::parse_command_line(argc, argv, desc), vm);
   po::notify(vm);

   if (vm.count("help")) {
      cout << desc << "\n";
      return false;
   }

   int setNbVehicles = vm["set_nb_vehicles"].as<int>();

   bool found_instance = false;
   string error = "";
   try{

      bool useTimeHorizon = false;
      double timeHorizon = 0.0;

      if(timeHorizonUsage == "default")
      {
         timeHorizon = 0.0;
         useTimeHorizon = true;
      }
      else if(timeHorizonUsage == "value")
      {
         useTimeHorizon = true;
         if(vm.count("time_horizon"))
         {
            timeHorizon = vm["time_horizon"].as<double>();
         }
         else throw std::invalid_argument("Missing time_horizon argument");
      }
      else if(timeHorizonUsage == "infinite")
      {
         useTimeHorizon = false;
         timeHorizon = 0.0;
      }
      else if(timeHorizonUsage == "compute")
      {
         useTimeHorizon = false;
         timeHorizon = 0.0;
      }
      else throw std::invalid_argument("Invalid timeHorizonUsage arg");

      if(timeHorizon > 0.0 && timeHorizonUsage == "compute")
      {
         std::cout << "incompatible options use_time_horizon and compute_time_horizon" << std::endl;
         throw std::invalid_argument("incompatible options use_time_horizon and compute_time_horizon");
      }
      
      if(vm.count("type") && vm.count("path"))
      {
         if(type == "V")
         {
            found_instance = ProblemData::readVincentInstance(vm["requests_path"].as<string>(), path + "/hospitals.txt", path + "/bases.txt", path + "/cleaning.txt", vm["v_index"].as<int>(), problemData, false, osmPath, useTimeHorizon, timeHorizon);
         }
         else if (type == "v10")
         {
            found_instance = ProblemData::readVincentInstance(vm["requests_path"].as<string>(), path + "/hospitals.txt", path + "/bases.txt", path + "/cleaning.txt", vm["v_index"].as<int>(), problemData, true, osmPath, useTimeHorizon, timeHorizon);
            
         }
         else
         {
            problemData = ProblemData(path, type);
            found_instance = true;
         }
      }
      else if(vm.count("instance_index"))
      {
         //find V instance by "usual index". Useful for lauching several jobs via command line
         std::cout << "osmPath:" << osmPath << std::endl;
         found_instance = FindVInstanceByIndex(path, vm["instance_index"].as<int>(), osmPath, problemData, timeHorizonUsage, timeHorizon, setNbVehicles);
      }
   }
   catch (std::exception& e) {
      std::cout << e.what() << std::endl;
      error = e.what();
   }
   catch (...) {
      error = "unknown error";
   }

   params.pricingAlgorithm = (PricingAlgorithm) vm["pricing_alg"].as<int>();
   params.max_time =  vm["max_time"].as<double>();
   params.maxTimeSinglePricing =  vm["max_pricing_time"].as<double>();
   params.max_memory = vm["max_memory"].as<double>();
   params.maxMemorySinglePricing = vm["max_pricing_memory"].as<double>();

   params.newRoutesPerPricing = vm["new_routes_per_pricing"].as<int>();

   params.nbRandomInitialRoutes = vm["n_random_initial_routes"].as<int>();
   params.route_gen_seed = vm["route_gen_seed"].as<int>();

   params.descriptiveString = vm["descriptiveString"].as<string>();

   problemData.waitingStationPolicy = (WaitingStationPolicy) vm["waitingStationPolicy"].as<int>();
   problemData.useTargetWaitTimeObjective = vm["use_target_objective"].as<int>() == 1;
   
   params.useBranchingOnVehicles = vm["branch_on_vehicles"].as<int>() == 1;
   params.useBranchingOnEdges = vm["branch_on_edges"].as<int>() == 1;

   params.solveRelaxedProblem = vm["relaxed"].as<int>() == 1;
   params.outputDuals = vm["outputDuals"].as<int>() == 1;

   params.heuristic_run = vm["heuristic_run"].as<int>();

   problemData.allowRerouting = vm["allow_rerouting"].as<int>() == 1;
   problemData.computeTimeHorizon = timeHorizonUsage == "compute";
   
   if(!found_instance)
   {
      return false;
   }
   else cout << "run instance " << problemData.name << endl;

   params.outputDirectory = output_dir;
   params.outputSuffix = suffix;

   return true;
}

int ComputingCanadaMain(int argc, char ** argv)
{
   Params params;
   ProblemData problemData;

   params.StartTime();

   bool ret = ParseCommandLine(argc, argv, params, problemData);

   if(ret)
   {
      SCIPSolver solver(&params, &problemData);
      ProblemSolution sol(&problemData);

      if(params.heuristic_run > 0)
      {
         sol.SetToInitialSolution(&problemData, &params, true, params.heuristic_run);
      }
      else
      {
         solver.solve(sol);
      }
      

      fs::path dir (params.outputDirectory);
      fs::path out_file = dir / fs::path(problemData.name + params.outputSuffix + ".sol");
      fs::path out_file2 = dir / fs::path(problemData.name + params.outputSuffix + ".req");
      sol.WriteSolution(out_file.string());
      sol.WriteRequestsOutput(out_file2.string());
      cout << "finished running" << problemData.name << endl;
   }
   else cout << "did not run" << endl;

   return 0;

}

#include <chrono>

int main(
   int                        argc,          /**< number of arguments from the shell */
   char**                     argv           /**< array of shell arguments */
   )
{

   //int _argc = 19;
   //char* _argv[] = {"./SCIPTesting", "--instance_index", "5020", "--path", "/mnt/d/Documents/NaoPSR/Ambulance_Allocation_CG/Scenarios_For_Tests", "--heuristic_run", "0",  "--pricing_alg", "2", "--waitingStationPolicy", "2", "--branch_on_edges", "1", "--branch_on_vehicles", "1", "--relaxed", "0", "--max_time", "1200" };


   //std::cout << "cos:" << cos(0.5) << std::endl;
   //int ret = 0;

   //int ret = ComputingCanadaMain(_argc, _argv);

   int ret = ComputingCanadaMain(argc, argv);
   return ret;

   // Params params;

   // params.pricingAlgorithm = PricingAlgorithm::spacedBellman;
   // params.max_time = 60;
   // params.outputDirectory = "/mnt/d/Documents/NaoPSR/MasterThesis/SCIPTesting/build/";

   // ProblemData instance;
   // bool ret = ProblemData::readVincentInstance("/mnt/d/Documents/NaoPSR/Ambulance_Allocation_CG/Scenarios_For_Tests", "/mnt/d/Documents/NaoPSR/Ambulance_Allocation_CG/Scenarios_For_Tests/Real_Data_Continuous_Scenarios/baseScenario.txt", 100, instance, false, "");
   // if(!ret) return 0;
   // //instance.name = prefixes[i] + instance.name;
   // std::cout << " running instance: " << instance.name << endl;

   // ProblemSolution solution(&instance);
   // SCIPSolver solver(&params, &instance);

   // solver.solve(solution);

   // //solution.ClosestAvailableVehicleSolution(&params);

   // solution.UpdateCost(); //update cost is also a way to validate a solution, as some sanity checks are done in the process

   // //solution.WriteSolution("/mnt/d/Documents/NaoPSR/MasterThesis/outputs/" + instance.name + ".sol");

   // return 0;
   
   //SCIP_RETCODE retcode;

   // Params params;

   // params.outFilePath = "./output.csv";
   // //params.waitingStationPolicy = WaitingStationPolicy::optionalStopInFixedStation;
   // params.waitingStationPolicy = WaitingStationPolicy::mandatoryStopInFixedStation;

   // //ProblemData problemData;
   // ProblemData problemData("/mnt/d/Documents/NaoPSR/MasterThesis/instances/pdptw/lc101.txt");
   // //ProblemData problemData("/mnt/d/Documents/NaoPSR/MasterThesis/instances/sdvrptw/pr11a.txt", "sdvrptw");

   // ProblemSolution solution(&problemData);
   // Solver solver(&params, &problemData);

   // solver.solveExactly(solution);

   // solution.UpdateCost(); //update cost is also a way to validate a solution, as some sanity checks are done in the process

   // solution.WriteSolution("/mnt/d/Documents/NaoPSR/MasterThesis/outputs/" + problemData.name + ".sol");

   // return 0;

   // std::ofstream myfile;
   // myfile.open("./output.csv");

   // myfile   << "instanceName,NbRequests,NbVehicles,total_cost,route_cost,pen_cost,NbUsedRoutes,total_time,NVars,NReoptRuns,NTotalNodes,Gap" << std::endl;
   
   // string dirPath = "/mnt/d/Documents/NaoPSR/MasterThesis/instances/pdptw/";
   // string dirPath2 = "/mnt/d/Documents/NaoPSR/MasterThesis/instances/sdvrptw/";

   // if (is_directory(dirPath)) {
   //              //pdptw
   //              for (auto& entry : boost::make_iterator_range(directory_iterator(dirPath), {})) {
            
   //                  std::cout << " running instance: " << entry.path().string() << endl;
   //                  Params params(entry.path().string(), "pdptw");
   //                  //ProblemSolution solution(&params.currentState);

   //                   SCIP_RETCODE retcode = runSCIP(&params.currentState);
   //                   if( retcode != SCIP_OKAY )
   //                   {
   //                      SCIPprintError(retcode);
   //                      return -1;
   //                   }

   //              }
   //              // sdvrptw
   //              for (auto& entry : boost::make_iterator_range(directory_iterator(dirPath2), {})) {

   //                  std::cout << " running instance: " << entry.path().string() << endl;
   //                  Params params(entry.path().string(), "sdvrptw");
   //                  // ProblemSolution solution(&params.currentState);

   //                   SCIP_RETCODE retcode = runSCIP(&params.currentState);
   //                   if( retcode != SCIP_OKAY )
   //                   {
   //                      SCIPprintError(retcode);
   //                      return -1;
   //                   }
   //              }
            
   //          }



   //std::string osmPath = "";
   //std::string osmPath = "/mnt/d/Documents/NaoPSR/OSRM_data/sudeste/sudeste.osrm";
   // std::string osmPath = "";


   // std::vector<unique_ptr<ProblemData>> instances;

   // //to-do: organize output files, cleanup etc. 

   // Params params;

   // params.outFilePath = "./output.csv";
   // //params.waitingStationPolicy = WaitingStationPolicy::optionalStopInFixedStation;
   // params.waitingStationPolicy = WaitingStationPolicy::mandatoryStopInFixedStation;

   // //clean output and print header
   // std::ofstream myfile;
   // myfile.open(params.outFilePath);
   // //to-do: add finish time, review timeHorizon
   // myfile   << "instanceName,descriptiveString,NbRequests,NbVehicles,solutionCost,routeCost,penaltyCost,NbRoutes,totalTime,SCIP_NbVars,SCIP_NReoptRuns,SCIP_NTotalNodes,SCIP_Gap,pricingAlgorithm" << std::endl;

   // for(int pricing_enum = 0; pricing_enum <= (int) PricingAlgorithm::spacedBellman2; pricing_enum++)
   // {
   //    PricingAlgorithm pricingAlgorithm = (PricingAlgorithm) pricing_enum;
   //    params.pricingAlgorithm = pricingAlgorithm;

   //    for(int i = 0; i < paths.size(); i++)
   //    {
   //       int j = 0;
   //       while(true)
   //       {
   //          ProblemData instance;
   //          bool ret = ProblemData::readVincentInstance(base_path, paths[i], j, instance, tenColumns[i], osmPath);
   //          if(!ret) break;
   //          instance.name = prefixes[i] + instance.name;
   //          std::cout << " running instance: " << instance.name << endl;

   //          ProblemSolution solution(&instance);
   //          Solver solver(&params, &instance);

   //          solver.solveExactly(solution);

   //          //solution.ClosestAvailableVehicleSolution(&params);

   //          solution.UpdateCost(); //update cost is also a way to validate a solution, as some sanity checks are done in the process

   //          solution.WriteSolution("/mnt/d/Documents/NaoPSR/MasterThesis/outputs/" + instance.name + ".sol");

   //          j++;
   //       }
   //    }
   // }
  


   return 0;
}
