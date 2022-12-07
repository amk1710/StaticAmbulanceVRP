
#include <ctime>
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

#include "SCIPSolver.h"

#include "scip/scip.h"
#include "scip/scipshell.h"
#include "scip/scipdefplugins.h"

#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

#include "branching.h"
#include "pricer_SPwCG.h"
//#include "reader_bpa.h
#include "probdata_SPwCG.h"

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "?"
#endif

namespace fs = boost::filesystem;
using std::endl;
using std::cout; 

/** scip execution: creates a SCIP instance with default plugins, loads rules, callbacks etc. 
 */
static
SCIP_RETCODE runSCIP(Params* params, ProblemData* problemData, ProblemSolution &solution)
{
   std::clock_t alg_start = std::clock();
   SCIP* scip = NULL;
   assert(problemData != NULL);

   /*********
    * Setup *
    *********/


   /* initialize SCIP */
   SCIP_CALL( SCIPcreate(&scip) );

   //set log file
   SCIPsetMessagehdlrLogfile(scip, "sciplog.txt");
   
   /* we explicitly enable the use of a debug solution for this main SCIP instance */
   SCIPenableDebugSol(scip);

   /* include binpacking branching and branching data */
   SCIP_CALL( SCIPincludeCustomBranchingRule(scip) );
   
   // i dont use constraint handlers anymore
   //SCIP_CALL( SCIPincludeConshdlrSumVehicle(scip) );
   
   /* include binpacking pricer  */
   SCIP_CALL( SCIPincludePricerSPwCG(scip) );

   /* include default SCIP plugins */
   SCIP_CALL( SCIPincludeDefaultPlugins(scip) );
 
   /* for column generation instances, disable restarts */
   SCIP_CALL( SCIPsetIntParam(scip,"presolving/maxrestarts",0) );

   SCIP_CALL( SCIPsetIntParam(scip,"propagating/rootredcost/freq",-1) );

   /* turn off all separation algorithms */
   SCIP_CALL( SCIPsetSeparating(scip, SCIP_PARAMSETTING_OFF, TRUE) );

   SCIP_CALL( loadProblem(scip, params, problemData) );

   /*******************
    * Problem Solving *
    *******************/

   //set time and memory limits

   //also consider setup and io time.
   double remaining_time = params->max_time - params->GetElapsedTime();
   if(remaining_time < 1)
   {
      SCIPwarningMessage(scip, "time limit of  %f was not sufficient to start optimizing. Program may run for longer than expected. \n", params->max_time);
      remaining_time = params->max_time;
   }

   SCIP_CALL( SCIPsetRealParam(scip, "limits/time", remaining_time) );
   SCIP_CALL( SCIPsetRealParam(scip, "limits/memory", params->max_memory) ); //memory in MBs

   /* solve problem */
   std::cout << "solve problem" << std::endl;
   std::cout << "=============" << std::endl;
   SCIP_CALL( SCIPsolve(scip) );

   std::clock_t alg_end = std::clock();

   std::cout << std::endl << "primal solution:" << std::endl;
   std::cout << "================" << std::endl << std::endl;
   SCIP_CALL( SCIPprintBestSol(scip, NULL, FALSE) );

   /**************
    * Statistics *
   **************/

   std::cout << std::endl << "Statistics" << std::endl;
   std::cout << "==========" << std::endl << std::endl;

   SCIP_CALL( SCIPprintStatistics(scip, NULL) );

   QuerySolution(scip, solution);

   double temp_cost = solution.cost; 

   //some solution stats:
   double avg_requests_per_route = 0;
   int max_req_in_route = 0;
   bool has_cycles = false;
   for(int i = 0; i < solution.routes.size(); i++)
   {
      Route* route = &solution.routes[i];
      has_cycles = has_cycles || route->has_cycles;
      int nreq = 0;
      for(int iV = 0; iV < route->vertices.size(); iV++)
      {
         if(problemData->IsRequest(route->vertices[iV].id)) nreq++;
      }
      avg_requests_per_route += nreq;
      max_req_in_route = std::max(max_req_in_route, nreq);
   }
   avg_requests_per_route = avg_requests_per_route / solution.routes.size();

   double total_time = ((double)(alg_end - alg_start)) / CLOCKS_PER_SEC; //seconds

   //have I timed-out? if that close to max_time, probably yes
   params->timeout = params->timeout || (total_time > params->max_time * 0.95);


   fs::path dir (params->outputDirectory);
   fs::path out_path = dir / fs::path(problemData->name + params->outputSuffix + ".out");
   std::ofstream myfile;
   myfile.open(out_path.string(), std::ios_base::app);

   ExecutionSummary summary = GetExecutionSummary(scip);

   ResponseSummary responseSummary = solution.GetResponseSummary();

   std::string commit_hash = GIT_COMMIT_HASH;

   std::scientific(myfile);
	myfile.precision(std::numeric_limits<double>::max_digits10);

   //myfile.write(problemData->name.c_str(), (problemData->name.size() + 1) * sizeof(char) );

   myfile   << problemData->name << "," << params->descriptiveString << "," << problemData->NbRequests() << "," << problemData->NbVehicles() << "," << problemData->timeHorizon << "," << (int) problemData->waitingStationPolicy << ","
            << solution.cost << "," << solution.route_cost << "," << solution.penalty_cost << "," << solution.routes.size() << "," << avg_requests_per_route << "," << max_req_in_route << "," << has_cycles << ","
            << responseSummary.nServiced << "," << responseSummary.nNotServiced << ","
            << responseSummary.meanResponseTime << "," << responseSummary.maxResponseTime << "," << responseSummary.meanWeightedResponseTime << "," << responseSummary.maxWeightedResponseTime << "," 
            << responseSummary.meanNonServicePenalty << "," << responseSummary.maxNonServicePenalty << ","
            << total_time << "," << summary.total_pricing_time << "," << summary.total_pricing_calls << "," << summary.total_pricing_timeouts << ","
            << summary.totalLabelsPriced << "," 
            << summary.totalLabelsStored << "," << summary.totalLabelsDeleted << "," << summary.sumOfMaxLabelsStoredSimultaneously << "," << summary.sumOfMostLabelsInRequest << "," << summary.sumOfNbConsideredRequests << ","
            << SCIPgetNVars(scip) << "," << SCIPgetNReoptRuns(scip) << "," << SCIPgetNTotalNodes(scip) << ","
            << SCIPgetGap(scip) << ","
            << summary.timesBranchedWithRule[0] << "," << summary.timesBranchedWithRule[1] << "," << summary.timesBranchedWithRule[2] << ","
            << (int) params->pricingAlgorithm << "," << params->timeout << "," << params->solveRelaxedProblem << "," << params->newRoutesPerPricing << "," << problemData->allowRerouting << "," << params->heuristic_run << ","
            << summary.timesRepeatedRouteWasPriced << "," << (summary.repeatedRoutesTotalReducedCost / summary.timesRepeatedRouteWasPriced) << ","
            << commit_hash
   << endl;

   if(params->outputDuals) OutputDuals(scip, params);

   /********************
    * Deinitialization *
    ********************/

   std::cout << "Free SCIP"  << std::endl << std::endl;
   SCIP_CALL( SCIPfree(&scip) );

   BMScheckEmptyMemory();

   return SCIP_OKAY;
}

/*
solve the problem using SCIP as the B&P framework.
*/
void SCIPSolver::solve(ProblemSolution& solution)
{
   SCIP_RETCODE retcode;

   retcode = runSCIP(params, problemData, solution);
   if( retcode != SCIP_OKAY )
   {
      SCIPprintError(retcode);
      throw std::runtime_error("SCIP error");
   }


}