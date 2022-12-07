#pragma once

#include <unordered_set>
#include <utility>

#include "scip/scip.h"
#include "scip/scipshell.h"
#include "scip/scipdefplugins.h"

#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

#include "ProblemData.h"
#include "ProblemSolution.h"
#include "Params.h"

using std::pair;

#ifdef __cplusplus
extern "C" {

/** sets up SCIP execution for a given (my) ProblemData structure */
SCIP_RETCODE loadProblem(
   SCIP*                 scip,               /**< SCIP data structure */
   Params*               params,              /**< globals params */
   ProblemData*          problemData         /**< my problem's data structure */
);

ProblemData* GetProblemData(SCIP_ProbData *probdata);
Params* GetParams(SCIP_ProbData *probdata);
bool IsVarRepeated(SCIP_ProbData *probdata, Route* route, SCIP* scip = NULL);

void QuerySolution(SCIP* scip, ProblemSolution &solution); 

/** returns array of all variables itemed in the way they got generated */
SCIP_VAR** SCIPprobdataGetVars(
   SCIP_PROBDATA*        probdata            /**< problem data */
);
/** returns number of variables */
int SCIPprobdataGetNVars(
   SCIP_PROBDATA*        probdata            /**< problem data */
);

/** returns array of all variables itemed in the way they got generated */
SCIP_CONS** SCIPprobdataGetCons(
   SCIP_PROBDATA*        probdata            /**< problem data */
);
/** returns number of variables */
int SCIPprobdataGetNCons(
   SCIP_PROBDATA*        probdata            /**< problem data */
); 

void AddVehicleBranchingCons(SCIP* scip, SCIP_PROBDATA* probdata, SCIP_CONS* cons, int vehicle_id);
const vector<SCIP_CONS*>* GetVehicleBranchingConstraints(SCIP_PROBDATA*        probdata);
const vector<int>* GetConstrainedVehicles(SCIP_PROBDATA* probdata);

void AddEdgeBranchingCons(SCIP* scip, SCIP_PROBDATA* probdata, SCIP_CONS* cons, pair<int, int> edge);
const vector<SCIP_CONS*>* GetEdgeBranchingConstraints(SCIP_PROBDATA* probdata);
const vector<pair<int,int>>* GetConstrainedEdges(SCIP_PROBDATA* probdata);

void IncrementUsedBranchingRule(SCIP_PROBDATA* probdata, int ruleIndex);
void LogRepeatedRoute(SCIP_PROBDATA* probdata, double reducedCost);


struct ExecutionSummary
{
   double total_pricing_time;
   size_t total_pricing_calls;
   size_t total_pricing_timeouts;
   size_t totalLabelsPriced;
   size_t totalLabelsStored;
   size_t totalLabelsDeleted;
   size_t sumOfMaxLabelsStoredSimultaneously;
   size_t sumOfMostLabelsInRequest;
   size_t sumOfNbConsideredRequests;
   size_t timesBranchedWithRule[3];
   size_t timesRepeatedRouteWasPriced;
   double repeatedRoutesTotalReducedCost;
};

ExecutionSummary GetExecutionSummary(SCIP *scip);

void OutputDuals(SCIP *scip, Params* params);


#endif

#ifdef __cplusplus
}
#endif