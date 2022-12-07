
#include "pricer_SPwCG.h"

#include <assert.h>
#include <string.h>
#include <ctime>

#include<unordered_set>

#include "scip/cons_knapsack.h"
#include "scip/cons_logicor.h"
#include "scip/cons_setppc.h"
#include "scip/cons_varbound.h"
#include "scip/scipdefplugins.h"

#include "vardata_SPwCG.h"
#include "probdata_SPwCG.h"


#include "BasePricing.h"
//#include "BellmanPricing.h"
#include "SpacedBellmanPricing.h"
//#include "SpacedBellmanPricing2.h"
//#include "DAGPricing.h"
//#include "HybridPricing.h"
//#include "PricerTester.h"



/**@name Pricer properties
 *
 * @{
 */

#define PRICER_NAME            "SPwCG"
#define PRICER_DESC            "pricer for pricing new routes"
#define PRICER_PRIORITY        0
#define PRICER_DELAY           TRUE     /* only call pricer if all problem variables have non-negative reduced costs */


static void checkForVarRedCosts(SCIP* scip);

/**@} */

/**name Callback methods
 *
 * @{
 */

static int ncons(ProblemData* problemData) 
{
   return problemData->NbVehicles() + problemData->NbRequests();
}

/** destructor of variable pricer to free user data (called when SCIP is exiting) */
static
SCIP_DECL_PRICERFREE(pricerFreeSPwCG)
{
   SCIP_PRICERDATA* pricerdata;

   assert(scip != NULL);
   assert(pricer != NULL);

   
   pricerdata = SCIPpricerGetData(pricer); 

   if( pricerdata != NULL)
   {
      /* free memory */
      SCIPfreeBlockMemoryArrayNull(scip, &pricerdata->conss, ncons(pricerdata->problemData));

      //must be called to prevent memory leak. SCIPfreeBlockMemory doesn't trigger it automatically!
      delete pricerdata->pricingAlgo;

      SCIPfreeBlockMemory(scip, &pricerdata);
   }

   return SCIP_OKAY;
}


/** initialization method of variable pricer (called after problem was transformed) */
static
SCIP_DECL_PRICERINIT(pricerInitSPwCG)
{  /*lint --e{715}*/
   SCIP_PRICERDATA* pricerdata;
   SCIP_CONS* cons;
   int c;

   SCIPdebugMsgPrint(scip, "pricerInitSPwCG\n");

   assert(scip != NULL);
   assert(pricer != NULL);


   pricerdata = SCIPpricerGetData(pricer);
   assert(pricerdata != NULL);
   assert(pricerdata->problemData != NULL);
   

   /* get transformed constraints */
   for( c = 0; c < ncons(pricerdata->problemData); ++c )
   {
      cons = pricerdata->conss[c];

      /* release original constraint */
      SCIP_CALL( SCIPreleaseCons(scip, &pricerdata->conss[c]) );
      
      /* get transformed constraint */
      SCIP_CALL( SCIPgetTransformedCons(scip, cons, &pricerdata->conss[c]) );
      
      /* capture transformed constraint */
      SCIP_CALL( SCIPcaptureCons(scip, pricerdata->conss[c]) );
      
   }   

   return SCIP_OKAY;
}


/** solving process deinitialization method of variable pricer (called before branch and bound process data is freed) */
static
SCIP_DECL_PRICEREXITSOL(pricerExitsolSPwCG)
{
   SCIP_PRICERDATA* pricerdata;
   int c;

   assert(scip != NULL);
   assert(pricer != NULL);

   pricerdata = SCIPpricerGetData(pricer);
   assert(pricerdata != NULL);

   /* get release constraints */
   for( c = 0; c < ncons(pricerdata->problemData); ++c )
   {
      /* release constraint */
      SCIP_CALL( SCIPreleaseCons(scip, &(pricerdata->conss[c])) );
   }

   return SCIP_OKAY;
}

using std::vector;

//true if are equal
static bool compareArrays(int* arr1, int size1, int* arr2, int size2)
{
   if(size1 != size2) return false;
   for(int i = 0; i < size1; i++)
   {
      if(arr1[i] != arr2[i]) return false;
   }
   return true;
}

static bool routeContainsRequest(ProblemData *problemData, Route* route, int reqId)
{
   for(int i = 0; i < route->vertices.size(); i++)
   {
      if(problemData->IsRequest(route->vertices[i].id))
      {
         if(route->vertices[i].id == reqId) return true;
      }
   }
   return false;
}

static bool DoesRouteViolateBranching(SCIP* scip, ProblemData* problemData, Route* route)
{
   SCIP_PROBDATA *probdata = SCIPgetProbData(scip);
   int n_vars = SCIPprobdataGetNVars(probdata);
   SCIP_VAR** vars = SCIPprobdataGetVars(probdata);

   for(int i = 0; i < route->vertices.size(); i++)
   {
      if(!problemData->IsRequest(route->vertices[i].id)) continue;

      int req_index = problemData->RequestIdToIndex(route->vertices[i].id);
      
      SCIP_VAR* var = vars[req_index];
      double lb = SCIPvarGetLbLocal(var);
      double ub = SCIPvarGetUbLocal(var);
      if(lb == ub && lb == 1.0) return true;
   }

   return false;
}

static void buildConsideredRequestsVector(SCIP* scip, ProblemData* problemData, std::vector<int> &outVec)
{
   //collect branching constraints on y variables:
   SCIP_PROBDATA *probdata = SCIPgetProbData(scip);
   int n_vars = SCIPprobdataGetNVars(probdata);
   SCIP_VAR** vars = SCIPprobdataGetVars(probdata);

   outVec.clear();

   for(int i = 0; i <  problemData->NbRequests(); i++)
   {
      const Request* req = problemData->GetRequestByIndex(i);
      SCIP_VAR* var = vars[i];
      double lb = SCIPvarGetLbLocal(var);
      double ub = SCIPvarGetUbLocal(var);
      
      // if LB != UB, y var is not fixed, generate routes as per usual
      if(lb != ub){
         outVec.push_back(req->id);
      }
      // if LB == UB == 1.0, request must not be serviced in this branch. Thus, we shouldn't generate new routes doing it
      else if(lb == ub && lb == 1.0) {

      }
      // if LB == UB == 0.0, request should be serviced by some route. Price as usual
      else if(lb == ub && ub == 0.0) {
         outVec.push_back(req->id);
      }
   }

   return;
}

static void buildForbiddenEdges(SCIP* scip, ProblemData* problemData, std::set<pair<int, int>> &outSet)
{
   outSet.clear();

   SCIP_PROBDATA *probdata = SCIPgetProbData(scip);

   const std::vector<SCIP_CONS*> *edgeBranchingConstraints = GetEdgeBranchingConstraints(probdata);
   assert(edgeBranchingConstraints != NULL);
   const std::vector<pair<int, int>> *constrainedEdges = GetConstrainedEdges(probdata);
   assert(constrainedEdges != NULL);
   assert(edgeBranchingConstraints->size() == constrainedEdges->size());
   for( int c = 0; c < (*edgeBranchingConstraints).size(); ++c )
   {
      SCIP_CONS* cons = (*edgeBranchingConstraints)[c];
      assert(cons != NULL);

      assert(!SCIPconsIsDeleted(cons));

      /* ignore constraints which are not active since these are not laying on the current active path of the search
      * tree
      
      */
      if( !SCIPconsIsActive(cons) )
         continue;

      //std::string t_name = SCIPconsGetName(cons);
      SCIP_CONSDATA* consdata = SCIPconsGetData (cons);
      assert(consdata != NULL);


      

      pair<int, int> edge = (*constrainedEdges)[c];
      double rhs = SCIPgetRhsLinear(scip,cons);
      if(rhs == 0.0) outSet.insert(edge);
   }
}

// // PROBLEM: am I sharing wrong info between different nodes?
// static bool checkIfRouteIsRepeated(SCIP* scip, SCIP_PROBDATA* probdata, Route* route)
// {
//    assert(route != NULL);

//    std::unordered_set<Route, Route::HashFunction>* hash_routes = GetHashedRoutes(probdata);
//    assert(hash_routes != NULL);

//    return hash_routes->find(*route) != hash_routes->end();

// }

/* 
   creates and add a new route variable to SCIP. 

   Can fail if exact route already has a variable, or if route violates branching rules. In these cases, returns false

   initial_var is used to ommit repeated vars checks

   caller must free var after using, if ret = true!
*/
bool createRouteVariable(SCIP* scip, Params* params, SCIP_CONS **all_cons, ProblemData *problemData, Route* route, SCIP_VAR** retVar, bool initial_var, double reducedCost)
{
   int iVeh = route->veh_index;
   assert(params != NULL);
   char name[SCIP_MAXSTRLEN];
   (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "x_%d", iVeh);
   int* consids;
   int* conscoeffs;
   int nconss = 0;
   SCIP_CALL( SCIPallocBufferArray(scip, &consids, ncons(problemData)) );
   SCIP_CALL( SCIPallocBufferArray(scip, &conscoeffs, ncons(problemData)) );
   

   //associate to the one vehicle constraint:
   consids[nconss] = iVeh;
   conscoeffs[nconss] = 1;
   nconss++;

   //get coefficients for each request by iterating route->
   // coeff == number of times request apears in route
   bool found_one = false;
   std::vector<int>  request_count(problemData->NbRequests());
   for(int i = 0; i < route->vertices.size(); i++)
   {
      const Vertex* vertex = &route->vertices[i];
      if(problemData->IsRequest(vertex->id))
      {
         int req_index = problemData->RequestIdToIndex(vertex->id);
         request_count[req_index]++;
         found_one = true;
      }
   }
   assert(found_one);

   for(int req_index = 0; req_index < problemData->NbRequests(); req_index++)
   {
      assert(request_count[req_index] <= 1 || route->has_cycles);
      if(request_count[req_index] > 0)
      {
         consids[nconss] = problemData->NbVehicles() + req_index;
         conscoeffs[nconss] = request_count[req_index];
         nconss++;
      }
   }

   //check if route has already been added. If it has, tries to update its obj value for the best. Else, fail

   bool repeated = false;
   if(initial_var) 
   {
      repeated = false;
   }
   else
   {
      repeated = IsVarRepeated(SCIPgetProbData(scip), route, scip);
      //repeated = false;
   }


   bool added = false;


   if(repeated)
   {
      assert(!initial_var);
      LogRepeatedRoute(SCIPgetProbData(scip), reducedCost);
   }
   else
   {
      SCIP_VAR* var;
      SCIP_VARDATA* vardata;

      SCIP_CALL( SCIPvardataCreateBinpacking(scip, &vardata, consids, conscoeffs, nconss, route) );

      // create variable for a new column with objective function coefficient equal to the route's cost */
      // bool, bool -> initial, removable
      SCIP_CALL( SCIPcreateVarBinpacking(scip, &var, NULL, route->total_lateness, initial_var, true, params->solveRelaxedProblem, vardata) );

      /* check which variables are fixed -> which orders belong to this packing */
      for( int v = 0; v < nconss; ++v )
      {

         SCIP_CONS* cons = all_cons[consids[v]];
         double coeff = (double) conscoeffs[v];
         bool b = SCIPconsIsEnabled(cons);
         const char *str = SCIPconsGetName(cons);
         assert(initial_var || b); // ???

         if(v > 0)
         {
            int count = route->GetRequestCount(problemData, problemData->GetRequest(consids[v])->id);
            if( count != coeff)
            {
               std::cout << count << " != " << coeff << "????" << std::endl;
            }
         }

         SCIP_CALL( SCIPaddCoefLinear(scip, cons, var, coeff) );
      }

      if(!initial_var)
      {

         SCIP_PROBDATA* probdata = SCIPgetProbData(scip);
         assert(probdata != NULL);
         const std::vector<SCIP_CONS*> *vehicleBranchingConstraints = GetVehicleBranchingConstraints(probdata);
         assert(vehicleBranchingConstraints != NULL);
         const std::vector<int> *constrainedVehicles = GetConstrainedVehicles(probdata);
         assert(vehicleBranchingConstraints != NULL);
         for( int c = 0; c < (*vehicleBranchingConstraints).size(); ++c )
         {
            assert(!initial_var);
            SCIP_CONS* cons = (*vehicleBranchingConstraints)[c];
            assert(cons != NULL);

            if((*constrainedVehicles)[c] == route->veh_index)
            {
               SCIP_CALL( SCIPaddCoefLinear(scip, cons, var, 1.0) );
            }
         }

         const std::vector<SCIP_CONS*> *edgeBranchingConstraints = GetEdgeBranchingConstraints(probdata);
         assert(edgeBranchingConstraints != NULL);
         const std::vector<pair<int, int>> *constrainedEdges = GetConstrainedEdges(probdata);
         assert(constrainedEdges != NULL);
         assert(edgeBranchingConstraints->size() == constrainedEdges->size());
         for( int c = 0; c < (*edgeBranchingConstraints).size(); ++c )
         {
            assert(!initial_var);
            SCIP_CONS* cons = (*edgeBranchingConstraints)[c];
            assert(cons != NULL);

            assert(!SCIPconsIsDeleted(cons));

            //std::string t_name = SCIPconsGetName(cons);
            SCIP_CONSDATA* consdata = SCIPconsGetData (cons);
            assert(consdata != NULL);

            pair<int, int> edge = (*constrainedEdges)[c];
            std::map<pair<int, int>, int> edgeUsage = route->GetEdgeUsage(problemData);
            if(edgeUsage.find(edge) != edgeUsage.end())
            {
               SCIP_CALL( SCIPaddCoefLinear(scip, cons, var, edgeUsage[edge]) );
            }
         }


      }
      
      *retVar = var;
      added = true;
   }
   

   SCIPfreeBufferArray(scip, &consids);
   SCIPfreeBufferArray(scip, &conscoeffs);
   return added;
}

static void checkForVarRedCosts(SCIP* scip)
{
    SCIP_PROBDATA* probdata = SCIPgetProbData(scip);
    assert(probdata != NULL);

    ProblemData *problemData = GetProblemData(probdata);
    assert(problemData != NULL);

    //iterate vars and add them to constraint
    int n_vars = SCIPprobdataGetNVars(probdata);
    SCIP_VAR** vars = SCIPprobdataGetVars(probdata);

    std::cout << "checkVars" << std::endl;
    std::cout << "nvars?: " << n_vars << std::endl;
    for( int i = 0; i < n_vars; ++i ) //first problemData->NbRequests() vars are y vars
    {
        SCIP_VAR *t_var = vars[i];

        double redCost = SCIPgetVarRedcost(scip, t_var);

        if(redCost < -0.1) std::cout << i << " , " << redCost << std::endl;

        if(SCIPvarIsInLP(t_var))
        {
            SCIP_COL *col = SCIPvarGetCol(t_var);
            double value = SCIPcolGetPrimsol(col);
            if(value > 0.05 && (redCost > 0.05 || redCost < -0.05) )
                  std::cout << i << " , " << value << " , " << redCost << std::endl;
        }
        
    }
}

static
SCIP_RETCODE DoPricing(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_PRICER*          pricer,             /**< pricer */
   SCIP_Bool             farkas,               /**< TRUE: Farkas pricing; FALSE: Redcost pricing */
   SCIP_RESULT* result
)
{ 
   SCIP_PRICERDATA* pricerdata;

   //(*result) = SCIP_SUCCESS;
   //return SCIP_OKAY;

   /* get the pricer data */
   pricerdata = SCIPpricerGetData(pricer);
   assert(pricerdata != NULL);
   assert(pricerdata->problemData != NULL);
   ProblemData *problemData = pricerdata->problemData;
   Params *params = GetParams(SCIPgetProbData(scip));
   assert(params != NULL);

   SCIP_PROBDATA *probdata = SCIPgetProbData(scip);
   int n_vars = SCIPprobdataGetNVars(probdata);
   SCIP_VAR** vars = SCIPprobdataGetVars(probdata);
   
   vector<double> alpha_duals(problemData->NbVehicles());
   vector<double> beta_duals(problemData->NbRequests());

   // do yvars have duals?
   int nVars = problemData->NbRequests();
   //SCIP_VAR** vars = SCIPprobdataGetVars(probdata);
   
   for( int v = 0; v < nVars; ++v )
   {
      assert(vars[v] != NULL);

      SCIP_VAR* var = vars[v];

      if(!SCIPvarIsActive(var)) continue;

      double lb = SCIPvarGetLbLocal(var);
      double ub = SCIPvarGetUbLocal(var);

      double dual = SCIPgetVarRedcost(scip, var);
      
      // if(lb == 1.0)
      //    std::cout << "var[" << SCIPvarGetName(var) << "] fixed to 1. RC: " << dual << std::endl;

      // if(ub == 0.0)
      //    std::cout << "var[" << SCIPvarGetName(var) << "] fixed to 0. RC: " << dual << std::endl;
      
   }

   // I should query scip`s branching decisions and fixed vars for this node and somehow pass them onto the pricing algorithm!
   
   // /* add constraint of the branching decisions */
   // SCIP_CALL( addBranchingDecisionConss(scip, subscip, vars, pricerdata->conshdlr) );

   // /* avoid to generate columns which are fixed to zero */
   // SCIP_CALL( addFixedVarsConss(scip, subscip, vars, conss, nitems) );

   bool addVar = false;
   for( int i = 0; i < problemData->NbVehicles(); ++i )
   {
      SCIP_CONS* cons = pricerdata->conss[i];

      assert(SCIPgetRhsLinear(scip, cons) == 1.0);

      // ???
      /* constraints which are (locally) disabled/redundant are not of
       * interest since the corresponding job is assigned to a packing
       */
      
      if( !SCIPconsIsActive(cons) )
      {
         alpha_duals[i] = 0.0;
         continue;
      }

      // minus?
      if(farkas) alpha_duals[i] = SCIPgetDualfarkasLinear(scip, cons);
      else alpha_duals[i] = SCIPgetDualsolLinear(scip, cons);

      //std::cout << "alpha[" << i << "] = " <<  alpha_duals[i] << std::endl;
      assert(!SCIPconsIsActive(cons) ? alpha_duals[i] == 0.0 : true);
   }

   //adjust alpha dual considering the branching constraints
   /* collect all branching decision constraints */
   const std::vector<SCIP_CONS*> *vehicleBranchingConstraints = GetVehicleBranchingConstraints(probdata);
   const std::vector<int> *constrainedVehicles = GetConstrainedVehicles(probdata);

   for( int c = 0; c < (*vehicleBranchingConstraints).size(); ++c )
   {

      assert(params->useBranchingOnVehicles);

      SCIP_CONS* cons = (*vehicleBranchingConstraints)[c];

      /* ignore constraints which are not active since these are not laying on the current active path of the search
       * tree
       */
      if( !SCIPconsIsActive(cons) )
         continue;

      int veh_id = (*constrainedVehicles)[c];

      double dual = farkas ? SCIPgetDualfarkasLinear(scip, cons) : SCIPgetDualsolLinear(scip, cons);
      alpha_duals[veh_id] = alpha_duals[veh_id] + dual;

      //std::cout << "veh " << veh_id << " dual: " << dual << std::endl;
   }

   //request constraints
   for( int i = 0; i < problemData->NbRequests(); ++i )
   {
      int con_index = i + problemData->NbVehicles();
      SCIP_CONS* cons = pricerdata->conss[con_index];

      assert(SCIPgetLhsLinear(scip, cons) == 1.0);

      if( !SCIPconsIsActive(cons) )
      {
         beta_duals[i] = 0.0;
         continue;
      }
      
      // ???
      /* constraints which are (locally) disabled/redundant are not of
       * interest since the corresponding job is assigned to a packing
       */
      //if( !SCIPconsIsEnabled(cons) )
      //   continue;

      // ???
      // if( SCIPgetNFixedonesSetppc(scip, cons) == 1 )
      // {
      //    /* disable constraint locally */
      //    SCIP_CALL( SCIPdelConsLocal(scip, cons) );
      //    continue;
      // }

      // per the complementary slackness rule, in an linearly optimal solution:
      /*
         - the constraint is saturated, or
         - the dual is 0.0
         - or both
      */
     
      double constraint_value = 0.0;
      SCIP_VAR** vars = SCIPgetVarsLinear(scip, cons);
      int nvars = SCIPgetNVarsLinear(scip, cons);

      for(int j = 0; j < nvars; j++)
      {
         if(SCIPvarIsInLP(vars[j]))
         {
            SCIP_VAR* var = vars[j];
            SCIP_COL *col = SCIPvarGetCol(var);
            double value = SCIPcolGetPrimsol(col);

            constraint_value += 1.0 * value;
         }
         
      }

      double dual = SCIPgetDualsolLinear(scip, cons);

      // if((!(constraint_value < 1.05 )) && dual > 0.05)
      // {
      //    std::cout << "WARN: complementary slackness violated: " << constraint_value << " / " << dual << std::endl;
      //    //throw std::runtime_error("complementary slackness violated");
      // }

      // minus?
      if(farkas) beta_duals[i] = SCIPgetDualfarkasLinear(scip, cons); 
      else beta_duals[i] = SCIPgetDualsolLinear(scip, cons);

      //std::cout << "beta[" << problemData->GetRequestByIndex(i)->id << "] = " <<  beta_duals[i] << std::endl;

      assert(!SCIPconsIsActive(cons) ? beta_duals[i] == 0.0 : true);

   }

   //adjust beta duals considering edge branching constraints:
   const std::vector<SCIP_CONS*> *edgeBranchingConstraints = GetEdgeBranchingConstraints(probdata);
   const std::vector<pair<int, int>> *constrainedEdges = GetConstrainedEdges(probdata);

   std::map<pair<int, int>, double> edgeDuals;
   for( int c = 0; c < (*edgeBranchingConstraints).size(); ++c )
   {
      assert(params->useBranchingOnEdges);
      SCIP_CONS* cons = (*edgeBranchingConstraints)[c];

      /* ignore constraints which are not active since these are not laying on the current active path of the search
       * tree
       */
      if( !SCIPconsIsActive(cons) )
         continue;

      unsigned int success;
      double lhs = SCIPconsGetLhs(scip, cons, &success);
      double rhs = SCIPconsGetRhs(scip, cons, &success);

      pair<int, int> edge = (*constrainedEdges)[c];

      assert(edge.first < edge.second);

      int reqIndex1 = problemData->RequestIdToIndex(edge.first);
      int reqIndex2 = problemData->RequestIdToIndex(edge.second);

      double dual = farkas ? SCIPgetDualfarkasLinear(scip, cons) : SCIPgetDualsolLinear(scip, cons);

      //std::cout << "edge(" << edge.first << " , " << edge.second << ") set to: " << lhs << " , " << rhs <<  " dual: " << dual << std::endl;

      edgeDuals[edge] = dual;
   }

   // for(int i = 0; i < alpha_duals.size(); i++)
   // {  
   //    std::cout << "alpha[" << i << "]: " << alpha_duals[i]<< std::endl;
   // }
   // for(int i = 0; i < beta_duals.size(); i++)
   // {  
   //    std::cout << "beta[" << problemData->GetRequestByIndex(i)->id << "]: " << beta_duals[i]<< std::endl;
   // }

   vector<int> consideredRequests;
   buildConsideredRequestsVector(scip, problemData, consideredRequests);


   PRICING_START:
   //prices out all vehicles:
   //to-do maybe randomize order?
   int iVeh = pricerdata->lastSuccessfullVehicle;
   int nbVehiclesTried = 0;
   while(!addVar && nbVehiclesTried < problemData->NbVehicles())
   {
      if(params->Timeout())
      {
         params->timeout = true;
         break;
      }
      vector<Route> outRoutes;

      pricerdata->pricingAlgo->SetMaxTime(params->maxTimeSinglePricing);
      pricerdata->pricingAlgo->SetMaxMemory(params->maxMemorySinglePricing);

      //temp gambiarra
      SpacedBellmanPricing *sbp = (SpacedBellmanPricing*) pricerdata->pricingAlgo;
      sbp->SetHeuristicPricing(pricerdata->heuristicPricing);

      set<pair<int, int>> forbiddenEdges;
      buildForbiddenEdges(scip, problemData, forbiddenEdges);
      
      std::clock_t alg_start = std::clock();
      PricingReturn ret = pricerdata->pricingAlgo->Price(iVeh, params->newRoutesPerPricing, alpha_duals, beta_duals, outRoutes, consideredRequests, forbiddenEdges, edgeDuals);
      std::clock_t alg_end = std::clock();
      double total_time = ((double)(alg_end - alg_start)) / CLOCKS_PER_SEC; //seconds
      pricerdata->total_pricing_time += total_time;
      pricerdata->total_pricing_calls += 1;
      pricerdata->labelsPriced += ret.labelsPriced;
      pricerdata->labelsStored += ret.labelsStored;
      pricerdata->labelsDeleted += ret.labelsDeleted;
      pricerdata->sumOfMaxLabelsStoredSimultaneously += ret.maxLabelsStoredSimultaneously;
      pricerdata->sumOfMostLabelsInRequest += ret.mostLabelsInRequest;
      pricerdata->sumOfNbConsideredRequests += ret.nbConsideredRequests;
      
      if(ret.timeout) 
      {
         pricerdata->total_pricing_timeouts++;
         params->timeout = true;
      }

      if(ret.status == PricingReturnStatus::OK)
      {
         for(int r = 0; r < outRoutes.size(); r++)
         {

            if(DoesRouteViolateBranching(scip, problemData, &outRoutes[r]))
               throw std::runtime_error("Priced route violates branching rules");

            assert(!DoesRouteViolateBranching(scip, problemData, &outRoutes[r]));

            assert(outRoutes[r].veh_index == iVeh);
            SCIP_VAR* newVar = NULL;
            bool ret2 = createRouteVariable(scip, params, pricerdata->conss, problemData, &outRoutes[r], &newVar, false, ret.reducedCostPerRoute[r]);
            if(ret2)
            {
                /* add the new variable to the pricer store */
               SCIP_CALL( SCIPaddPricedVar(scip, newVar, 1.0) ); // 1.0 -> pricing score? The larger, the better the varible... ???
               SCIP_CALL( SCIPreleaseVar(scip, &newVar) );
               addVar = true;
               pricerdata->lastSuccessfullVehicle = iVeh;
            }
         }
      }
      else
      {
         iVeh = (iVeh + 1) % problemData->NbVehicles();
         nbVehiclesTried++;
      }
   }

   if(!addVar && pricerdata->heuristicPricing && !params->timeout)
   {
      //if we had been doing heuristic pricing until now, but every vehicle has just failed, disable heuristic pricing and try again
      std::cout << "switch to exact pricing" << std::endl;
      pricerdata->heuristicPricing = false;
      goto PRICING_START;
   }

   //result is success regardless of wether vars were added or not
   (*result) = SCIP_SUCCESS;
   return SCIP_OKAY;
   
}

/** reduced cost pricing method of variable pricer for feasible LPs */
static
SCIP_DECL_PRICERREDCOST(pricerRedcostSPwCG)
{  /*lint --e{715}*/
   return DoPricing(scip, pricer, FALSE, result);
}

/** farkas pricing method of variable pricer for infeasible LPs */
// the branching is simply infeasible and no further actions are necessary (?)
static
SCIP_DECL_PRICERFARKAS(pricerFarkasBinpacking)
{  
   return DoPricing(scip, pricer, TRUE, result);
}

/**@} */


/** creates the binpacking variable pricer and includes it in SCIP */
SCIP_RETCODE SCIPincludePricerSPwCG(
   SCIP*                 scip                /**< SCIP data structure */
   )
{
   SCIP_PRICERDATA* pricerdata;
   SCIP_PRICER* pricer;

   /* create SPwCG variable pricer data */
   SCIP_CALL( SCIPallocBlockMemory(scip, &pricerdata) );

   //pricerdata->conshdlr = SCIPfindConshdlr(scip, "samediff");
   //assert(pricerdata->conshdlr != NULL);

   pricerdata->conss = NULL;
   pricerdata->params = NULL;
   pricerdata->problemData = NULL;
   pricerdata->heuristicPricing = true;
   pricerdata->lastSuccessfullVehicle = 0;
   //pricerdata->pricingAlgo;
   pricerdata->total_pricing_time = 0.0;
   pricerdata->total_pricing_calls = 0;
   pricerdata->total_pricing_timeouts = 0;
   pricerdata->labelsPriced = 0;
   pricerdata->labelsStored = 0;
   pricerdata->labelsDeleted = 0;
   pricerdata->sumOfMaxLabelsStoredSimultaneously = 0;
   pricerdata->sumOfMostLabelsInRequest = 0;
   pricerdata->sumOfNbConsideredRequests = 0;


   

   /* include variable pricer */
   SCIP_CALL( SCIPincludePricerBasic(scip, &pricer, PRICER_NAME, PRICER_DESC, PRICER_PRIORITY, PRICER_DELAY,
         pricerRedcostSPwCG, pricerFarkasBinpacking, pricerdata) );

   SCIP_CALL( SCIPsetPricerFree(scip, pricer, pricerFreeSPwCG) );
   SCIP_CALL( SCIPsetPricerInit(scip, pricer, pricerInitSPwCG) );
   SCIP_CALL( SCIPsetPricerExitsol(scip, pricer, pricerExitsolSPwCG) );

   // /* add binpacking variable pricer parameters */
   // /* TODO: (optional) add variable pricer specific parameters with SCIPaddTypeParam() here */

   return SCIP_OKAY;
}

/** added problem specific data to pricer and activates pricer */
SCIP_RETCODE SCIPpricerSPwCGActivate(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_CONS**           conss,              /**< set covering constraints for the items */
   Params*               params,
   ProblemData*          problemData
   )
{
   SCIP_PRICER* pricer;
   SCIP_PRICERDATA* pricerdata;
   int c;

   assert(scip != NULL);
   assert(conss != NULL);
   assert(problemData != NULL);
   

   pricer = SCIPfindPricer(scip, PRICER_NAME);
   assert(pricer != NULL);

   pricerdata = SCIPpricerGetData(pricer);
   assert(pricerdata != NULL);

   pricerdata->params = params;
   pricerdata->problemData = problemData;


   if(params->pricingAlgorithm == PricingAlgorithm::spacedBellman)
   {
      auto sbp = new SpacedBellmanPricing(params, pricerdata->problemData);
      pricerdata->pricingAlgo = sbp;
   }
   else
   {
      std::cerr << "Pricing algorithm no longer used " << (int) params->pricingAlgorithm << std::endl;
      return SCIP_ERROR;
   }


   /* copy arrays */
   SCIP_CALL( SCIPduplicateBlockMemoryArray(scip, &pricerdata->conss, conss, ncons(problemData)) );

   
   /* capture all constraints */
   for( c = 0; c < ncons(problemData); ++c )
   {
      SCIP_CALL( SCIPcaptureCons(scip, conss[c]) );
   }

   /* activate pricer */
   SCIP_CALL( SCIPactivatePricer(scip, pricer) );


   SCIPdebugMsgPrint(scip, "pricer activated \n");
   return SCIP_OKAY;
}
