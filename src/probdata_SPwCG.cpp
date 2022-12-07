#include <string.h>
#include <limits.h>

#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

#include "probdata_SPwCG.h"
//#include "vardata_binpacking.h"
#include "pricer_SPwCG.h"

#include "vardata_SPwCG.h"
#include "pricer_SPwCG.h"

#include "scip/cons_setppc.h"
#include "scip/scip.h"
#include "scip/type_var.h"

#include <assert.h>
#include <unordered_set>

/** @brief Problem data which is accessible in all places
 *
 * This problem data is used to store the input of the binpacking, all variables which are created, and all
 * constraints.
 */
struct SCIP_ProbData
{
   SCIP_VAR**            vars;         /**< all exiting variables in the problem */
   SCIP_CONS**           conss;        /**< set partitioning constraints for each item exactly one */
   int                   nvars;        /**< number of generated variables */
   int                   varssize;     /**< size of the variable array */
   Params*               params;       /* params class, with meta-parameters for solving */
   ProblemData*          problemData;  /** all data for this problem instance is acessible via this C++ Class */


   vector<SCIP_CONS*> *vehicleBranchingConstraints;
   vector<int> *constrainedVehicles;

   /*
      set of 'edges' constrained. An 'edge' in this context is actually not an actual graph edge, but simply the servicing of i,j (or j,i) in a row!
      some weird consequences of this:
       - we define i < j always
       - some valid routes may not have any 'edges' because they're only servicing one request!
   */
   vector<SCIP_CONS*> *edgeBranchingConstraints;
   vector<pair<int, int>> *constraintedEdges;

   std::multimap<size_t, SCIP_VAR*>* RouteToVarMap; /* hash set to check for variable duplicity */

   //LOGGING:
   size_t timesBranchedWithRule[3];
   size_t timesRepeatedRouteWasPriced;
   double repeatedRoutesTotalReducedCost;
};

ProblemData* GetProblemData(SCIP_ProbData *probdata)
{
   return probdata->problemData;
}
Params* GetParams(SCIP_ProbData *probdata)
{
   return probdata->params;
}

bool IsVarRepeated(SCIP_ProbData *probdata, Route* route, SCIP* scip)
{
   size_t route_hash = route->GetHash(probdata->problemData);
   //check if any active variables are associated with the same route:
   // in a multimap, elements with the same key are always sequential, and find gets the first one.
   // using a mulimap was necessary because variables using the route but in different branches should be allowed to coexist
   // ???
   for (auto itr = probdata->RouteToVarMap->find(route_hash); itr != probdata->RouteToVarMap->end() && itr->first == route_hash; itr++)
   {
      SCIP_Var *var = itr->second;
      SCIP_Real redCost = SCIPgetVarRedcost(scip, var);
      std::cout << "found var " << SCIPvarGetName(var) << ". Active? " <<  SCIPvarIsActive(itr->second) << "in LP?: " << SCIPvarIsInLP(var) << " redCost:" << redCost << std::endl;
      //exit(1);
      // SCIPvarIsActive true iff var is in current node?
      if(SCIPvarIsActive(itr->second)) {

         //if the var is active and the hash matches, and only then, we check the entire route one by one
         bool identical = true;
         Route *otherRoute = SCIPvardataGetRoute(SCIPvarGetData(itr->second));

         if(route->vertices.size() != otherRoute->vertices.size()) identical = false;
         for(int i = 0; i < route->vertices.size() && identical == true; i++)
         {
            if(route->vertices[i].id != otherRoute->vertices[i].id) identical = false;
         }

         

         std::cout << "identical?:" << identical << std::endl;
         
         if(identical) 
         {            
            //what is vars value in the current solution? 
            double value = 0.0;
            if(SCIPvarIsInLP(var))
            {
               SCIP_COL *col = SCIPvarGetCol(var);
               value = SCIPcolGetPrimsol(col);

               double myRC = route->total_lateness;
               std::cout << "total lateness:" << route->total_lateness << std::endl;
               SCIP_VARDATA *vardata = SCIPvarGetData(var);
               int ncons = SCIPvardataGetNConsids(vardata);
               int* consids = SCIPvardataGetConsids(vardata);

               std::cout << "consids:";
               for(int i = 0; i < ncons; i++)
               {
                  SCIP_CONS* cons = probdata->conss[consids[i]];

                  double dual = SCIPgetDualsolLinear(scip, cons);

                  std::cout << consids[i] << " , ";

                  myRC -= dual;
               }

               std::cout << std::endl;

               for (int iver = 0; iver < route->vertices.size(); iver++)
               {
                  int id = route->vertices[iver].id;
                  char c = 'X';
                  //if (problemData->IsInitialPosition(id)) c = 'i';
                  //if (problemData->IsRequest(id)) c = 'r';
                  //if (problemData->IsWaitingStation(id)) c = 'w';

                  //servicedRequests.emplace(id);
                  std::cout << id << "(" << c << ")";
               }

               std::cout << endl;

               std::cout << "Repeated Var(" << SCIPvarGetName(var)<< ")" << "RedCost: " << redCost << " MyRedCost: " << myRC <<  " used?: " << value << " in LP?: " << SCIPvarIsInLP(var) << " initial?: " << SCIPvarIsInitial(var) << "cost: " << SCIPvarGetObj(var) << "loops?: " << route->has_cycles << std::endl;
               //exit(1);

            }
            
            return true;
         }
            
      }
   }
   return false;

}

void AddVehicleBranchingCons(SCIP* scip, SCIP_PROBDATA* probdata, SCIP_CONS* cons, int vehicle_id)
{
   SCIPcaptureCons(scip, cons);
   probdata->vehicleBranchingConstraints->push_back(cons);
   probdata->constrainedVehicles->push_back(vehicle_id);
}

void AddEdgeBranchingCons(SCIP* scip, SCIP_PROBDATA* probdata, SCIP_CONS* cons, pair<int, int> edge)
{
   assert(edge.first < edge.second);
   SCIPcaptureCons(scip, cons);
   probdata->edgeBranchingConstraints->push_back(cons);
   probdata->constraintedEdges->push_back(edge);
}

const vector<SCIP_CONS*>* GetVehicleBranchingConstraints(SCIP_PROBDATA* probdata)
{
   return  probdata->vehicleBranchingConstraints;
}
const vector<int>* GetConstrainedVehicles(SCIP_PROBDATA* probdata)
{
   return  probdata->constrainedVehicles;
}

const vector<SCIP_CONS*>* GetEdgeBranchingConstraints(SCIP_PROBDATA* probdata)
{
   return  probdata->edgeBranchingConstraints;
}
const vector<pair<int, int>>* GetConstrainedEdges(SCIP_PROBDATA* probdata)
{
   return  probdata->constraintedEdges;
}

void IncrementUsedBranchingRule(SCIP_PROBDATA* probdata, int ruleIndex)
{
   assert(ruleIndex >= 0 && ruleIndex < 3);
   probdata->timesBranchedWithRule[ruleIndex]++;
}

void LogRepeatedRoute(SCIP_PROBDATA* probdata, double reducedCost)
{
   probdata->timesRepeatedRouteWasPriced++;
   probdata->repeatedRoutesTotalReducedCost += reducedCost;
}

/** returns array of all variables itemed in the way they got generated */
SCIP_VAR** SCIPprobdataGetVars(
   SCIP_PROBDATA*        probdata            /**< problem data */
   )
{
   return probdata->vars;
}
/** returns number of variables */
int SCIPprobdataGetNVars(
   SCIP_PROBDATA*        probdata            /**< problem data */
   )
{
   return probdata->nvars;
}

/** returns array of all variables itemed in the way they got generated */
SCIP_CONS** SCIPprobdataGetCons(
   SCIP_PROBDATA*        probdata            /**< problem data */
)

{
   return probdata->conss;
}
/** returns number of variables */
int SCIPprobdataGetNCons(
   SCIP_PROBDATA*        probdata            /**< problem data */
)
{
   return probdata->problemData->NbRequests() + probdata->problemData->NbVehicles();
}

/**@name Event handler properties
 *
 * @{
 */

#define EVENTHDLR_NAME         "addedvar"
#define EVENTHDLR_DESC         "event handler for catching added variables"


/**@name Local methods
 *
 * @{
 */

static int ncons(ProblemData* problemData) 
{
   return problemData->NbVehicles() + problemData->NbRequests();
}

/** creates problem data */
static
SCIP_RETCODE probdataCreate(
   SCIP*                 scip,              /**< SCIP data structure */
   SCIP_PROBDATA**       probdata,          /**< pointer to problem data */
   SCIP_VAR**            vars,              /**< array of ALL vars */
   SCIP_CONS**           conss,             /**< array of all constraints */
   int                   nvars,             /**< number of route variables */
   Params*               params,            /**< global params */
   ProblemData*          problemData        /**< general data of this problem instance */
   )
{
   assert(scip != NULL);
   assert(probdata != NULL);
   assert(problemData != NULL);

   /* allocate memory */
   SCIP_CALL( SCIPallocBlockMemory(scip, probdata) );

   if( nvars > 0 )
   {
      /* copy variable array */
      SCIP_CALL( SCIPduplicateBlockMemoryArray(scip, &(*probdata)->vars, vars, nvars) );
   }
   else
      (*probdata)->vars = NULL;

   
   SCIP_CALL( SCIPduplicateBlockMemoryArray(scip, &(*probdata)->conss, conss, ncons(problemData)) );

   (*probdata)->nvars = nvars;
   (*probdata)->varssize = nvars;
   (*probdata)->problemData = problemData;
   (*probdata)->params = params;

   (*probdata)->timesBranchedWithRule[0] = 0;
   (*probdata)->timesBranchedWithRule[1] = 0;
   (*probdata)->timesBranchedWithRule[2] = 0;
   (*probdata)->timesRepeatedRouteWasPriced = 0;
   (*probdata)->repeatedRoutesTotalReducedCost = 0.0;
   


   (*probdata)->RouteToVarMap = new std::multimap<size_t, SCIP_VAR*>();

   //add initial routes:
   for(int i = problemData->NbRequests(); i < nvars; i++) //first problemData->NbRequests() are y vars
   {
      SCIP_VAR* var = vars[i];
      SCIP_VARDATA *vardata = SCIPvarGetData(var);
      assert(vardata != NULL);
      Route *route = SCIPvardataGetRoute(vardata);
      assert(route != NULL);
      size_t hash = route->GetHash((*probdata)->problemData);
      (*probdata)->RouteToVarMap->insert(std::make_pair(hash, var));
   }

   (*probdata)->vehicleBranchingConstraints = new std::vector<SCIP_CONS*>(); //branching constraints are empty when probdata is initialized
   (*probdata)->constrainedVehicles = new std::vector<int>(); //branching constraints are empty when probdata is initialized   

   (*probdata)->edgeBranchingConstraints = new std::vector<SCIP_CONS*>(); //branching constraints are empty when probdata is initialized
   (*probdata)->constraintedEdges = new std::vector<pair<int, int>>(); //branching constraints are empty when probdata is initialized   
   
   return SCIP_OKAY;
}

/** frees the memory of the given problem data */
static
SCIP_RETCODE probdataFree(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_PROBDATA**       probdata            /**< pointer to problem data */
   )
{
   int i;

   assert(scip != NULL);
   assert(probdata != NULL);

   /* release all variables */
   for( i = 0; i < (*probdata)->nvars; ++i )
   {
      SCIP_CALL( SCIPreleaseVar(scip, &(*probdata)->vars[i]) );
   }

   /* release all constraints */
   for( i = 0; i < ncons((*probdata)->problemData); ++i )
   {
      SCIP_CALL( SCIPreleaseCons(scip, &(*probdata)->conss[i]) );
   }

   for(int i = 0; i < (*probdata)->vehicleBranchingConstraints->size(); i++)
   {
      vector<SCIP_CONS*>* vecPtr = (*probdata)->vehicleBranchingConstraints;
      SCIP_CALL( SCIPreleaseCons(scip, &(*vecPtr)[i]) );
   }

   for(int i = 0; i < (*probdata)->edgeBranchingConstraints->size(); i++)
   {
      vector<SCIP_CONS*>* vecPtr = (*probdata)->edgeBranchingConstraints;
      SCIP_CALL( SCIPreleaseCons(scip, &(*vecPtr)[i]) );
   }

   /* free memory of arrays */
   SCIPfreeBlockMemoryArray(scip, &(*probdata)->vars, (*probdata)->varssize);
   SCIPfreeBlockMemoryArray(scip, &(*probdata)->conss, ncons((*probdata)->problemData));

   delete (*probdata)->RouteToVarMap;
   delete (*probdata)->vehicleBranchingConstraints;
   delete (*probdata)->constrainedVehicles;
   delete (*probdata)->edgeBranchingConstraints;
   delete (*probdata)->constraintedEdges;
   
   /* free probdata */
   SCIPfreeBlockMemory(scip, probdata);

   return SCIP_OKAY;
}

/**@} */

/**@name Callback methods of event handler
 *
 * @{
 */

/** adds given variable to the problem data */
SCIP_RETCODE SCIPprobdataAddVar(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_PROBDATA*        probdata,           /**< problem data */
   SCIP_VAR*             var                 /**< variables to add */
   )
{

   SCIP_VARDATA *vardata = SCIPvarGetData(var);

   /* check if enough memory is left */
   if( probdata->varssize == probdata->nvars )
   {
      int newsize;
      newsize = MAX(100, probdata->varssize * 2);
      SCIP_CALL( SCIPreallocBlockMemoryArray(scip, &probdata->vars, probdata->varssize, newsize) );
      probdata->varssize = newsize;
   }

   /* caputure variables */
   SCIP_CALL( SCIPcaptureVar(scip, var) );

   probdata->vars[probdata->nvars] = var;
   probdata->nvars++;

   //add var to hashed set
   Route *route = SCIPvardataGetRoute(vardata);
   //assert(probdata->hashedRoutes->find(*SCIPvardataGetRoute(vardata)) == probdata->hashedRoutes->end());
   assert(!IsVarRepeated(probdata, route));

   //probdata->RouteToVarMap->insert(std::pair<size_t, SCIP_VAR*>( (*route)::HashFunction(), var ));

   size_t hash = route->GetHash(probdata->problemData);
   probdata->RouteToVarMap->insert(std::make_pair(hash, var));

   SCIPdebugMsg(scip, "added variable to probdata; nvars = %d\n", probdata->nvars);

   return SCIP_OKAY;
}

/** execution method of event handler */
static
SCIP_DECL_EVENTEXEC(eventExecAddedVar)
{  /*lint --e{715}*/
   assert(eventhdlr != NULL);
   assert(strcmp(SCIPeventhdlrGetName(eventhdlr), EVENTHDLR_NAME) == 0);
   assert(event != NULL);
   assert(SCIPeventGetType(event) == SCIP_EVENTTYPE_VARADDED);

   SCIPdebugMsg(scip, "exec method of event handler for added variable to probdata\n");

   /* add new variable to probdata */
   SCIP_CALL( SCIPprobdataAddVar(scip, SCIPgetProbData(scip), SCIPeventGetVar(event)) );

   return SCIP_OKAY;
}

/**@} */


/**@name Callback methods of problem data
 *
 * @{
 */

/** frees user data of original problem (called when the original problem is freed) */
static
SCIP_DECL_PROBDELORIG(probdelorigSPwCG)
{
   SCIPdebugMsg(scip, "free original problem data\n");

   SCIP_CALL( probdataFree(scip, probdata) );

   return SCIP_OKAY;
}

/** frees user data of transformed problem (called when the transformed problem is freed) */
static
SCIP_DECL_PROBDELTRANS(probdeltransSPwCG)
{
   SCIPdebugMsg(scip, "free transformed problem data\n");

   SCIP_CALL( probdataFree(scip, probdata) );

   return SCIP_OKAY;
}

/** creates user data of transformed problem by transforming the original user problem data
 *  (called after problem was transformed) */
static
SCIP_DECL_PROBTRANS(probtransSPwCG)
{
   /* create transform probdata */
   SCIP_CALL( probdataCreate(scip, targetdata, sourcedata->vars, sourcedata->conss, sourcedata->nvars, sourcedata->params, sourcedata->problemData) );

   /* transform all constraints */
   SCIP_CALL( SCIPtransformConss(scip, ncons((*targetdata)->problemData), (*targetdata)->conss, (*targetdata)->conss) );

   /* transform all variables */
   SCIP_CALL( SCIPtransformVars(scip, (*targetdata)->nvars, (*targetdata)->vars, (*targetdata)->vars) );

   return SCIP_OKAY;
}

/** solving process initialization method of transformed data (called before the branch and bound process begins) */
static
SCIP_DECL_PROBINITSOL(probinitsolSPwCG)
{
   SCIP_EVENTHDLR* eventhdlr;

   assert(probdata != NULL);

   /* catch variable added event */
   eventhdlr = SCIPfindEventhdlr(scip, "addedvar");
   assert(eventhdlr != NULL);

   SCIP_CALL( SCIPcatchEvent(scip, SCIP_EVENTTYPE_VARADDED, eventhdlr, NULL, NULL) );

   return SCIP_OKAY;
}

/** solving process deinitialization method of transformed data (called before the branch and bound data is freed) */
static
SCIP_DECL_PROBEXITSOL(probexitsolSPwCG)
{  /*lint --e{715}*/
   SCIP_EVENTHDLR* eventhdlr;

   assert(probdata != NULL);

   SCIPdebugMsg(scip, "----------------\n");
   SCIPdebugMsg(scip, "----------------\n");
   SCIPdebugMsg(scip, "----------------\n");
   SCIPdebugMsg(scip, "probexitsolSPwCG\n");
   SCIPdebugMsg(scip, "----------------\n");
   SCIPdebugMsg(scip, "----------------\n");
   SCIPdebugMsg(scip, "----------------\n");

   /* drop variable added event */
   eventhdlr = SCIPfindEventhdlr(scip, "addedvar");
   assert(eventhdlr != NULL);

   SCIP_CALL( SCIPdropEvent(scip, SCIP_EVENTTYPE_VARADDED, eventhdlr, NULL, -1) );

   return SCIP_OKAY;
}

/**@} */

/**@name Interface methods
 *
 * @{
 */

/** sets up SCIP execution for a given (my) ProblemData structure */
SCIP_RETCODE loadProblem(
   SCIP*                 scip,               /**< SCIP data structure */
   Params*               params,             /**< global params */
   ProblemData*          problemData         /**< my problem's data structure */
)
{
   SCIP_PROBDATA* probdata;
   SCIP_CONS** conss;
   SCIP_VAR** vars;
   char name[SCIP_MAXSTRLEN];
   int i;

   assert(scip != NULL);
   assert(problemData != NULL);
   assert(params != NULL);

   /* create event handler if it does not exist yet */
   if( SCIPfindEventhdlr(scip, EVENTHDLR_NAME) == NULL )
   {
      SCIP_CALL( SCIPincludeEventhdlrBasic(scip, NULL, EVENTHDLR_NAME, EVENTHDLR_DESC, eventExecAddedVar, NULL) );
   }

   /* create problem in SCIP and add non-NULL callbacks via setter functions */
   SCIP_CALL( SCIPcreateProbBasic(scip, problemData->name.c_str()) );

   SCIP_CALL( SCIPsetProbDelorig(scip, probdelorigSPwCG) );
   SCIP_CALL( SCIPsetProbTrans(scip, probtransSPwCG) );
   SCIP_CALL( SCIPsetProbDeltrans(scip, probdeltransSPwCG) );
   SCIP_CALL( SCIPsetProbInitsol(scip, probinitsolSPwCG) );
   SCIP_CALL( SCIPsetProbExitsol(scip, probexitsolSPwCG) );

   /* set objective sense */
   SCIP_CALL( SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE) );

   //initialize a 'simple coverage' solution. This will be used to ensure that the initial LP is feasible
   vector<Route> initial_routes = ProblemSolution::ClosestAvailableVehicleSolution(problemData, params);

   if(params->nbRandomInitialRoutes > 0)
   {
      ProblemSolution::RandomRoutes(problemData, params, params->nbRandomInitialRoutes, (uint32_t) params->route_gen_seed, initial_routes);
   }

   if(problemData->computeTimeHorizon)
   {
      double max_time = 0.0;
      for(Route &route : initial_routes)
      {
         assert(problemData->IsRequest(route.vertices.back().id));
         max_time = std::max(max_time, route.arrival_times.back());
      }
      assert(max_time > 0);
      problemData->timeHorizon = max_time * 1.5; //1.5 ?
      std::cout << "time horizon computed as " << problemData->timeHorizon << std::endl;
   }
   

   //arrays for (initial) variables and constraints
   SCIP_CALL( SCIPallocBufferArray(scip, &conss, ncons(problemData) ));
   SCIP_CALL( SCIPallocBufferArray(scip, &vars, problemData->NbRequests() + initial_routes.size()) );

   //create y variables:
   for(i = 0; i < problemData->NbRequests(); i++)
   {
      const Request* req = problemData->GetRequestByIndex(i);
      (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "y_%d", i);

      SCIP_VARTYPE var_type = params->solveRelaxedProblem ? SCIP_VARTYPE_CONTINUOUS : SCIP_VARTYPE_BINARY;

      if(req->non_service_penalty == std::numeric_limits<double>::infinity())
      {
         /* to-do: rework this.
            some instances require that some(or all) requests must be serviced. In terms of ProblemData, this is signaled with non_service_penalty == infinity
            however, I coded the scip problem without planning for this scenario. Thus, if y variables are ommited, there will be problems when branching, pricing etc.
            so, as a cheat, I create them but fix them to 0
         */
         SCIP_CALL(SCIPcreateVar(scip, &vars[i], name, 0.0, 0.0, 1.0,
                           var_type, TRUE, FALSE, // initial, removable ???
                          NULL, NULL, NULL, NULL, NULL));

         //fix to 0.0 via lb ub

         //std::cout << "ERROR" << std::endl;
         //exit(1);

      }
      else
      {

      
         SCIP_CALL(SCIPcreateVar(scip, &vars[i], name, 0.0, 1.0, req->non_service_penalty,
                              var_type, TRUE, FALSE,
                              NULL, NULL, NULL, NULL, NULL));

         //lazy bound:
         SCIP_CALL( SCIPchgVarUbLazy(scip, vars[i], 1.0) );
      }

      SCIP_CALL(SCIPaddVar(scip, vars[i]));

   }

   /* create constraints */
   //vehicle constraints
   for( i = 0; i < problemData->NbVehicles(); ++i )
   {
      int var_index = i + problemData->NbRequests(); //initial route var related to this constraint
      (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "veh_cons%d", i);

      // ??
      //SCIP_CALL( SCIPcreateConsBasicSetcover(scip, &conss[i], name, 0, NULL) );

      SCIP_CALL(SCIPcreateConsLinear(scip, &conss[i], name,
                                   0, NULL, NULL, 0.0, 1.0, TRUE,
                                   TRUE, TRUE, TRUE, TRUE, FALSE,
                                   FALSE, FALSE, FALSE, FALSE));

      /* declare constraint modifiable for adding variables during pricing */
      SCIP_CALL( SCIPsetConsModifiable(scip, conss[i], TRUE) );

      //this constraint has no active coefficients in the y variables, thus no coeffs are added here
      //SCIP_CALL(SCIPaddCoefLinear(scip, cons, var, 1.0));

      SCIP_CALL( SCIPaddCons(scip, conss[i]) );

   }

   //request constraints
   for( i = 0; i < problemData->NbRequests(); ++i )
   {
      int con_index = i + problemData->NbVehicles();
      (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "req_cons%d", i);

      // ??
      //SCIP_CALL( SCIPcreateConsBasicSetcover(scip, &conss[i], name, 0, NULL) );

      //SCIPinfinity(scip)

      SCIP_CALL(SCIPcreateConsLinear(scip, &conss[con_index], name,
                                   0, NULL, NULL, 1.0, 1.0, TRUE,
                                   TRUE, TRUE, TRUE, TRUE, FALSE,
                                   FALSE, FALSE, FALSE, FALSE));

      /* declare constraint modifiable for adding variables during pricing */
      SCIP_CALL( SCIPsetConsModifiable(scip, conss[con_index], TRUE) );

      // this constraint is related to the i-th y variable:
      const Request* req = problemData->GetRequestByIndex(i);
      if(req->non_service_penalty == std::numeric_limits<double>::infinity())
      {
         //dont add!
      }
      else
      {
         SCIP_CALL(SCIPaddCoefLinear(scip, conss[con_index], vars[i], 1.0));
      }
      

      SCIP_CALL( SCIPaddCons(scip, conss[con_index]) );

   }

   vector<SCIP_CONS*> tempBoundCons;
   //create initial x variables
   for(i = 0; i < initial_routes.size(); i++)
   {
      int v_index = i + problemData->NbRequests();
      int size = initial_routes.size();
      bool ret = createRouteVariable(scip, params, conss, problemData, &initial_routes[i], &vars[v_index], true);

      if(ret)
      {
         SCIP_CALL(SCIPaddVar(scip, vars[v_index]));
      }
      else throw std::runtime_error("SCIP problem creation : couldn't create initial x variables");
      
   }

    /* create problem data */
   SCIP_CALL( probdataCreate(scip, &probdata, vars, conss, problemData->NbRequests() + initial_routes.size(), params, problemData) );

   /* set user problem data */
   SCIP_CALL( SCIPsetProbData(scip, probdata) );

   SCIP_CALL( SCIPpricerSPwCGActivate(scip, conss, params, problemData) );



   /* free local buffer arrays */
   SCIPfreeBufferArray(scip, &conss);
   SCIPfreeBufferArray(scip, &vars);

   std::cout << "time horizon?:" << problemData->timeHorizon << std::endl;
   std::cout << "how many constraints?: " << SCIPgetNConss(scip) << "vs ncons():" << problemData->NbVehicles() + problemData->NbRequests() << std::endl;
   std::cout << "how many vars?: " << SCIPgetNVars(scip) << std::endl;
   std::cout << "how many vehs?: " << problemData->NbVehicles() << std::endl;
   

   return SCIP_OKAY;
}


#include "ProblemSolution.h"
void QuerySolution(SCIP *scip, ProblemSolution &solution)
{
   solution.routes.clear();
   solution.coeffs.clear();
   SCIP_ProbData* probdata = SCIPgetProbData(scip);

   int nvars = SCIPprobdataGetNVars(probdata);
   SCIP_VAR** vars = SCIPprobdataGetVars(probdata);

   SCIP_SOL * scip_sol = SCIPgetBestSol(scip);

   double scip_cost = SCIPsolGetOrigObj(scip_sol);

   //first vars are not route vars. They're 'non-service-penalty' vars
   for(int i = solution.problemData->NbRequests(); i < nvars; i++)
   {
      SCIP_Real value = SCIPgetSolVal(scip, scip_sol, vars[i]);
      if(value > 0.1)
      {
         SCIP_VARDATA* vardata = SCIPvarGetData(vars[i]);
         assert(vardata != NULL);
         solution.routes.push_back(*SCIPvardataGetRoute(vardata)); //copy
         solution.coeffs.push_back(value);
      }
   }

   //maybe order routes by vehicle?

   solution.UpdateCost();

   std::cout << scip_cost << " / " << solution.cost << std::endl;

}

#include "scip/struct_pricer.h"

ExecutionSummary GetExecutionSummary(SCIP *scip)
{
   ExecutionSummary summary;

   SCIP_PROBDATA* probdata = SCIPgetProbData(scip);

   SCIP_Pricer *pricer = SCIPfindPricer(scip, "SPwCG");
   assert(pricer != NULL);
   SCIP_PRICERDATA *pricerdata = SCIPpricerGetData(pricer);
   assert(pricerdata != NULL);
   summary.total_pricing_time = pricerdata->total_pricing_time;
   summary.total_pricing_calls = pricerdata->total_pricing_calls;
   summary.total_pricing_timeouts = pricerdata->total_pricing_timeouts;
   summary.totalLabelsPriced = pricerdata->labelsPriced;
   summary.totalLabelsStored = pricerdata->labelsStored;
   summary.totalLabelsDeleted = pricerdata->labelsDeleted;
   summary.sumOfMaxLabelsStoredSimultaneously = pricerdata->sumOfMaxLabelsStoredSimultaneously;
   summary.sumOfMostLabelsInRequest = pricerdata->sumOfMostLabelsInRequest;
   summary.sumOfNbConsideredRequests = pricerdata->sumOfNbConsideredRequests;
   
   summary.timesBranchedWithRule[0] = probdata->timesBranchedWithRule[0];
   summary.timesBranchedWithRule[1] = probdata->timesBranchedWithRule[1];
   summary.timesBranchedWithRule[2] = probdata->timesBranchedWithRule[2];

   summary.timesRepeatedRouteWasPriced = probdata->timesRepeatedRouteWasPriced;
   summary.repeatedRoutesTotalReducedCost = probdata->repeatedRoutesTotalReducedCost;
   
   return summary;
}


namespace fs = boost::filesystem;
void OutputDuals(SCIP *scip, Params* params)
{

   SCIP_PRICER* pricer;
   pricer = SCIPfindPricer(scip, "SPwCG");
   assert(pricer != NULL);

   /* get the pricer data */
   SCIP_PRICERDATA* pricerdata = SCIPpricerGetData(pricer);
   assert(pricerdata != NULL);
   assert(pricerdata->problemData != NULL);
   ProblemData *problemData = pricerdata->problemData;

   fs::path dir (params->outputDirectory);
   fs::path out_path = dir / fs::path(problemData->name + params->outputSuffix + "_duals.dl");
   std::ofstream myfile;
   myfile.open(out_path.string());

   myfile << "alpha" << std::endl;
   for( int i = 0; i < problemData->NbVehicles(); ++i )
   {
      SCIP_CONS* cons = pricerdata->conss[i];
      
      // minus?
      double value = -SCIPgetDualsolLinear(scip, cons);
      myfile << value << ",";
   }
   myfile << std::endl;

   //request constraints
   myfile << "beta" << std::endl;
   for( int i = 0; i < problemData->NbRequests(); ++i )
   {
      int con_index = i + problemData->NbVehicles();
      SCIP_CONS* cons = pricerdata->conss[con_index];

      // minus?
      // minus?
      double value = -SCIPgetDualsolLinear(scip, cons);
      myfile << value << ",";

   }

}


/**@} */