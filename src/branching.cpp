

/**@file   branching.cpp
 * @ingroup BRANCHINGRULES
 * @brief  Implementation of custom branching rules
 * @author André Mazal Krauss
 *
 * This file implements branching rules for the problem.
 *
 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/

#include <assert.h>
#include <string.h>
#include <math.h> 

#include <utility>
using std::pair;

#include <cstddef>

#include "branching.h"

#include "scip/scip.h"

#include "probdata_SPwCG.h"
#include "vardata_SPwCG.h"
#include "probdata_SPwCG.h"

#include "ProblemData.h"
#include "ProblemSolution.h"

/**@name Branching rule properties
 *
 * @{
 */

#define BRANCHRULE_NAME            "CustomRules"
#define BRANCHRULE_DESC            "Custom made branching rule"
#define BRANCHRULE_PRIORITY        50000
#define BRANCHRULE_MAXDEPTH        -1
#define BRANCHRULE_MAXBOUNDDIST    1.0

/**@} */

/**@name Helper methods
 *
 * @{
 */

/**
 * @brief Creates a new branching constraint constraining the usage of a vehicle to a certain value (0/1)
 *  
*/
SCIP_RETCODE SCIPcreateConsSumVehicle(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_CONS**           cons,               /**< pointer to hold the created constraint */
   const char*           name,               /**< name of constraint */
   int                   vehicle_id,         
   double                desired_sum,         
   SCIP_NODE*            node,               /**< the node in the B&B-tree at which the cons is sticking */
   SCIP_Bool             local               /**< is constraint only valid locally? */
   )
{
    SCIPdebugMsgPrint(scip, "SCIPcreateConsSumVehicle \n");

    SCIP_PROBDATA* probdata = SCIPgetProbData(scip);
    assert(probdata != NULL);

    ProblemData *problemData = GetProblemData(probdata);
    assert(problemData != NULL);
   

   SCIP_CALL(SCIPcreateConsLinear(scip, cons, name,
                                0, NULL, NULL, desired_sum, desired_sum, TRUE,
                                TRUE, TRUE, TRUE, TRUE, 
                                local, // <--- 'local' == true -> branching constraint
                                TRUE, // <-- 'modifiable' 
                                FALSE, FALSE, FALSE));
   
   
    /* declare constraint modifiable for adding variables during pricing */
    SCIP_CALL( SCIPsetConsModifiable(scip, *cons, TRUE) );

    //iterate vars and add them to constraint
    int n_vars = SCIPprobdataGetNVars(probdata);
    SCIP_VAR** vars = SCIPprobdataGetVars(probdata);

    for( int i = problemData->NbRequests(); i < n_vars; ++i ) //first problemData->NbRequests() vars are y vars
    {
        SCIP_VAR *t_var = vars[i];
        assert(t_var != NULL);
        SCIP_VARDATA* t_vardata = SCIPvarGetData(t_var);
        assert(t_vardata != NULL);
        Route* t_route = SCIPvardataGetRoute(t_vardata);
        assert(t_route != NULL);

        if(t_route->veh_index == vehicle_id)
        {
            SCIP_CALL(SCIPaddCoefLinear(scip, *cons, t_var, 1.0));
        }
    }

   SCIPdebugMsg(scip, "created constraint: veh constraint ");
   
   return SCIP_OKAY;
}

/**
 * @brief Creates a new branching constraint constraining the usage of an edge to a certain value (0/1)
 *  
*/
SCIP_RETCODE SCIPcreateConsSumEdge(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_CONS**           cons,               /**< pointer to hold the created constraint */
   const char*           name,               /**< name of constraint */
   pair<int, int>        edge,               /* edge i,j with i < j*/        
   double                desired_sum,         
   SCIP_NODE*            node,               /**< the node in the B&B-tree at which the cons is sticking */
   SCIP_Bool             local               /**< is constraint only valid locally? */
   )
{
    SCIPdebugMsgPrint(scip, "SCIPcreateConsSumEdge \n");

    assert(edge.first < edge.second);

    SCIP_PROBDATA* probdata = SCIPgetProbData(scip);
    assert(probdata != NULL);

    ProblemData *problemData = GetProblemData(probdata);
    assert(problemData != NULL);
   
   SCIP_CALL(SCIPcreateConsLinear(scip, cons, name,
                                0, NULL, NULL, desired_sum, desired_sum, TRUE,
                                TRUE, TRUE, TRUE, TRUE, 
                                local, // <--- 'local' == true -> branching constraint
                                TRUE, // <-- 'modifiable' 
                                FALSE, FALSE, 
                                /*FALSE*/ TRUE ));


   SCIP_CONSDATA* consdata = SCIPconsGetData (*cons);
   assert(consdata != NULL);
   
   
    /* declare constraint modifiable for adding variables during pricing */
    SCIP_CALL( SCIPsetConsModifiable(scip, *cons, TRUE) );

    //iterate vars and add them to constraint
    int n_vars = SCIPprobdataGetNVars(probdata);
    SCIP_VAR** vars = SCIPprobdataGetVars(probdata);

   //no y_var's associated to edge

   for( int i = problemData->NbRequests(); i < n_vars; ++i ) //first problemData->NbRequests() vars are y vars
   {
      SCIP_VAR *t_var = vars[i];
      assert(t_var != NULL);
      SCIP_VARDATA* t_vardata = SCIPvarGetData(t_var);
      assert(t_vardata != NULL);
      Route* t_route = SCIPvardataGetRoute(t_vardata);
      assert(t_route != NULL);

      std::map<pair<int, int>, int> edgeUsage = t_route->GetEdgeUsage(problemData);

      if(edgeUsage.find(edge) != edgeUsage.end())
      {
         SCIP_CALL(SCIPaddCoefLinear(scip, *cons, t_var, edgeUsage[edge]));
      }
   }


   SCIPdebugMsg(scip, "created constraint: edge constraint ");
   
   return SCIP_OKAY;
}

/**
 * @brief Checks if variable has already been branched at for the current node
 *  
*/
static bool IsSimpleBranchingRepeated(SCIP* scip, ProblemData* problemData, SCIP_VAR* var)
{
   //if var is active, it isn't fixed
   return !SCIPvarIsActive(var);
}

/**
 * @brief Checks if vehicle branching rule has already been applied for a given vehicle at the current node
 *  
*/
static bool IsVehicleBranchingRepeated(SCIP* scip, ProblemData* problemData, int chosenVehicle)
{
   SCIP_PROBDATA *probdata = SCIPgetProbData(scip);
   /* collect all branching decision constraints */
   const std::vector<SCIP_CONS*> *vehicleBranchingConstraints = GetVehicleBranchingConstraints(probdata);
   const std::vector<int> *constrainedVehicles = GetConstrainedVehicles(probdata);

   for( int c = 0; c < (*vehicleBranchingConstraints).size(); ++c )
   {

      //assert(params->useBranchingOnVehicles);

      SCIP_CONS* cons = (*vehicleBranchingConstraints)[c];

      /* ignore constraints which are not active since these are not laying on the current active path of the search
       * tree
       */
      if( !SCIPconsIsActive(cons) )
         continue;

      int veh_id = (*constrainedVehicles)[c];

      if(veh_id == chosenVehicle) return true;

   }

   return false;
}

/**
 * @brief Checks if edge branching rule has already been applied for a given edge at the current node
 *  
*/
static bool IsEdgeBranchingRepeated(SCIP* scip, ProblemData* problemData, pair<int, int> chosenEdge)
{
   SCIP_PROBDATA *probdata = SCIPgetProbData(scip);

   /* collect all branching decision constraints */
   const std::vector<SCIP_CONS*> *edgeBranchingConstraints = GetEdgeBranchingConstraints(probdata);
   const std::vector<pair<int, int>> *constrainedEdges = GetConstrainedEdges(probdata);

   for( int c = 0; c < (*edgeBranchingConstraints).size(); ++c )
   {
      SCIP_CONS* cons = (*edgeBranchingConstraints)[c];

      /* ignore constraints which are not active since these are not laying on the current active path of the search
         * tree
         */
      if( !SCIPconsIsActive(cons) || SCIPconsIsDeleted(cons))
         continue;

      pair<int, int> cons_edge = (*constrainedEdges)[c];

      if(cons_edge == chosenEdge) return true;
   }

   return false;
}
  
/**
 * @brief Debug function for printing out branching constraints for a given value
 *  
*/
static SCIP_RETCODE PrintConstraintValues(SCIP* scip,  std::map<SCIP_VAR*, double> &var_solutionValue)
{

   SCIP_PROBDATA *probdata = SCIPgetProbData(scip);
   ProblemData* problemData = GetProblemData(probdata);

   Params *params = GetParams(probdata);

   SCIP_CONS **conss = SCIPprobdataGetCons(probdata);


   for( int i = 0; i < problemData->NbVehicles(); ++i )
   {
      SCIP_CONS* cons = conss[i];

       if( !SCIPconsIsActive(cons) )
         continue;

      unsigned int success;
      SCIP_VAR** consvars;
      int nconsvars;

      /* getting the variables that are in the constraint */
      SCIP_CALL( SCIPgetConsNVars(scip, cons, &nconsvars, &success) );
      SCIP_CALL( SCIPallocBufferArray(scip, &consvars, nconsvars) );

      SCIP_CALL( SCIPgetConsVars(scip, cons, consvars, nconsvars, &success) );

      double c_value = 0.0;
      for(int v = 0; v < nconsvars; v++)
      {
         c_value += var_solutionValue[consvars[v]] * 1.0;
      }

      std::cout << SCIPconsGetName(cons) << ", " << c_value << std::endl;


      assert(success);
   }

    //request constraints
   for( int i = 0; i < problemData->NbRequests(); ++i )
   {
      int con_index = i + problemData->NbVehicles();
      SCIP_CONS* cons = conss[con_index];

      int req_id = problemData->GetRequestByIndex(i)->id;

       if( !SCIPconsIsActive(cons) )
         continue;

      unsigned int success;
      SCIP_VAR** consvars;
      int nconsvars;

      /* getting the variables that are in the constraint */
      SCIP_CALL( SCIPgetConsNVars(scip, cons, &nconsvars, &success) );
      SCIP_CALL( SCIPallocBufferArray(scip, &consvars, nconsvars) );

      SCIP_CALL( SCIPgetConsVars(scip, cons, consvars, nconsvars, &success) );

      double c_value = 0.0;
      for(int v = 0; v < nconsvars; v++)
      {
         SCIP_VAR* var = consvars[v];
         SCIP_VARDATA* t_vardata = SCIPvarGetData(var);
         if(t_vardata == NULL) 
         {
            c_value += var_solutionValue[consvars[v]] * 1.0;
         }
         else
         {
            Route* t_route = SCIPvardataGetRoute(t_vardata);
            assert(t_route != NULL);

            int count = t_route->GetRequestCount(problemData, req_id);

            c_value += var_solutionValue[consvars[v]] * count;
         }
      }

      if(true)
      {
         std::cout << SCIPconsGetName(cons) << ", " << c_value << std::endl;
         std::cout << "-------------" << nconsvars << std::endl;
         // //print vars in cons
         // for(int v = 0; v < nconsvars; v++)
         // {
         //    SCIP_VAR* var = consvars[v];
         //    SCIP_VARDATA* t_vardata = SCIPvarGetData(var);
         //    if(t_vardata == NULL) continue;
         //    Route* t_route = SCIPvardataGetRoute(t_vardata);
         //    assert(t_route != NULL);

         //    std::cout << SCIPvarGetName(var) << " / " << std::endl;
         // }
         std::cout << "-------------" << std::endl;

      }


      assert(success);

   }

   const std::vector<SCIP_CONS*> *edgeBranchingConstraints = GetEdgeBranchingConstraints(probdata);
   const std::vector<pair<int, int>> *constrainedEdges = GetConstrainedEdges(probdata);

   for( int c = 0; c < (*edgeBranchingConstraints).size(); ++c )
   {
      SCIP_CONS* cons = (*edgeBranchingConstraints)[c];

      /* ignore constraints which are not active since these are not laying on the current active path of the search
       * tree
       */
      if( !SCIPconsIsActive(cons) || SCIPconsIsDeleted(cons))
         continue;

      pair<int, int> edge = (*constrainedEdges)[c];

      unsigned int success;
      SCIP_VAR** consvars;
      int nconsvars;

      /* getting the variables that are in the constraint */
      SCIP_CALL( SCIPgetConsNVars(scip, cons, &nconsvars, &success) );
      SCIP_CALL( SCIPallocBufferArray(scip, &consvars, nconsvars) );

      SCIP_CALL( SCIPgetConsVars(scip, cons, consvars, nconsvars, &success) );

      double c_value = 0.0;
      for(int v = 0; v < nconsvars; v++)
      {
         SCIP_VAR* var = consvars[v];
         SCIP_VARDATA* t_vardata = SCIPvarGetData(var);
         if(t_vardata == NULL) 
         {
            std::cout << "???????" << std::endl;
            assert(false);
            c_value += var_solutionValue[consvars[v]] * 1.0;
         }
         else
         {
            Route* t_route = SCIPvardataGetRoute(t_vardata);
            assert(t_route != NULL);

            std::map<pair<int, int>, int> edgeUsage = t_route->GetEdgeUsage(problemData);

            c_value += var_solutionValue[consvars[v]] * edgeUsage[edge];
         }
         
      }

      if(true || c_value > params->RCEpsilon && c_value < 1.0 - params->RCEpsilon)
      {
         std::cout << SCIPconsGetName(cons) << "-" << edge.first << "," << edge.second << ", " << c_value << std::endl;
         std::cout << "-------------" << nconsvars << std::endl;
         //print vars in cons
         // for(int v = 0; v < nconsvars; v++)
         // {
         //    SCIP_VAR* var = consvars[v];
         //    SCIP_VARDATA* t_vardata = SCIPvarGetData(var);
         //    if(t_vardata == NULL) continue;
         //    Route* t_route = SCIPvardataGetRoute(t_vardata);
         //    assert(t_route != NULL);

         //    int count = t_route->GetRequestCount(problemData, req_id);

         //    std::cout << SCIPvarGetName(var) << " / " << count << std::endl;
         // }
         std::cout << "-------------" << std::endl;

      }

      


      assert(success);
      
   }

   return SCIP_OKAY;
}

/**@name Callback methods
 *
 * @{
 */

/** 
 * Branching execution method for fractional LP solutions. This method is called by SCIP when it is needs to perform branching
 * In our implementation, this single function inspects the variables at the current node, decides on the best of the 3 branching rules to apply, 
 * and applies it. If branching options have been exhausted, it reports this to scip
 * 
 * Its definition must be exactly so to dialogue correctly with SCIP. We refer to SCIP's documentation for SCIP_DECL_BRANCHEXECLP
 * 
 * */
//this function has quite a few inefficiencies regarding querying/storing solution values and stuff like this
// however, it really isn't the bottleneck for the entire program, so I'm not too worried about it
static SCIP_RETCODE branchExeclpBranchingRules (SCIP* scip, SCIP_BRANCHRULE* branchrule, SCIP_Bool allowaddcons, SCIP_RESULT* result)
{  /*lint --e{715}*/
   SCIP_PROBDATA* probdata;
   SCIP_VAR** lpcands;
   SCIP_Real* lpcandsfrac;
   int nlpcands;
   SCIP_Real bestValue;
   int* consids;
   int nconsids;

   int v;

   assert(scip != NULL);
   assert(branchrule != NULL);
   assert(strcmp(SCIPbranchruleGetName(branchrule), BRANCHRULE_NAME) == 0);
   assert(result != NULL);

   probdata = SCIPgetProbData(scip);
   assert(probdata != NULL);
   Params *params = GetParams(probdata);
   assert(params != NULL);

   SCIPdebugMsg(scip, "start branching at node %"SCIP_LONGINT_FORMAT", depth %d\n", SCIPgetNNodes(scip), SCIPgetDepth(scip));

   if(!SCIPhasCurrentNodeLP(scip))
      throw std::runtime_error("unexpected LP status at branching"); 
   
   SCIP_COL ** cols;
   int ncols;

   SCIP_CALL(SCIPgetLPColsData(scip,&cols,&ncols));
   
   //map of (active) vars to their solution value
   std::map<SCIP_VAR*, double> var_solutionValue;
   for(int i = 0; i < ncols; i++)
   {
      SCIP_COL *col = cols[i];
      SCIP_VAR* var = SCIPcolGetVar(col);
      double value = SCIPcolGetPrimsol(col);
      var_solutionValue[var] = value;

      assert(SCIPvarIsActive(var));

   }

   
   ProblemData* problemData = GetProblemData(probdata);
   assert(problemData != NULL);

   /* 
      decide which variable and branching rule to branch on 
      select most fractional
   */

  /* 
      branching types:
         0 -> branch on y variable
         1 -> branch on vehicle
         2 -> branch on request
         -1 -> no viable branching strategy found
   */
   int branchingType = -1;
   bestValue = 0.0;
   SCIP_VAR *bestVariable = NULL;   

   //1st: branch on yVars
   int nVars = problemData->NbRequests();
   SCIP_VAR** vars = SCIPprobdataGetVars(probdata);
   
   for( v = 0; v < nVars; ++v )
   {
      assert(vars[v] != NULL);

      SCIP_VAR* var = vars[v];

      bool active = SCIPvarIsActive(var);

      double value = var_solutionValue[var];
      double frac_value = MIN(value, 1 - value);

      if(!SCIPvarIsActive(var)) continue;

      #ifdef _DEBUG
      SCIP_VARDATA* vardata = SCIPvarGetData(var);
      assert(vardata == NULL); //y vars have no vardata
      #endif

      
            
      if(frac_value > bestValue) //eps?
      {
         bestValue = frac_value;
         bestVariable = var;
         branchingType = 0;
      }
      
   }
   
   pair<int, int> chosenEdge = std::make_pair(-1, -1);
   int chosenVehIndex = -1;
   int id1, id2;

   //inspect branching on vehicles and edges
   if(params->useBranchingOnVehicles || params->useBranchingOnEdges)
   {
      //is sum_omega(k) delta_sigma fractional?
      std::vector<double> sumVeh(problemData->NbVehicles(), 0.0);

      int n_vars = SCIPprobdataGetNVars(probdata);
      SCIP_VAR** vars = SCIPprobdataGetVars(probdata);

      std::map<pair<int, int>, double> sum_pairs;

      //y vars dont affect sum_veh
      for( int i = problemData->NbRequests(); i < n_vars; ++i ) //first problemData->NbRequests() vars are y vars. The rest are x vars
      {
         SCIP_VAR *t_var = vars[i];
         assert(t_var != NULL);
         SCIP_VARDATA* t_vardata = SCIPvarGetData(t_var);
         assert(t_vardata != NULL);
         Route* t_route = SCIPvardataGetRoute(t_vardata);
         assert(t_route != NULL);

         assert(!(SCIPvarGetLbLocal(t_var) == SCIPvarGetUbLocal(t_var) && var_solutionValue.find(t_var) == var_solutionValue.end()));
         assert(SCIPvarIsActive(t_var));

         if(var_solutionValue.find(t_var) != var_solutionValue.end())
         {
            double v = var_solutionValue[t_var];
            sumVeh[t_route->veh_index] += v;
         }

         std::map<std::pair<int, int>, int> edgeUsage = t_route->GetEdgeUsage(problemData);

         for(auto const& x : edgeUsage)
         {
            //x.first is the pair. x.second is the count value
            if(sum_pairs.find(x.first) == sum_pairs.end())
            {
               sum_pairs[x.first] = x.second * var_solutionValue[t_var];
            }
            else sum_pairs[x.first] = sum_pairs[x.first] + x.second * var_solutionValue[t_var];
         }        
      }

      double integer_part;
      for(int i = 0; i < sumVeh.size(); i++)
      {
         double v = sumVeh[i];
         double mf = std::modf(v, &integer_part);
         mf = MIN(mf, 1.0 - mf);
         
         if( params->useBranchingOnVehicles && mf > bestValue + params->fractionalityEpsilon && mf > params->fractionalityEpsilon ) //most fractional
         {
            branchingType = 1;
            chosenVehIndex = i;
            bestValue = mf;
         }
      }
         
      for (auto const& x : sum_pairs)
      {
         double v = x.second;
         double mf = std::modf(v, &integer_part);
         mf = MIN(mf, 1.0 - mf);
         if( params->useBranchingOnEdges &&  mf > bestValue + params->fractionalityEpsilon && mf > params->fractionalityEpsilon ) //most fractional
         {
            branchingType = 2;
            chosenEdge = x.first;
            bestValue = mf;
         }
      }
   
   }
   
   // for y variables (or if req pairs couldnt be found): simple branching, create two children, one with y = 0.0 other with y = 1.0
   if(branchingType == -1)
   {
      throw std::runtime_error("failed to find suitable branching");
   }
   else if(branchingType == 1) //branch on vehicle
   {
      //std::cout << "branched type 1" << std::endl;
      assert(params->useBranchingOnVehicles);
      IncrementUsedBranchingRule(probdata, 1);

      //std::cout << "branching on vehicle: " << chosenVehIndex << "repeated? : " << IsVehicleBranchingRepeated(scip, problemData, chosenVehIndex) << std::endl;
      assert(!IsVehicleBranchingRepeated(scip, problemData, chosenVehIndex));

      // if(IsVehicleBranchingRepeated(scip, problemData, chosenVehIndex))
      // {
      //    PrintConstraintValues(scip, var_solutionValue);
      // }

      SCIP_NODE* childSum0;
      SCIP_NODE* childSum1;
      SCIP_CONS* consSum0;
      SCIP_CONS* consSum1;

      /* create the branch-and-bound tree child nodes of the current node */
      SCIP_CALL( SCIPcreateChild(scip, &childSum0, 0.0, SCIPgetLocalTransEstimate(scip)) );
      SCIP_CALL( SCIPcreateChild(scip, &childSum1, 0.0, SCIPgetLocalTransEstimate(scip)) );

      int vehicle_id = chosenVehIndex;
      
    
      SCIPcreateConsSumVehicle(scip, &consSum0, "vehCons0", vehicle_id, 0.0, childSum0, true);
      SCIPcreateConsSumVehicle(scip, &consSum1, "vehCons1", vehicle_id, 1.0, childSum1, true);


      /* add constraints to nodes */
      SCIP_CALL( SCIPaddConsNode(scip, childSum0, consSum0, NULL) );
      SCIP_CALL( SCIPaddConsNode(scip, childSum1, consSum1, NULL) );

      //add constraint to probdata vector:
      AddVehicleBranchingCons(scip, probdata, consSum0, vehicle_id);
      AddVehicleBranchingCons(scip, probdata, consSum1, vehicle_id);

      /* release constraints */
      SCIP_CALL( SCIPreleaseCons(scip, &consSum0) );
      SCIP_CALL( SCIPreleaseCons(scip, &consSum1) );

      *result = SCIP_BRANCHED;
   }
   else if(branchingType == 2)
   {
      //std::cout << "branched type 2" << std::endl;
      assert(params->useBranchingOnEdges);
      IncrementUsedBranchingRule(probdata, 2);

      //std::cout << "branching on pair: " << chosenEdge.first << "," << chosenEdge.second << " repeated? : " << IsEdgeBranchingRepeated(scip, problemData, chosenEdge) << std::endl;

      assert(!IsEdgeBranchingRepeated(scip, problemData, chosenEdge));

      // if(IsEdgeBranchingRepeated(scip, problemData, chosenEdge))
      // {
      //    std::cout << "bestValue:" << bestValue << std::endl;
      //    PrintConstraintValues(scip, var_solutionValue);
      //    exit(1);
      // }

      SCIP_NODE* childSum0;
      SCIP_NODE* childSum1;
      SCIP_CONS* consSum0;
      SCIP_CONS* consSum1;

      /* create the branch-and-bound tree child nodes of the current node */
      SCIP_CALL( SCIPcreateChild(scip, &childSum0, 0.0, SCIPgetLocalTransEstimate(scip)) );
      SCIP_CALL( SCIPcreateChild(scip, &childSum1, 0.0, SCIPgetLocalTransEstimate(scip)) );      
    
      SCIPcreateConsSumEdge(scip, &consSum0, "edgeCons0", chosenEdge, 0.0, childSum0, true);
      SCIPcreateConsSumEdge(scip, &consSum1, "edgeCons1", chosenEdge, 1.0, childSum1, true);

      /* add constraints to nodes */
      SCIP_CALL( SCIPaddConsNode(scip, childSum0, consSum0, NULL) );
      SCIP_CALL( SCIPaddConsNode(scip, childSum1, consSum1, NULL) );

      //add constraint to probdata vector:
      AddEdgeBranchingCons(scip, probdata, consSum0, chosenEdge);
      AddEdgeBranchingCons(scip, probdata, consSum1, chosenEdge);

      /* release constraints */
      SCIP_CALL( SCIPreleaseCons(scip, &consSum0) );
      SCIP_CALL( SCIPreleaseCons(scip, &consSum1) );

      *result = SCIP_BRANCHED;


   }
   else if(branchingType == 0) {

      //std::cout << "branched type 0" << std::endl;
      IncrementUsedBranchingRule(probdata, 0);
      //std::cout << "branching on var: " << SCIPvarGetName(bestVariable) << " repeated? : " << IsSimpleBranchingRepeated(scip, problemData, bestVariable) << std::endl;

      assert(!IsSimpleBranchingRepeated(scip, problemData, bestVariable));

      SCIP_NODE* child0;
      SCIP_NODE* child1;

      /* create the branch-and-bound tree child nodes of the current node */
      SCIP_CALL( SCIPcreateChild(scip, &child0, 0.0, SCIPgetLocalTransEstimate(scip)) );
      SCIP_CALL( SCIPcreateChild(scip, &child1, 0.0, SCIPgetLocalTransEstimate(scip)) );

      //at child0. set var UB to 0.0
      SCIPchgVarUbNode (scip,child0, bestVariable, 0.0);
      
      // at child1, set var LB to 1.0
      SCIPchgVarLbNode (scip,child1, bestVariable, 1.0);
       	

      // double coeff = 1.0;
      // //fix var in children
      // SCIP_CONS* cons0;
      // SCIP_CALL(SCIPcreateConsLinear(scip, &cons0, "cons0",
      //                               1, &bestVariable, &coeff, 0.0, 0.0, TRUE,
      //                               TRUE, TRUE, TRUE, TRUE, 
      //                               TRUE, // <--- 'local' = true -> branching constraint
      //                               FALSE, // <-- 'modifiable' 
      //                               FALSE, FALSE, FALSE));
      // SCIP_CALL( SCIPaddConsNode(scip, child0, cons0, NULL));
      // SCIP_CALL( SCIPreleaseCons(scip, &cons0) );

      // SCIP_CONS* cons1;
      // SCIP_CALL( SCIPcreateConsLinear(scip, &cons1, "cons1",
      //                               1, &bestVariable, &coeff, 1.0, 1.0, TRUE,
      //                               TRUE, TRUE, TRUE, TRUE, 
      //                               TRUE, // <--- 'local' = true -> branching constraint
      //                               FALSE, // ... 
      //                               FALSE, FALSE, FALSE));
      // SCIP_CALL( SCIPaddConsNode(scip, child1, cons1, NULL));
      // SCIP_CALL( SCIPreleaseCons(scip, &cons1) );

      *result = SCIP_BRANCHED;
   }
   else throw std::runtime_error("unexpected value at branching enumeration");
   
   return SCIP_OKAY;
}


/**@} */

/**@name Interface methods
 *
 * @{
 */

SCIP_RETCODE SCIPincludeCustomBranchingRule(
   SCIP*                 scip                /**< SCIP data structure */
   )
{
   SCIP_BRANCHRULEDATA* branchruledata;
   SCIP_BRANCHRULE* branchrule;

   /* optionally create ryan foster branching rule data */
   branchruledata = NULL;
   branchrule = NULL;
   /* include branching rule */
   SCIP_CALL( SCIPincludeBranchruleBasic(scip, &branchrule, BRANCHRULE_NAME, BRANCHRULE_DESC, BRANCHRULE_PRIORITY, BRANCHRULE_MAXDEPTH,
         BRANCHRULE_MAXBOUNDDIST, branchruledata) );
   assert(branchrule != NULL);

   SCIP_CALL( SCIPsetBranchruleExecLp(scip, branchrule, branchExeclpBranchingRules) );

   SCIPdebugMsg(scip, "branching rule added\n");

   return SCIP_OKAY;
}

/**@} */
