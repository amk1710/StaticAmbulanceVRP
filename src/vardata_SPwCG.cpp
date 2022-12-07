/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/*                  This file is part of the program and library             */
/*         SCIP --- Solving Constraint Integer Programs                      */
/*                                                                           */
/*    Copyright (C) 2002-2021 Konrad-Zuse-Zentrum                            */
/*                            fuer Informationstechnik Berlin                */
/*                                                                           */
/*  SCIP is distributed under the terms of the ZIB Academic License.         */
/*                                                                           */
/*  You should have received a copy of the ZIB Academic License              */
/*  along with SCIP; see the file COPYING. If not visit scipopt.org.         */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/**@file   vardata_binpacking.c
 * @brief  Variable data containing the ids of constraints in which the variable appears
 * @author Timo Berthold
 * @author Stefan Heinz
 *
 * This file implements the handling of the variable data which is attached to each file. See SCIP_VarData and \ref BINPACKING_PRICER.
 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/

#include "ProblemSolution.h"

#include "probdata_SPwCG.h"
#include "vardata_SPwCG.h"

/** Variable data which is attached to all variables.
 *
 *  This variable data is used to store in which constraints this variable appears. Therefore, the variable data
 *  contains the ids of constraints in which the variable is part of. Hence, that data give us a column view.
 */
struct SCIP_VarData
{
   int*                  consids;
   int*                  conscoeffs; //this var's coefficients at each constraint 
   int                   nconsids;
   Route*                 route;
};

/**@name Local methods
 *
 * @{
 */

/** create a vardata */
static
SCIP_RETCODE vardataCreate(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_VARDATA**        vardata,            /**< pointer to vardata */
   int*                  consids,            /**< array of constraints ids */
   int*                  conscoeffs,          /**< array of constraints coefficients */
   int                   nconsids,            /**< number of constraints */
   Route*                route               //** another data structure for this route*/
   )
{
   SCIP_CALL( SCIPallocBlockMemory(scip, vardata) );

   SCIP_CALL( SCIPduplicateBlockMemoryArray(scip, &(*vardata)->consids, consids, nconsids) );
   SCIP_CALL( SCIPduplicateBlockMemoryArray(scip, &(*vardata)->conscoeffs, conscoeffs, nconsids) );
   
   SCIPsortIntInt((*vardata)->consids, (*vardata)->conscoeffs, nconsids);

   (*vardata)->nconsids = nconsids;
   Route *my_route = new Route();
   *my_route = (*route); //copy
   (*vardata)->route = my_route; 

   return SCIP_OKAY;
}

/** frees user data of variable */
static
SCIP_RETCODE vardataDelete(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_VARDATA**        vardata             /**< vardata to delete */
   )
{
   SCIPfreeBlockMemoryArray(scip, &(*vardata)->consids, (*vardata)->nconsids);
   SCIPfreeBlockMemoryArray(scip, &(*vardata)->conscoeffs, (*vardata)->nconsids);
   delete (*vardata)->route;
   SCIPfreeBlockMemory(scip, vardata);


   

   return SCIP_OKAY;
}

/**@} */


/**@name Callback methods
 *
 * @{
 */

/** frees user data of transformed variable (called when the transformed variable is freed) */
static
SCIP_DECL_VARDELTRANS(vardataDelTrans)
{
   SCIP_CALL( vardataDelete(scip, vardata) );

   return SCIP_OKAY;
}/*lint !e715*/

/**@} */


/**@name Interface methods
 *
 * @{
 */

/** create variable data */
SCIP_RETCODE SCIPvardataCreateBinpacking(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_VARDATA**        vardata,            /**< pointer to vardata */
   int*                  consids,            /**< array of constraints ids */
   int*                  conscoeffs,         /**< array of contraint coefficients */
   int                   nconsids,            /**< number of constraints */
   Route*                route               //** another data structure for this route*/
   )
{
   SCIP_CALL( vardataCreate(scip, vardata, consids, conscoeffs, nconsids, route) );

   return SCIP_OKAY;
}

/** get number of constraints */
int SCIPvardataGetNConsids(
   SCIP_VARDATA*         vardata             /**< variable data */
   )
{
   return vardata->nconsids;
}

/** returns route pointer */
Route* SCIPvardataGetRoute(
   SCIP_VARDATA*         vardata             /**< variable data */
   )
{
   return vardata->route;  
}

/** returns sorted constraint id array */
int* SCIPvardataGetConsids(
   SCIP_VARDATA*         vardata             /**< variable data */
   )
{
   /* check if the consids are sorted */
#ifndef NDEBUG
   {
      int i;

      for( i = 1; i < vardata->nconsids; ++i )
         assert( vardata->consids[i-1] < vardata->consids[i]);
   }
#endif

   return vardata->consids;
}

/** returns constraint coeffs */
int* SCIPvardataGetConsCoeffs(
   SCIP_VARDATA*         vardata             /**< variable data */
   )
{

   return vardata->conscoeffs;
}

/** creates variable */
SCIP_RETCODE SCIPcreateVarBinpacking(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_VAR**            var,                /**< pointer to variable object */
   const char*           name,               /**< name of variable, or NULL for automatic name creation */
   SCIP_Real             obj,                /**< objective function value */
   SCIP_Bool             initial,            /**< should var's column be present in the initial root LP? */
   SCIP_Bool             removable,          /**< is var's column removable from the LP (due to aging or cleanup)? */
   bool                  relaxed,            /** should the var be binary (false) or continuous (true) ? */
   SCIP_VARDATA*         vardata             /**< user data for this specific variable */
   )
{
   assert(scip != NULL);
   assert(var != NULL);

   /* create a basic variable object */
   if(relaxed)
   {
      SCIP_CALL( SCIPcreateVarBasic(scip, var, name, 0.0, 1.0, obj, SCIP_VARTYPE_CONTINUOUS) );
   }
   else
   {
      SCIP_CALL( SCIPcreateVarBasic(scip, var, name, 0.0, 1.0, obj, SCIP_VARTYPE_BINARY) );
   }
   assert(*var != NULL);

   SCIP_CALL( SCIPchgVarUbLazy(scip, *var, 1.0) );

   /* set callback functions */
   SCIPvarSetData(*var, vardata);
   SCIPvarSetDeltransData(*var, vardataDelTrans);

   /* set initial and removable flag */
   SCIP_CALL( SCIPvarSetInitial(*var, initial) );
   SCIP_CALL( SCIPvarSetRemovable(*var, removable) );

   SCIPvarMarkDeletable(*var);

   //SCIPdebug( SCIPprintVar(scip, *var, NULL) );

   return SCIP_OKAY;
}

/** prints vardata to file stream */
void SCIPvardataPrint(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_VARDATA*         vardata,            /**< variable data */
   FILE*                 file                /**< the text file to store the information into */
   )
{
   SCIP_PROBDATA* probdata;
   int* ids;
   int i;

   probdata = SCIPgetProbData(scip);
   assert(probdata != NULL);

   ProblemData* problemData = GetProblemData(probdata);

   SCIPinfoMessage(scip, file, "consids = {");

   for( i = 0; i < vardata->nconsids; ++i )
   {
      if(vardata->consids[i] > problemData->NbVehicles())
      {
         const Request* req = problemData->GetRequestByIndex(vardata->consids[i]);
         SCIPinfoMessage(scip, file, "%d->%d", req->id, vardata->consids[i]);
      }
      
      if( i < vardata->nconsids - 1 )
         SCIPinfoMessage(scip, file, ",");
   }

   SCIPinfoMessage(scip, file, "}\n");
}

/**@} */
