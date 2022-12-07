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

/**@file   pricer_binpacking.h
 * @brief  Binpacking variable pricer
 * @author Timo Berthold
 * @author Stefan Heinz
 *
 * This file implements the variable pricer which check if variables exist with negative reduced cost. See for more
 * details \ref BINPACKING_PRICER.
 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/

#pragma once

#include "scip/scip.h"
#include "scip/struct_pricer.h"

#include "Params.h"
#include "BasePricing.h"
#include "ProblemData.h"
#include "ProblemSolution.h"

/*
 * Data structures
 */

/** @brief Variable pricer data used in the \ref pricer_binpacking.c "pricer" */
struct SCIP_PricerData
{
   SCIP_CONSHDLR*        conshdlr;           /**< comstraint handler for "same" and "diff" constraints */
   SCIP_CONS**           conss;              /**< set covering constraints for the items */
   Params*               params;
   ProblemData*          problemData;        /**< general problem info */
   BasePricing*          pricingAlgo;        /** < implementation of pricing algorithm */

   bool heuristicPricing;
   int lastSuccessfullVehicle;
   
   // logging:
   double                total_pricing_time; //in seconds
   int                   total_pricing_calls;
   int                   total_pricing_fails;
   int                   total_pricing_timeouts;
   size_t labelsPriced;
   size_t labelsStored;
   size_t labelsDeleted;
   size_t sumOfMaxLabelsStoredSimultaneously;
   size_t sumOfMostLabelsInRequest;
   size_t sumOfNbConsideredRequests;

};


/** creates the binpacking variable pricer and includes it in SCIP */
SCIP_RETCODE SCIPincludePricerSPwCG(
   SCIP*                 scip                /**< SCIP data structure */
   );

/** added problem specific data to pricer and activates pricer */
SCIP_RETCODE SCIPpricerSPwCGActivate(
   SCIP*                 scip,               /**< SCIP data structure */
   SCIP_CONS**           conss,              /**< set covering constraints for the items */
   Params*               params,
   ProblemData*          problemData
   );

bool createRouteVariable(SCIP* scip, Params* params, SCIP_CONS **all_cons, ProblemData *problemData, Route* route, SCIP_VAR** retVar, bool initial_var, double reducedCost = 0.0);

