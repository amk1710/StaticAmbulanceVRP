/**@file   branching.h
 * @brief  Implementation of Custom Branching Rules
 * @author André Mazal Krauss
 *
 * This file implements branching rules for the problem.

 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/

#pragma once

#include "scip/scip.h"


/** creates the branching rule and includes it in SCIP */
SCIP_RETCODE SCIPincludeCustomBranchingRule(
   SCIP*                 scip                /**< SCIP data structure */
   );

