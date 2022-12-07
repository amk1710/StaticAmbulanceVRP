#pragma once

#include "Params.h"
#include "ProblemSolution.h"

class SCIPSolver
{
private:
    Params *params;
    ProblemData* problemData;
public:
    SCIPSolver(Params *params, ProblemData *problemData) : params(params), problemData(problemData)
    {

    }

    void solve(ProblemSolution& solution);
};