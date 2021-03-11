/*******************************************************************************

  Copyright (c) 2020, Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef GRAS_ASTAR_H
#define GRAS_ASTAR_H

#include "ExplorationStrategy.h"
#include "SearchNode.h"  // \todo: Should be removed (only exists because of vec2Str()

namespace Gras
{

class Astar
{
public:

  /*! \ingroup Gras
   *  \brief A* search algorithm to find the optimal path in a graph.
   *
   *  \param[in] expl       Exploration strategy class
   *  \param[in] start      State vector comprising starting state
   *  \return Solution path from start state to goal. States are stored in the
   *          rows of the solution path.
   */
  static std::vector<std::vector<int> > search(const ExplorationStrategy& expl,
                                               std::vector<int> start);


  static void printSolution(const ExplorationStrategy& explorer,
                            const std::vector<std::vector<int> >& solutionPath);
};

}

#endif // GRAS_ASTAR_H
