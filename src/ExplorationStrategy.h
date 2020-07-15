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

#ifndef GRAS_EXPLORATIONSTRATEGY_H
#define GRAS_EXPLORATIONSTRATEGY_H

#include <vector>


namespace Gras
{

/*! \ingroup Gras
 *  \brief The ExplorationStrategy class is a base class for search problems.
 *         It implements problem-specific functions for the search. The
 *         explore() function returns the set of reachible successor states of
 *         any given state, including its g-cost and h-cost (see SearchNode
 *         class for details). The euristic cost needs to be implemented in
 *         the calculateHCost() function. The goalReached() function is
 *         determining the end o the search, it needs to implement the
 *         criterion of when the search has reached the goal. A naive
 *         implementation would be a comparison of the current state to a
 *         goal state vector. However, also other criteria may be implemented
 *         (for instance a subset of the states are matching etc.).
 */
class ExplorationStrategy
{
public:

  virtual ~ExplorationStrategy()
  {
  }

  virtual std::vector<std::vector<int> > explore(const std::vector<int>& state) const = 0;
  virtual double heuristicCost(const std::vector<int>& value) const = 0;
  virtual double transitionCost(const std::vector<int>& from,
                                const std::vector<int>& to) const = 0;
  virtual bool goalReached(const std::vector<int>& state) const = 0;
};

}

#endif // GRAS_EXPLORATIONSTRATEGY_H
