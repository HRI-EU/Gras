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

#ifndef GRAS_SEARCHALGORITHM_H
#define GRAS_SEARCHALGORITHM_H

#include "ExplorationStrategy.h"
#include "SearchNode.h"
#include "OpenList.h"
#include "ClosedList.h"

namespace Gras
{

class SearchAlgorithm
{
public:

  enum Algo
  {
    AStar,
    Dijkstra,
    Greedy
  };

  virtual ~SearchAlgorithm();

  /*! \ingroup Gras
   *  \brief A* search algorithm to find the optimal path in a weighted graph.
   *
   *  \param[in] explorer   Exploration strategy class
   *  \param[in] start      State vector comprising starting state
   *  \param[in] algo       Search algorithm type
   *  \return Solution path from start state to goal. States are stored in the
   *          rows of the solution path.
   */
  std::vector<std::vector<int> > search(const ExplorationStrategy& explorer,
                                        std::vector<int> start,
                                        Algo algo=AStar);


  static void printSolution(const ExplorationStrategy& explorer,
                            const std::vector<std::vector<int> >& solutionPath,
                            Algo algo = AStar);

  const OpenList& getOpenList() const
  {
    return openList;
  }

  const ClosedList& getClosedList() const
  {
    return closedList;
  }

  class ExploreCallback
  {
  public:
    virtual ~ExploreCallback()
    {
    }
    virtual void callback() = 0;
  };

  void registerCallback(ExploreCallback* cb)
  {
    callback.push_back(cb);
  }

  void deleteCallbacks()
  {
    for (size_t i=0; i<callback.size(); ++i)
    {
      delete callback[i];
    }

    callback.clear();
  }

  size_t getNumCallbacks() const
  {
    return callback.size();
  }


protected:

  static std::vector<SearchNode> explore(const ExplorationStrategy& explorer,
                                         const SearchNode& state,
                                         double hCostScaler);

  static void getScalingFactors(Algo algo, double& gScaling, double& hScaling);

  OpenList openList;
  ClosedList closedList;
  std::vector<ExploreCallback*> callback;
};

}

#endif // GRAS_SEARCHALGORITHM_H
