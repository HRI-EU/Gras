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

#include "SearchAlgorithm.h"

#include <utility>
#include <limits>



namespace Gras
{

std::vector<SearchNode> SearchAlgorithm::explore(const ExplorationStrategy& explorer,
                                                 const SearchNode& state,
                                                 double gCostScaler)
{
  std::vector<std::vector<int> > successors = explorer.explore(state.getValue());

  std::vector<SearchNode> nextNodes;

  for (size_t i = 0; i < successors.size(); ++i)
  {
    double tCost = gCostScaler*explorer.transitionCost(state.getValue(), successors[i]);

    if (tCost < std::numeric_limits<double>::max())
    {
      double gNew = state.getGCost() + tCost;
      nextNodes.push_back(SearchNode(successors[i], gNew, state.getValue()));
    }
  }

  return nextNodes;
}

void SearchAlgorithm::getScalingFactors(Algo algo, double& gScaling, double& hScaling)
{
  switch (algo)
  {
    case Dijkstra:
      gScaling = 1.0;
      hScaling = 0.0;
      break;
    case Greedy:
      gScaling = 0.0;
      hScaling = 1.0;
      break;
    default:
      gScaling = 1.0;
      hScaling = 1.0;
      break;
  }

}

SearchAlgorithm::~SearchAlgorithm()
{
  deleteCallbacks();
}

std::vector<std::vector<int> > SearchAlgorithm::search(const ExplorationStrategy& expl,
                                                       std::vector<int> startState,
                                                       Algo algo)
{
  SearchNode goal;
  SearchNode start(startState);
  start.setGCost(0.0);
  start.setFCost(expl.heuristicCost(startState));
  closedList.clear();
  openList.clear();
  openList.insert(start);
  double gCostScaler, hCostScaler;
  getScalingFactors(algo, gCostScaler, hCostScaler);

  // Continue while there is states to be searched
  while (!openList.empty())
  {
    // Get first element and remove from queue
    SearchNode current = openList.pop();

    // Add to close list
    closedList.insert(current);

    // Check if the current state is the goal state
    if (expl.goalReached(current.getValue()))
    {
      goal = current;
      break;
    }

    // Etract new states with g-cost.
    std::vector<SearchNode> statesToExplore = explore(expl, current, gCostScaler);

    // Loop through all potential new states
    for (size_t i = 0; i < statesToExplore.size(); i++)
    {
      SearchNode nextState = statesToExplore.at(i);

      // Check if already on closed list. If yes, we don't need to consider
      // it again. This is the case for admissable heuristice. For
      // non-admissable heuristics, there might be cases where the state is
      // already contained in the closed set, but the explored next state has
      // a better cost as seen before. In this cases, we would need to add
      // the state once again to the open set. We don't handle this here to not
      // deviate from textbook algorithms, and since for non-admissable
      // heuristics, there is no optimality guarantee anyways.
      if (closedList.find(nextState.getValue()))
      {
        continue;
      }

      // It is not yet in the closed list. We update the f-cost by adding the
      // heuristic cost and add it to the open list. The open list takes care
      // of ignoring it if some better equal state already exists.
      double hCost = hCostScaler*expl.heuristicCost(nextState.getValue());
      nextState.setFCost(nextState.getGCost() + hCost);
      openList.insertIfBetter(nextState);
    }

    // Call all registered callbacks after the open and closed sets have been
    // updated with the new explorations.
    for (size_t i = 0; i < callback.size(); ++i)
    {
      callback[i]->callback();
    }

  }   // while (!openList.empty())

  // Declare the vector to store the shortest path
  std::vector<SearchNode> shortestPath;

  // Start from the end state
  SearchNode currentState = goal;

  // Trace back the solution, until the start state is reached
  // (The start state has no parent)
  while (currentState.valid())
  {
    shortestPath.push_back(currentState);
    currentState = closedList.get(currentState.getParent());
  }

  // Assemble solution path in reverse order: The first state is the goal
  // state, then it propagates backwards to the start.
  std::vector<std::vector<int> > solution;

  for (size_t i = shortestPath.size(); i != 0; i--)
  {
    solution.push_back((shortestPath[i - 1]).getValue());
  }

  return solution;
}

void SearchAlgorithm::printSolution(const ExplorationStrategy& explorer,
                                    const std::vector<std::vector<int> >& solutionPath,
                                    Algo algo)
{
  double gCostScaler, hCostScaler;
  getScalingFactors(algo, gCostScaler, hCostScaler);

  std::cout << std::endl << "Solution path has length "
            << solutionPath.size() << ":" << std::endl;

  double gCostIntegral = 0.0;

  for (size_t i = 0; i < solutionPath.size(); ++i)
  {
    SearchNode node_i(solutionPath[i]);
    node_i.setGCost(gCostIntegral);
    node_i.setFCost(gCostIntegral + hCostScaler*explorer.heuristicCost(solutionPath[i]));

    std::cout << node_i << std::endl;

    if (i < solutionPath.size() - 1)
    {
      //gCostIntegral += gCostScaler * explorer.transitionCost(solutionPath[i],
      //  solutionPath[i + 1]);
      gCostIntegral += explorer.transitionCost(solutionPath[i],
                                               solutionPath[i + 1]);
    }

  }

}


}   // namespace Gras
