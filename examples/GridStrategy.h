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

#ifndef RCS_GRIDSTRATEGY_H
#define RCS_GRIDSTRATEGY_H

#include <ExplorationStrategy.h>

#include <cmath>
#include <limits>
#include <algorithm>



namespace Gras
{

/*! \ingroup Gras
 */

class GridStrategy : public ExplorationStrategy
{
public:
  GridStrategy(int xGoal, int yGoal, int xlb=0, int ylb=0, int xub=19, int yub=19)
  {
    x_min = xlb;
    y_min = ylb;
    x_max = xub;
    y_max = yub;
    goal.push_back(xGoal);
    goal.push_back(yGoal);
  }

  bool isObstacle(std::vector<int> value) const
  {
    return (std::find(obstacles.begin(), obstacles.end(), value) != obstacles.end());
  }

  void addObstacle(std::vector<int> newObstacle)
  {
    obstacles.push_back(newObstacle);
  }

  void clearObstacles()
  {
    obstacles.clear();
  }

  /*! \brief Checks if the state (x, y) is valid: each of the neigbouring grid
   *         cells (including the diagonal ones) if they are within the grid,
   *         and they are not an obstacle.
    */
  bool checkState(std::vector<int> value) const
  {
    return ((value[0] >= x_min)
            && (value[0] <= x_max)
            && (value[1] >= y_min)
            && (value[1] <= y_max)
            && (!isObstacle(value)));
  }

  /*! \brief Checks if the state (x, y) is valid, and if so, appends it to
   *         the vector next.
   */
  void checkAndAdd(std::vector<std::vector<int> >& next, int x, int y) const
  {
    std::vector<int> valuesNew;
    valuesNew.push_back(x);
    valuesNew.push_back(y);
    if (checkState(valuesNew))
    {
      next.push_back(valuesNew);
    }
  }

  /*! \brief Returns a vector of successors for a given state. In this simple
   *         example, we check each of the neigbouring grid cells.
   */
  std::vector<std::vector<int> > explore(const std::vector<int>& state) const
  {
    std::vector<std::vector<int> > nextStates;
    int x = state[0];
    int y = state[1];

    checkAndAdd(nextStates, x-1, y);     // left
    checkAndAdd(nextStates, x+1, y);     // right
    checkAndAdd(nextStates, x, y-1);     // down
    checkAndAdd(nextStates, x, y+1);     // up
    checkAndAdd(nextStates, x-1, y-1);   // left down
    checkAndAdd(nextStates, x-1, y+1);   // left up
    checkAndAdd(nextStates, x+1, y-1);   // right down
    checkAndAdd(nextStates, x+1, y+1);   // right up

    return nextStates;
  }

  /*!  \brief Estimated cost to reach the goal from the given state. In order
   *          to lead to an optimal solution for the A* search, this cost
   *          estimate must be:
   *          - admissible: Never over-estimate the true cost
   *          - monotone / consistent: <= heuristic cost from any neighbouring
   *            state to the goal, plus the true cost of reaching the neighbour.
   *          In this simple grid example, we provide two costs: The Euclidean
   *          and the Manhattan distance between two grid cells. The latter
   *          one violates the first condition for diagonal moves, possibly
   *          leading to suboptimal solutions.
   */
  double heuristicCost(const std::vector<int>& values) const
  {
    // Manhattan distance
    // return std::abs(goal[0]-values[0]) + std::abs(goal[1] - values[1]);

    // Euclidean distance
    return std::sqrt(pow(values[0]-goal[0], 2) + pow(values[1]-goal[1], 2));
  }

  /*! \brief The cost corresponds to the distance of the grid centers. For a
   *         diagonal move, it is sqrt(2*edgeLength), otherwise it is the
   *         edge length of the grid cell. We assume it to be 1.
   */
  double transitionCost(const std::vector<int>& from,
                        const std::vector<int>& to) const
  {
    if ((from[0] == to[0]) || (from[1] == to[1]))
    {
      return 1.0;
    }

    return sqrt(2.0);
  }

  /*! \brief Returns true if the goal is reached, false otherwise. In this
   *         example, it is trivial. owever, there may be more complex cases,
   *         where for instance a sub-set of state elements needs to be checked,
   *         or there is a set of goals rather tan a single one.
   */
  bool goalReached(const std::vector<int>& state) const
  {
    return (goal==state) ? true : false;
  }

protected:

  int x_min, y_min, x_max, y_max;
  std::vector<std::vector<int> > obstacles;
  std::vector<int> goal;
};

}

#endif // RCSGRIDSTRATEGY_H
