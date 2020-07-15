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

#include "GridStrategy.h"

#include <SearchAlgorithm.h>

#include <algorithm>


/*******************************************************************************
 *
 ******************************************************************************/
static void printGrid(const Gras::OpenList& openSet,
                      const Gras::ClosedList& closedList,
                      const std::vector<int>& start,
                      const std::vector<int>& goal,
                      const std::vector<std::vector<int> >& solutionPath,
                      const Gras::GridStrategy& explorer)
{
  // Define grid bounds
  int llx = start[0];
  int lly = start[1];
  int ulx = start[0];
  int uly = start[1];

  std::cout << "Start is " << start[0] << " " << start[1] << std::endl;
  std::cout << "Goal is " << goal[0] << " " << goal[1] << std::endl;

  std::cout << "Open set has size " << openSet.size() << std::endl;

  for (size_t i = 0; i < openSet.size(); ++i)
  {
    //std::cout << "Open set [" << i << "]: " << openSet[i][0] << " " << openSet[i][1] << std::endl;
    llx = std::min(llx, openSet[i][0]);
    lly = std::min(lly, openSet[i][1]);
    ulx = std::max(ulx, openSet[i][0]);
    uly = std::max(uly, openSet[i][1]);
  }

  std::cout << "Closed set has size " << closedList.size() << std::endl;

  for (size_t i = 0; i < closedList.size(); ++i)
  {
    //std::cout << "Closed set [" << i << "]: " << closedList[i][0] << " " << closedList[i][1] << std::endl;
    llx = std::min(llx, closedList[i][0]);
    lly = std::min(lly, closedList[i][1]);
    ulx = std::max(ulx, closedList[i][0]);
    uly = std::max(uly, closedList[i][1]);
  }

  std::cout << "llx = " << llx << " lly = " << lly
            << " ulx = " << ulx << " uly = " << uly << std::endl;

  for (int y = uly; y >= lly; --y)
  {
    for (int x = llx; x <= ulx; ++x)
    {
      std::vector<int> current(2);
      current[0] = x;
      current[1] = y;

      if (start == current)
      {
        std::cout << "S";
      }
      else if (goal == current)
      {
        std::cout << "G";
      }
      else if (std::find(solutionPath.begin(), solutionPath.end(), current) != solutionPath.end())
      {
        std::cout << "*";
      }
      else if (!explorer.checkState(current))
      {
        std::cout << "x";
      }
      else if (openSet.contains(current))
      {
        std::cout << "o";
      }
      else if (closedList.find(current))
      {
        std::cout << "c";
      }
      else
      {
        std::cout << ".";
      }

    }

    std::cout << std::endl;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  int xStart = 2, yStart = 2;
  int xGoal = 65, yGoal = 15;
  std::string searchAlgo = "AStar";

  Gras::SearchAlgorithm::Algo salgo;

  if (searchAlgo == "AStar")
  {
    salgo = Gras::SearchAlgorithm::AStar;
  }
  else if (searchAlgo == "Dijkstra")
  {
    salgo = Gras::SearchAlgorithm::Dijkstra;
  }
  else if (searchAlgo == "Greedy")
  {
    salgo = Gras::SearchAlgorithm::Greedy;
  }
  else
  {
    std::cout << "Unknown search algorithm: " << searchAlgo << std::endl;
    return -1;
  }

  // Start and goal states
  std::vector<int> start, goal;
  start.push_back(xStart);
  start.push_back(yStart);
  goal.push_back(xGoal);
  goal.push_back(yGoal);

  Gras::GridStrategy explorer(xGoal, yGoal, 0, 0, xGoal+5, yGoal+5);

  for (int i=0; i<xGoal+5; ++i)
  {
    std::vector<int> obstacle(2);
    obstacle[0] = i;
    obstacle[1] = 5;
    explorer.addObstacle(obstacle);
    obstacle[0] = i+1;
    obstacle[1] = 10;
    explorer.addObstacle(obstacle);
  }

  // Start search
  std::vector<std::vector<int> > solution;
  Gras::SearchAlgorithm algo;
  solution = algo.search(explorer, start, salgo);

  if (!solution.empty())
  {
    algo.printSolution(explorer, solution, salgo);
    printGrid(algo.getOpenList(), algo.getClosedList(),
              start, goal, solution, explorer);
  }
  else
  {
    std::cout << "No solution found" << std::endl;
  }

  return algo.getOpenList().size();
}
