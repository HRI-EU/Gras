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

#ifndef GRAS_MONKEYSTRATEGY_H
#define GRAS_MONKEYSTRATEGY_H

#include <ExplorationStrategy.h>

#include <cmath>
#include <iostream>


/* State vector:
 * - one dimension for each grid cell: height of the stack at the cell position
 * - last dimension is position of the monkey
 */
class MonkeyStrategy : public Gras::ExplorationStrategy
{
public:
  MonkeyStrategy(int nCells_, int stackHeight_, int stackPos_) :
    nCells(nCells_), stackHeight(stackHeight_), stackPos(stackPos_)
  {
    std::vector<int> value(nCells+1, 0);
    value[stackPos] = stackHeight;
  }

  double heuristicCost(const std::vector<int>& value) const
  {
    return fabs(value[stackPos] - stackHeight);
  }

  double transitionCost(const std::vector<int>& from, const std::vector<int>& to) const
  {
    for (size_t i=0; i<from.size()-1; ++i)
    {
      if (from[i] != to[i])
      {
        return 0.1;  // Something has moved
      }
    }

    // Nothing was moved
    return 0.0;
  }

  bool goalReached(const std::vector<int>& state) const
  {
    return (state[stackPos] == stackHeight) ? true : false;
  }

  std::vector<std::vector<int>> explore(const std::vector<int>& currentState) const
  {
    std::vector<std::vector<int>> nextStates;

    int monkeyPos = currentState[nCells];

    // Just move: The boxes don't change
    if (monkeyPos==0)   // Only 1 step right when at left boundary
    {
      std::vector<int> newVal = currentState;
      newVal[nCells]++;
      nextStates.push_back(newVal);
    }
    else if (monkeyPos==nCells-1)   // Only 1 step left when at right boundary
    {
      std::vector<int> newVal = currentState;
      newVal[nCells]--;
      nextStates.push_back(newVal);
    }
    else   // 1 step in both directions
    {
      std::vector<int> newVal = currentState;
      newVal[nCells]++;
      nextStates.push_back(newVal);
      newVal = currentState;
      newVal[nCells]--;
      nextStates.push_back(newVal);
    }


    // Push right
    if (monkeyPos < nCells-2)
    {
      int stackHeightRight = currentState[monkeyPos+1];
      if (stackHeightRight>0)
      {
        std::vector<int> newVal = currentState;
        newVal[nCells]++;   // Move monkey right
        newVal[monkeyPos+1]--;   //
        newVal[monkeyPos+2]++;   // Add box on top of next next right state
        nextStates.push_back(newVal);
      }
    }


    // Push left
    if (monkeyPos > 1)
    {
      int stackHeightLeft = currentState[monkeyPos-1];
      if (stackHeightLeft>0)
      {
        std::vector<int> newVal = currentState;
        newVal[nCells]--;   // Move monkey left
        newVal[monkeyPos-1]--;   //
        newVal[monkeyPos-2]++;   // Add box on top of next next left state
        nextStates.push_back(newVal);
      }
    }

    return nextStates;
  }


private:
  int nCells, stackHeight, stackPos;
};

#endif // GRAS_MONKEYSTRATEGY_H
