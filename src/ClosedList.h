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

#ifndef GRAS_CLOSEDLIST_H
#define GRAS_CLOSEDLIST_H

#include "SearchNode.h"

#include <map>


namespace Gras
{

/*! \ingroup Gras
 *  \brief Data structure to store already completely visited (black) nodes
 */
class ClosedList
{
public:

  inline void clear()
  {
    closedList.clear();
  }

  inline void insert(const SearchNode& state)
  {
    closedList[state.getValue()] = state;
  }

  inline bool find(const std::vector<int>& state) const
  {
    std::map<std::vector<int>, SearchNode>::const_iterator map_it;
    map_it = closedList.find(state);

    if (map_it != closedList.end())
    {
      return true;
    }

    return false;
  }

  inline void remove(const SearchNode& state)
  {
    closedList.erase(state.getValue());
  }

  inline size_t size() const
  {
    return closedList.size();
  }

  inline bool empty() const
  {
    return closedList.empty();
  }

  inline SearchNode get(const std::vector<int>& state) const
  {
    std::map<std::vector<int>, SearchNode>::const_iterator map_it;
    map_it = closedList.find(state);

    if (map_it != closedList.end())
    {
      return map_it->second;
    }

    return SearchNode();
  }

  inline SearchNode operator[](const size_t& index) const
  {
    SearchNode res;

    if (index < closedList.size())
    {
      std::map<std::vector<int>, SearchNode>::const_iterator q_it;
      q_it = closedList.begin();
      std::advance(q_it, index);
      res = q_it->second;
    }

    return res;
  }



protected:

  std::map<std::vector<int>, SearchNode> closedList;
};

}

#endif // GRAS_CLOSEDLIST_H
