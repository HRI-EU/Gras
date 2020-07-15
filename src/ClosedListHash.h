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

#ifndef GRAS_CLOSEDLISTHASH_H
#define GRAS_CLOSEDLISTHASH_H

#include "SearchNode.h"

#include <unordered_set>

namespace Gras
{

/*! \ingroup Gras
 *  \brief Data structure to store already completely visited (black) nodes
 */
class CloseList
{
public:

  inline void clear()
  {
    closeMap.clear();
  }

  inline void insert(const SearchNode& state)
  {
    closeMap.insert(state);
  }

  inline bool find(const SearchNode& state) const
  {
    auto map_it = closeMap.find(state);

    if (map_it != closeMap.end())
    {
      return true;
    }

    return false;
  }

  inline void remove(const SearchNode& state)
  {
    closeMap.erase(state);
  }

  inline size_t size() const
  {
    return closeMap.size();
  }

  inline bool empty() const
  {
    return closeMap.empty();
  }

  inline SearchNode get(const std::vector<int>& state) const
  {
    auto it = closeMap.find(state);

    if (it != closeMap.end())
    {
      return *it;
    }

    return SearchNode();
  }

  struct StateHash
  {
    // https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
    inline void hash_combine(size_t& seed, const int& v) const
    {
      std::hash<int> hasher;
      seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    size_t operator()(SearchNode const& state) const
    {
      size_t result = 0;
      for (size_t i = 0; i < state.size(); ++i)
      {
        hash_combine(result, state[i]);
      }
      return result;
    }

  };

  struct StateEq
  {
    bool operator()(SearchNode const& lhs, SearchNode const& rhs) const
    {
      return lhs.getValue() == rhs.getValue();
    }
  };


protected:

  std::unordered_set<SearchNode, StateHash, StateEq> closeMap;
};

}

#endif // GRAS_CLOSEDLISTHASH_H
