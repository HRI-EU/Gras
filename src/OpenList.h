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

#ifndef GRAS_OPENLIST_H
#define GRAS_OPENLIST_H

#include "SearchNode.h"

#include <set>
#include <map>
#include <iostream>
#include <algorithm>



namespace Gras
{

/*! \brief The OpenList class stores search states that are at the threshold
 *         between the explored and the unexplored states. The states are
 *         sorted by increasing f cost
 */
class OpenList
{
  /*! \brief The operator << print all states stored in the open list
   *  \param os    Stream to print output to.
   *  \param list  OpenList to print
   *  \return The ostream reference passed as argument one.
   */
  friend std::ostream& operator<< (std::ostream& os, const OpenList& list)
  {
    if (list.size()==0)
    {
      os << "list is empty!" << std::endl;
    }
    else
    {
      for (size_t i = 0; i < list.size(); i++)
      {
        os << "[" << i << "]" << list[i] << std::endl;
      }

    }
    os << std::endl;
    return os;
  }

public:

  /*! \brief Insert adds a new state to the open list and places it at the
   *         its position in the sorted queue. If a state with the same key
   *         already exists, the state will not be inserted.
   *  \param[in] stateToInsert State to add to the open list
   *  \return False if a state with the same key already exists, true otherwise.
   */
  inline bool insert(const SearchNode& stateToInsert)
  {
    std::vector<int> key = stateToInsert.getValue();
    std::map<std::vector<int>, SearchNode>::iterator it = searchMap.find(key);

    // Check if we already have this state in the search queue
    // (query map for efficiency)
    if (it == searchMap.end())
    {
      // Not found -> just add to search queue
      searchQueue.insert(stateToInsert);
      searchMap[key] = stateToInsert;
      return true;
    }

    return false;
  }

  /*! \brief Insert adds a new state to the open list and places it at the
   *         its position in the sorted queue. If a state with the same key
   *         already exists, the state will only be inserted if its f-cost
   *         (see SearchNode for details) is less than the f-cost of the
   *         state that is already in the list.
   *  \param[in] stateToInsert State to add to the open list
   */
  void insertIfBetter(const SearchNode& stateToInsert)
  {
    std::vector<int> key = stateToInsert.getValue();
    std::map<std::vector<int>, SearchNode>::iterator it = searchMap.find(key);

    // Check if we already have this state in the search queue. If it isn't,
    // it is added and we are done.
    if (it == searchMap.end())
    {
      searchQueue.insert(stateToInsert);
      searchMap[key] = stateToInsert;
      return;
    }

    // State is already in the search queue: compare cost to see if we
    // need to update. If costs are equal or higher, we are done.
    SearchNode existing = it->second;

    if (stateToInsert >= existing)
    {
      return;
    }

    // new cost is lower than previous cost, remove and reinsert with new
    // cost and parent
    std::pair<std::multiset<SearchNode, SearchNode>::iterator,
        std::multiset<SearchNode, SearchNode>::iterator> ret;
    ret = searchQueue.equal_range(existing);
    std::multiset<SearchNode, SearchNode>::iterator it2;
    it2 = std::find(ret.first, ret.second, existing);
    searchQueue.erase(it2);
    searchQueue.insert(stateToInsert);
    searchMap[key] = stateToInsert;
  }

  /*! \brief Deletes the search node with the least f cost from the open list
   *         and returns it.
   *  \return Search state with the least f cost
   */
  inline SearchNode pop()
  {
    if (searchQueue.empty())
    {
      return SearchNode();
    }

    SearchNode front = *searchQueue.begin();
    searchQueue.erase(searchQueue.begin());
    searchMap.erase(front.getValue());

    return front;
  }

  /*! \brief Empties all internal data structures. After clear(), the class
   *         instance is like right after construction.
   */
  inline void clear()
  {
    searchQueue.clear();
    searchMap.clear();
  }

  /*! \brief Retrieves a SearchNode with the given key from the open list.
   *  \return SearchNode with given key, or empty SearchNode if key was
   *          not found.
   */
  inline SearchNode find(std::vector<int> key) const
  {
    std::map<std::vector<int>, SearchNode>::const_iterator map_it ;
    map_it = searchMap.find(key);

    // check if we already have this state in the search queue (query map
    // for efficiency)
    if (map_it != searchMap.end())
    {
      return map_it->second;
    }

    return SearchNode();
  }

  inline bool contains(const std::vector<int>& key) const
  {
    std::map<std::vector<int>, SearchNode>::const_iterator it;
    it = searchMap.find(key);
    return (it != searchMap.end()) ? true : false;
  }

  /*! \brief Retrieves the SearchNode with the given key, and removes it from
   *         the list (if it was found).
   *  \param[in] key vector of integers.
   */
  void remove(std::vector<int> key)
  {
    std::map<std::vector<int>, SearchNode>::iterator map_it;
    map_it = searchMap.find(key);

    // Check if element exists for key. If it doesn't, we are done.
    if (map_it == searchMap.end())
    {
      return;
    }

    SearchNode existing = map_it->second;
    std::pair<std::multiset<SearchNode, SearchNode>::iterator,
        std::multiset<SearchNode, SearchNode>::iterator> ret;
    ret = searchQueue.equal_range(existing);

    if ((ret.first != searchQueue.end()) && (ret.first != ret.second))
    {
      // find the right one in case of several equal scores
      std::multiset<SearchNode, SearchNode>::iterator q_it = ret.first;

      while (q_it != ret.second)
      {
        std::vector<int> key2 = q_it->getValue();
        if (key == key2)
        {
          break;
        }
        q_it++;
      }

      // just double check to be sure
      if (q_it != searchQueue.end())
      {
        // erase it
        searchQueue.erase(q_it);
      }
    }

    searchMap.erase(map_it);
  }

  /*! \brief operator [] returns the search state that has the position
   *         index in the open list. The list is sorted by increasing
   *         f cost.
   *
   *  \param index rank in the queue
   *  \return search state with the rank index
   */
  inline SearchNode operator[](const size_t& index) const
  {
    SearchNode res;

    if (index < searchQueue.size())
    {
      std::multiset<SearchNode, SearchNode>::const_iterator q_it;
      q_it = searchQueue.begin();
      std::advance(q_it, index);
      res = *q_it;
    }

    return res;
  }

  /*! \brief Size a method to get the number of states in the open list
   *  \return The size of the open list
   */
  inline size_t size() const
  {
    return searchQueue.size();
  }

  /*! \brief Check if list is empty.
   *  \return True for empty, false otherwise.
   */
  inline bool empty() const
  {
    return (size()==0);
  }

protected:
  /*!
   * \brief priorityQueue queue of search state sorted by increasing f cost
   */
  std::multiset<SearchNode, SearchNode> searchQueue;
  std::map<std::vector<int>, SearchNode> searchMap;
};

std::ostream& operator<< (std::ostream& os, const OpenList& list);

}

#endif // GRAS_OPENLIST_H
