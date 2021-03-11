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

#ifndef GRAS_OPENQUEUE_H
#define GRAS_OPENQUEUE_H

#include "SearchNode.h"

#include <queue>
#include <set>
#include <map>
#include <iostream>



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
        os << "[" << i << "]" << list[i];
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
    searchQueue.push(stateToInsert);
    return true;
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
    searchQueue.push(stateToInsert);
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

    SearchNode front = searchQueue.top();
    searchQueue.pop();

    return front;
  }

  /*! \brief Empties all internal data structures. After clear(), the class
   *         instance is like right after construction.
   */
  inline void clear()
  {
    std::cerr << "OpenQueue::clear() not implemented yet" << std::endl;
  }



  /*! \brief operator [] returns the search state that has the position index
   *         in the open list. The list is sorted by increasing f cost.
   *
   *  \param[in] index rank in the queue
   *  \return search state with the rank index
   */
  inline SearchNode operator[](const size_t& index) const
  {
    SearchNode res;

    std::cerr << "OpenQueue::[] not implemented yet" << std::endl;


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

  struct SearchNodeGreater
  {
    bool operator()(const SearchNode& a, const SearchNode& b) const
    {
      return a > b;
    }
  };

  typedef std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeGreater> OpenSetQueue;
  OpenSetQueue searchQueue;
};

std::ostream& operator<< (std::ostream& os, const OpenList& list);

}

#endif // GRAS_OPENQUEUE_H
