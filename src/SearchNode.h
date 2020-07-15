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

#ifndef GRAS_SEARCHNODE_H
#define GRAS_SEARCHNODE_H

#include <string>
#include <vector>
#include <limits>
#include <sstream>
#include <iomanip>
#include <functional>
#include <iostream>



namespace Gras
{

std::string vecToStr(std::vector<int> vec);

/*! \ingroup Gras
 *
 * \brief A generic class representing an n-dimensional discrete state. It
 *        comprises the vector of discrete state elements and some cost
 *        terms required for the planning algorithms. It should not be
 *        required to expose this to somewhere else than the search algorithm.
 */
class SearchNode
{

  /*! \brief operator << displays the values of the SearchNode in the following
   *          way: ("first discrete value", "second discrete value", ...)
   *
   * \param os
   * \param state the state to display
   * \return ("first discrete value", "second discrete value", ...)
   */
  friend std::ostream& operator<< (std::ostream& os, const SearchNode& state);

public:

  SearchNode() :
    gCost(std::numeric_limits<double>::max()),
    fCost(std::numeric_limits<double>::max())
  {
  }

  /*! \brief SearchNode constructor based only on the discrete values passed as
   *         std vector.
   *
   * \param value the state values
   */
  SearchNode(std::vector<int> value_) :
    value(value_),
    gCost(std::numeric_limits<double>::max()),
    fCost(std::numeric_limits<double>::max())
  {
  }

  /*! \brief SearchNode constructor
   *
   * \param value the discrete state values
   * \param gCost cost of the path from the start state to this state
   * \param hCost estimated cost from this state to the goal state. The
   *              estimation is based on
   * a heuristic function
   * \param p_parent optional pointer to the predecessor state. The default is
   *                 the null pointer.
   * This should be the case of the start state
   */
  SearchNode(std::vector<int> value_, double gCost_,
             std::vector<int> parentState) :
    value(value_),
    parent(parentState),
    gCost(gCost_),
    fCost(std::numeric_limits<double>::max())
  {
  }

  /*! \brief Assigns the value to the state's g-cost.
   */
  inline void setGCost(const double& value)
  {
    this->gCost = value;
  }

  /*! \brief Assigns the value to the state's f-cost.
   */
  inline void setFCost(const double& value)
  {
    this->fCost = value;
  }

  /*! \brief operator == check if two states are the same. Two state are the
   *         same if they discrete values are the same. The costs are not
   *         considered.
   *  \param[in] other the state on right hand side of the == operator
   *  \return True if it is the same state, false otherwise
   */
  inline bool operator == (const SearchNode& other) const
  {
    return (value==other.value) ? true : false;
  }

  /*! \brief Two SearchNodes are not identical if their discrete values are
  *          different. The costs are not considered.
   *  \param[in] other the state on right hand side of the != operator
   *  \return True if it is not the same state, false otherwise
   */
  inline bool operator != (const SearchNode& other) const
  {
    return (value!=other.value) ? true : false;;
  }

  inline bool operator >(const SearchNode& other) const
  {
    if (fCost == other.fCost)
    {
      return gCost > other.gCost;
    }
    return fCost > other.fCost;
  }

  inline bool operator >=(const SearchNode& other) const
  {
    if (fCost == other.fCost)
    {
      return gCost >= other.gCost;
    }
    return fCost >= other.fCost;
  }

  inline bool operator <(const SearchNode& other) const
  {
    if (fCost == other.fCost)
    {
      return gCost < other.gCost;
    }
    return fCost < other.fCost;
  }

  inline bool operator <=(const SearchNode& other) const
  {
    if (fCost == other.fCost)
    {
      return gCost <= other.gCost;
    }
    return fCost <= other.fCost;
  }

  // Needed for ordering in multiset
  bool operator()(const SearchNode& left, const SearchNode& right) const
  {
    return left.less(right);
  }

  // Needed for ordering in multiset
  inline bool less(const SearchNode& other) const
  {
    if (fCost == other.fCost)
    {
      return gCost < other.gCost;
    }

    return fCost < other.fCost;
  }

  inline int operator [](size_t idx) const
  {
    return value[idx];
  }

  inline std::vector<int> getValue() const
  {
    return value;
  }

  inline void setValue(std::vector<int> value_in)
  {
    value = value_in;
  }

  inline int getValue(size_t idx) const
  {
    return value[idx];
  }

  std::vector<int> getParent() const
  {
    return parent;
  }

  inline double getGCost() const
  {
    return gCost;
  }

  inline double getFCost() const
  {
    return fCost;
  }

  inline std::string toString() const
  {
    return vecToStr(value);
  }

  inline bool valid() const
  {
    return !value.empty();
  }

  inline size_t size() const
  {
    return value.size();
  }

protected:

  std::vector<int> value;   // Vector of integers representing the state
  std::vector<int> parent;  // Following parents leads to optimal path
  double gCost; // Accumulated actual cost to reach this state
  double fCost; // gCost + heuristic cost, required for sorting open list
};


std::ostream& operator<< (std::ostream& os, const SearchNode& state);

}
#endif // GRAS_SEARCHNODE_H
