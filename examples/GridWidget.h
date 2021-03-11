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

#ifndef GRAS_GRIDWIDGET_H
#define GRAS_GRIDWIDGET_H

#include <SearchAlgorithm.h>

#include <QWidget>
#include <QLabel>
#include <QPushButton>




namespace Gras
{
enum GridCellType {Start, Goal, Obstacle, Solution, OpenSet, ClosedSet, Free};

class GridLabel : public QLabel
{
  Q_OBJECT
public:
  GridLabel(int row_, int col_, QWidget* parent=Q_NULLPTR,
            Qt::WindowFlags f=Qt::WindowFlags()) :
    QLabel("", parent, f), type(Free), row(row_), col(col_)
  {
    setFixedSize(QSize(10,10));
    setStyleSheet("background-color : white;");
  }

  std::vector<int> getState() const
  {
    std::vector<int> state;
    state.push_back(row);
    state.push_back(col);
    return state;
  }

  GridCellType type;
  int row;
  int col;

public slots:
  void change(GridCellType newType)
  {
    if (type==newType)
    {
      return;
    }

    type = newType;

    switch (type)
    {
      case Start:
        setStyleSheet("QLabel { background-color : magenta; }");
        break;
      case Goal:
        setStyleSheet("QLabel { background-color : green; }");
        break;
      case Obstacle:
        setStyleSheet("QLabel { background-color : red; }");
        break;
      case OpenSet:
        setStyleSheet("QLabel { background-color : gray; }");
        break;
      case ClosedSet:
        setStyleSheet("QLabel { background-color : black; }");
        break;
      case Solution:
        setStyleSheet("QLabel { background-color : blue; }");
        break;
      default:
        setStyleSheet("QLabel { background-color : white; }");
        break;
    }

  }
};

class GridWidget : public QWidget
{
  Q_OBJECT

public:

  GridWidget(size_t rows, size_t columns);
  GridWidget(const QString& fileName);

  virtual ~GridWidget();

  void mousePressEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  GridLabel* getLabel(size_t x, size_t y);

  size_t rows;
  size_t columns;
  bool animate;
  GridCellType cellTypeSelection;
  GridLabel* start;
  GridLabel* goal;
  std::set<GridLabel*> obstacles;
  std::set<GridLabel*> solutionPath;
  std::vector<std::vector<GridLabel*>> labelMatrix;
  QPushButton* bAlgo;
  QPushButton* bAnimate;
  QLabel* msg;
  SearchAlgorithm::Algo algo;

protected slots:
  void changeMode(int);
  void changeAnimate();
  void clear();
  void plan();
  void save();
  void setAStar();
  void setGreedy();
  void setDijkstra();
private:
  void init();
};

}

#endif // GRAS_GRIDWIDGET_H
