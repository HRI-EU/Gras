/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement: This product includes
   software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "GridWidget.h"
#include "GridStrategy.h"

#include <QLayout>
#include <QMouseEvent>
#include <QDebug>
#include <QRadioButton>
#include <QButtonGroup>
#include <QMenu>
#include <QAction>
#include <QCoreApplication>

#include <iostream>
#include <fstream>



namespace Gras
{

class GridCallback : public SearchAlgorithm::ExploreCallback
{
public:
  GridCallback(SearchAlgorithm& planner_,
               GridWidget* grid_) : planner(planner_), grid(grid_)
  {
  }

  virtual ~GridCallback()
  {
  }

  void callback()
  {
    const OpenList& oSet = planner.getOpenList();
    for (size_t i = 0; i < oSet.size(); ++i)
    {
      GridLabel* li = grid->getLabel(oSet[i][0], oSet[i][1]);
      li->change(OpenSet);
    }

    const ClosedList& cSet = planner.getClosedList();
    for (size_t i = 0; i < cSet.size(); ++i)
    {
      GridLabel* li = grid->getLabel(cSet[i][0], cSet[i][1]);
      li->change(ClosedSet);
    }

    // Draw start and goal state, in case they have been overwritten
    grid->start->change(Start);
    grid->goal->change(Goal);

    // Manually trigger the Qt event loop to update the GridWidget. That's
    // really bad and should go into a thread or so.
    QCoreApplication::processEvents();
  }

  SearchAlgorithm& planner;
  GridWidget* grid;
};

GridWidget::GridWidget(size_t rows_, size_t columns_) :
  QWidget(), rows(rows_), columns(columns_), animate(true), cellTypeSelection(Start),
  start(Q_NULLPTR), goal(Q_NULLPTR), bAlgo(Q_NULLPTR),
  algo(SearchAlgorithm::Algo::AStar)
{
  init();
}

GridWidget::GridWidget(const QString& fileName) :
  QWidget(), rows(0), columns(0), animate(true), cellTypeSelection(Start),
  start(Q_NULLPTR), goal(Q_NULLPTR), bAlgo(Q_NULLPTR),
  algo(SearchAlgorithm::Algo::AStar)
{
  std::ifstream fd("astar.txt");

  if (!fd)
  {
    rows = 50;
    columns = 50;
    init();
  }
  else
  {
    std::string tmp;
    fd >> tmp >> rows >> tmp >> columns;

    init();

    int x, y;

    // Start state
    fd >> tmp >> x >> y;
    start = labelMatrix[x][y];
    start->change(Start);

    // Goal state
    fd >> tmp >> x >> y;
    goal = labelMatrix[x][y];
    goal->change(Goal);

    // Obstacles
    size_t nObstacles;
    fd >> tmp >> nObstacles;

    for (size_t i = 0; i < nObstacles; ++i)
    {
      fd >> x >> y;
      GridLabel* obsi = labelMatrix[x][y];
      obstacles.insert(obsi);
      obsi->change(Obstacle);
    }

    fd.close();
  }
}

void GridWidget::init()
{
  // Horizontal box with Start-Goal-Obstacle readio buttons
  QRadioButton* bStart = new QRadioButton("Select Start");
  QRadioButton* bGoal = new QRadioButton("Select Goal");
  QRadioButton* bObstacle = new QRadioButton("Select Obstacles");
  bStart->setChecked(true);

  QHBoxLayout* bbox = new QHBoxLayout;
  bbox->addWidget(bStart);
  bbox->addWidget(bGoal);
  bbox->addWidget(bObstacle);
  bbox->addStretch();

  // Horizontal box with Clear-Plan-Select algorithm
  QPushButton* bClear = new QPushButton("Clear");
  QPushButton* bPlan = new QPushButton("Plan");
  QPushButton* bSave = new QPushButton("Save");
  bAnimate = new QPushButton("Animate");
  bAnimate->setCheckable(true);
  bAnimate->setChecked(true);
  bAnimate->setAutoFillBackground(true);
  bAnimate->setStyleSheet("background-color : yellow; border: 2px solid grey; ");

  QMenu* aMenu = new QMenu();
  this->bAlgo = new QPushButton();
  bAlgo->setMenu(aMenu);
  bAlgo->setText("A-Star");

  QAction* actAStar = new QAction(tr("&AStar..."), this);
  connect(actAStar, &QAction::triggered, this, &GridWidget::setAStar);
  aMenu->addAction(actAStar);
  QAction* actDijkstra = new QAction(tr("&Dijkstra..."), this);
  connect(actDijkstra, &QAction::triggered, this, &GridWidget::setDijkstra);
  aMenu->addAction(actDijkstra);
  QAction* actGreedy = new QAction(tr("&Greedy..."), this);
  connect(actGreedy, &QAction::triggered, this, &GridWidget::setGreedy);
  aMenu->addAction(actGreedy);

  QHBoxLayout* bbox2 = new QHBoxLayout;
  bbox2->addWidget(bClear);
  bbox2->addWidget(bPlan);
  bbox2->addWidget(bAlgo);
  bbox2->addWidget(bSave);
  bbox2->addWidget(bAnimate);
  bbox2->addStretch();

  QButtonGroup* buttonGroup = new QButtonGroup;
  buttonGroup->addButton(bStart, 0);
  buttonGroup->addButton(bGoal, 1);
  buttonGroup->addButton(bObstacle, 2);
  connect(buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(changeMode(int)));
  connect(bAnimate, SIGNAL(clicked()), this, SLOT(changeAnimate()));
  connect(bClear, SIGNAL(clicked()), this, SLOT(clear()));
  connect(bPlan, SIGNAL(clicked()), this, SLOT(plan()));
  connect(bSave, SIGNAL(clicked()), this, SLOT(save()));

  // Text lable for displaying search results
  msg = new QLabel();

  // Search grid
  QHBoxLayout* hgrid = new QHBoxLayout;
  QGridLayout* const grid = new QGridLayout();
  grid->setHorizontalSpacing(0);
  grid->setVerticalSpacing(0);
  for (size_t row = 0; row < rows; ++row)
  {
    std::vector<GridLabel*> labelMatRow;

    for (size_t col = 0; col < columns; ++col)
    {
      GridLabel* label = new GridLabel(row, col);
      grid->addWidget(label, row, col);
      labelMatRow.push_back(label);
    }

    labelMatrix.push_back(labelMatRow);
  }
  hgrid->addLayout(grid);
  hgrid->addStretch();

  // Align the above layouts vertically
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(bbox);
  layout->addLayout(bbox2);
  layout->addWidget(msg);
  layout->addLayout(hgrid);

  setLayout(layout);
}

GridWidget::~GridWidget()
{
}

void GridWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget* const widget = childAt(event->pos());
  if (!widget)
  {
    return;
  }

  GridLabel* label = qobject_cast<GridLabel*>(widget);
  if (!label)
  {
    return;
  }

  switch (cellTypeSelection)
  {
    case Start:
      label->change(cellTypeSelection);
      if (start)
      {
        start->change(Free);
      }
      start = label;
      break;

    case Goal:
      label->change(cellTypeSelection);
      if (goal)
      {
        goal->change(Free);
      }
      goal = label;
      break;

    case Obstacle:
      label->change(cellTypeSelection);
      obstacles.insert(label);
      break;

    default:
      break;
  }

}

void GridWidget::mouseMoveEvent(QMouseEvent* event)
{
  mousePressEvent(event);
}

void GridWidget::changeMode(int buttonId)
{
  switch (buttonId)
  {
    case 0:
      cellTypeSelection = Start;
      break;
    case 1:
      cellTypeSelection = Goal;
      break;
    case 2:
      cellTypeSelection = Obstacle;
      break;
    default:
      cellTypeSelection = Free;
      break;
  }
}

void GridWidget::changeAnimate()
{
  if (bAnimate->isChecked())
  {
    bAnimate->setStyleSheet("background-color : yellow; border: 2px solid grey; ");
  }
  else
  {
    bAnimate->setStyleSheet("background-color : gray; border: 2px solid grey; ");
  }

  animate = bAnimate->isChecked();
  bAnimate->show();
}

void GridWidget::clear()
{
  if (start)
  {
    start->change(Free);
    start = Q_NULLPTR;
  }

  if (goal)
  {
    goal->change(Free);
    goal = Q_NULLPTR;
  }

  obstacles.clear();
  solutionPath.clear();

  // Clear everything else than start, goal and obstacles
  for (size_t row = 0; row < labelMatrix.size(); ++row)
  {
    for (size_t col = 0; col < labelMatrix[row].size(); ++col)
    {
      labelMatrix[row][col]->change(Free);
    }
  }

  msg->setText("");
}

GridLabel* GridWidget::getLabel(size_t x, size_t y)
{
  if ((x >= rows) || (y >= columns))
  {
    qDebug() << "Label index out of range - go fix your code";
    exit(0);
  }

  return labelMatrix[x][y];
}

void GridWidget::plan()
{
  msg->setText("Planning ...");

  if (!start)
  {
    msg->setText("Start state missing - skipping plan");
    return;
  }

  if (!goal)
  {
    msg->setText("Goal state missing - skipping plan");
    return;
  }

  std::vector<int> startState = start->getState();
  std::vector<int> goalState = goal->getState();

  GridStrategy explorer(goalState[0], goalState[1],
                        0, 0, (int)rows-1, (int)columns-1);

  std::set<GridLabel*>::iterator it;

  for (it = obstacles.begin(); it != obstacles.end(); ++it)
  {
    GridLabel* label = *it;
    explorer.addObstacle(label->getState());
  }

  // Clear everything else than start, goal and obstacles
  for (size_t row = 0; row < labelMatrix.size(); ++row)
  {
    for (size_t col = 0; col < labelMatrix[row].size(); ++col)
    {
      GridLabel* li = labelMatrix[row][col];

      if ((li->type!=Start) && (li->type!=Goal) && (li->type!=Obstacle))
      {
        labelMatrix[row][col]->change(Free);
      }
    }
  }

  std::vector<std::vector<int>> solution;
  SearchAlgorithm planner;

  if (!animate)
  {
    planner.deleteCallbacks();
  }
  else
  {
    if (planner.getNumCallbacks()==0)
    {
      planner.registerCallback(new GridCallback(planner, this));
    }
  }

  solution = planner.search(explorer, startState, algo);

  if (solution.empty())
  {
    msg->setText("No solution found");
    return;
  }

  // The planner found a solution, let's clear the previous solution and
  // start drawing.
  solutionPath.clear();

  // Draw the open set
  const OpenList& oSet = planner.getOpenList();
  for (size_t i = 0; i < oSet.size(); ++i)
  {
    GridLabel* li = getLabel(oSet[i][0], oSet[i][1]);
    li->change(OpenSet);
  }

  // Draw the closed set
  const ClosedList& cSet = planner.getClosedList();
  for (size_t i = 0; i < cSet.size(); ++i)
  {
    GridLabel* li = getLabel(cSet[i][0], cSet[i][1]);
    li->change(ClosedSet);
  }

  // Draw the solution path
  for (size_t i = 0; i < solution.size(); ++i)
  {
    GridLabel* li = getLabel(solution[i][0], solution[i][1]);
    solutionPath.insert(li);
    li->change(Solution);
  }

  // Draw start and goal state, in case they have been overwritten
  start->change(Start);
  goal->change(Goal);

  // Reconstruct goal cost using transition cost integral. For the Greedy
  // algorithm, it is not really correct, since the g-cost is zero. We still
  // compute it to be able to compare it against the other algorithms.
  double gCostIntegral = 0.0;
  for (size_t i = 0; i < solution.size()-1; ++i)
  {
    gCostIntegral += explorer.transitionCost(solution[i],
                                             solution[i + 1]);
  }

  char a[256];
  sprintf(a, "Solution with cost %f has %zu steps",
          gCostIntegral, solution.size());
  msg->setText(a);

  // For A-Star and Dijkstra, the integrated cost must correspond to the one
  // stored in the state. If it is not the case, there must be a mistake
  // somewhere and we fatally exit.
  if (algo != SearchAlgorithm::Algo::Greedy)
  {
    SearchNode res = planner.getClosedList().get(solution.back());

    if (fabs(res.getGCost() - gCostIntegral) > 1.0e-8)
    {
      qDebug() << "g-cost mismatch: " << res.getGCost() << " != "
               << gCostIntegral;
      exit(0);
    }
  }

}

void GridWidget::setAStar()
{
  bAlgo->setText("A-Star");
  algo = SearchAlgorithm::Algo::AStar;
}

void GridWidget::setGreedy()
{
  bAlgo->setText("Greedy");
  algo = SearchAlgorithm::Algo::Greedy;
}

void GridWidget::setDijkstra()
{
  bAlgo->setText("Dijkstra");
  algo = SearchAlgorithm::Algo::Dijkstra;
}

void GridWidget::save()
{
  if (start == NULL || goal == NULL)
  {
    qDebug() << "Search not fully defined - not saving";
    return;
  }

  std::ofstream fd;
  fd.open("astar.txt");
  fd << "rows " << rows << " columns " << columns << std::endl;
  fd << "start " << start->row << " " << start->col << std::endl;
  fd << "goal " << goal->row << " " << goal->col << std::endl;
  fd << "obstacles " << obstacles.size() << std::endl;

  std::set<GridLabel*>::iterator it;
  for (it = obstacles.begin(); it != obstacles.end(); ++it)
  {
    GridLabel* label = *it;
    fd << label->row << " " << label->col << std::endl;
  }

  fd.close();
}

}   // namespace Gras
