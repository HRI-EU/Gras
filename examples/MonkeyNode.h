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

#ifndef RCS_MONKEYNODE_H
#define RCS_MONKEYNODE_H

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgGA/GUIEventHandler>


/*******************************************************************************
 * Handle some keys etc.
 ******************************************************************************/
namespace
{










  class BoxNode : public osg::PositionAttitudeTransform
  {

  public:

    BoxNode(osg::Vec3 center, osg::Vec3 extents, osg::Vec4 color)
    {
      this->box = new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f),
                               extents[0], extents[1], extents[2]);
      osg::ref_ptr <osg::ShapeDrawable> shape = new osg::ShapeDrawable(box.get());
      shape->setColor(color);
      osg::ref_ptr<osg::Geode> geode = new osg::Geode();
      geode->addDrawable(shape.get());
      addChild(geode.get());
      setPosition(center);
    }

  protected:

    osg::ref_ptr<osg::Box> box;
  };







  class MonkeyNode: public osg::Group
  {

  public:

    MonkeyNode(std::vector<std::vector<int>> solution)
    {
      unsigned int nBoxes = 0;
      std::vector<int> state = solution[0];

      for (size_t i=0; i<state.size()-1; ++i)
      {
        nBoxes += state[i];
        osg::Vec3 center(0.1f*i, 0.0f, -0.06f);
        osg::Vec3 extents(0.09f, 0.25f, 0.01f);
        osg::Vec4 green(0.0f, 1.0f, 0.0f, 1.0f);
        osg::ref_ptr<BoxNode> tmp = new BoxNode(center, extents, green);
        addChild(tmp.get());
      }

      for (unsigned int i=0; i<nBoxes; ++i)
      {
        osg::Vec3 center;
        osg::Vec3 extents(0.09f, 0.09f, 0.1f);
        osg::Vec4 red(1.0f, 0.0f, 0.0f, 1.0f);
        osg::ref_ptr<BoxNode> tmp = new BoxNode(center, extents, red);
        addChild(tmp.get());
        box.push_back(tmp);
      }

      osg::Vec3 center;
      osg::Vec3 extents(0.07f, 0.07f, 0.07f);
      osg::Vec4 yellow(1.0, 1.0, 0.0, 1.0f);
      monkey = new BoxNode(center, extents, yellow);
      addChild(monkey.get());

      update(state);

      setEventCallback(new MonkeyEventHandler(this, solution));
    }

    void update(std::vector<int> state)
    {
      // Display monkey
      monkey->setPosition(osg::Vec3(0.1f*state.back(), 0.1f, 0.0f));

      // Display stack
      unsigned int boxIdx = 0;
      for (size_t i=0; i<state.size()-1; ++i)
      {
        for (int j=0; j<state[i]; ++j)
        {
          box[boxIdx]->setPosition(osg::Vec3(0.1*i, 0, 0.1*j));
          boxIdx++;
        }

      }
    }

    std::vector<osg::ref_ptr<BoxNode>> box;
    osg::ref_ptr<BoxNode> monkey;
    class MonkeyEventHandler : public osgGA::GUIEventHandler
    {
    public:

    MonkeyEventHandler(MonkeyNode* node, std::vector<std::vector<int>> sln) :
      monkeyNode(node), solution(sln), solutionIdx(0)
      {
      }

      virtual bool handle(const osgGA::GUIEventAdapter& ea,
                          osgGA::GUIActionAdapter& aa)
      {

        switch (ea.getEventType())
        {

          case (osgGA::GUIEventAdapter::FRAME):
          {
            break;
          }

          case (osgGA::GUIEventAdapter::KEYDOWN):
          {
            if (ea.getKey() == '+')
            {
              solutionIdx++;
              if (solutionIdx>(int)solution.size()-1)
              {
                solutionIdx = (int)solution.size()-1;
              }
              monkeyNode->update(solution[solutionIdx]);
            }
            else if (ea.getKey() == '-')
            {
              solutionIdx--;
              if (solutionIdx<0)
              {
                solutionIdx = 0;
              }
              monkeyNode->update(solution[solutionIdx]);
            }

            std::cout << "Solution index: " << solutionIdx << " of "
                      << solution.size() << std::endl;

            break;
          }

          default:
            break;

        }   // switch(...)


        return false;
      }


    protected:
      virtual ~MonkeyEventHandler()
      {
      }

      MonkeyNode* monkeyNode;
      std::vector<std::vector<int>> solution;
      int solutionIdx;
    };

  };

}   // anonymous namespace

#endif // RCS_MONKEYNODE_H
