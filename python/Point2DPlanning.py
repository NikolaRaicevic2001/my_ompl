#!/usr/bin/env python

from os.path import abspath, dirname, join
import sys
try:
   from ompl import util as ou
   from ompl import base as ob
   from ompl import geometric as og
except ImportError:
   # if the ompl module is not in the PYTHONPATH assume it is installed in a
   # subdirectory of the parent directory called "py-bindings."
   sys.path.insert(0, '/home/erl-tianyu/Nikola_ws/ros2_ws/src/ompl-1.6.0/py-bindings')
   from ompl import util as ou
   from ompl import base as ob
   from ompl import geometric as og
from functools import partial

class Plane2DEnvironment:
   def __init__(self, ppm_file):
       self.ppm_ = ou.PPM()
       self.ppm_.loadFile(ppm_file)
       space = ob.RealVectorStateSpace()
       space.addDimension(0.0, self.ppm_.getWidth())
       space.addDimension(0.0, self.ppm_.getHeight())
       self.maxWidth_ = self.ppm_.getWidth() - 1
       self.maxHeight_ = self.ppm_.getHeight() - 1
       self.ss_ = og.SimpleSetup(space)

       # set state validity checking for this space
       self.ss_.setStateValidityChecker(ob.StateValidityCheckerFn(
           partial(Plane2DEnvironment.isStateValid, self)))
       space.setup()
       self.ss_.getSpaceInformation().setStateValidityCheckingResolution( \
           1.0 / space.getMaximumExtent())
       #      self.ss_.setPlanner(og.RRTConnect(self.ss_.getSpaceInformation()))

   def plan(self, start_row, start_col, goal_row, goal_col):
       if not self.ss_:
           return False
       start = ob.State(self.ss_.getStateSpace())
       start()[0] = start_row
       start()[1] = start_col
       goal = ob.State(self.ss_.getStateSpace())
       goal()[0] = goal_row
       goal()[1] = goal_col
       self.ss_.setStartAndGoalStates(start, goal)
       # generate a few solutions; all will be added to the goal
       for _ in range(10):
           if self.ss_.getPlanner():
               self.ss_.getPlanner().clear()
           self.ss_.solve()
       ns = self.ss_.getProblemDefinition().getSolutionCount()
       print("Found %d solutions" % ns)
       print("\n%s" % self.ss_.getSolutionPath().printAsMatrix())
       
       if self.ss_.haveSolutionPath():
           self.ss_.simplifySolution()
           p = self.ss_.getSolutionPath()
           ps = og.PathSimplifier(self.ss_.getSpaceInformation())
           ps.simplifyMax(p)
           ps.smoothBSpline(p)

           # print the path to a file
           file = open('path.txt', 'w')
           file.write(self.ss_.getSolutionPath().printAsMatrix())
           file.close()
           return True
       return False

   def recordSolution(self):
       if not self.ss_ or not self.ss_.haveSolutionPath():
           return
       p = self.ss_.getSolutionPath()
       p.interpolate()
       for i in range(p.getStateCount()):
           w = min(self.maxWidth_, int(p.getState(i)[0]))
           h = min(self.maxHeight_, int(p.getState(i)[1]))
           c = self.ppm_.getPixel(h, w)
           c.red = 255
           c.green = 0
           c.blue = 0

   def save(self, filename):
       if not self.ss_:
           return
       self.ppm_.saveFile(filename)

   def isStateValid(self, state):
       w = min(int(state[0]), self.maxWidth_)
       h = min(int(state[1]), self.maxHeight_)

       c = self.ppm_.getPixel(h, w)
       return c.red > 127 and c.green > 127 and c.blue > 127


if __name__ == "__main__":
   fname = join(join(join(dirname(dirname(abspath(__file__))), \
       'python'), 'data'), 'floor.ppm')
   env = Plane2DEnvironment(fname)

   if env.plan(0, 0, 777, 1265):
       env.recordSolution()
       env.save("result_demo.ppm")
