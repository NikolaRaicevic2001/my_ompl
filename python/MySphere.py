#!/usr/bin/env python

import numpy as np
import vtkplotter as vplt; vplt.embedWindow(False)

# ompl
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from functools import partial


class MyEnv(object):
  def __init__(self):
    self.obstacle = vplt.Sphere(pos=(0.5, 0.5, 0.5),r=0.25)
  def isStateValid(self, state):
    return (not self.obstacle.isInside([state[0],state[1],state[2]]))
  def show(self):
    cam = vplt.Arrows(np.zeros((4,3)),np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0],[1.0,0,0]]),c=np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0],[1.0,0,0]]))
    vplt.show( self.obstacle, cam, interactive=1 )


class ValidityChecker(ob.StateValidityChecker):
   # Returns whether the given state's position overlaps the
   # circular obstacle
   def isValid(self, state):
       return self.clearance(state) > 0.0

   # Returns the distance from the given state's position to the
   # boundary of the circular obstacle.
   def clearance(self, state):
       # Extract the robot's (x,y) position from its state
       x = state[0]
       y = state[1]
       z = state[2]

       # Distance formula between two points, offset by the circle's
       # radius
       return np.sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) + z*z) - 0.25
       

def getPathLengthObjective(si):
   return ob.PathLengthOptimizationObjective(si)


def getThresholdPathLengthObj(si):
   obj = ob.PathLengthOptimizationObjective(si)
   obj.setCostThreshold(ob.Cost(1.51))
   return obj


class ClearanceObjective(ob.StateCostIntegralObjective):
   def __init__(self, si):
       super(ClearanceObjective, self).__init__(si, True)
       self.si_ = si

   # Our requirement is to maximize path clearance from obstacles,
   # but we want to represent the objective as a path cost
   # minimization. Therefore, we set each state's cost to be the
   # reciprocal of its clearance, so that as state clearance
   # increases, the state cost decreases.
   def stateCost(self, s):
       return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
           sys.float_info.min))


def getClearanceObjective(si):
   return ClearanceObjective(si)


def getBalancedObjective1(si):
   lengthObj = ob.PathLengthOptimizationObjective(si)
   clearObj = ClearanceObjective(si)

   opt = ob.MultiOptimizationObjective(si)
   opt.addObjective(lengthObj, 5.0)
   opt.addObjective(clearObj, 1.0)

   return opt


def getPathLengthObjWithCostToGo(si):
   obj = ob.PathLengthOptimizationObjective(si)
   obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
   return obj
   
# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
   if plannerType.lower() == "bfmtstar":
       return og.BFMT(si)
   elif plannerType.lower() == "bitstar":
       return og.BITstar(si)
   elif plannerType.lower() == "fmtstar":
       return og.FMT(si)
   elif plannerType.lower() == "informedrrtstar":
       return og.InformedRRTstar(si)
   elif plannerType.lower() == "prmstar":
       return og.PRMstar(si)
   elif plannerType.lower() == "rrtconnect":
       return og.RRTConnect(si)
   elif plannerType.lower() == "rrtstar":
       return og.RRTstar(si)
   elif plannerType.lower() == "sorrtstar":
       return og.SORRTstar(si)
   else:
       ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
   if objectiveType.lower() == "pathclearance":
       return getClearanceObjective(si)
   elif objectiveType.lower() == "pathlength":
       return getPathLengthObjective(si)
   elif objectiveType.lower() == "thresholdpathlength":
       return getThresholdPathLengthObj(si)
   elif objectiveType.lower() == "weightedlengthandclearancecombo":
       return getBalancedObjective1(si)
   else:
       ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


if __name__ == '__main__':
  
  env = MyEnv()
  space = ob.RealVectorStateSpace(3)
  space.setBounds(0.0, 1.0)
  si = ob.SpaceInformation(space)
  
  #validityChecker = ValidityChecker(si)
  #si.setStateValidityChecker(validityChecker)
  si.setStateValidityChecker(ob.StateValidityCheckerFn(env.isStateValid))
  si.setup()
  
  start = ob.State(space)
  start[0] = 0.1
  start[1] = 0.1
  start[2] = 0.1
  
  goal = ob.State(space)
  goal[0] = 0.9
  goal[1] = 0.9
  goal[2] = 0.9
  
  
  pdef = ob.ProblemDefinition(si)
  pdef.setStartAndGoalStates(start, goal)
  pdef.setOptimizationObjective(allocateObjective(si, "pathlength"))
  planner = allocatePlanner(si, "bitstar")
  planner.setProblemDefinition(pdef)
  planner.setup()
  solved = planner.solve(2.0)
  
  if solved:
    # get the goal representation from the problem definition (not the same as the goal state)
    # and inquire about the found path
    path = pdef.getSolutionPath()
    cost = path.cost(pdef.getOptimizationObjective()).value()
    print("Found solution:\n%s" % path)
    print("Cost: %.4f" % cost)
    
    
    for i in range(path.getStateCount()):
      print([path.getState(i)[0],path.getState(i)[1],path.getState(i)[2]],"\n")
      
      
  else:
    print("No solution found")


  plotter = vplt.Plotter(bg='white')
  env.show()
  
  
