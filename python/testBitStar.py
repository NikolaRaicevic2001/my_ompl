import math
# ompl
import sys
sys.path.insert(0, '/home/erl-tianyu/Nikola_ws/ros2_ws/src/ompl-1.6.0/py-bindings')
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from functools import partial


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
       return math.sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) + (z-0.5)*(z-0.5)) - 0.25


if __name__ == "__main__":
  space = ob.RealVectorStateSpace(3)
  space.setBounds(0.0, 1.0)
  si = ob.SpaceInformation(space)
  
  validityChecker = ValidityChecker(si)
  si.setStateValidityChecker(validityChecker)
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
  pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
  planner = og.BITstar(si)
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
    

