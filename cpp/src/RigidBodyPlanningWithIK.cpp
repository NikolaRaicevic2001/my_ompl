#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/geometric/GeneticSearch.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// describe an arbitrary representation of a goal region in SE(3)
class MyGoalRegion : public ob::GoalRegion
{
public:

   MyGoalRegion(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
   {
       setThreshold(1e-2);
   }

   double distanceGoal(const ob::State *state) const override
   {
       // goal region is given by states where x + y = z and orientation is close to identity
       double d = fabs(state->as<ob::SE3StateSpace::StateType>()->getX()
                       + state->as<ob::SE3StateSpace::StateType>()->getY()
                       - state->as<ob::SE3StateSpace::StateType>()->getZ())
           + fabs(state->as<ob::SE3StateSpace::StateType>()->rotation().w - 1.0);
       return d;
   }

};

// Goal regions such as the one above cannot be sampled, so
// bi-directional trees cannot be used for solving. However, we can
// transform such goal regions into ones that can be sampled. The
// caveat is that it should be possible to find states in this region
// with some other algorithm. Genetic algorithms that essentially
// perform inverse kinematics in the general sense can be used:

bool regionSamplingWithGS(const ob::SpaceInformationPtr &si, const ob::ProblemDefinitionPtr &pd, const ob::GoalRegion *region, const ob::GoalLazySamples *gls, ob::State *result)
{
   og::GeneticSearch g(si);

   // we can use a larger time duration for solve(), but we want to demo the ability
   // of GeneticSearch to continue from where it left off
   bool cont = false;
   for (int i = 0 ; i < 100 ; ++i)
       if (g.solve(0.05, *region, result))
       {
           cont = true;
           break;
       }

   if (cont)
   {
       std::cout << "Found goal state: " << std::endl;
       si->printState(result);
   }

   // we continue sampling while we are able to find solutions, we have found not more than 2 previous solutions and we have not yet solved the problem
   return cont && gls->maxSampleCount() < 3 && !pd->hasSolution();
}

void planWithIK()
{
   // construct the state space we are planning in
   auto space(std::make_shared<ob::SE3StateSpace>());

   // set the bounds for the R^3 part of SE(3)
   ob::RealVectorBounds bounds(3);
   bounds.setLow(-1);
   bounds.setHigh(1);

   space->setBounds(bounds);

   // define a simple setup class
   og::SimpleSetup ss(space);

   // create a random start state
   ob::ScopedState<ob::SE3StateSpace> start(space);
   start->setXYZ(0, 0, 0);
   start->rotation().setIdentity();
   ss.addStartState(start);

   // define our goal region
   MyGoalRegion region(ss.getSpaceInformation());

   // bind a sampling function that fills its argument with a sampled state
   // and returns true while it can produce new samples we don't need to
   // check if new samples are different from ones previously computed as
   // this is pefromed automatically by GoalLazySamples
   ob::GoalSamplingFn samplingFunction = [&ss, &region](const ob::GoalLazySamples *gls, ob::State *result)
       {
           return regionSamplingWithGS(ss.getSpaceInformation(), ss.getProblemDefinition(), &region,
               gls, result);
       };

   // create an instance of GoalLazySamples:
   auto goal(std::make_shared<ob::GoalLazySamples>(ss.getSpaceInformation(), samplingFunction));

   // we set a goal that is sampleable, but it in fact corresponds to a region that is not sampleable by default
   ss.setGoal(goal);

   // attempt to solve the problem
   ob::PlannerStatus solved = ss.solve(3.0);

   if (solved)
   {
       std::cout << "Found solution:" << std::endl;
       // print the path to screen
       ss.simplifySolution();
       ss.getSolutionPath().print(std::cout);
   }
   else
       std::cout << "No solution found" << std::endl;

   // the region variable will now go out of scope. To make sure it is not used in the sampling function any more
   // (i.e., the sampling thread was able to terminate), we make sure sampling has terminated
   goal->as<ob::GoalLazySamples>()->stopSampling();
}

int main(int /*argc*/, char ** /*argv*/)
{
   std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

   planWithIK();

   return 0;
}
