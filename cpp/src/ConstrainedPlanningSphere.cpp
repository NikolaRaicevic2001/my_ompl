#include "ConstrainedPlanningCommon.h"

class SphereConstraint : public ob::Constraint
{
public:
    SphereConstraint() : ob::Constraint(3, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        out[0] = x.norm() - 1;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }
};

class SphereProjection : public ob::ProjectionEvaluator
{
public:
    SphereProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void defaultCellSizes() override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();
        projection(0) = atan2(x[1], x[0]);
        projection(1) = acos(x[2]);
    }
};

bool obstacles(const ob::State *state)
{
    auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

    if (-0.80 < x[2] && x[2] < -0.6)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] > 0;
        return false;
    }
    else if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    else if (0.6 < x[2] && x[2] < 0.80)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] < 0;
        return false;
    }

    return true;
}

bool spherePlanningOnce(ConstrainedProblem &cp, enum PLANNER_TYPE planner, bool output)
{
    cp.setPlanner(planner, "sphere");

    // Solve the problem
    ob::PlannerStatus stat = cp.solveOnce(output, "sphere");

    if (output)
    {
        OMPL_INFORM("Dumping problem information to `sphere_info.txt`.");
        std::ofstream infofile("sphere_info.txt");
        infofile << cp.type << std::endl;
        infofile.close();
    }

    cp.atlasStats();

    if (output)
        cp.dumpGraph("sphere");

    return stat;
}

bool spherePlanningBench(ConstrainedProblem &cp, std::vector<enum PLANNER_TYPE> &planners)
{
    cp.setupBenchmark(planners, "sphere");
    cp.runBenchmark();
    return false;
}

bool spherePlanning(bool output, enum SPACE_TYPE space, std::vector<enum PLANNER_TYPE> &planners,
                    struct ConstrainedOptions &c_opt, struct AtlasOptions &a_opt, bool bench)
{
    // Create the ambient space state space for the problem.
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-2);
    bounds.setHigh(2);

    rvss->setBounds(bounds);

    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<SphereConstraint>();

    ConstrainedProblem cp(space, rvss, constraint);
    cp.setConstrainedOptions(c_opt);
    cp.setAtlasOptions(a_opt);

    cp.css->registerProjection("sphere", std::make_shared<SphereProjection>(cp.css));

    Eigen::VectorXd start(3), goal(3);
    start << 0, 0, -1;
    goal << 0, 0, 1;

    cp.setStartAndGoalStates(start, goal);
    cp.ss->setStateValidityChecker(obstacles);

    if (!bench)
        return spherePlanningOnce(cp, planners[0], output);
    else
        return spherePlanningBench(cp, planners);
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text and planning graph in GraphML to "
                  "`sphere_path.txt` and `sphere_graph.graphml` respectively.";
auto bench_msg = "Do benchmarking on provided planner list.";

int main(int argc, char **argv)
{
    bool output, bench;
    enum SPACE_TYPE space = PJ;
    std::vector<enum PLANNER_TYPE> planners = {RRT};

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
    desc.add_options()("bench", po::bool_switch(&bench)->default_value(false), bench_msg);

    addSpaceOption(desc, &space);
    addPlannerOption(desc, &planners);
    addConstrainedOptions(desc, &c_opt);
    addAtlasOptions(desc, &a_opt);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return 1;
    }

    return spherePlanning(output, space, planners, c_opt, a_opt, bench);
}
