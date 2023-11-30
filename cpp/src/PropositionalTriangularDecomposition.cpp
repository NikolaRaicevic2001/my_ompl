#include "ompl/extensions/triangle/PropositionalTriangularDecomposition.h"
#include "ompl/extensions/triangle/TriangularDecomposition.h"
#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <ostream>
#include <set>
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    /* PropositionalTriangularDecomposition creates a WrapperDecomposition under-the-hood
       (which represents a triangulation over the given bounds that uses
       PropositionalTriangularDecomposition's user-defined project and sample methods)
       and hands it upward to the PropositionalDecomposition superclass constructor. */
    class WrapperDecomposition : public oc::TriangularDecomposition
    {
    public:
        using Polygon = TriangularDecomposition::Polygon;
        using Vertex = TriangularDecomposition::Vertex;
        WrapperDecomposition(const oc::Decomposition *decomp, const ob::RealVectorBounds &bounds,
                             const std::vector<Polygon> &holes, const std::vector<Polygon> &props);
        ~WrapperDecomposition() override = default;
        void project(const ob::State *s, std::vector<double> &coord) const override;
        void sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const override;
        void sampleFullState(const ob::StateSamplerPtr &sampler, const std::vector<double> &coord,
                             ob::State *s) const override;

    protected:
        const oc::Decomposition *decomp_;
    };
}

oc::PropositionalTriangularDecomposition::PropositionalTriangularDecomposition(const ob::RealVectorBounds &bounds,
                                                                               const std::vector<Polygon> &holes,
                                                                               const std::vector<Polygon> &props)
  : PropositionalDecomposition(std::make_shared<WrapperDecomposition>(this, bounds, holes, props))
  , triDecomp_(static_cast<TriangularDecomposition *>(decomp_.get()))
{
}

int oc::PropositionalTriangularDecomposition::getNumProps() const
{
    return triDecomp_->getNumRegionsOfInterest();
}

oc::World oc::PropositionalTriangularDecomposition::worldAtRegion(int triID)
{
    int numProps = getNumProps();
    World world(numProps);
    for (int p = 0; p < numProps; ++p)
        world[p] = false;
    if (triID == -1)
        return world;
    int prop = triDecomp_->getRegionOfInterestAt(triID);
    if (prop >= 0)
        world[prop] = true;
    return world;
}

void oc::PropositionalTriangularDecomposition::setup()
{
    triDecomp_->setup();
}

void oc::PropositionalTriangularDecomposition::addHole(const Polygon &hole)
{
    triDecomp_->addHole(hole);
}

void oc::PropositionalTriangularDecomposition::addProposition(const Polygon &prop)
{
    triDecomp_->addRegionOfInterest(prop);
}

const std::vector<oc::PropositionalTriangularDecomposition::Polygon> &
oc::PropositionalTriangularDecomposition::getHoles() const
{
    return triDecomp_->getHoles();
}

const std::vector<oc::PropositionalTriangularDecomposition::Polygon> &
oc::PropositionalTriangularDecomposition::getPropositions() const
{
    return triDecomp_->getAreasOfInterest();
}

void oc::PropositionalTriangularDecomposition::print(std::ostream &out) const
{
    triDecomp_->print(out);
}

namespace
{
    WrapperDecomposition::WrapperDecomposition(const oc::Decomposition *decomp, const ob::RealVectorBounds &bounds,
                                               const std::vector<Polygon> &holes, const std::vector<Polygon> &props)
      : oc::TriangularDecomposition(bounds, holes, props), decomp_(decomp)
    {
    }

    void WrapperDecomposition::project(const ob::State *s, std::vector<double> &coord) const
    {
        decomp_->project(s, coord);
    }

    void WrapperDecomposition::sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const
    {
        decomp_->sampleFromRegion(rid, rng, coord);
    }

    void WrapperDecomposition::sampleFullState(const ob::StateSamplerPtr &sampler, const std::vector<double> &coord,
                                               ob::State *s) const
    {
        decomp_->sampleFullState(sampler, coord, s);
    }
}
