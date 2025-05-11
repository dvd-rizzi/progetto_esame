#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
using namespace project;


TEST_CASE("Checking the lower distance function") {
    boid boid1{0.,0.,1.,1.};
    SUBCASE("second boid way farther than lower distance") {
        boid boid2{10.,10.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{ 2,flockvect,50.,5.,0.,0.,0.};

    }
}