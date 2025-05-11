#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
using namespace project;

TEST_CASE("Checking the lower distance function") {
    boid boid1{0.,0.,1.,1.};
    SUBCASE("second boid way farther than lower distance") {
        boid boid2{10.,10.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,50.,5.,0.,0.,0.};
        CHECK(flock.lower_distance(boid1,boid2)==true);
    }
    SUBCASE("second boid very close to lower distance") {
        boid boid2{5.,1.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,50.,5.,0.,0.,0.};
        CHECK(flock.lower_distance(boid1,boid2)==true);
    }
    SUBCASE("second boid on lower distance") {
        boid boid2{5.,0.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,50.,5.,0.,0.,0.};
        CHECK(flock.lower_distance(boid1,boid2)==false);
    }
    SUBCASE("second boid closer than lower distance") {
        boid boid2{1.,1.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,50.,5.,0.,0.,0.};
        CHECK(flock.lower_distance(boid1,boid2)==false);
    }
}

TEST_CASE("Checking the upper distance function") {
    boid boid1{0.,0.,1.,1.};
    SUBCASE("second boid way farther than upper distance") {
        boid boid2{20.,20.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,10.,2.,0.,0.,0.};
        CHECK(flock.upper_distance(boid1,boid2)==false);
    }
    SUBCASE("second boid just farther than upper distance") {
        boid boid2{10.,1.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,10.,2.,0.,0.,0.};
        CHECK(flock.upper_distance(boid1,boid2)==false);
    }
    SUBCASE("second boid exactly on upper distance") {
        boid boid2{0.,10.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,10.,2.,0.,0.,0.};
        CHECK(flock.upper_distance(boid1,boid2)==true);
    }
    SUBCASE("second boid closer than upper distance") {
        boid boid2{4.,3.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,10.,2.,0.,0.,0.};
        CHECK(flock.upper_distance(boid1,boid2)==true);
    }

}