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
        CHECK(flock.upper_distance(boid1,boid2)==false);
    }
    SUBCASE("second boid closer than upper distance") {
        boid boid2{4.,3.,1.,1.};
        std::vector<boid> flockvect={boid1,boid2};
        boids_flock flock{2,flockvect,10.,2.,0.,0.,0.};
        CHECK(flock.upper_distance(boid1,boid2)==true);
    }
}

TEST_CASE("testing the separation rule function") {
    SUBCASE("two boids with same y position") {
    boid boid1{0.,2.,0.,0.};
    boid boid2{2.,2.,0.,0.};
    std::vector<boid> flockvect={boid1,boid2};
    boids_flock flock{2,flockvect,20.,3.,2.,0.,0.};
    CHECK(flock.separation_rule_y(boid1)==doctest::Approx(0));
    CHECK(flock.separation_rule_x(boid1)==doctest::Approx(-4));
    }

    SUBCASE("two boids with same x position") {
    boid boid1{2.,0.,0.,0.};
    boid boid2{2.,2.,0.,0.};
    std::vector<boid> flockvect={boid1,boid2};
    boids_flock flock{2,flockvect,20.,3.,2.,0.,0.};
    CHECK(flock.separation_rule_y(boid1)==doctest::Approx(-4));
    CHECK(flock.separation_rule_x(boid1)==doctest::Approx(0));
    }

    SUBCASE("two boids with both different") {
    boid boid1{0.,0.,0.,0.};
    boid boid2{4.,3.,0.,0.};
    std::vector<boid> flockvect={boid1,boid2};
    boids_flock flock{2,flockvect,20.,6.,3.,0.,0.};
    CHECK(flock.separation_rule_y(boid1)==doctest::Approx(-9));
    CHECK(flock.separation_rule_x(boid1)==doctest::Approx(-12));
    
    }
}