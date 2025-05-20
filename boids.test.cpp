#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"

#include "doctest.h"
using namespace project;

TEST_CASE("Checking the lower distance function") {
  boid boid1{0., 0., 1., 1.};
  SUBCASE("second boid way farther than lower distance") {
    boid boid2{10., 10., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.lower_distance(boid1, boid2) == true);
  }
  SUBCASE("second boid very close to lower distance") {
    boid boid2{5., 1., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.lower_distance(boid1, boid2) == true);
  }
  SUBCASE("second boid on lower distance") {
    boid boid2{5., 0., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.lower_distance(boid1, boid2) == false);
  }
  SUBCASE("second boid closer than lower distance") {
    boid boid2{1., 1., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.lower_distance(boid1, boid2) == false);
  }
}

TEST_CASE("Checking the upper distance function") {
  boid boid1{0., 0., 1., 1.};
  SUBCASE("second boid way farther than upper distance") {
    boid boid2{20., 20., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.upper_distance(boid1, boid2) == false);
  }
  SUBCASE("second boid just farther than upper distance") {
    boid boid2{10., 1., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.upper_distance(boid1, boid2) == false);
  }
  SUBCASE("second boid exactly on upper distance") {
    boid boid2{0., 10., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.upper_distance(boid1, boid2) == false);
  }
  SUBCASE("second boid closer than upper distance") {
    boid boid2{4., 3., 1., 1.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.upper_distance(boid1, boid2) == true);
  }
}

TEST_CASE("testing the separation rule function") {
  SUBCASE("two boids with same y position") {
    boid boid1{0., 2., 0., 0.};
    boid boid2{2., 2., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 20., 3., 2., 0., 0.};
    CHECK(flock.separation_rule_y(boid1) == doctest::Approx(0));
    CHECK(flock.separation_rule_x(boid1) == doctest::Approx(-4));
    CHECK(flock.separation_rule_y(boid2) == doctest::Approx(0));
    CHECK(flock.separation_rule_x(boid2) == doctest::Approx(4));
  }

  SUBCASE("two boids with same x position") {
    boid boid1{2., 0., 0., 0.};
    boid boid2{2., 2., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 20., 3., 2., 0., 0.};
    CHECK(flock.separation_rule_y(boid1) == doctest::Approx(-4));
    CHECK(flock.separation_rule_x(boid1) == doctest::Approx(0));
    CHECK(flock.separation_rule_y(boid2) == doctest::Approx(4));
    CHECK(flock.separation_rule_x(boid2) == doctest::Approx(0));
  }

  SUBCASE("two boids with different x and y values") {
    boid boid1{0., 0., 0., 0.};
    boid boid2{4., 3., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 20., 6., 3., 0., 0.};
    CHECK(flock.separation_rule_y(boid1) == doctest::Approx(-9));
    CHECK(flock.separation_rule_x(boid1) == doctest::Approx(-12));
    CHECK(flock.separation_rule_y(boid2) == doctest::Approx(9));
    CHECK(flock.separation_rule_x(boid2) == doctest::Approx(12));
  }

  SUBCASE("two boids exactly on lower distance") {
    boid boid1{0., 0., 0., 0.};
    boid boid2{4., 3., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2};
    boids_flock flock{2, flockvect, 20., 5., 3., 0., 0.};
    CHECK(flock.separation_rule_y(boid1) == doctest::Approx(-9));
    CHECK(flock.separation_rule_x(boid1) == doctest::Approx(-12));
    CHECK(flock.separation_rule_y(boid2) == doctest::Approx(9));
    CHECK(flock.separation_rule_x(boid2) == doctest::Approx(12));
  }
}

TEST_CASE("Testing the alignment rule function") {
  SUBCASE("The boids are still") {
    boid boid1{0., 0., 0., 0.};
    boid boid2{10., 10., 0., 0.};
    boid boid3{14., 3., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2, boid3};
    boids_flock flock{3, flockvect, 20., 2., 0., 0.5, 0.};
    CHECK(flock.alignment_rule_x(boid1) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(boid1) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_x(boid2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(boid2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_x(boid3) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(boid3) == doctest::Approx(0.));
  }
  //AGGIUNGERE MOLTI ALTRI TEST!!!!
}


TEST_CASE("testing the center of mass function") {
  SUBCASE("three boids") {
    boid boid1{1., 1., 0., 0.};
    boid boid2{4., -3., 0., 0.};
    boid boid3{-3., 6., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2, boid3};
    boids_flock flock{3, flockvect, 20., 5., 0., 0., 0.};
    CHECK(flock.center_of_mass_x() == doctest::Approx(0.666).epsilon(0.01));
    CHECK(flock.center_of_mass_y() == doctest::Approx(1.333).epsilon(0.01));
  }

  SUBCASE("five boids") {
    boid boid1{-2., -2., 0., 0.};
    boid boid2{6., -3., 0., 0.};
    boid boid3{-1., 9., 0., 0.};
    boid boid4{-10., 0., 0., 0.};
    boid boid5{2., 4., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2, boid3, boid4, boid5};
    boids_flock flock{5, flockvect, 20., 5., 0., 0., 0.};
    CHECK(flock.center_of_mass_x() == doctest::Approx(-1));
    CHECK(flock.center_of_mass_y() == doctest::Approx(1.6));
  }
    
  SUBCASE("three boids in the same position") {
    boid boid1{1., 1., 0., 0.};
    boid boid2{1., 1., 0., 0.};
    boid boid3{1., 1., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2, boid3};
    boids_flock flock{3, flockvect, 20., 5., 0., 0., 0.};
    CHECK(flock.center_of_mass_x() == doctest::Approx(1));
    CHECK(flock.center_of_mass_y() == doctest::Approx(1));
  }

  SUBCASE("three, widely apart boids") {
    boid boid1{-75., -80., 0., 0.};
    boid boid2{-30., 50., 0., 0.};
    boid boid3{90., 90., 0., 0.};
    std::vector<boid> flockvect = {boid1, boid2, boid3};
    boids_flock flock{3, flockvect, 20., 5., 0., 0., 0.};
    CHECK(flock.center_of_mass_x() == doctest::Approx(-5));
    CHECK(flock.center_of_mass_y() == doctest::Approx(20));
  }

  
}

TEST_CASE("Checking the cohesion rule") {
  //test
}

TEST_CASE("Checking the corner behaviour") {
  SUBCASE("The boid hits the upper/right/lower/left corner") {
    boid boid1{2., 20., -3., 10.};
    boid expected1{2., 20., -3., -10.};
    boid boid2{20., -10., 8., 3.};
    boid expected2{20., -10., -8., 3.};
    boid boid3{-15., -20., 10., -7.};
    boid expected3{-15., -20., 10., 7.};
    boid boid4{-20., 11., -10., -7.};
    boid expected4{-20., 11., 10., -7.};
    std::vector<boid> flockvect = {boid1, boid2, boid3, boid4};
    boids_flock flock{4, flockvect, 20., 5., 0., 0., 0.};
    flock.corner_behaviour();
    const boid& result1 = flock.get_flock()[0];
    const boid& result2 = flock.get_flock()[1];
    const boid& result3 = flock.get_flock()[2];
    const boid& result4 = flock.get_flock()[3];
    CHECK(result1.v_y == expected1.v_y);
    CHECK(result2.v_x == expected2.v_x);
    CHECK(result3.v_y == expected3.v_y);
    CHECK(result4.v_x == expected4.v_x);
  }

  SUBCASE("boids exactly in the corners") {
    boid boid1{20., 20., -3., 10.};
    boid expected1{20., 20., 3., -10.};
    boid boid2{20., -20., 8., 3.};
    boid expected2{20., -20., -8., -3.};
    boid boid3{-20., -20., 10., -7.};
    boid expected3{-20., -20., -10., 7.};
    boid boid4{-20., 20., -10., -7.};
    boid expected4{-20., 20., 10., 7.};
    std::vector<boid> flockvect = {boid1, boid2, boid3, boid4};
    boids_flock flock{4, flockvect, 20., 5., 0., 0., 0.};
    flock.corner_behaviour();
    const boid& result1 = flock.get_flock()[0];
    const boid& result2 = flock.get_flock()[1];
    const boid& result3 = flock.get_flock()[2];
    const boid& result4 = flock.get_flock()[3];
    CHECK(result1.v_x == expected1.v_x);
    CHECK(result1.v_y == expected1.v_y);
    CHECK(result2.v_x == expected2.v_x);
    CHECK(result2.v_y == expected2.v_y);
    CHECK(result3.v_x == expected3.v_x);
    CHECK(result3.v_y == expected3.v_y);
    CHECK(result4.v_x == expected4.v_x);
    CHECK(result4.v_y == expected4.v_y);
  }
}
