#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"

#include "doctest.h"

TEST_CASE("Checking the lower distance function") {
  project::boid b1{0., 0., 1., 1.};
  SUBCASE("second boid way farther than lower distance") {
    project::boid b2{10., 10., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.get_lower_distance(b1, b2) == true);
  }
  SUBCASE("second boid very close to lower distance") {
    project::boid b2{5., 1., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.get_lower_distance(b1, b2) == true);
  }
  SUBCASE("second boid on lower distance") {
    project::boid b2{5., 0., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.get_lower_distance(b1, b2) == false);
  }
  SUBCASE("second boid closer than lower distance") {
    project::boid b2{1., 1., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 50., 5., 0., 0., 0.};
    CHECK(flock.get_lower_distance(b1, b2) == false);
  }
}

TEST_CASE("Checking the upper distance function") {
  project::boid b1{0., 0., 1., 1.};
  SUBCASE("second boid way farther than upper distance") {
    project::boid b2{20., 20., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.get_upper_distance(b1, b2) == false);
  }
  SUBCASE("second boid just farther than upper distance") {
    project::boid b2{10., 1., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.get_upper_distance(b1, b2) == false);
  }
  SUBCASE("second boid exactly on upper distance") {
    project::boid b2{0., 10., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.get_upper_distance(b1, b2) == false);
  }
  SUBCASE("second boid closer than upper distance") {
    project::boid b2{4., 3., 1., 1.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 10., 2., 0., 0., 0.};
    CHECK(flock.get_upper_distance(b1, b2) == true);
  }
}

TEST_CASE("testing the separation rule function") {
  SUBCASE("two boids with same y position") {
    project::boid b1{0., 2., 0., 0.};
    project::boid b2{2., 2., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 20., 3., 2., 0., 0.};
    CHECK(flock.separation_rule_y(b1) == doctest::Approx(0));
    CHECK(flock.separation_rule_x(b1) == doctest::Approx(-4));
    CHECK(flock.separation_rule_y(b2) == doctest::Approx(0));
    CHECK(flock.separation_rule_x(b2) == doctest::Approx(4));
  }

  SUBCASE("two boids with same x position") {
    project::boid b1{2., 0., 0., 0.};
    project::boid b2{2., 2., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 20., 3., 2., 0., 0.};
    CHECK(flock.separation_rule_y(b1) == doctest::Approx(-4));
    CHECK(flock.separation_rule_x(b1) == doctest::Approx(0));
    CHECK(flock.separation_rule_y(b2) == doctest::Approx(4));
    CHECK(flock.separation_rule_x(b2) == doctest::Approx(0));
  }

  SUBCASE("two boids with different x and y values") {
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{4., 3., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 20., 6., 3., 0., 0.};
    CHECK(flock.separation_rule_y(b1) == doctest::Approx(-9));
    CHECK(flock.separation_rule_x(b1) == doctest::Approx(-12));
    CHECK(flock.separation_rule_y(b2) == doctest::Approx(9));
    CHECK(flock.separation_rule_x(b2) == doctest::Approx(12));
  }

  SUBCASE("two boids exactly on lower distance") {
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{4., 3., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 20., 5., 3., 0., 0.};
    CHECK(flock.separation_rule_y(b1) == doctest::Approx(-9));
    CHECK(flock.separation_rule_x(b1) == doctest::Approx(-12));
    CHECK(flock.separation_rule_y(b2) == doctest::Approx(9));
    CHECK(flock.separation_rule_x(b2) == doctest::Approx(12));
  }
}

TEST_CASE("Testing the alignment rule function") {
  SUBCASE("The boids are still") {
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{10., 10., 0., 0.};
    project::boid b3{14., 3., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2, b3};
    project::boids_flock flock{3, flockvect, 20., 2., 0., 0.5, 0.};
    CHECK(flock.alignment_rule_x(b1) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b1) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_x(b2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_x(b3) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b3) == doctest::Approx(0.));
  }

  SUBCASE("Two boids with same velocities") {
    project::boid b1{0., 0., 5., 10.};
    project::boid b2{10., 0., 5., 10.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 20., 2., 0., 0.5, 0.};
    CHECK(flock.alignment_rule_x(b1) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b1) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_x(b2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b2) == doctest::Approx(0.));
  }
  

  SUBCASE("Two boids, one initially still") {
    project::boid b1{10., 10., -5., -5.};
    project::boid b2{20., 0., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 20., 2., 0., 0.5, 0.};
    CHECK(flock.alignment_rule_x(b1) == doctest::Approx(2.5));
    CHECK(flock.alignment_rule_y(b1) == doctest::Approx(2.5));
    CHECK(flock.alignment_rule_x(b2) == doctest::Approx(-2.5));
    CHECK(flock.alignment_rule_y(b2) == doctest::Approx(-2.5));
  }

  SUBCASE("Two boids with opposite velocities") {
    project::boid b1{10., 0., 0., 10.};
    project::boid b2{0., 0., 0., -10.};
    std::vector<project::boid> flockvect = {b1, b2};
    project::boids_flock flock{2, flockvect, 20., 2., 0., 0.5, 0.};
    CHECK(flock.alignment_rule_x(b1) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b1) == doctest::Approx(-10.));
    CHECK(flock.alignment_rule_x(b2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b2) == doctest::Approx(10.));
  }

  SUBCASE("Three boids, one initially still") {
    project::boid b1{-5.,5.,10.,10.};
    project::boid b2{0., 0., 0., 0.};
    project::boid b3{5.,-5.,-10.,-10.};
    std::vector<project::boid> flockvect = {b1, b2, b3};
    project::boids_flock flock{3, flockvect, 20., 2., 0., 0.5, 0.};
    CHECK(flock.alignment_rule_x(b1) == doctest::Approx(-7.5));
    CHECK(flock.alignment_rule_y(b1) == doctest::Approx(-7.5));
    CHECK(flock.alignment_rule_x(b2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_y(b2) == doctest::Approx(0.));
    CHECK(flock.alignment_rule_x(b3) == doctest::Approx(7.5));
    CHECK(flock.alignment_rule_y(b3) == doctest::Approx(7.5));
  }

  SUBCASE("Three boids, all too far to align") {
    project::boid b1{0.,0.,0.,0.};
    project::boid b2{100.,100.,10.,10.};
    project::boid b3{-100.,-100.,10.,10.};
    std::vector<project::boid> flockvect = {b1,b2,b3};
    project::boids_flock flock{3, flockvect, 25., 2., 0., 0.5, 0.};
    CHECK_THROWS(flock.alignment_rule_x(b1));
    CHECK_THROWS(flock.alignment_rule_y(b1));
    CHECK_THROWS(flock.alignment_rule_x(b2));
    CHECK_THROWS(flock.alignment_rule_y(b2));
    CHECK_THROWS(flock.alignment_rule_x(b3));
    CHECK_THROWS(flock.alignment_rule_y(b3));
  }


  
}


TEST_CASE("testing the center of mass function") {
  SUBCASE("three boids") {
    project::boid b1{1., 1., 0., 0.};
    project::boid b2{4., -3., 0., 0.};
    project::boid b3{-3., 6., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2, b3};
    project::boids_flock flock{3, flockvect, 20., 0.5, 0., 0., 0.};
    CHECK(flock.get_center_of_mass_x_nearby(b1) == doctest::Approx(0.500).epsilon(0.01));
    CHECK(flock.get_center_of_mass_y_nearby(b1) == doctest::Approx(1.500).epsilon(0.01));
  }

  SUBCASE("five boids") {
    project::boid b1{-2., -2., 0., 0.};
    project::boid b2{6., -3., 0., 0.};
    project::boid b3{-1., 9., 0., 0.};
    project::boid b4{-10., 0., 0., 0.};
    project::boid b5{2., 4., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2, b3, b4, b5};
    project::boids_flock flock{5, flockvect, 20., 5., 0., 0., 0.};
    CHECK(flock.get_center_of_mass_x_nearby(b1) == doctest::Approx(-0.750));
    CHECK(flock.get_center_of_mass_y_nearby(b1) == doctest::Approx(2.5));
  }
    
  SUBCASE("three boids in the same position") {
    project::boid b1{1., 1., 0., 0.};
    project::boid b2{1., 1., 0., 0.};
    project::boid b3{1., 1., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2, b3};
    project::boids_flock flock{3, flockvect, 20., 5., 0., 0., 0.};
    CHECK_THROWS(flock.get_center_of_mass_x_nearby(b1));
    CHECK_THROWS(flock.get_center_of_mass_y_nearby(b1));
    CHECK_THROWS(flock.get_center_of_mass_x_nearby(b2));
    CHECK_THROWS(flock.get_center_of_mass_y_nearby(b2));
    CHECK_THROWS(flock.get_center_of_mass_x_nearby(b3));
    CHECK_THROWS(flock.get_center_of_mass_y_nearby(b3));
  }
  

  SUBCASE("three, widely apart boids") {
    project::boid b1{-75., -80., 0., 0.};
    project::boid b2{-30., 50., 0., 0.};
    project::boid b3{90., 90., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2, b3};
    project::boids_flock flock{3, flockvect, 20., 5., 0., 0., 0.};
    CHECK_THROWS(flock.get_center_of_mass_x_nearby(b1));
    CHECK_THROWS(flock.get_center_of_mass_y_nearby(b1));
    CHECK_THROWS(flock.get_center_of_mass_x_nearby(b2));
    CHECK_THROWS(flock.get_center_of_mass_y_nearby(b2));
    CHECK_THROWS(flock.get_center_of_mass_x_nearby(b3));
    CHECK_THROWS(flock.get_center_of_mass_y_nearby(b3));
  }

  
}

TEST_CASE("Checking the cohesion rule") {
  SUBCASE("three boids, all in range") {
    project::boid b1{0.,0.,0.,0.};
    project::boid b2{4.,0.,0.,0.};
    project::boid b3{2.,2.,0.,0.};
    std::vector<project::boid> flockvect = {b1, b2, b3};
    project::boids_flock flock{3, flockvect, 20., 2., 0., 0., 2.};
    CHECK(flock.cohesion_rule_x(b1) == doctest::Approx(6));
    CHECK(flock.cohesion_rule_y(b1) == doctest::Approx(2));
    CHECK(flock.cohesion_rule_x(b2) == doctest::Approx(-6));
    CHECK(flock.cohesion_rule_y(b2) == doctest::Approx(2));
    CHECK(flock.cohesion_rule_x(b3) == doctest::Approx(0));
    CHECK(flock.cohesion_rule_y(b3) == doctest::Approx(-4));
  }

  SUBCASE("four boids in different quadrants") {
    project::boid b1{4.,4.,0.,0.};
    project::boid b2{-4.,4.,0.,0.};
    project::boid b3{-4.,-4.,0.,0.};
    project::boid b4{4.,-4.,0.,0.};
    //vengono tutti periodici ;-;
    std::vector<project::boid> flockvect = {b1, b2, b3, b4};
    project::boids_flock flock{4, flockvect, 21., 2., 0., 0., 2.};
    CHECK(flock.cohesion_rule_x(b1) == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule_y(b1) == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule_x(b2) == doctest::Approx(10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule_y(b2) == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule_x(b3) == doctest::Approx(10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule_y(b3) == doctest::Approx(10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule_x(b4) == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule_y(b4) == doctest::Approx(10.67).epsilon(0.01));
  }
}

TEST_CASE("Checking the corner behaviour") {
  SUBCASE("The boid hits the upper/right/lower/left corner") {
    project::boid b1{2., 20., -3., 10.};
    project::boid expected1{2., 20., -3., -10.};
    project::boid b2{20., -10., 8., 3.};
    project::boid expected2{20., -10., -8., 3.};
    project::boid b3{-15., -20., 10., -7.};
    project::boid expected3{-15., -20., 10., 7.};
    project::boid b4{-20., 11., -10., -7.};
    project::boid expected4{-20., 11., 10., -7.};
    std::vector<project::boid> flockvect = {b1, b2, b3, b4};
    project::boids_flock flock{4, flockvect, 20., 5., 0., 0., 0.};
    flock.corner_behaviour();
    const project::boid& result1 = flock.get_flock()[0];
    const project::boid& result2 = flock.get_flock()[1];
    const project::boid& result3 = flock.get_flock()[2];
    const project::boid& result4 = flock.get_flock()[3];
    CHECK(result1.v_y == expected1.v_y);
    CHECK(result2.v_x == expected2.v_x);
    CHECK(result3.v_y == expected3.v_y);
    CHECK(result4.v_x == expected4.v_x);
  }

  SUBCASE("boids exactly in the corners") {
    project::boid b1{20., 20., -3., 10.};
    project::boid expected1{20., 20., 3., -10.};
    project::boid b2{20., -20., 8., 3.};
    project::boid expected2{20., -20., -8., -3.};
    project::boid b3{-20., -20., 10., -7.};
    project::boid expected3{-20., -20., -10., 7.};
    project::boid b4{-20., 20., -10., -7.};
    project::boid expected4{-20., 20., 10., 7.};
    std::vector<project::boid> flockvect = {b1, b2, b3, b4};
    project::boids_flock flock{4, flockvect, 20., 5., 0., 0., 0.};
    flock.corner_behaviour();
    const project::boid& result1 = flock.get_flock()[0];
    const project::boid& result2 = flock.get_flock()[1];
    const project::boid& result3 = flock.get_flock()[2];
    const project::boid& result4 = flock.get_flock()[3];
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

TEST_CASE("Testing the mean velocity function") {
  //even if it is not possible the boids will have all the same position to make the test writing easier 
  project::boid b1{0., 0., 1., 1.};
  project::boid b2{0., 0., 0.7, 0.9};
  project::boid b3{0., 0., -4.5, 3.6};
  project::boid b4{0., 0., -10., 0.};
  project::boid b5{0., 0., -3., -2.};
  project::boid b6{0., 0., 0.01, -1.};
  project::boid b7{0., 0., 2., 0.};
  project::boid b8{0., 0., 0., 2.};
  std::vector<project::boid> flockvect = {b1, b2, b3, b4, b5, b6, b7, b8};
  project::boids_flock flock{8, flockvect, 20., 5., 0., 0., 0.};
  CHECK(flock.mean_velocity().mean_velocity==doctest::Approx(1.8132));
  CHECK(flock.mean_velocity().theta == doctest::Approx(161.9272826));
}

TEST_CASE("Test the velocity_st_deviation function") {

}

TEST_CASE("Testing the mean_distance function") {
  SUBCASE("four boids"){
    project::boid b1{2., 2., 0., 0.};
    project::boid b2{2., -2., 0., 0.};
    project::boid b3{-2., -2., 0., 0.};
    project::boid b4{-2., 2., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2, b3, b4};
    project::boids_flock flock{4, flockvect, 20., 5., 0., 0., 0.};
    CHECK(flock.mean_distance() == doctest::Approx(4.55228));
  }

  SUBCASE("three boids") {
    project::boid b1{20., 11., 0., 0.};
    project::boid b2{-11., -4., 0., 0.};
    project::boid b3{7., -8., 0., 0.};
    std::vector<project::boid> flockvect = {b1, b2, b3};
    project::boids_flock flock{3, flockvect, 20., 5., 0., 0., 0.};
    CHECK(flock.mean_distance() == doctest::Approx(25.2997));
  }
}

TEST_CASE("Testing the velocities update function (molto sperimentale)") {
  SUBCASE("two boids, very basic scenario"){
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{3., 0., 3., 0.};
    std::vector<project::boid> flockvect = {};
    project::boids_flock flock{2, flockvect, 20., 2., 1., 0.5, 0.5};
    //flock.velocities_update();
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.velocities_update();
    CHECK(flock.get_flock()[0].v_x==doctest::Approx(3));
    CHECK(flock.get_flock()[1].v_x==doctest::Approx(1.5));
  }

  /*
  La funzione sembra andare nel modo giusto, ma è come se le modifiche agli oggetti rimanessero
  "bloccate" nella funzione velocities update.
  ho aggiunto dei cout per vedere prima e dopo la funzione. Finora non ho avuto grandi risultati... 
  */

}