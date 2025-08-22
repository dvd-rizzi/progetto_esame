#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"

#include "doctest.h"
TEST_CASE("Test operatore !=") {
  SUBCASE("Boid is the same") {
    project::boid b1{2., 78., 1.9, 9.};
    project::boid b2{2., 78., 1.9, 9.};
    CHECK((b1 != b2) == false);
  }

  SUBCASE("Different Boids") {
    project::boid b1{2., 79., 1.9, 9.};
    project::boid b2{2., 78., 1.9, 9.};
    CHECK((b1 != b2) == true);
  }
}

TEST_CASE("Checking the lower distance function") {
  project::boid b1{0., 0., 1., 1.};
  SUBCASE("second boid way farther than lower distance") {
    project::boid b2{10., 10., 1., 1.};
    project::boids_flock flock{2, 50., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_lower_distance(b1, b2) == true);
  }
  SUBCASE("second boid very close to lower distance") {
    project::boid b2{5., 1., 1., 1.};
    project::boids_flock flock{2, 50., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_lower_distance(b1, b2) == true);
  }
  SUBCASE("second boid on lower distance") {
    project::boid b2{5., 0., 1., 1.};
    project::boids_flock flock{2, 50., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_lower_distance(b1, b2) == false);
  }
  SUBCASE("second boid closer than lower distance") {
    project::boid b2{1., 1., 1., 1.};
    project::boids_flock flock{2, 50., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_lower_distance(b1, b2) == false);
  }
}

TEST_CASE("Checking the upper distance function") {
  project::boid b1{0., 0., 1., 1.};
  SUBCASE("second boid way farther than upper distance") {
    project::boid b2{20., 20., 1., 1.};
    project::boids_flock flock{2, 10., 2., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_upper_distance(b1, b2) == false);
  }
  SUBCASE("second boid just farther than upper distance") {
    project::boid b2{10., 1., 1., 1.};
    project::boids_flock flock{2, 10., 2., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_upper_distance(b1, b2) == false);
  }
  SUBCASE("second boid exactly on upper distance") {
    project::boid b2{0., 10., 1., 1.};
    project::boids_flock flock{2, 10., 2., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_upper_distance(b1, b2) == false);
  }
  SUBCASE("second boid closer than upper distance") {
    project::boid b2{4., 3., 1., 1.};
    project::boids_flock flock{2, 10., 2., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.get_upper_distance(b1, b2) == true);
  }
}

TEST_CASE("Testing the reciprocal_distance functions") {
  project::boid b1{4., 9., 0., 0.};
  project::boid b2{-89., 23., 0., 0.};
  project::boid b3{189., -33., 0., 0.};
  project::boid b4{10., -10., 0., 0.};
  project::boid b5{-99., -39., 0., 0.};
  project::boid b6{0., 0., 0., 0.};
  project::boids_flock flock{6, 15., 3., 3., 0.2, 0.1};
  flock.addBoid(b1);
  flock.addBoid(b2);
  flock.addBoid(b3);
  flock.addBoid(b4);
  flock.addBoid(b5);
  flock.addBoid(b6);
  project::module expected12{-93.,14.};
  project::module expected13{185.,-42.};
  project::module expected14{6.,-19.};
  project::module expected15{-103.,-48.};
  project::module expected16{-4.,-9.};
  project::module expected32{-278.,56.};
  project::module expected62{-89.,23.};

  CHECK(flock.get_reciprocal_distance(b1,b2) == expected12);
  CHECK(flock.get_reciprocal_distance(b1,b3) == expected13);
  CHECK(flock.get_reciprocal_distance(b1,b4) == expected14);
  CHECK(flock.get_reciprocal_distance(b1,b5) == expected15);
  CHECK(flock.get_reciprocal_distance(b1,b6) == expected16);
  CHECK(flock.get_reciprocal_distance(b3,b2) == expected32);
  CHECK(flock.get_reciprocal_distance(b6,b2) == expected62);
}

TEST_CASE("testing the separation rule function") {
  SUBCASE("two boids with same y position") {
    project::boid b1{0., 2., 0., 0.};
    project::boid b2{2., 2., 0., 0.};
    project::module expected1{-4.,0.};
    project::module expected2{4.,0.};
    project::boids_flock flock{2, 20., 3., 2., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.separation_rule(b1) == expected1);
    CHECK(flock.separation_rule(b2) == expected2);
  }

  SUBCASE("two boids with same x position") {
    project::boid b1{2., 0., 0., 0.};
    project::boid b2{2., 2., 0., 0.};
    project::module expected1{0.,-4.};
    project::module expected2{0.,4.};
    project::boids_flock flock{2, 20., 3., 2., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.separation_rule(b1) == expected1);
    CHECK(flock.separation_rule(b2) == expected2);
  }

  SUBCASE("two boids with different x and y values") {
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{4., 3., 0., 0.};
    project::module expected1{-12.,-9.};
    project::module expected2{12.,9.};
    project::boids_flock flock{2, 20., 6., 3., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.separation_rule(b1) == expected1);
    CHECK(flock.separation_rule(b2) == expected2);
  }

  SUBCASE("two boids exactly on lower distance") {
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{4., 3., 0., 0.};
    project::module expected1{-12.,-9.};
    project::module expected2{12.,9.};
    project::boids_flock flock{2, 20., 5., 3., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.separation_rule(b1) == expected1);
    CHECK(flock.separation_rule(b2) == expected2);
  }
}

TEST_CASE("Testing the alignment rule function") {
  SUBCASE("The boids are still") {
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{10., 10., 0., 0.};
    project::boid b3{14., 3., 0., 0.};
    project::module expected{0.,0.};
    project::boids_flock flock{3, 20., 2., 0., 0.5, 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    CHECK(flock.alignment_rule(b1) == expected);
    CHECK(flock.alignment_rule(b2) == expected);
    CHECK(flock.alignment_rule(b3) == expected);
  }

  SUBCASE("Two boids with same velocities") {
    project::boid b1{0., 0., 5., 10.};
    project::boid b2{10., 0., 5., 10.};
    project::module expected{0.,0.};
    project::boids_flock flock{3, 20., 2., 0., 0.5, 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.alignment_rule(b1) == expected);
    CHECK(flock.alignment_rule(b2) == expected);
  }

  SUBCASE("Two boids, one initially still") {
    project::boid b1{10., 10., -5., -5.};
    project::boid b2{20., 0., 0., 0.};
    project::module expected1{2.5,2.5};
    project::module expected2{-2.5,-2.5};
    project::boids_flock flock{2, 20., 2., 0., 0.5, 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.alignment_rule(b1) == expected1);
    CHECK(flock.alignment_rule(b2) == expected2);
  }

  SUBCASE("Two boids with opposite velocities") {
    project::boid b1{10., 0., 0., 10.};
    project::boid b2{0., 0., 0., -10.};
    project::module expected1{0.,-10.};
    project::module expected2{0.,10.};
    project::boids_flock flock{2, 20., 2., 0., 0.5, 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.alignment_rule(b1) == expected1);
    CHECK(flock.alignment_rule(b2) == expected2);
  }

  SUBCASE("Three boids, one initially still") {
    project::boid b1{-5., 5., 10., 10.};
    project::boid b2{0., 0., 0., 0.};
    project::boid b3{5., -5., -10., -10.};
    project::module expected1{-7.5,-7.5};
    project::module expected2{0.,0.};
    project::module expected3{7.5,7.5};
    project::boids_flock flock{3, 20., 2., 0., 0.5, 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    CHECK(flock.alignment_rule(b1) == expected1);
    CHECK(flock.alignment_rule(b2) == expected2);
    CHECK(flock.alignment_rule(b3) == expected3);
  }

  SUBCASE("Testing the return when boids are too far apart") {
    project::boid b1{100., 100., 10., 10.};
    project::boid b2{100., -100., 10., -10.};
    project::boid b3{-100., 100., -10., 10.};
    project::boid b4{-100., -100., -10., -10.};
    project::module expected{0.,0.};
    project::boids_flock flock{4, 20., 2., 0., 0.5, 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    CHECK(flock.alignment_rule(b1) == expected);
    CHECK(flock.alignment_rule(b2) == expected);
    CHECK(flock.alignment_rule(b3) == expected);
    CHECK(flock.alignment_rule(b4) == expected);
  }
}

TEST_CASE("testing the center of mass function") {
  SUBCASE("three boids") {
    project::boid b1{1., 1., 0., 0.};
    project::boid b2{4., -3., 0., 0.};
    project::boid b3{-3., 6., 0., 0.};
    project::module expected{0.5,1.5};
    project::boids_flock flock{3, 20., 0.5, 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    CHECK(flock.get_center_of_mass_nearby(b1) == expected);
  }

  SUBCASE("five boids") {
    project::boid b1{-2., -2., 0., 0.};
    project::boid b2{6., -3., 0., 0.};
    project::boid b3{-1., 9., 0., 0.};
    project::boid b4{-10., 0., 0., 0.};
    project::boid b5{2., 4., 0., 0.};
    project::module expected{-0.75,2.5};
    project::boids_flock flock{5, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    CHECK(flock.get_center_of_mass_nearby(b1) == expected);
  }
}

TEST_CASE("Checking the cohesion rule") {
  SUBCASE("three boids, all in range") {
    project::boid b1{0., 0., 0., 0.};
    project::boid b2{4., 0., 0., 0.};
    project::boid b3{2., 2., 0., 0.};
    project::module expected1{6.,2.};
    project::module expected2{-6.,2.};
    project::module expected3{0.,-4.};
    project::boids_flock flock{3, 20., 2., 0., 0., 2.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    CHECK(flock.cohesion_rule(b1) == expected1);
    CHECK(flock.cohesion_rule(b2) == expected2);
    CHECK(flock.cohesion_rule(b3) == expected3);
  }

  SUBCASE("four boids in different quadrants") {
    project::boid b1{4., 4., 0., 0.};
    project::boid b2{-4., 4., 0., 0.};
    project::boid b3{-4., -4., 0., 0.};
    project::boid b4{4., -4., 0., 0.};
    project::boids_flock flock{4, 21., 2., 0., 0., 2.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    CHECK(flock.cohesion_rule(b1).x == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule(b1).y == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule(b2).x == doctest::Approx(10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule(b2).y == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule(b3).x == doctest::Approx(10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule(b3).y == doctest::Approx(10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule(b4).x == doctest::Approx(-10.67).epsilon(0.01));
    CHECK(flock.cohesion_rule(b4).y == doctest::Approx(10.67).epsilon(0.01));
  }
}

TEST_CASE("Checking the corner force") {
  SUBCASE("The boid hits the upper/right/lower/left corner") {
    project::boid b1{2., 140., -3., 10.};
    project::boid expected1{2., 140., -3., 8.5};
    project::boid b2{140., -10., 8., 3.};
    project::boid expected2{140., -10., 6.5, 3.};
    project::boid b3{-15., -140., 10., -7.};
    project::boid expected3{-15., -140., 10., -5.5};
    project::boid b4{-140., 11., -10., -7.};
    project::boid expected4{-140., 11., -8.5, -7.};
    project::boids_flock flock{4, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.corner_force();
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
    project::boid b1{140., 140., -3., 10.};
    project::boid expected1{140., 140., -4.5, 8.5};
    project::boid b2{140., -140., 8., 3.};
    project::boid expected2{140., -140., 6.5, 4.5};
    project::boid b3{-140., -140., 10., -7.};
    project::boid expected3{-140., -140., 11.5, -5.5};
    project::boid b4{-140., 140., -10., -7.};
    project::boid expected4{-140., 140., -8.5, -8.5};
    project::boids_flock flock{4, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.corner_force();
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

TEST_CASE("Testing the flock_velocity function") {
  SUBCASE("Random boids") {
    project::boid b1{0., 0., 1., 1.};
    project::boid b2{0., 0., 0.7, 0.9};
    project::boid b3{0., 0., -4.5, 3.6};
    project::boid b4{0., 0., -10., 0.};
    project::boid b5{0., 0., -3., -2.};
    project::boid b6{0., 0., 0.01, -1.};
    project::boid b7{0., 0., 2., 0.};
    project::boid b8{0., 0., 0., 2.};
    project::boids_flock flock{8, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    flock.addBoid(b6);
    flock.addBoid(b7);
    flock.addBoid(b8);
    CHECK(flock.flock_velocity() == doctest::Approx(1.813207).epsilon(0.01));
  }

  SUBCASE("Boids whose velocities cancel out each other") {
    project::boid b1{0., 0., 1., 1.};
    project::boid b2{0., 0., -1., -1.};
    project::boid b3{0., 0., -4.5, -4.5};
    project::boid b4{0., 0., 4.5, 4.5};
    project::boid b5{0., 0., -2., 3.};
    project::boid b6{0., 0., 2., -3.};
    project::boid b7{0., 0., 10., 0.};
    project::boid b8{0., 0., -10., 0.};
    project::boids_flock flock{8, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    flock.addBoid(b6);
    flock.addBoid(b7);
    flock.addBoid(b8);
    CHECK(flock.flock_velocity() == doctest::Approx(0.000));
  }
}

TEST_CASE("Testing the mean velocity function") {
  SUBCASE("Random boids") {
    project::boid b1{0., 0., 1., 1.};
    project::boid b2{0., 0., 0.7, 0.9};
    project::boid b3{0., 0., -4.5, 3.6};
    project::boid b4{0., 0., -10., 0.};
    project::boid b5{0., 0., -3., -2.};
    project::boid b6{0., 0., 0.01, -1.};
    project::boid b7{0., 0., 2., 0.};
    project::boid b8{0., 0., 0., 2.};
    project::boids_flock flock{8, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    flock.addBoid(b6);
    flock.addBoid(b7);
    flock.addBoid(b8);
    CHECK(flock.mean_velocity() == doctest::Approx(3.3658).epsilon(0.01));
  }

  SUBCASE("Velocities components cancelling out each other") {
    project::boid b1{0., 0., 1., 1.};
    project::boid b2{0., 0., -1., -1.};
    project::boid b3{0., 0., -4.5, -4.5};
    project::boid b4{0., 0., 4.5, 4.5};
    project::boid b5{0., 0., -2., 3.};
    project::boid b6{0., 0., 2., -3.};
    project::boid b7{0., 0., 10., 0.};
    project::boid b8{0., 0., -10., 0.};
    project::boids_flock flock{8, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    flock.addBoid(b6);
    flock.addBoid(b7);
    flock.addBoid(b8);
    CHECK(flock.mean_velocity() == doctest::Approx(5.345931).epsilon(0.01));
  }
}

TEST_CASE("Test the velocity_st_deviation function") {
  SUBCASE("Two boids, simple situation") {
    project::boid b1{0., 0., 3., 4.};
    project::boid b2{0., 0., 8., 6.};
    project::boids_flock flock{2, 20., 1., 0.5, 0.5, 0.5};
    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.mean_velocity() == doctest::Approx(7.5));
    CHECK(flock.velocity_st_deviation() ==
          doctest::Approx(3.535).epsilon(0.01));
  }

  SUBCASE("Eight boids") {
    project::boid b1{0., 0., 1., 1.};
    project::boid b2{0., 0., 0.7, 0.9};
    project::boid b3{0., 0., -4.5, 3.6};
    project::boid b4{0., 0., -10., 0.};
    project::boid b5{0., 0., -3., -2.};
    project::boid b6{0., 0., 0.01, -1.};
    project::boid b7{0., 0., 2., 0.};
    project::boid b8{0., 0., 0., 2.};
    project::boids_flock flock{8, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    flock.addBoid(b6);
    flock.addBoid(b7);
    flock.addBoid(b8);
    CHECK(flock.velocity_st_deviation() ==
          doctest::Approx(3.1151).epsilon(0.01));
  }
}

TEST_CASE("Testing the mean_distance function") {
  SUBCASE("four boids") {
    project::boid b1{2., 2., 0., 0.};
    project::boid b2{2., -2., 0., 0.};
    project::boid b3{-2., -2., 0., 0.};
    project::boid b4{-2., 2., 0., 0.};
    project::boids_flock flock{4, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    CHECK(flock.mean_distance() == doctest::Approx(4.55228));
  }

  SUBCASE("three boids") {
    project::boid b1{20., 11., 0., 0.};
    project::boid b2{-11., -4., 0., 0.};
    project::boid b3{7., -8., 0., 0.};
    project::boids_flock flock{3, 20., 5., 0., 0., 0.};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    CHECK(flock.mean_distance() == doctest::Approx(25.2997));
  }
}

TEST_CASE("Testing the velocities update function") {
  SUBCASE("two boids, very basic scenario") {
    project::boid b1{0., 0., 18., 0.};
    project::boid b2{0., 8., 0., 18.};
    project::boids_flock flock{2, 20., 2., 0.5, 0.5, 0.5};
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.velocities();
    CHECK(flock.get_flock()[0].v_x == doctest::Approx(9.));
    CHECK(flock.get_flock()[0].v_y == doctest::Approx(13.));
    CHECK(flock.get_flock()[1].v_x == doctest::Approx(9.));
    CHECK(flock.get_flock()[1].v_y == doctest::Approx(5.));
  }
}

TEST_CASE("Testing the position update function") {
  project::boid b1{0., 0., 0., 0.};
  project::boid b2{0., 0., 0., 10.};
  project::boid b3{0., 0., 10., 0.};
  project::boid b4{0., 0., 10., 10.};
  project::boid b5{0., 0., 0., -10.};
  project::boid b6{0., 0., -10., 0.};
  project::boid b7{0., 0., -10., -10.};
  project::boid b1e{0., 0., 0., 0.};
  project::boid b2e{0., 0.15, 0., 10.};
  project::boid b3e{0.15, 0., 10., 0.};
  project::boid b4e{0.15, 0.15, 10., 10.};
  project::boid b5e{0., -0.15, 0., -10.};
  project::boid b6e{-0.15, 0., -10., 0.};
  project::boid b7e{-0.15, -0.15, -10., -10.};
  project::boids_flock flock{7, 30., 2., 0.5, 0.5, 0.5};
  flock.addBoid(b1);
  flock.addBoid(b2);
  flock.addBoid(b3);
  flock.addBoid(b4);
  flock.addBoid(b5);
  flock.addBoid(b6);
  flock.addBoid(b7);
  flock.position_update();
  const project::boid& result1 = flock.get_flock()[0];
  const project::boid& result2 = flock.get_flock()[1];
  const project::boid& result3 = flock.get_flock()[2];
  const project::boid& result4 = flock.get_flock()[3];
  const project::boid& result5 = flock.get_flock()[4];
  const project::boid& result6 = flock.get_flock()[5];
  const project::boid& result7 = flock.get_flock()[6];
  CHECK(result1.v_x == b1e.v_x);
  CHECK(result1.v_y == b1e.v_y);
  CHECK(result2.v_x == b2e.v_x);
  CHECK(result2.v_y == b2e.v_y);
  CHECK(result3.v_x == b3e.v_x);
  CHECK(result3.v_y == b3e.v_y);
  CHECK(result4.v_x == b4e.v_x);
  CHECK(result4.v_y == b4e.v_y);
  CHECK(result5.v_x == b5e.v_x);
  CHECK(result5.v_y == b5e.v_y);
  CHECK(result6.v_x == b6e.v_x);
  CHECK(result6.v_y == b6e.v_y);
  CHECK(result7.v_x == b7e.v_x);
  CHECK(result7.v_y == b7e.v_y);
}
