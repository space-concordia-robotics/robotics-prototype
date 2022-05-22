//
// Created by william on 22/05/22.
//

#include <gtest/gtest.h>
#include "ik-solver.h"

// Demonstrate some basic assertions.
TEST(IK, EndPosTest) {
    float test = getJointAngle2For2D(10, 15);

    printf("%f\n", test);

    EXPECT_EQ(7 * 6, 42);
}