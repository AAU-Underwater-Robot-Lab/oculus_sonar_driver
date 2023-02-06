// Copyright 2020-2023 UW-APL
// Authors: Aaron Marburg

#include <gtest/gtest.h>

// Run all the declared gtests
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
