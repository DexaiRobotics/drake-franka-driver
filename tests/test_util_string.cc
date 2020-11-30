/// @file test_util_string.cc -- part of a googletest suite
#include <gtest/gtest.h>

#include "util_string.h"

// TODO: Add test for hostname_string()

// Expected: convert to uppercase
TEST(UtilString, to_uppercase)
{
    const std::string lowercase = "what_do_we_say_to_the_god_of_death?";
    const std::string uppercase = "WHAT_DO_WE_SAY_TO_THE_GOD_OF_DEATH?";

    std::string lower_to_upper = utils::to_uppercase(lowercase);

    EXPECT_EQ(lower_to_upper, uppercase);

    const std::string mixed = "Not today!";
    
    std::string mixed_to_upper = utils::to_uppercase(mixed);

    EXPECT_EQ(mixed_to_upper, "NOT TODAY!");
}
