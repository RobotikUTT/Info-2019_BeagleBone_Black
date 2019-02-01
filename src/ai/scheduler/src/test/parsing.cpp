#include "scheduler/ActionsParser.hpp"

#include <gtest/gtest.h>

TEST(ActionsParser, parseSingleFile) {
}

TEST(TestingTests, passing) {
    EXPECT_TRUE(1==1);
}
TEST(TestingTests, error) {
    EXPECT_TRUE(1==0);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}