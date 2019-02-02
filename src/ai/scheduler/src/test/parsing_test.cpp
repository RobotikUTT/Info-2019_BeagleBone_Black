#include "scheduler/ActionsParser.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <string>

// Static content for tests
class ParsingFixture : public ::testing::Test {
  protected:
    std::string foldername;
    ros::NodeHandle nh;

  // Setup
  ParsingFixture() : nh() {
    nh.param<std::string>("test_directory", foldername, "");
  }
};


TEST_F(ParsingFixture, retrieveFolder) {
    ASSERT_FALSE(foldername.empty()) << "Unable to retrieve folder parameter";
    ASSERT_TRUE(false) << foldername;
}

TEST_F(ParsingFixture, passing) {
    ASSERT_TRUE(1==1);
}
TEST_F(ParsingFixture, error) {
    ASSERT_TRUE(1==0);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "parsing_test");
    //ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}