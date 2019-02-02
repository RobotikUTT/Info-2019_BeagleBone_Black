#include "scheduler/ActionsParser.hpp"

#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

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

  const char* make_path(const char* string) {
    std::stringstream path(foldername);
    path << string;
    return path.c_str();
  }
};

/* TESTED JSON FILE :
  {
      "name": "atomic test",
      "performer": "none",
      "args": {"usefull": 0, "style": 30},
      "sync": false,
      "points": 1
  }
*/
TEST_F(ParsingFixture, atomicAction) {
  ActionsParser parser(make_path("atomic_action.json"));

  AtomicAction comparison("atomic test", "none");
  comparison.setSync(false);
  comparison.setBasePoints(1);
  comparison.addArg(ai_msgs::Argument("usefull", 0));
  comparison.addArg(ai_msgs::Argument("style", 30));

  ASSERT_EQ(parser.getAction(), comparison);
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