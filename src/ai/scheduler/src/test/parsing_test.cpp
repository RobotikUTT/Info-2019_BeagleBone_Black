#include "scheduler/ActionsParser.hpp"

#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <string>
#include <sstream>

// Static content for tests
class ParsingFixture : public ::testing::Test {
  protected:
    std::string foldername;
    ros::NodeHandle nh;
    Action* saved;

  // Setup
  ParsingFixture() : nh() {
    nh.param<std::string>("test_directory", foldername, "");
  }

  std::string make_path(const char* string) {
    std::stringstream stream;
    stream << foldername << string;
    return stream.str();
  }

  ai_msgs::Argument make_arg(std::string name, double value) {
    ai_msgs::Argument arg;
    arg.name = name;
    arg.value = value;

    return arg;
  }
  
};

/* TESTED JSON FILE
  {
      "name": "atomic test",
      "performer": "none",
      "args": {"usefull": 0, "style": 30},
      "sync": false,
      "points": 1
  }
*/
TEST_F(ParsingFixture, atomicAction) {
  AtomicAction* expected = new AtomicAction("atomic test", "none");
  expected->setSync(false);
  expected->setBasePoints(1);
  expected->addArg(make_arg("usefull", 0));
  expected->addArg(make_arg("style", 30));

  // Save for further tests
  saved = expected;

  ActionsParser* parser;
  try {
    parser = new ActionsParser(make_path("atomic_action.json"));
  } catch(std::string message) {
    FAIL() << message;
  }
  
  ASSERT_TRUE(parser->getAction()->equals(*expected));
}

/* TESTED JSON FILE
  [
    {
      "name": "action group",
      "points": 3,
      "sync": true
    },
    {
        "name": "atomic test",
        "performer": "none",
        "args": {"usefull": 0, "style": 30},
        "sync": false,
        "points": 1
    },
    {
        "name": "atomic test 2",
        "performer": "none"
    }
  ]
*/
TEST_F(ParsingFixture, actionBlock) {
  // Action block expected and parsed
  ActionBlock* expected = new ActionBlock("action group");
  expected->setBasePoints(3);
  expected->setSync(true);

  expected->addAction(std::make_shared<Action>(*saved));
  expected->addAction(std::make_shared<Action>(AtomicAction("atomic test 2", "none")));
  saved = expected; // save for next test
  try {
    ActionPtr parsed = ActionsParser(make_path("action_block.json")).getAction();
  
    ASSERT_TRUE(expected->equals(*parsed));
  } catch(std::string message) {
    FAIL() << message;
  }

}

/* TESTED JSON FILE
  [
    {
      "name": "action group",
      "points": 3,
      "sync": true
    },
    {
        "file": "atomic_action"
    },
    {
        "name": "atomic test 2",
        "performer": "none"
    }
  ]
*/
TEST_F(ParsingFixture, fileInclusion) {
  // TOFIX prevent test to terminate
  try {
    ActionPtr parsed = ActionsParser(make_path("inclusion.json")).getAction();
  
    ASSERT_TRUE(parsed->equals(*saved));
  } catch(std::string message) {
    FAIL() << message;
  }

}

/* TESTED JSON FILES
  == first.json ==
  {
    "file": "second"
  }

  == second.json ==
  {
    "file": "first"
  }
*/
TEST_F(ParsingFixture, circularInclusionPrevention) {
  // Parser should throw an exception in this case
  EXPECT_THROW({
    ActionsParser(make_path("first.json"));
  }, std::string);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "parsing_test");
    
    return RUN_ALL_TESTS();
}