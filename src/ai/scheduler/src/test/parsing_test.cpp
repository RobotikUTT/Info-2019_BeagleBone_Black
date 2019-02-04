#include "scheduler/ActionsParser.hpp"
#include "scheduler/ActionFilePath.hpp"

#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <string>
#include <sstream>

// Static content for tests
class ParsingFixture : public ::testing::Test {
  protected:
    ros::NodeHandle nh;
    Action* saved;
    AtomicAction* atomicSavec;
    ActionFilePath* path;

  // Setup
  ParsingFixture() : nh() {
    // retrieve args
    std::string root;
    nh.param<std::string>("test_directory", root, "");

    // create default path
    path = new ActionFilePath(std::string(), root);
  }

  ActionFilePath& make_path(const char* file) {
    path->file = file;
    return *path;
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
  atomicSavec = expected;

  ActionsParser* parser;
  try {
    parser = new ActionsParser(make_path("atomic_action"));
  } catch(const char* message) {
    FAIL() << message;
  }
  
  ASSERT_TRUE(parser->getAction()->equals(*expected))
      << *parser->getAction() << std::endl << "parsed result is not equal to" << std::endl << *expected;
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
  // Action block expected
  ActionBlock* expected = new ActionBlock("action group");
  expected->setBasePoints(3);
  expected->setSync(true);

  expected->addAction(std::make_shared<AtomicAction>(*atomicSavec));
  expected->addAction(std::make_shared<AtomicAction>(AtomicAction("atomic test 2", "none")));
  saved = expected; // save for next test

  try {
    // Parse from file
    ActionPtr parsed = ActionsParser(make_path("action_block")).getAction();
    
    ASSERT_TRUE(expected->equals(*parsed))
      << *parsed << std::endl << "parsed result is not equal to" << std::endl << *expected;
  } catch(const char* message) {
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
  try {
    ActionPtr parsed = ActionsParser(make_path("inclusion")).getAction();
  
    ASSERT_TRUE(parsed->equals(*saved))
      << *parsed << std::endl << "parsed result is not equal to" << std::endl << *saved;

  } catch(const char* message) {
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
    ActionsParser(make_path("first"));
  }, const char*);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "parsing_test");
    
    return RUN_ALL_TESTS();
}