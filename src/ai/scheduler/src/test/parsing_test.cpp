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

    ActionBlockPtr block;
    AtomicActionPtr atomic;

    ActionFilePath* path;

  // Setup
  ParsingFixture() : nh() {
    // retrieve args
    std::string root;
    nh.param<std::string>("test_directory", root, "");

    // create default path
    path = new ActionFilePath(std::string(), root);

    // init theorical blocks
    atomic = std::make_shared<AtomicAction>("atomic test", "none");
    atomic->setSync(false);
    atomic->setBasePoints(1);
    atomic->addArg(make_arg("usefull", 0));
    atomic->addArg(make_arg("style", 30));

    block = std::make_shared<ActionBlock>("action group");
    block->setBasePoints(3);
    block->setSync(true);

    block->addAction(atomic);
    block->addAction(std::make_shared<AtomicAction>("atomic test 2", "none"));
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
  ActionsParser* parser;
  try {
    parser = new ActionsParser(make_path("atomic_action"));
  } catch(const char* message) {
    FAIL() << message;
  }
  
  ASSERT_TRUE(parser->getAction()->equals(*atomic))
      << *parser->getAction() << std::endl << "parsed result is not equal to" << std::endl << *atomic;
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
  try {
    // Parse from file
    ActionPtr parsed = ActionsParser(make_path("action_block")).getAction();
    
    ASSERT_TRUE(parsed->equals(*block))
      << *parsed << std::endl << "parsed result is not equal to" << std::endl << *block;
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
  
    ASSERT_TRUE(parsed->equals(*block))
      << *parsed << std::endl << "parsed result is not equal to" << std::endl << *block;

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