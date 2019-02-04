#ifndef ACTION_FILE_PATH_HPP
#define ACTION_FILE_PATH_HPP

#include <iostream>
#include <string>
#include <sstream>

class ActionFilePath
{
private:
public:
    ActionFilePath(std::string file, std::string root, std::string folder = ".");

    std::string root;
    std::string folder;
    std::string file;

    std::string compile();
    ActionFilePath relative(std::string file);
};

bool operator==(const ActionFilePath& lhs, const ActionFilePath& rhs);

#endif