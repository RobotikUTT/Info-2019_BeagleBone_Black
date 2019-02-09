#include "scheduler/ActionFilePath.hpp"
#include <sstream>

#define FILE_EXTENSION ".json"

ActionFilePath::ActionFilePath(std::string file, std::string root, std::string folder /* = "." */) :
    file(file), root(root), folder(folder) { }

std::string ActionFilePath::compile() {
    std::ostringstream output;
    output << root << "/";
    output << folder << "/";
    output << file << FILE_EXTENSION;

    return output.str();
}

ActionFilePath ActionFilePath::relative(std::string path) {
    std::string folder, filename;
    int separatorIndex = path.find_last_of("/");
    
    filename = path.substr(separatorIndex + 1);
    
    if (separatorIndex != -1) {
        folder = this->folder + path.substr(0, separatorIndex);
    } else if (separatorIndex == 0) {
        folder = path.substr(0, separatorIndex);
    } else {
        folder = this->folder;
    }
    
    return ActionFilePath(filename, this->root, folder);
}

bool operator==(const ActionFilePath& lhs, const ActionFilePath& rhs) {
    return lhs.folder == rhs.folder &&
        lhs.root == rhs.root &&
        lhs.file == rhs.file;
}