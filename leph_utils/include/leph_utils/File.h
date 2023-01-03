#ifndef LEPH_UTILS_FILE_H
#define LEPH_UTILS_FILE_H

#include <string>
#include <fstream>

namespace leph {

/**
 * Read and return the content of given filepath
 */
inline std::string FileRead(const std::string& filepath)
{
    //Open file
    std::ifstream file;
    file.open(filepath);
    if (!file.is_open()) {
        throw std::runtime_error(
            "leph::ReadFile: Unabled to open: " 
            + filepath);
    }
    //Read file content
    std::string filecontent(
        (std::istreambuf_iterator<char>(file)), 
        std::istreambuf_iterator<char>());
    file.close();

    return filecontent;
}

}

#endif

