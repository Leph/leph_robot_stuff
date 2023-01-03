#ifndef LEPH_UTILS_STRING_H
#define LEPH_UTILS_STRING_H

#include <string>
#include <vector>

namespace leph {

/**
 * Split a string in string chucks
 * separated by a delimiter character.
 *
 * @param str String to be splited
 * @param delimiter Delimiter used 
 * to split the string
 */
inline std::vector<std::string> StrExplode(
    const std::string& str, char delimiter)
{
    std::vector<std::string> parts;
    size_t index = 0;
    while (index < str.length()) {
        size_t pos = str.find_first_of(delimiter, index);
        if (pos != std::string::npos) {
            if (pos == index) {
                index++;
                continue;
            }
            parts.push_back(str.substr(index, pos-index));
            index = pos+1;
        } else {
            parts.push_back(str.substr(index));
            break;
        }
    }

    return parts;
}

}

#endif

