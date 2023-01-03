#ifndef LEPH_UTILS_IMPORTERCSV_H
#define LEPH_UTILS_IMPORTERCSV_H

#include <string>
#include <fstream>
#include <vector>
#include <stdexcept>

namespace leph {

/**
 * ImporterCSV
 *
 * Import CSV data into Eigen matrix.
 * @param filename Path to the CSV file.
 * @param CSV delimiter character.
 * @return nested vector with rows x columns.
 */
inline std::vector<std::vector<double>> ImporterCSV(const std::string& filepath, char delimiter)
{
    std::vector<std::vector<double>> data;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error(
            "leph::ImporterCSV: error opening file: " + filepath);
    }
    while (file.good()) {
        std::string line;
        std::getline(file, line);
        if (line.length() == 0) {
            continue;
        }
        data.push_back({});
        size_t tmpPosStart = 0;
        while (true) {
            if (tmpPosStart == std::string::npos || tmpPosStart >= line.length()) {
                break;
            }
            while (
                tmpPosStart < line.length() && 
                !std::isdigit(line[tmpPosStart]) && 
                line[tmpPosStart] != '+' &&
                line[tmpPosStart] != '-'
            ) {
                tmpPosStart++;
            }
            size_t tmpPosEnd = line.find(delimiter, tmpPosStart);
            if (tmpPosEnd == std::string::npos) {
                tmpPosEnd = line.length();
            }
            if (tmpPosEnd > tmpPosStart) {
                double val = std::stod(line.substr(tmpPosStart, tmpPosEnd-tmpPosStart));
                data.back().push_back(val);
            }
            tmpPosStart = tmpPosEnd + 1;
        }
    }
    file.close();

    return data;
}

}

#endif

