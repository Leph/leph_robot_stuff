#include <iostream>
#include <leph_utils/ImporterCSV.h>

int main()
{
    std::vector<std::vector<double>> data = leph::ImporterCSV("/tmp/test_csv.csv", ' ');
    for (size_t i=0;i<data.size();i++) {
        std::cout << "Row " << i << ": ";
        for (size_t j=0;j<data[i].size();j++) {
            std::cout << data[i][j] << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}

