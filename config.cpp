//
// Created by dkssu on 2022-06-17.
//

#include <iostream>
#include <algorithm>
#include <map>
#include <string>
#include <fstream>

int main()
{
    std::string path = "/workspace/testc/src/config.yaml";
    std::map<std::string, std::string> m_table;
    std::ifstream openFile(path);
    if (openFile.is_open()) {
        std::string line;
        while (getline(openFile, line)) {
            std::string delimiter = ": ";
            if (std::string::npos == line.find(delimiter)) delimiter = " : ";

            // substr(start, count), return [start, start + count)
            // find(first, last, val), point of first value of val from first to last, if same value not exist in str, return last.
            std::string token1 = line.substr(0, line.find(delimiter));
            if (line.find("#") == std::string::npos) // if not find value, return npos = -1
                int length = line.length();
            else
            {
                int length = line.find("#");
                std::cout << length << std::endl;
            }

            std::string token2 = line.substr(line.find(delimiter) + delimiter.length(), line.find("#") - line.find(delimiter) - 2);
            m_table[token1] = token2;

            std::cout << "name is " << token1 << ", value is " << token2 << std::endl << std::endl;
        }
        openFile.close();
    }
    else
    {
        std::cout << "do not exist file" << std::endl;
    }


    std::string name1 = "ORBextractor.nFeatures";
    if (m_table.find(name1) == m_table.end())
        throw std::invalid_argument("Not exist name");
    else
    {
        int ORBextractor_nFeatures = std::stoi(m_table[name1]);
        std::cout << "ORB feature number is " << ORBextractor_nFeatures << std::endl;
    }

    std::string name2 = "RANSAC method";
    if (m_table.find(name2) == m_table.end())
        throw std::invalid_argument("Not exist name");
    else
    {
        std::string RANSAC_method = m_table[name2];

        if (RANSAC_method.compare("RANSAC")) {
            std::cout << "filtering method is RANSAC" << std::endl;
        }
        else if(RANSAC_method.compare("PROSAC")) {
            std::cout << "filtering method is PROSAC" << std::endl;
        }
    }
}