#include "config_parser.h"

std::unordered_map<std::string, std::string> parseConfigFile(std::string filename, std::string delimiter){
    std::string line;
    std::ifstream file(filename);
    std::unordered_map<std::string, std::string> inputs;
    if(file.is_open()){
        while(std::getline(file, line)){

            // Remove leading and trailing spaces
            line = std::regex_replace(line, std::regex("^ +| +$|( ) +"), "$1");

            // Split std::string into two parts using delimiter
            int delimiter_position = line.find(delimiter);
            std::string key = line.substr(0, delimiter_position);
            std::string value = line.substr(delimiter_position + 1);
            key = std::regex_replace(key, std::regex("^ +| +$|( ) +"), "$1");
            value = std::regex_replace(value, std::regex("^ +| +$|( ) +"), "$1");

            
            // If either of them is empty ignore the line
            if(key == "" || value == "")
                continue;

            inputs[key] = value;
        }
        file.close();
    }
    return inputs;
}