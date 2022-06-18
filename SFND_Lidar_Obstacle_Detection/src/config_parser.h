#ifndef CONFIG_PARSER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CONFIG_PARSER_H

#include <string>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <regex>

std::unordered_map<std::string, std::string> parseConfigFile(std::string filename, std::string delimiter);

#endif