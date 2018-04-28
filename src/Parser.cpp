#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>

#include "../headers/Parser.h"

Parser::Parser(std::string config_name) {
    _config_name = config_name;
}

std::string Parser::get_key_value(std::string key) {
    std::ifstream config_file;
   
    config_file.open(_config_name);
    
    std::string line;
    while(std::getline(config_file, line)) {
        
        std::istringstream is_line(line);
        
        std::string found_key;
        if(std::getline(is_line, found_key, '='))
        {
            std::string value;
             
            if(std::getline(is_line, value)) {
                
                if(key == found_key) {
                   
                    return value;
                }
            }
        }
    }   
}

std::vector<std::string> Parser::get_key_values(std::string key) {
    std::vector<std::string> values;

    std::ifstream config_file;
    config_file.open(_config_name);

    std::string line;
    while(std::getline(config_file, line)) {
        std::istringstream is_line(line);
        std::string found_key;
        if(std::getline(is_line, found_key, '=') )
        {
            std::string value;
            if(std::getline(is_line, value)) {
                if(key == found_key) {
                    values.push_back(value);
                }
            }
        }
    }

    return values;   
}