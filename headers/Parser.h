#ifndef PARSER_H
#define PARSER_H

class Parser {

    public:
        Parser(std::string config_name);

        std::string get_key_value(std::string key);

        std::vector<std::string> get_key_values(std::string key);

    private:
        std::string _config_name;

};

#endif