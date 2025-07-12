#include "yaml_parser.h"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <algorithm> 

namespace odin_ros_driver {

YamlParser::YamlParser(const std::string& config_file) 
    : config_file_(config_file) {} // // Using config_file_ member variable

bool YamlParser::loadConfig() {
    try {
        //  Using member variable config_file_
        std::cerr << "Loading config file: " << config_file_ << std::endl;
        
        // Check if file exists
        if (!std::filesystem::exists(config_file_)) {
            std::cerr << "Config file not found: " << config_file_ << std::endl;
            return false;
        }
        
        // Print file contents
        std::ifstream file(config_file_);
        std::string content((std::istreambuf_iterator<char>(file)), 
                           std::istreambuf_iterator<char>());
        std::cerr << "Config file content:\n" << content << "\n--- End of file ---" << std::endl;
        
        // Load YAML
        YAML::Node config = YAML::LoadFile(config_file_);
        
        // Check if 'register_keys' node exists
        if (!config["register_keys"]) {
            std::cerr << "Missing 'register_keys' section in config file" << std::endl;
            return false;
        }
        
        YAML::Node register_keys = config["register_keys"];
        register_keys_.clear();
        
        // Print number of key-value pairs found
        std::cerr << "Found " << register_keys.size() << " keys in config" << std::endl;
        
        for (YAML::const_iterator it = register_keys.begin(); it != register_keys.end(); ++it) {
            std::string key = it->first.as<std::string>();
            int value = it->second.as<int>();
            
            // Convert key to lowercase
            std::transform(key.begin(), key.end(), key.begin(), 
                           [](unsigned char c){ return std::tolower(c); });
            
            register_keys_[key] = value;
            std::cerr << "Loaded key: " << key << " = " << value << std::endl;
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML exception: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return false;
    }
}

const std::map<std::string, int>& YamlParser::getRegisterKeys() const {
    return register_keys_;
}

void YamlParser::printConfig() const {
    std::cerr << "Configuration Keys:" << std::endl;
    if (register_keys_.empty()) {
        std::cerr << "  (empty)" << std::endl;
        return;
    }
    
    for (const auto& [key, value] : register_keys_) {
        std::cerr << "  " << key << ": " << value << std::endl;
    }
}

} 