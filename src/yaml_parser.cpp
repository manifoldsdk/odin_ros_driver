#include "yaml_parser.h"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <algorithm> // 用于 std::transform

namespace odin_ros_driver {

// 使用一致的成员变量名
YamlParser::YamlParser(const std::string& config_file) 
    : config_file_(config_file) {} // 这里使用 config_file_

bool YamlParser::loadConfig() {
    try {
        // 使用成员变量 config_file_
        std::cerr << "Loading config file: " << config_file_ << std::endl;
        
        // 检查文件是否存在
        if (!std::filesystem::exists(config_file_)) {
            std::cerr << "Config file not found: " << config_file_ << std::endl;
            return false;
        }
        
        // 打印文件内容
        std::ifstream file(config_file_);
        std::string content((std::istreambuf_iterator<char>(file)), 
                           std::istreambuf_iterator<char>());
        std::cerr << "Config file content:\n" << content << "\n--- End of file ---" << std::endl;
        
        // 加载 YAML - 使用成员变量 config_file_
        YAML::Node config = YAML::LoadFile(config_file_);
        
        // 检查是否存在 register_keys 节点
        if (!config["register_keys"]) {
            std::cerr << "Missing 'register_keys' section in config file" << std::endl;
            return false;
        }
        
        YAML::Node register_keys = config["register_keys"];
        register_keys_.clear();
        
        // 打印键值对数量
        std::cerr << "Found " << register_keys.size() << " keys in config" << std::endl;
        
        for (YAML::const_iterator it = register_keys.begin(); it != register_keys.end(); ++it) {
            std::string key = it->first.as<std::string>();
            int value = it->second.as<int>();
            
            // 转换为小写
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

} // namespace odin_ros_driver