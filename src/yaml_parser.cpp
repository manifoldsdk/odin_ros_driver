/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "yaml_parser.h"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <iomanip>

#include <cstring>
namespace odin_ros_driver {

YamlParser::YamlParser(const std::string& config_file)
    : config_file_(config_file) {}

bool YamlParser::loadConfig() {
    try {
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
        register_keys_str_val_.clear();
        custom_parameters_.clear();

        // Print number of key-value pairs found
        std::cerr << "Found " << register_keys.size() << " keys in config" << std::endl;

        for (YAML::const_iterator it = register_keys.begin(); it != register_keys.end(); ++it) {
            std::string key = it->first.as<std::string>();
            const YAML::Node& value_node = it->second;

            // Convert key to lowercase
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c){ return std::tolower(c); });

            // Check if this is a custom parameter
            if (key.substr(0, 7) == "custom_") {
                std::string param_name = key.substr(7);

                // Handle different value types
                if (value_node.IsScalar()) {
                    // Single scalar value (int or float)
                    try {
                        int int_value = value_node.as<int>();
                        ParameterValue param_value;
                        param_value.type = DataType::INT_TYPE;
                        param_value.setData(int_value);
                        custom_parameters_[param_name] = param_value;
                        std::cerr << "Loaded custom parameter (int): " << param_name << " = " << int_value << std::endl;
                    } catch (...) {
                        try {
                            double float_value = value_node.as<double>();
                            ParameterValue param_value;
                            param_value.type = DataType::FLOAT_ARRAY_TYPE;
                            std::vector<float> float_array = {static_cast<float>(float_value)};
                            param_value.setArray(float_array);
                            custom_parameters_[param_name] = param_value;
                            std::cerr << "Loaded custom parameter (float): " << param_name << " = " << float_value << std::endl;
                        } catch (const std::exception& e) {
                            std::cerr << "Failed to parse custom parameter " << param_name << ": " << e.what() << std::endl;
                        }
                    }
                } else if (value_node.IsSequence()) {
                    // Array of values
                    size_t array_size = value_node.size();
                    if (array_size == 0) {
                        std::cerr << "Empty array for custom parameter: " << param_name << std::endl;
                        continue;
                    }

                    // Try to detect if it's a float or int array based on first element
                    try {
                        // Try to parse as float array first
                        std::vector<float> float_array;
                        for (size_t i = 0; i < array_size; ++i) {
                            float_array.push_back(value_node[i].as<float>());
                        }
                        ParameterValue param_value;
                        param_value.type = DataType::FLOAT_ARRAY_TYPE;
                        param_value.setArray(float_array);
                        custom_parameters_[param_name] = param_value;

                        std::cerr << "Loaded custom parameter (float array): " << param_name << " = [";
                        for (size_t i = 0; i < float_array.size(); ++i) {
                            if (i > 0) std::cerr << ", ";
                            std::cerr << std::fixed << std::setprecision(4) << float_array[i];
                        }
                        std::cerr << "]" << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "Failed to parse custom parameter array " << param_name << ": " << e.what() << std::endl;
                    }
                }
            } else if (allowed_key_w_str_val.find(key) != allowed_key_w_str_val.end()) {
                try {
                    std::string value = value_node.as<std::string>();
                    register_keys_str_val_[key] = value;
                    std::cerr << "Loaded key: " << key << " = " << value << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Failed to parse key " << key << ": " << e.what() << std::endl;
                }
            } else {
                // Regular (non-custom) integer parameter
                try {
                    int value = value_node.as<int>();
                    register_keys_[key] = value;
                    std::cerr << "Loaded key: " << key << " = " << value << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Failed to parse key " << key << ": " << e.what() << std::endl;
                }
            }
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

const std::map<std::string, ParameterValue>& YamlParser::getCustomParameters() const {
    return custom_parameters_;
}

const std::map<std::string, std::string>& YamlParser::getRegisterKeysStrVal() const {
    return register_keys_str_val_;
}

void YamlParser::printConfig() const {
    std::cerr << "Configuration Keys:" << std::endl;
    if (register_keys_.empty()) {
        std::cerr << "  (int val empty)" << std::endl;
    } else {
        for (const auto& [key, value] : register_keys_) {
            std::cerr << "  " << key << ": " << value << std::endl;
        }
    }

    if (register_keys_str_val_.empty()) {
        std::cerr << "  (str_val empty)" << std::endl;
    } else {
        for (const auto& [key, value] : register_keys_str_val_) {
            std::cerr << "  " << key << ": " << value << std::endl;
        }
    }

    std::cerr << "Custom Parameters:" << std::endl;
    if (custom_parameters_.empty()) {
        std::cerr << "  (custom param empty)" << std::endl;
    } else {
        for (const auto& [key, param_val] : custom_parameters_) {
            std::cerr << "  " << key << ": (size=" << param_val.getSize() << " bytes)";
            if (param_val.type == DataType::INT_TYPE && param_val.getSize() == sizeof(int)) {
                int int_val = *reinterpret_cast<const int*>(param_val.getData());
                std::cerr << " = " << int_val;
            } else if (param_val.type == DataType::FLOAT_ARRAY_TYPE && param_val.getSize() % sizeof(float) == 0) {
                size_t count = param_val.getSize() / sizeof(float);
                const float* float_arr = reinterpret_cast<const float*>(param_val.getData());
                std::cerr << " = [";
                for (size_t i = 0; i < count; ++i) {
                    if (i > 0) std::cerr << ", ";
                    std::cerr << std::fixed << std::setprecision(4) << float_arr[i];
                }
                std::cerr << "]";
            }
            std::cerr << std::endl;
        }
    }
}

bool YamlParser::applyCustomParameters(device_handle device) {
    bool success = true;

    for (const auto& [param_name, param_value] : custom_parameters_) {
        std::cerr << "Setting custom parameter: " << param_name << " (size=" << param_value.getSize() << " bytes)" << std::endl;

        int result = lidar_set_custom_parameter(device, param_name.c_str(), param_value.getData(), param_value.getSize());
        if (result != 0) {
            std::cerr << "Failed to set custom parameter " << param_name << ": error code " << result << std::endl;
            success = false;
        } else {
            std::cerr << "Successfully set custom parameter " << param_name << std::endl;
        }
    }

    return success;
}

int YamlParser::getCustomParameterInt(const std::string& param_name, int default_value) const {
    auto it = custom_parameters_.find(param_name);
    if (it != custom_parameters_.end()) {
        const ParameterValue& param_value = it->second;
        if (param_value.type == DataType::INT_TYPE && param_value.getSize() == sizeof(int)) {
            return *reinterpret_cast<const int*>(param_value.getData());
        }
    }
    return default_value;
}

} 