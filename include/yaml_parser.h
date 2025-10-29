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
#ifndef YAML_PARSER_H
#define YAML_PARSER_H

#include <cstdio>
#include <string>
#include <map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "lidar_api.h"

namespace odin_ros_driver {

// Data type enum for supporting different value types
enum class DataType {
    INT_TYPE,
    FLOAT_ARRAY_TYPE,
    INT_ARRAY_TYPE,
};

// Generic parameter value holder
struct ParameterValue {
    DataType type;
    std::vector<uint8_t> data;

    ParameterValue() : type(DataType::INT_TYPE) {}

    template<typename T>
    void setData(const T& value) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
        data.assign(ptr, ptr + sizeof(T));
    }

    template<typename T>
    void setArray(const std::vector<T>& arr) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(arr.data());
        data.assign(ptr, ptr + arr.size() * sizeof(T));
    }

    size_t getSize() const {
        return data.size();
    }

    const void* getData() const {
        return data.empty() ? nullptr : data.data();
    }
};

class YamlParser {
public:
    YamlParser(const std::string& config_file);

    bool loadConfig();
    const std::map<std::string, int>& getRegisterKeys() const;
    const std::map<std::string, std::string>& getRegisterKeysStrVal() const;
    const std::map<std::string, ParameterValue>& getCustomParameters() const;
    void printConfig() const;
    bool applyCustomParameters(device_handle device);
    int getCustomParameterInt(const std::string& param_name, int default_value) const;

    int getCustomMapMode(int default_value) const {
        auto it = custom_parameters_.find("map_mode");
        if (it != custom_parameters_.end() && it->second.type == DataType::INT_TYPE) {
            printf("custom_map_mode = %d\n", *(int*)it->second.getData());
            return *(int*)it->second.getData();
        } else {
            return default_value;
        }
    };

private:
    std::string config_file_;
    std::map<std::string, int> register_keys_;
    std::map<std::string, std::string> register_keys_str_val_;
    std::map<std::string, ParameterValue> custom_parameters_;

    std::unordered_set<std::string> allowed_key_w_str_val = {"relocalization_map_abs_path", "mapping_result_dest_dir", "mapping_result_file_name"};
};

}

#endif