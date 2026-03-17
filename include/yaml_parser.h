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

/**
 * @file yaml_parser.h
 * @brief YAML 配置文件解析模块
 *
 * 解析 control_command.yaml，提取寄存器键值参数和自定义算法参数，
 * 并将参数同步发送到设备。
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

/// 参数值支持的数据类型枚举
enum class DataType {
    INT_TYPE,         ///< 整型
    FLOAT_ARRAY_TYPE, ///< 浮点数组
    INT_ARRAY_TYPE,   ///< 整型数组
};

/**
 * @brief 通用参数值容器
 *
 * 以二进制方式存储不同类型的参数值，支持整数和数组。
 */
struct ParameterValue {
    DataType type;              ///< 当前存储的数据类型
    std::vector<uint8_t> data;  ///< 原始字节数据

    ParameterValue() : type(DataType::INT_TYPE) {}

    /// 设置单个值（模板化，支持任意 POD 类型）
    template<typename T>
    void setData(const T& value) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
        data.assign(ptr, ptr + sizeof(T));
    }

    /// 设置数组值（模板化，支持任意 POD 类型数组）
    template<typename T>
    void setArray(const std::vector<T>& arr) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(arr.data());
        data.assign(ptr, ptr + arr.size() * sizeof(T));
    }

    /// 返回数据字节数
    size_t getSize() const {
        return data.size();
    }

    /// 返回数据原始指针
    const void* getData() const {
        return data.empty() ? nullptr : data.data();
    }
};

/**
 * @brief YAML 配置解析器
 *
 * 负责加载和解析 control_command.yaml，
 * 区分寄存器键（register_keys）和自定义参数（custom_parameters）。
 */
class YamlParser {
public:
    /// 构造函数，传入配置文件路径
    YamlParser(const std::string& config_file);

    /// 加载并解析配置文件，返回是否成功
    bool loadConfig();

    /// 获取整型寄存器键值映射表
    const std::map<std::string, int>& getRegisterKeys() const;

    /// 获取字符串类型寄存器键值映射表（路径类参数）
    const std::map<std::string, std::string>& getRegisterKeysStrVal() const;

    /// 获取自定义参数映射表（用于发送到设备）
    const std::map<std::string, ParameterValue>& getCustomParameters() const;

    /// 打印当前所有配置参数（调试用）
    void printConfig() const;

    /// 将自定义参数通过 lidar API 发送到设备
    bool applyCustomParameters(device_handle device);

    /// 获取指定自定义参数的整型值，不存在时返回默认值
    int getCustomParameterInt(const std::string& param_name, int default_value) const;

    /// 获取地图模式参数（0: 里程计, 1: SLAM, 2: 重定位）
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
    std::string config_file_;                                      ///< 配置文件路径
    std::map<std::string, int> register_keys_;                     ///< 整型键值对
    std::map<std::string, std::string> register_keys_str_val_;     ///< 字符串键值对
    std::map<std::string, ParameterValue> custom_parameters_;      ///< 自定义参数映射

    /// 允许字符串类型值的键名集合（路径类参数）
    std::unordered_set<std::string> allowed_key_w_str_val = {
        "relocalization_map_abs_path",
        "mapping_result_dest_dir",
        "mapping_result_file_name"
    };
};

} // namespace odin_ros_driver

#endif
