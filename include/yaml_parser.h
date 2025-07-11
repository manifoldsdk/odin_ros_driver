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

#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

namespace odin_ros_driver {

class YamlParser {
public:
    // 使用一致的成员变量名
    YamlParser(const std::string& config_file);
    
    bool loadConfig();
    const std::map<std::string, int>& getRegisterKeys() const;
    void printConfig() const;

private:
    std::string config_file_; 
    std::map<std::string, int> register_keys_;
};

} // namespace odin_ros_driver

#endif // YAML_PARSER_H