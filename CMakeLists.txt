cmake_minimum_required(VERSION 3.5)
project(odin_ros_driver)


if(DEFINED BUILD_SYSTEM)
    set(ROS_VERSION ${BUILD_SYSTEM})
    message(STATUS "使用命令行指定的构建系统: ${ROS_VERSION}")
elseif(DEFINED ENV{ROS_DISTRO})
    if("$ENV{ROS_DISTRO}" MATCHES "foxy|galactic|humble|iron|rolling")
        set(ROS_VERSION "ROS2")
    else()
        set(ROS_VERSION "ROS1")
    endif()
elseif(DEFINED ENV{ROS_VERSION})
    if("$ENV{ROS_VERSION}" EQUAL "2")
        set(ROS_VERSION "ROS2")
    else()
        set(ROS_VERSION "ROS1")
    endif()
else()
    # 尝试自动检测
    if(COMMAND catkin_package)
        set(ROS_VERSION "ROS1")
    elseif(COMMAND ament_package)
        set(ROS_VERSION "ROS2")
    else()
        # 默认使用ROS2
        set(ROS_VERSION "ROS2")
        message(WARNING "无法确定ROS版本，默认使用ROS2")
    endif()
endif()

# 检测 ROS 版本后添加编译宏
if(ROS_VERSION STREQUAL "ROS2")
    add_definitions(-DROS2)
    message(STATUS "定义 ROS2 宏")
else()
    add_definitions(-DROS1)
    message(STATUS "定义 ROS1 宏")
endif()

message(STATUS "构建系统: ${ROS_VERSION}")

# 平台检测
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(ARCH STREQUAL "x86_64")
    set(TARGET_PLATFORM "x86")
    message(STATUS "检测到 x86_64 架构")
elseif(ARCH MATCHES "arm|aarch64")
    set(TARGET_PLATFORM "arm")
    message(STATUS "检测到 ARM 架构: ${ARCH}")
else()
    message(WARNING "不支持的架构: ${ARCH}. 使用默认设置")
    set(TARGET_PLATFORM "unknown")
endif()

# 设置库路径
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "库目录: ${LIB_DIR}")

# 根据平台设置库名称
if(TARGET_PLATFORM STREQUAL "arm")
    set(LYD_HOST_API_LIB_NAME "lydHostApi_arm")
else()
    set(LYD_HOST_API_LIB_NAME "lydHostApi_amd")
endif()

# 查找预编译的 lydHostApi 库
find_library(LYD_HOST_API_LIB
    NAMES 
        ${LYD_HOST_API_LIB_NAME}
        lib${LYD_HOST_API_LIB_NAME}.a
        lib${LYD_HOST_API_LIB_NAME}.so
    PATHS ${LIB_DIR}
    NO_DEFAULT_PATH
)

if(LYD_HOST_API_LIB)
    message(STATUS "找到 lydHostApi 库: ${LYD_HOST_API_LIB}")
else()
    file(GLOB LIB_FILES "${LIB_DIR}/lib${LYD_HOST_API_LIB_NAME}.*")
    if(LIB_FILES)
        message(STATUS "找到库文件: ${LIB_FILES}")
        set(LYD_HOST_API_LIB ${LIB_FILES})
    else()
        message(FATAL_ERROR "在 ${LIB_DIR} 中找不到预编译的 lydHostApi 库")
    endif()
endif()

# 设置公共编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 公共依赖查找
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# 共享包含目录
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIR}
    ${LIBUSB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 共享链接库列表
set(COMMON_LIBS
    ${OpenCV_LIBS}
    ${yaml-cpp_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    pthread
    rt
    ${CMAKE_DL_LIBS}
    ${LYD_HOST_API_LIB}
)

# ===== ROS1 专属配置 =====
if(ROS_VERSION STREQUAL "ROS1")
    message(STATUS "配置为 ROS1 构建")
    
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
    )
    
    include_directories(${catkin_INCLUDE_DIRS})
    
    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs cv_bridge image_transport
        INCLUDE_DIRS include
    )
    
    # 设置输出目录
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
    
    add_executable(host_sdk_sample 
        src/host_sdk_sample.cpp
        src/yaml_parser.cpp
    )
    target_link_libraries(host_sdk_sample
        ${catkin_LIBRARIES}
        ${COMMON_LIBS}
	${LYD_HOST_API_LIB}
    	${LIBUSB_LIBRARIES}
    	yaml-cpp
    	${OpenCV_LIBS}
    	pthread  
        usb-1.0
    )
    
    # 安装规则
    install(TARGETS host_sdk_sample
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    
    install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
    )
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
        install(DIRECTORY config/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )
    endif()

# ===== ROS2 配置 =====
elseif(ROS_VERSION STREQUAL "ROS2")
    message(STATUS "配置为 ROS2 构建")
    
    # 查找所有必要的ROS2包
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    
    # 创建可执行文件
    add_executable(host_sdk_sample 
        src/host_sdk_sample.cpp
        src/yaml_parser.cpp  
        )
    
    # 链接库
    target_link_libraries(host_sdk_sample
        ${catkin_LIBRARIES}
        ${COMMON_LIBS}
        yaml-cpp 
        usb-1.0
    )
    
    # 添加ROS2依赖
    ament_target_dependencies(host_sdk_sample
        rclcpp 
        std_msgs 
        sensor_msgs 
        nav_msgs 
        cv_bridge
        image_transport
    )
    
    # 安装规则 - 确保所有安装目标在 ament_package() 之前定义
    # 安装可执行文件
    install(TARGETS host_sdk_sample
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    
    # 安装 package.xml
    install(FILES package.xml
        DESTINATION share/${PROJECT_NAME}
    )
    
    # 安装头文件
    install(DIRECTORY include/
        DESTINATION include
    )
    # 安装 launch_ROS2 目录
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch_ROS2")
        install(DIRECTORY launch_ROS2/
            DESTINATION share/${PROJECT_NAME}/launch
        )
        message(STATUS "安装 launch_ROS2 目录到 share/${PROJECT_NAME}/launch")
    else()
        message(WARNING "未找到 launch_ROS2 目录")
    endif()
    # 安装配置文件 (如果存在)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
        install(DIRECTORY config/
            DESTINATION share/${PROJECT_NAME}/config
        )
    endif()
    
    # 安装启动文件 (如果存在)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
        install(DIRECTORY launch/
            DESTINATION share/${PROJECT_NAME}/launch
        )
    endif()
    ament_export_targets(export_${PROJECT_NAME})
    # 声明依赖
    ament_export_dependencies(
        rclcpp 
        std_msgs 
        sensor_msgs 
        nav_msgs 
        cv_bridge
        image_transport
    )
    
    # 可选的lint检查
    
    # 完成包配置 - 必须在所有安装规则之后调用
    ament_package()
    message(STATUS "安装目标已添加")
    
else()
    message(FATAL_ERROR "无效的 ROS_VERSION: ${ROS_VERSION}")
endif()

# ARM平台特定的链接选项
if(TARGET_PLATFORM STREQUAL "arm")
    set_target_properties(host_sdk_sample PROPERTIES
        LINK_FLAGS "-Wl,--no-as-needed -Wl,--rpath=${LIB_DIR}"
    )
    message(STATUS "添加 ARM 特定的链接选项和 RPATH")
endif()

# 添加调试信息
message(STATUS "=======================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "ROS 版本: ${ROS_VERSION}")
message(STATUS "Target platform: ${TARGET_PLATFORM}")
message(STATUS "lydHostApi library: ${LYD_HOST_API_LIB}")
message(STATUS "libusb library: ${LIBUSB_LIBRARIES}")
message(STATUS "=======================================")
