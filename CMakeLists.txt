cmake_minimum_required(VERSION 3.5)
project(odin_ros_driver)


if(DEFINED BUILD_SYSTEM)
    set(ROS_VERSION ${BUILD_SYSTEM})
    message(STATUS "ROS_VERSION: ${ROS_VERSION}")
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
    # Attempt automatic detection
    if(COMMAND catkin_package)
        set(ROS_VERSION "ROS1")
    elseif(COMMAND ament_package)
        set(ROS_VERSION "ROS2")
    else()
        # Default to ROS2
        set(ROS_VERSION "ROS2")
        message(WARNING "Unable to determine ROS version, defaulting to ROS2")
    endif()
endif()

# Add compile definitions after detecting ROS version
if(ROS_VERSION STREQUAL "ROS2")
    add_definitions(-DROS2)
    message(STATUS "Defining ROS2")
else()
    add_definitions(-DROS1)
    message(STATUS "Defining ROS1")
endif()

message(STATUS "Build system: ${ROS_VERSION}")

# Platform detection
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(ARCH STREQUAL "x86_64")
    set(TARGET_PLATFORM "x86")
    message(STATUS "Detected x86_64 architecture")
elseif(ARCH MATCHES "arm|aarch64")
    set(TARGET_PLATFORM "arm")
    message(STATUS "Detected ARM architecture: ${ARCH}")
else()
    message(WARNING "Unsupported architecture: ${ARCH}. Using default settings")
    set(TARGET_PLATFORM "unknown")
endif()

# Set library path
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "Library directory: ${LIB_DIR}")

# Set library name based on platform
if(TARGET_PLATFORM STREQUAL "arm")
    set(LYD_HOST_API_LIB_NAME "lydHostApi_arm")
else()
    set(LYD_HOST_API_LIB_NAME "lydHostApi_amd")
endif()

# Find precompiled lydHostApi library
find_library(LYD_HOST_API_LIB
    NAMES 
        ${LYD_HOST_API_LIB_NAME}
        lib${LYD_HOST_API_LIB_NAME}.a
        lib${LYD_HOST_API_LIB_NAME}.so
    PATHS ${LIB_DIR}
    NO_DEFAULT_PATH
)

if(LYD_HOST_API_LIB)
    message(STATUS "Found lydHostApi library: ${LYD_HOST_API_LIB}")
else()
    file(GLOB LIB_FILES "${LIB_DIR}/lib${LYD_HOST_API_LIB_NAME}.*")
    if(LIB_FILES)
        message(STATUS "Found library files: ${LIB_FILES}")
        set(LYD_HOST_API_LIB ${LIB_FILES})
    else()
        message(FATAL_ERROR "Could not find precompiled lydHostApi library in ${LIB_DIR}")
    endif()
endif()

# Set common compile options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find common dependencies
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Shared include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIR}
    ${LIBUSB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Shared library list
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

# ===== ROS1 Configuration =====
if(ROS_VERSION STREQUAL "ROS1")
    message(STATUS "Configuring for ROS1 build")
    
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
    
    # Set output directories
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
    
    # Installation rules
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

# ===== ROS2 Configuration =====
elseif(ROS_VERSION STREQUAL "ROS2")
    message(STATUS "Configuring for ROS2 build")
    
    # Find all necessary ROS2 packages
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    
    # Create executable
    add_executable(host_sdk_sample 
        src/host_sdk_sample.cpp
        src/yaml_parser.cpp  
        )
    
    # Link libraries
    target_link_libraries(host_sdk_sample
        ${catkin_LIBRARIES}
        ${COMMON_LIBS}
        yaml-cpp 
        usb-1.0
    )
    
    # Add ROS2 dependencies
    ament_target_dependencies(host_sdk_sample
        rclcpp 
        std_msgs 
        sensor_msgs 
        nav_msgs 
        cv_bridge
        image_transport
    )
    
    # Installation rules - ensure all install targets are defined before ament_package()
    # Install executable
    install(TARGETS host_sdk_sample
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    
    # Install package.xml
    install(FILES package.xml
        DESTINATION share/${PROJECT_NAME}
    )
    
    # Install headers
    install(DIRECTORY include/
        DESTINATION include
    )
    # Install launch_ROS2 directory
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch_ROS2")
        install(DIRECTORY launch_ROS2/
            DESTINATION share/${PROJECT_NAME}/launch
        )
        message(STATUS "Installing launch_ROS2 directory to share/${PROJECT_NAME}/launch")
    else()
        message(WARNING "launch_ROS2 directory not found")
    endif()
    # Install config files
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
        install(DIRECTORY config/
            DESTINATION share/${PROJECT_NAME}/config
        )
    endif()
    
    # Install launch files
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
        install(DIRECTORY launch/
            DESTINATION share/${PROJECT_NAME}/launch
        )
    endif()
    ament_export_targets(export_${PROJECT_NAME})
    # Declare dependencies
    ament_export_dependencies(
        rclcpp 
        std_msgs 
        sensor_msgs 
        nav_msgs 
        cv_bridge
        image_transport
    )
    
    ament_package()
    message(STATUS "Install targets added")
    
else()
    message(FATAL_ERROR "Invalid ROS_VERSION: ${ROS_VERSION}")
endif()

# ARM platform specific link options
if(TARGET_PLATFORM STREQUAL "arm")
    set_target_properties(host_sdk_sample PROPERTIES
        LINK_FLAGS "-Wl,--no-as-needed -Wl,--rpath=${LIB_DIR}"
    )
    message(STATUS "Adding ARM-specific link options and RPATH")
endif()

# Add debug information
message(STATUS "=======================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "ROS_VERSION: ${ROS_VERSION}")
message(STATUS "Target platform: ${TARGET_PLATFORM}")
message(STATUS "lydHostApi library: ${LYD_HOST_API_LIB}")
message(STATUS "libusb library: ${LIBUSB_LIBRARIES}")
message(STATUS "=======================================")
