#!/bin/bash

# Get the directory where the script is located (Odin_ROS_Driver directory)
PKG_DIR="$(cd "$(dirname "$0")/.."; pwd)"
# Calculate the workspace root directory (contains devel, build, src)
WORKSPACE_ROOT="$(dirname "$(dirname "$PKG_DIR")")"
# Workspace source directory (contains all packages)
WORKSPACE_SRC="${WORKSPACE_ROOT}/src"
PROJECT_NAME="odin_ros_driver"
PACKAGE_DIR_NAME=$(basename "$PKG_DIR")

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Extract package name from package.xml
get_package_name() {
    local package_xml="$1"
    if [ -f "$package_xml" ]; then
        # Extract content of <name> tag
        grep -oP '<name>\K[^<]+' "$package_xml" | head -1
    else
        echo ""
    fi
}

# Clean workspace function
clean_workspace() {
    echo -e "${YELLOW}Cleaning build directories${NC}"
    
    # Clean build artifacts in workspace
    rm -rf "${WORKSPACE_ROOT}/build" 
    rm -rf "${WORKSPACE_ROOT}/install" 
    rm -rf "${WORKSPACE_ROOT}/log"
    rm -rf "${WORKSPACE_ROOT}/devel"
    
    echo -e "${GREEN}Cleanup complete${NC}"
}

# Run node function
run_node() {
    echo -e "${YELLOW}Running ROS2 node${NC}"
    
    # Check if environment file exists
    if [ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
        echo -e "${RED}Could not find install/setup.bash, please build the project with ./build_ros2.sh first${NC}"
        return 1
    fi
    
    # Source environment and run node
    source "${WORKSPACE_ROOT}/install/setup.bash"
    
}

# Build workspace function
build_workspace() {
    echo -e "${YELLOW}Workspace structure:${NC}"
    echo "  Workspace root: ${WORKSPACE_ROOT}"
    echo "  Source directory: ${WORKSPACE_SRC}"
    echo "  Package directory: ${PKG_DIR}"
    echo "  Directory name: ${PACKAGE_DIR_NAME}"
    echo "  ROS version: ROS2"
    
    echo -e "${YELLOW}Starting ROS2 project build...${NC}"

    cd $WS_DIR
    rm -rf build install log
    # Ensure ROS2 environment is loaded
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        source "/opt/ros/foxy/setup.bash"
    elif [ -f "/opt/ros/galactic/setup.bash" ]; then
        source "/opt/ros/galactic/setup.bash"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source "/opt/ros/humble/setup.bash"
    else
        echo -e "${RED}Could not find ROS2 setup.bash file. Please ensure ROS2 is installed.${NC}"
        return 1
    fi
    
    # Create temporary package.xml
    if [ -f "${PKG_DIR}/package_ros2.xml" ]; then
        echo "Creating temporary package.xml (using package_ros2.xml)"
        cp "${PKG_DIR}/package_ros2.xml" "${PKG_DIR}/package.xml"
        TEMP_PACKAGE=true
    elif [ -f "${PKG_DIR}/package.xml" ]; then
         echo "Using existing package.xml"
        TEMP_PACKAGE=false
    else
        echo -e "${RED}Could not find package.xml in package directory${NC}"
        return 1
    fi
    
    # Extract package name from package.xml
    PACKAGE_NAME=$(get_package_name "${PKG_DIR}/package.xml")
    if [ -z "$PACKAGE_NAME" ]; then
        echo -e "${RED}Failed to extract package name from package.xml${NC}"
        return 1
    fi
    echo "  Package name: ${PACKAGE_NAME}"
    
    # Set build system variable
    export BUILD_SYSTEM=ROS2
    
    # Switch to workspace root and build
    cd "${WORKSPACE_ROOT}" || return 1
    
    # Build with correct package name
    colcon build \
        --packages-select "${PACKAGE_NAME}" \
        --parallel-workers $(nproc) \
        --cmake-args \
            -DBUILD_SYSTEM=ROS2 \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    
    BUILD_RESULT=$?
    
    # If build successful, source environment
    if [[ $BUILD_RESULT -eq 0 ]]; then
        echo -e "${GREEN}ROS2 build successful, loading environment: source install/setup.bash${NC}"
        source "${WORKSPACE_ROOT}/install/setup.bash"
        
    else
        echo -e "${RED}ROS2 build failed, please check error logs${NC}"
    fi
    
}

# Help function
show_help() {
    echo -e "${YELLOW}Usage:${NC}"
    echo "  ./build_ros2.sh          # Build project"
    echo "  ./build_ros2.sh -c       # Clean build artifacts"
    echo "  ./build_ros2.sh -h       # Show help information"
    echo ""
    echo -e "${YELLOW}Current configuration:${NC}"
    echo "  Project name: ${PROJECT_NAME}"
    echo "  Package directory: ${PKG_DIR}"
    echo "  Workspace root: ${WORKSPACE_ROOT}"
    echo "  Source directory: ${WORKSPACE_SRC}"
}

# Main program
case "$1" in
    -c|--clean)
        clean_workspace
        ;;
    -r|--run)
        run_node
        ;;
    -h|--help)
        show_help
        ;;
    *)
        build_workspace
        ;;
esac