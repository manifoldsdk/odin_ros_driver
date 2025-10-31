#!/bin/bash

# Get the directory where the script is located (Odin_ROS_Driver directory)
PKG_DIR="$(cd "$(dirname "$0")/.."; pwd)"
# Calculate the workspace root directory (contains devel, build, src)
WORKSPACE_ROOT="$(dirname "$(dirname "$PKG_DIR")")"
# Workspace source directory (contains all packages)
WORKSPACE_SRC="${WORKSPACE_ROOT}/src"
PROJECT_NAME="odin_ros_driver"

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' 

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
    echo -e "${YELLOW}Running ROS1 node${NC}"
    
    # Check if environment file exists
    if [ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
        echo -e "${RED}Could not find devel/setup.bash, please build the project with ./build_ros1.sh first${NC}"
        return 1
    fi
    
    # Source environment and run node
    source "${WORKSPACE_ROOT}/devel/setup.bash"
    
}

# Build workspace function
build_workspace() {
    echo -e "${YELLOW}Workspace structure:${NC}"
    echo "  Workspace root: ${WORKSPACE_ROOT}"
    echo "  Source directory: ${WORKSPACE_SRC}"
    echo "  Package directory: ${PKG_DIR}"
    echo "  ROS version: ROS1"
    
    echo -e "${YELLOW}Starting ROS1 project build...${NC}"
    
    # Clean
    cd $WS_DIR
    rm -rf build devel install
    
    # Ensure ROS1 environment is loaded
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source "/opt/ros/noetic/setup.bash"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source "/opt/ros/melodic/setup.bash"
    else
        echo -e "${RED}Could not find ROS1 setup.bash file. Please ensure ROS1 is installed.${NC}"
        return 1
    fi
    
    # Create temporary package.xml
    if [ -f "${PKG_DIR}/package_ros1.xml" ]; then
        echo "Creating temporary package.xml (using package_ros1.xml)"
        cp "${PKG_DIR}/package_ros1.xml" "${PKG_DIR}/package.xml"
        TEMP_PACKAGE=true
    elif [ -f "${PKG_DIR}/package.xml" ]; then
        echo "Using existing package.xml"
    else
        echo -e "${RED}Could not find package.xml in package directory${NC}"
        return 1
    fi
    
    # Set build system variable
    export BUILD_SYSTEM=ROS1
    
    # Switch to workspace root and build
    cd "${WORKSPACE_ROOT}" || return 1
    catkin_make -DBUILD_SYSTEM=ROS1 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -j$(nproc)
    BUILD_RESULT=$?
    
    # If build successful, source environment
    if [[ $BUILD_RESULT -eq 0 ]]; then
        echo -e "${GREEN}ROS1 build successful, loading environment: source devel/setup.bash${NC}"
        source "${WORKSPACE_ROOT}/devel/setup.bash"
    else
        echo -e "${RED}ROS1 build failed, please check error logs${NC}"
    fi
    

}

# Help function
show_help() {
    echo -e "${YELLOW}Usage:${NC}"
    echo "  ./build_ros.sh          # Build project"
    echo "  ./build_ros.sh -c       # Clean build artifacts"
    echo "  ./build_ros.sh -h       # Show help information"
    echo ""
    echo -e "${YELLOW}Current configuration:${NC}"
    echo "  Project name: ${PROJECT_NAME}"
    echo "  Package directory: ${PKG_DIR}"
    echo "  Workspace root: ${WORKSPACE_ROOT}"
    echo "  Source directory: ${WORKSPACE_SRC}"
}

# Main
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