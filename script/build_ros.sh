#!/bin/bash

# 获取脚本所在目录（Odin_ROS_Driver 目录）
PKG_DIR="$(cd "$(dirname "$0")/.."; pwd)"
# 计算工作空间根目录（包含 devel、build、src 的目录）
WORKSPACE_ROOT="$(dirname "$(dirname "$PKG_DIR")")"
# 工作空间源码目录（包含所有包的 src）
WORKSPACE_SRC="${WORKSPACE_ROOT}/src"
PROJECT_NAME="odin_ros_driver"

# 定义颜色代码
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # 无颜色

# 清理函数
clean_workspace() {
    echo -e "${YELLOW}🧹 清理构建目录${NC}"
    
    # 清理工作空间根目录下的构建产物
    rm -rf "${WORKSPACE_ROOT}/build" 
    rm -rf "${WORKSPACE_ROOT}/install" 
    rm -rf "${WORKSPACE_ROOT}/log"
    rm -rf "${WORKSPACE_ROOT}/devel"
    
    echo -e "${GREEN}✅ 清理完成${NC}"
}

# 运行函数
run_node() {
    echo -e "${YELLOW}🏃‍♂️‍➡️ 运行 ROS1 节点${NC}"
    
    # 检查环境文件是否存在
    if [ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
        echo -e "${RED}❌ 找不到 devel/setup.bash，请先执行 ./build_ros1.sh 构建项目${NC}"
        return 1
    fi
    
    # Source 环境并运行节点
    source "${WORKSPACE_ROOT}/devel/setup.bash"
    
}

# 构建函数
build_workspace() {
    echo -e "${YELLOW}🔍 工作空间结构:${NC}"
    echo "  工作空间根目录: ${WORKSPACE_ROOT}"
    echo "  源码目录: ${WORKSPACE_SRC}"
    echo "  包目录: ${PKG_DIR}"
    echo "  ROS版本: ROS1"
    
    echo -e "${YELLOW}🔧 开始构建 ROS1 工程...${NC}"
    
    # 清理
    cd $WS_DIR
    rm -rf build devel install
    
    # 确保 ROS1 环境已加载
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source "/opt/ros/noetic/setup.bash"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source "/opt/ros/melodic/setup.bash"
    else
        echo -e "${RED}❌ 找不到 ROS1 的 setup.bash 文件。请确保 ROS1 已安装。${NC}"
        return 1
    fi
    
    # 创建临时 package.xml
    if [ -f "${PKG_DIR}/package_ros1.xml" ]; then
        echo "🔄 创建临时 package.xml（使用 package_ros1.xml）"
        cp "${PKG_DIR}/package_ros1.xml" "${PKG_DIR}/package.xml"
        TEMP_PACKAGE=true
    elif [ -f "${PKG_DIR}/package.xml" ]; then
        echo "ℹ️ 使用现有的 package.xml"
    else
        echo -e "${RED}❌ 在包目录中找不到 package.xml${NC}"
        return 1
    fi
    
    # 设置构建系统变量
    export BUILD_SYSTEM=ROS1
    
    # 切换到工作空间根目录并构建
    cd "${WORKSPACE_ROOT}" || return 1
    catkin_make -DBUILD_SYSTEM=ROS1 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -j$(nproc)
    BUILD_RESULT=$?
    
    # 构建成功，source 环境
    if [[ $BUILD_RESULT -eq 0 ]]; then
        echo -e "${GREEN}✅ ROS1 构建成功，载入环境变量：source devel/setup.bash${NC}"
        source "${WORKSPACE_ROOT}/devel/setup.bash"
    else
        echo -e "${RED}❌ ROS1 构建失败，请检查错误日志${NC}"
    fi
    

}

# 帮助函数
show_help() {
    echo -e "${YELLOW}使用说明:${NC}"
    echo "  ./build_ros.sh          # 构建项目"
    echo "  ./build_ros.sh -c       # 清理构建产物"
    echo "  ./build_ros.sh -h       # 显示帮助信息"
    echo ""
    echo -e "${YELLOW}当前配置:${NC}"
    echo "  项目名称: ${PROJECT_NAME}"
    echo "  包目录: ${PKG_DIR}"
    echo "  工作空间根目录: ${WORKSPACE_ROOT}"
    echo "  源码目录: ${WORKSPACE_SRC}"
}

# 主程序
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