# Git 子模块开发指南

## 项目结构

本SLAM项目现已采用Git子模块结构管理，包含三个独立的仓库：

```
src/
├── navigation_control/     # 自制导航控制包 (dkjsiogu/navigation_control)
├── sllidar_ros2/          # RPLIDAR驱动包 (Slamtec/sllidar_ros2)
└── get-toy/               # 机器人抓取功能包 (zZ-strawberry/get-toy)
```

## 子模块信息

| 子模块 | 远程仓库 | 描述 |
|--------|----------|------|
| navigation_control | https://github.com/dkjsiogu/navigation_control.git | SLAM导航、里程计、Cartographer配置 |
| sllidar_ros2 | https://github.com/Slamtec/sllidar_ros2.git | RPLIDAR激光雷达ROS2驱动 |
| get-toy | https://github.com/zZ-strawberry/get-toy.git | 机器人抓取和目标检测功能 |

## 开发工作流

### 初次克隆项目

```bash
# 克隆主仓库
git clone https://github.com/dkjsiogu/slam.git
cd slam

# 初始化并更新所有子模块
git submodule update --init --recursive
```

### 更新子模块

```bash
# 更新特定子模块到最新版本
git submodule update --remote src/navigation_control

# 更新所有子模块
git submodule update --remote

# 如果子模块有变更，提交到主仓库
git add .
git commit -m "Update submodules"
```

### 在子模块中开发

```bash
# 进入子模块目录
cd src/navigation_control

# 确保在正确分支上（默认会处于detached HEAD状态）
git checkout main

# 正常开发...
echo "some changes" >> file.txt
git add file.txt
git commit -m "Add some changes"

# 推送到子模块仓库
git push origin main

# 回到主仓库，更新子模块引用
cd ../..
git add src/navigation_control
git commit -m "Update navigation_control submodule"
git push
```

### 构建系统

```bash
# 正常的ROS2构建流程保持不变
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 注意事项

### 子模块状态管理

1. **Detached HEAD**: 子模块默认处于detached HEAD状态，开发前请切换到具体分支
2. **同步问题**: 子模块更新后需要在主仓库中提交引用变更
3. **递归操作**: 某些Git操作需要使用 `--recursive` 参数

### 权限管理

- `navigation_control`: 你有写权限，可以直接推送
- `sllidar_ros2`: 上游Slamtec仓库，建议Fork后使用你的Fork版本
- `get-toy`: 其他开发者仓库，需确认写权限

### 建议工作流

1. **功能开发**: 在对应子模块中进行，使用正常Git工作流
2. **集成测试**: 在主仓库中测试整个系统集成
3. **版本发布**: 先发布子模块版本，再更新主仓库中的引用

## 故障排除

### 子模块未正确下载

```bash
git submodule update --init --recursive --force
```

### 子模块指向错误版本

```bash
cd src/navigation_control
git checkout main
cd ../..
git add src/navigation_control
git commit -m "Fix submodule reference"
```

### 恢复到特定版本

```bash
# 查看子模块历史
git log --oneline --graph --submodule

# 重置到特定提交
git checkout <commit-hash>
git submodule update --recursive
```

## 相关命令参考

```bash
# 查看子模块状态
git submodule status

# 查看子模块配置
cat .gitmodules

# 删除子模块（如需要）
git submodule deinit src/navigation_control
git rm src/navigation_control
rm -rf .git/modules/src/navigation_control

# 添加新子模块
git submodule add <repository-url> <local-path>
```

这种结构让每个功能包都能独立开发和版本管理，同时保持整个SLAM系统的集成性。