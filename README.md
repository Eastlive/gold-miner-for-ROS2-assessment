# gold-miner-for-ROS2-assessment
该仓库为四川大学火锅战队视觉组24届ROS2考核仓库，内容为黄金矿工游戏。

该仓库可作为ROS2基础模板参考和使用。
## 项目说明
- [miner_interfaces](./miner_interfaces/): 黄金矿工自定义ROS消息接口
- [mine_publisher](./mine_publisher/): 矿石生成器及矿石位置发布者
- [mine_service](./mine_service/): 矿石地图服务器节点
- [miner_client](./miner_client/): 采矿者客户端节点

## 项目依赖
- ROS2 Humble

## 使用方法
1. 克隆仓库到本地
```bash
git clone
```
2. 编译
```bash
colcon build
```
3. 运行
```bash
ros2 run mine_publisher mine_publisher_node 
ros2 run mine_service mine_service_node 
ros2 run miner_client miner_client_node 
```
## 项目结构
```
.
├── mine_publisher
│   ├── CMakeLists.txt
│   ├── config
│   │   └── mine_publisher.yaml
│   ├── include
│   │   └── mine_publisher
│   │       └── mine_publisher_node.hpp
│   ├── launch
│   │   └── mine_publisher.launch.py
│   ├── package.xml
│   ├── src
│   │   └── mine_publisher_node.cpp
│   └── test
│       └── CMakeLists.txt
├── miner_client
│   ├── CMakeLists.txt
│   ├── include
│   │   └── miner_client
│   │       └── miner_client_node.hpp
│   ├── launch
│   │   └── miner_client.launch.py
│   ├── package.xml
│   └── src
│       └── miner_client_node.cpp
├── miner_interfaces
│   ├── CMakeLists.txt
│   ├── msg
│   │   ├── Ore.msg
│   │   └── Ores.msg
│   ├── package.xml
│   └── srv
│       └── MineMap.srv
├── mine_service
│   ├── CMakeLists.txt
│   ├── include
│   │   └── mine_service
│   │       └── mine_service_node.hpp
│   ├── launch
│   ├── package.xml
│   ├── src
│   │   └── mine_service_node.cpp
│   └── test
│       ├── CMakeLists.txt
│       └── test_mine_service.cpp
└── README.md
```
## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
