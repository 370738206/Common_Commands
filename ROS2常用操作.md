# Learn ROS2

参考资料 *ROS2入门21讲 · 古月*

- 课程视频：[https://class.guyuehome.com/detail/p_628f4288e4b01c509ab5bc7a/6](https://class.guyuehome.com/detail/p_628f4288e4b01c509ab5bc7a/6)
- 图文教程：[https://book.guyuehome.com/](https://book.guyuehome.com/)

## 检查环境变量,主要注意*rosdistro*

```Shell
printenv | grep -i ROS
```

## 创建功能包

在**工作空间/src**目录下，使用

```Shell
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>
```

### 普通的工具包需要增加的内容

在*setup.py*文件中指定功能入口,起始位置在21行,实例如下

```Python
    entry_points={
        'console_scripts': [
            'l_service_add_client = llearn_service.l_service_add_client:main',
            'l_service_add_server = llearn_service.l_service_add_server:main',
        ],
```

### 接口文件的工具包需要增加的内容

> CmakeLists.txt文件,起始添加位置为13行

```C++
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/AddTwoInts.srv"
    "srv/GetObjectPosition.srv"
    )
```

> package.xml文件,起始添加位置为15行

```XML
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

## 自动安装依赖并编译

### 安装依赖,在**工作空间**目录下，使用

```Shell
rosdep install -i --from-path src --rosdistro galactic -y
```

依赖安装无误,且已经在*setup.py*中指定功能入口后,编译

```Shell
colcon build
```

## 调包方式,启动相机并发布话题

```Shell
ros2 run usb_cam usb_cam_node_exe
```

## 固定结构

```Python
def __init__(self, name):
    super().__init__(name) #ROS2 父类节点初始化

def main(args=None):
    rclpy.init(args=args)
    node = 
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
```

### Topic

```Python
    # Pub
    self.pub = self.create_publisher(String, 'chatter_name', 10)    # 创建发布者对象（消息类型，话题名，队列长度）

    self.pub.publish(data)

    # Sub
    self.sub = self.create_subscription(String, 'chatter_name', self.sub_call, 10)      # 创建接收者对象（消息类型，话题名，回调函数，队列长度）
```

### Service

```Python
    # Server
    self.client = self.create_client(GetObjectPosition, 'object_detect')
    while not self.client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not aviliable, waiting again...')   # 循环等待服务器启动成功
    self.request = GetObjectPosition.Request()  # 创建服务请求的数据对象

    def send_request(self):
        self.request.get = True
        self.process = self.client.call_async(self.request)  # 异步方式发送服务请求

    #Client
    self.sub = self.create_subscription(Image, 'image_raw', self.sub_call, 10)      # 创建接收者对象（消息类型，话题名，回调函数，队列长度）
```

### Action

```Python
    # Server
    self.action_server = ActionServer(
        self, MoveCircle, 'move_circle_test', self.execute_callback)
    # Client
    self.action_client = ActionClient(self, MoveCircle, 'move_circle_test')

    def send_goal(self, flag):                                          # 发送目标动作的函数

        self.send_goal_feature = self.action_client.send_goal_async(
            goal_mes, feedback_callback=self.feedback_callback)         # 处理周期反馈消息的回调函数

        self.send_goal_feature.add_done_callback(
            self.goal_response_callback) 
            
    def goal_response_callback(self, data):
        action_goal = data.result()                                     # 接收动作的结果

        self.get_result_feature = action_goal.get_result_async()        # 异步获取动作最终执行的结果反馈
        self.get_result_feature.add_done_callback(
            self.get_result_callback)                                   # 设置一个收到最终结果的回调函数
     
```

## 文件路径

原始文件
---/workspace/src/<package_name>

编译后的文件
---/workspace/install/<package_name>/lib/python3.8/site-packages/<package_name>

## DOMAIN

export ROS_DOMAIN_ID=<your_domain_id>
