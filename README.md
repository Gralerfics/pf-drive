# tr-drive
Teach and repeat driving controller.

## Information

**Language**: Python

**Dependence**: ROS Noetic, OpenCV, DearPyGUI (imgui)

**OS**: Ubuntu 20.04

**Simulator**: Webots

**Reference**: https://arxiv.org/pdf/2010.11326.pdf

## Structure

`config/`：参数文件，由 .launch 文件引用。此为**默认**参数，预期 GUI 启动后可更改。

---

`launch/`：ROS 启动文件。

使用见后。

---

`nodes/`：launch 中启动的节点脚本。

添加节点脚本注意在 CMakeLists.txt 中 170 左右 catkin_install_python 内注册。

其他依赖同理。

---

`rviz/`：预设 rviz 文件。

---

`src/`：源代码，**具体见各脚本内部注释**。

`src/tr_drive/`：tr_drive 包主体，所有功能都在该包内，注册于 setup.py 内，source 后可被引用。

`src/tr_drive/controller`：控制器，包含 goal_controller.py 内的 GoalController，接收 Odometry，并根据设置的目标位姿，发布 Twist 到 cmd_vel。

`src/tr_drive/operator`：作为节点的脚本，即 Teacher 和 Repeater。

`src/tr_drive/persistent`：持久化存储相关，保存和读取 recording（即示教数据）。提供从文件夹读取到类实例、从实例保存到文件夹等操作。

`src/tr_drive/sensor`：包括 `Camera` 和 `Odom`。前者订阅摄像头图像并进行降采样、
patch normalization 处理，后者订阅里程计 Odometry 数据，转为 Frame 并减掉设定的 bias（置零操作）。

`src/tr_drive/util`：工具类。`geometry.py` 中定义了 `Vec3`、`Quat`、`Frame` 等空间运算类，并提供与 ROS 消息间的转换，统一类型操作；`debug.py` 中定义了统一的 publisher 管理器，用于简便地发布调试消息（用 rviz 查看），按约定每个组件类中都会定义一个 `self.debugger`；`image.py` 中提供了统一的 `DigitialImage` 图像类，和本项目需要用到的一些图像处理方法；`namespace.py` 仅提供一个递归定义的、用于字典转对象结构的类。

---

`simulations/`：仿真资源，例如 Webots 场景文件。

## Usages

### 启动

启动仿真环境（Turtlebot 3）：

```
roslaunch tr_drive tb3_wbt.launch [rviz:=false]
```

启动 Teacher：

```
roslaunch tr_drive tb3_teach_wbt.launch folder:="..." [name:='.../.auto'] [teleop_keyboard:=false]
```

启动 Repeater：

```
roslaunch tr_drive tb3_repeat_wbt.launch path:="..." [report:=".../.auto/.none"]
```

### 评估

在 repeater 启动时传入 report 参数作为报告的路径，运行结束后会产生路径文件。

截至编辑时，report 文件夹内容为 TUM 格式的文本两个 (traj_teach.txt 和 traj_repeat.txt)。

可以使用 evo 绘图或评估，例如：

```
evo_traj tum traj_repeat.txt --ref=traj_teach.txt -p --plot_mode=xy
evo_ape tum traj_teach.txt traj_repeat.txt --plot --plot_mode=xy
```

## Notes

### 线程安全

对外 public、可能在回调函数中或同时在多处被引用的成员都应定义对应的 `..._lock = threading.Lock()` 加锁，并提供 `get_/set_` 等安全的操作方法。

### 参数服务器

参数最外级名称为 `tr`，内部为各组件的命名空间，统一便于管理。使用 `src/tr_drive/util/namespace.py` 中的 `DictRegulator` 将 `dict` 转为对象结构，例如：

```
(teacher.py)
...
self.params = DictRegulator(rospy.get_param('/tr'))
...
```

不在参数文件中，需要后续添加的参数操作样例见 `src/tr_drive/operator/repeater.py` 中 `__init__` 中的参数部分。

读取参数服务器皆在 `operator` 中直接作为节点的脚本中，`Camera` 和 `Odom` 等的参数皆应依靠构造器传递。

### TODO
