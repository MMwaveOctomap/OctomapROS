# ROS简介

机器人操作系统（ROS）是用于编写机器人软件的灵活框架。它是工具，库和约定的集合，旨在简化跨各种机器人平台创建复杂而强大的机器人行为的任务。

为什么？因为创建真正强大的通用机器人软件很困难。从机器人的角度来看，在人类看来微不足道的问题通常在任务实例和环境实例之间千差万别。处理这些变化是如此困难，以至没有任何一个个人，实验室或机构可以希望自己做。

因此，ROS是从头开始构建的，以鼓励协作机器人软件的开发。例如，一个实验室可能拥有室内环境地图绘制方面的专家，并且可以为创建地图提供世界一流的系统。另一个小组可能有专家使用地图进行导航，而另一个小组可能已经发现了一种计算机视觉方法，可以很好地识别杂乱的小物体。 ROS是专门为此类团体设计的，它们可以相互协作并以彼此的工作为基础。

## 介绍

### 1.ROS是什么

ROS 是一个适用于机器人的开源的元操作系统。它提供了操作系统应有的服务，包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包管理。它也提供用于获取、编译、编写、和跨计算机运行代码所需的工具和库函数。在某些方面ROS相当于一种“机器人框架（robot frameworks）”类似的“机器人框架”有：[Player](http://playerstage.sf.net/)，[YARP](http://eris.liralab.it/yarp/)，[Orocos](http://www.orocos.org/)，[CARMEN](http://carmen.sourceforge.net/)，[Orca](http://orca-robotics.sourceforge.net/)，[MOOS](http://www.robots.ox.ac.uk/~pnewman/TheMOOS/index.html)和 [Microsoft Robotics Studio](http://msdn.microsoft.com/en-us/robotics/default.aspx)。 

ROS 运行时的“蓝图”是一种基于ROS通信基础结构的松耦合点对点进程网络。ROS实现了几种不同的通信方式，包括基于同步RPC样式通信的[服务（services）](http://wiki.ros.org/Services)机制，基于异步流媒体数据的[话题（topics）](http://wiki.ros.org/Topics)机制以及用于数据存储的[参数服务器（Parameter Server）](http://wiki.ros.org/Parameter Server)。 

ROS并不是一个实时的框架，但ROS可以嵌入实时程序。Willow Garage的PR2机器人使用了一种叫做[pr2_etherCAT](http://wiki.ros.org/pr2_etherCAT)的系统来实时发送或接收ROS消息。ROS也可以[与Orocos实时工具包无缝集成](http://www.willowgarage.com/blog/2009/06/10/orocos-rtt-and-ros-integrated)。 

### 2.目的

很多人都在问“ROS与其它机器人软件平台有什么不同？”这是一个很难解答的问题。因为ROS不是一个集成了大多数功能或特征的框架。事实上，ROS 的主要目标是为机器人研究和开发提供代码复用的支持。ROS是一个分布式的进程（也就是节点）框架，这些进程被封装在易于被分享和发布的程序包和功能包集中。ROS也支持一种类似于代码储存库的联合系统，这个系统也可以实现工程的协作及发布。这个设计可以使一个工程的开发和实现从文件系统到用户接口完全独立决策（不受ROS限制）。同时，所有的工程都可以被ROS的基础工具整合在一起。 

为了支持分享和协作的主要目的，ROS框架也有其它几个目标： 

- 小型化：ROS尽可能设计的很小 -- 我们不封装您的 main() 函数 -- 所以为ROS编写的代码可以轻松的在其它机器人软件平台上使用。 由此得出的必然结论是ROS可以轻松集成在其它机器人软件平台：ROS已经可以与OpenRAVE，Orocos和Player集成。 
- ROS不敏感库：ROS的首选开发模型都是用不依赖ROS的干净的库函数编写而成。 
- 语言独立：ROS框架可以简单地使用任何的现代编程语言实现。我们已经实现了[Python版本](http://wiki.ros.org/rospy)，[C++版本](http://wiki.ros.org/roscpp)和 [Lisp版本](http://wiki.ros.org/roslisp)。同时，我们也拥有Java 和 Lua版本的实验库。 
- 方便测试：ROS内建一个了叫做[rostest](http://wiki.ros.org/rostest)的单元/集成测试框架，可以轻松安装或卸载测试模块。 
- 可扩展：ROS可以适用于大型运行时系统和大型开发进程。 

所以，“ROS与其它机器人软件平台有什么不同？”很难得到一个适用于所有情况的答案，但是，如果你选择使用其它机器人软件平台，我们希望你仍然可以使用到很多基于ROS发布的库函数。至于更多细节，这封Brian Gerkey（同时涉猎 Player 和 ROS）向ROS用户所写的关于ROS和Player区别的电子邮件（包括OpenCV 的集成）可以为我们提供一些比较： 

这个问题的答案，和许多问题一样，视情况而定。特别是取悦于你想要干什么。Player非常适合简洁的非铰接的移动平台。它的设计为那些激光雷达的先锋提供了简单的传感器和电机操作方法。 然而，ROS是围绕着基于驱动传感器（倾斜式激光，盘式/斜试头部传感器，机械臂传感器）的复杂移动处理平台设计的。与Player相比，ROS可以更方便的借助分布式计算设备，而且我可以肯定，越高级的应用越适用于ROS而不是Player。换句话说，Player提供了更多的硬件驱动，而ROS提供了更多算法。 我认为，说ROS比Player更加灵活强大是公平的。但是，现实情况是，更加灵活强大意味着更加复杂。尽管我们很努力的使ROS更加简单易用，ROS仍然需要一个很长的学习过程。当然，熟悉Player会对学习ROS有很大帮助，因为它们很多基本的方面都是相似的。 关于你们针对OpenCV集成提出的问题，我想你们会发现ROS集成OpenCV的代码要比Player多一点。未来，当ROS和OpenCV团队明显重叠时，你们会发现这种差异将变得更大。 我发现ROS利用了大量的来自于Player工程的代码。ROS节点的代码重用了许多Player的驱动，而且Stage和Gazebo可在ROS社区中得到广泛的支持和良好的应用。

### 3.操作系统

ROS目前只能在基于Unix的平台上运行。ROS的软件主要在Ubuntu和Mac OS X 系统上测试，同时ROS社区仍持续支持Fedora，Gentoo，Arch Linux和其它Linux平台。 

与此同时，Microsoft Windows端口的ROS已经实现，但并未完全开发完成。 

### 4.发布

ROS核心系统及各种工具和库函数通常在[ROS 发行版本](http://wiki.ros.org/Distributions)中发布。ROS发行版本类似于Linux发行版本，并提供了一系列兼容此版本的可被使用或开发的软件。 

### 5.贡献

ROS是开源项目，任何人都可以为ROS或兼容ROS的库函数做出贡献。

## 基本概念

ROS的概念分为三個层次：文件系统层、计算图层、社区层，這些层次以及概念将会在接下來的章节介绍。 

除了这三个层次的概念，ROS同样定义两个[names](http://wiki.ros.org/Names)类型，Package Resource Names and Graph Resource Names -- 下面会讨论 

### 1.ROS文件系统层

文件系统层概念主要指在硬盘里能看到的ROS目录和文件, 例如： 

- **[Packages](http://wiki.ros.org/Packages)**: Packages是在ROS中整理及组织软件的主要单元。一个Packages包含节点（ROS runtime processes）、ROS依赖库（ROS-dependent library）、数据集（datasets）、配置文件（configuration files）以及任何可以很好地组织在一起的部件。Package是ROS中最具原子性的构建项和发布项。也就是说，Packages是您在ROS中能建立及发布的最小单元。 
- **[Metapackages](http://wiki.ros.org/Metapackages)**: Metapackages 是一组具体的服务相关的功能包。大部分的metpackages 只作为转换[rosbuild](http://wiki.ros.org/rosbuild) [Stacks](http://wiki.ros.org/rosbuild/ROS/Concepts#Stacks)的向后兼容的备选。 
- **[Package Manifests](http://wiki.ros.org/catkin/package.xml)**: Manifests (`package.xml`) 描述一个package的元信息，包括了package的名字，版本，功能简述，证书信息，依赖关系，以及一些其他的被export的package所有的信息。关于`package.xml` 的文件说明，参考[REP-0127](http://www.ros.org/reps/rep-0127.html). 
- **Repositories**: 代码仓库是使用VCS版本控制系统的软件包集合，软件包利用版本控制维持同一版本，它能使用catkin自动发布工具[bloom](http://wiki.ros.org/bloom)进行发布。这些代码仓库常通过映射来进行转换 [rosbuild](http://wiki.ros.org/rosbuild) [Stacks](http://wiki.ros.org/rosbuild/ROS/Concepts#Stacks)。仓库可以只有一个软件包。 
- **[Message (msg) types](http://wiki.ros.org/msg)**: 存储在`my_package/msg/MyMessageType.msg`的Message文件，主要定义了ROS系统的[messages](http://wiki.ros.org/Messages)传输的数据结构。 
- **[Service (srv) types](http://wiki.ros.org/srv)**: 存储在 `my_package/srv/MyServiceType.srv`的服务[services](http://wiki.ros.org/Services)文件，定义了ROS的服务通信时的请求（request ）和响应（response ）相关的数据结构。 

### 2.ROS计算图层

计算图是ROS在点对点网络里整合并处理数据的过程。基本计算图概念是节点、 主机、 参数服务器、 消息、 服务、 话题、 数据包，它们通过不同的方式提供数据给图层。 

这些概念是在[ros_comm](http://wiki.ros.org/ros_comm)库里实现的 

- **[Nodes](http://wiki.ros.org/Nodes)**:节点是执行计算处理进程 。ROS被设计为细粒度的模块化的系统;一个机器人控制系统通常有很多节点组成 。例如，一个节点控制激光测距仪、一个节点控制轮电机、一个节点执行定位、一个节点执行路径规划、一个节点提供系统图形界面等等。一个ROS节点通过ROS客户端库 [client library](http://wiki.ros.org/Client Libraries)编写，例如 [roscpp](http://wiki.ros.org/roscpp) o或[rospy](http://wiki.ros.org/rospy) 
- **[Master](http://wiki.ros.org/Master)**:ROS Master提供名称注册和对计算图其余部分的查找。 没有Master，节点将无法彼此查找，交换消息或调用服务。
- **[Parameter Server](http://wiki.ros.org/Parameter Server)**: 参数服务器允许通过密钥将数据存储在中央位置。 它目前是Master的一部分。 
- **[Messages](http://wiki.ros.org/Messages)**:节点之间通过传递消息进行通信。 消息只是一个数据结构，包括类型字段。 支持标准基本类型（整数，浮点数，布尔值等），以及基本类型数组。 消息可以包含任意嵌套的结构和数组（非常类似于C结构）。
- **[Topics](http://wiki.ros.org/Topics)**: 消息通过具有发布/订阅语义的传输系统进行路由。节点通过将消息发布到给定主题来发出消息。主题是用于标识消息内容的名称。对某种类型的数据感兴趣的节点将订阅适当的主题。 单个主题可能有多个并发发布者和订阅者，并且单个节点可以发布和订阅多个主题。 通常，发布者和订阅者不了解彼此的存在。 这个想法是要使信息的生产与其消耗解耦。 从逻辑上讲，人们可以将主题视为强类型的消息总线。 每条总线都有一个名称，只要它们是正确的类型，任何人都可以连接到该总线以发送或接收消息。
- **[Services](http://wiki.ros.org/Services)**:发布/订阅模型是一种非常灵活的通信范例，但是其多对多单向传输不适用于请求/应答交互，而这在分布式系统中通常是必需的。 请求/回复是通过服务完成的，服务由一对消息结构定义：一个用于请求，一个用于回复。 一个提供节点使用名称提供服务，客户端通过发送请求消息并等待回复来使用该服务。 ROS客户端库通常将此交互呈现给程序员，就像是远程过程调用一样。
- **[Bags](http://wiki.ros.org/Bags)**: 包是用于保存和播放ROS消息数据的格式。包是用于存储数据（例如，传感器数据）的重要机制，这些数据可能很难收集，但对于开发和测试算法是必需的。

ROS主机在ROS计算图中充当名称服务。它存储ROS节点的主题和服务注册信息。 节点与主节点通信以报告其注册信息。这些节点与主节点通信时，它们可以接收有关其他已注册节点的信息并进行适当的连接。当此注册信息更改时，主服务器还将对这些节点进行回调，从而允许节点在运行新节点时动态创建连接。

节点直接连接到其他节点； 主机仅提供查找信息，就像DNS服务器一样。订阅主题的节点将请求发布该主题的节点的连接，并将通过已同意的连接协议建立该连接。 ROS中最常用的协议称为TCPROS，它使用标准的TCP / IP套接字。

这种架构允许解耦操作，其中名称是构建更大、更复杂的系统的主要手段。 名称在ROS中扮演着非常重要的角色：节点、主题、服务和参数都具有名称。 每个ROS客户端库都支持名称的命令行重映射，这意味着可以在运行时重新配置编译后的程序，以在不同的计算图拓扑中运行。

例如，要控制Hokuyo激光测距仪，我们可以启动hokuyo_node驱动程序，该驱动程序与激光对话并在扫描主题上发布sensor_msgs / LaserScan消息。 为了处理该数据，我们可以使用laser_filters编写一个节点，该节点订阅有关扫描主题的消息。 订阅后，我们的过滤器将自动开始接收来自Laser的消息。

请注意两侧如何分离。hokuyo_node节点所做的只是发布扫描，而不知道是否有人订阅。筛选器所做的只是订阅扫描，而不知道是否有人在发布扫描。可以以任何顺序启动，终止和重新启动两个节点，而不会引起任何错误情况。

之后，我们可能会向机器人添加另一个激光器，因此我们需要重新配置系统。我们需要做的就是重新映射使用的名称。当我们启动第一个hokuyo_node时，我们可以告诉它将扫描重新映射到base_scan，并对过滤器节点执行相同的操作。现在，这两个节点将改为使用base_scan主题进行通信，并且不会听到有关scan主题的消息。然后，我们可以为新的激光测距仪启动另一个hokuyo_node。

 ![ROS_basic_concepts](http://ros.org/images/wiki/ROS_basic_concepts.png)

### 3.ROS社区级别

ROS社区级别的概念是ROS资源，使单独的社区能够交换软件和知识。 这些资源包括：

- **[Distributions](http://wiki.ros.org/Distributions)**: ROS分发是可以安装的版本堆栈的集合。发行版起着与Linux发行版相似的作用：它们使安装软件集合更加容易，并且它们在一组软件之间保持一致的版本。
- **[Repositories](http://wiki.ros.org/Repositories)**: ROS依赖于代码存储库的联合网络，不同的机构可以在其中开发和发布自己的机器人软件组件。
- **[The ROS Wiki](http://wiki.ros.org/Documentation)**: ROS社区Wiki是记录有关ROS信息的主要论坛。 任何人都可以注册一个帐户并贡献自己的文档，提供更正或更新，编写教程等。
- **Bug Ticket System**: 请参阅[Tickets](http://wiki.ros.org/Tickets)以获取有关file tickets的信息。
- **[Mailing Lists](http://wiki.ros.org/Mailing Lists)**:ros-users邮件列表是有关ROS的新更新的主要交流渠道，也是一个提出有关ROS软件问题的论坛。
- **[ROS Answers](http://answers.ros.org/)**:一个问答站点，用于回答与ROS有关的问题。
- **[Blog](http://www.willowgarage.com/blog)**:  [Willow Garage Blog](http://www.willowgarage.com/blog) 提供定期更新，包括照片和视频。

### 4.名称

#### 4.1 图资源名称

图资源名称提供了一种分层的命名结构，该结构用于ROS计算图中的所有资源，例如节点，参数，主题和服务。 这些名称在ROS中非常强大，并且对于ROS中组成更大和更复杂的系统至关重要，因此了解这些名称的工作方式以及如何操作它们至关重要。

在进一步描述名称之前，下面是一些示例名称：

- `/` (the global namespace) 
- `/foo` 
- `/stanford/robot/name` 
- `/wg/node1` 

图形资源名称是ROS中提供封装的重要机制。 每个资源都在命名空间中定义，可以与许多其他资源共享。 通常，资源可以在其名称空间内创建资源，并且可以访问其自身名称空间内或上方的资源。 可以在不同命名空间中的资源之间建立连接，但这通常是通过两个命名空间上方的集成代码来完成的。 这种封装可将系统的不同部分隔离起来，以免意外捕获错误的命名资源或全局劫持名称。

名称是相对解析的，因此资源不必知道它们在哪个名称空间中。这简化了编程，因为可以将协同工作的节点写成好像它们都在顶级名称空间中一样。 将这些节点集成到较大的系统中时，可以将它们下推到定义其代码集合的名称空间中。 例如，可以进行一个Stanford演示和Willow Garage演示，然后将它们合并到带有stanford和wg子图的新演示中。 如果两个演示都具有一个名为“ camera”的节点，则它们不会冲突。 顶级节点可以创建整个图形都需要看到的工具（例如图形可视化）以及参数（例如demo_name）。

##### 4.1.1 有效名称

有效名称具有以下特征：

1. 第一个字符是字母字符 ([a-z|A-Z]), 波浪号 (`~`) 或正斜杠 (`/`) 
2. 后续字符可以是字母数字([0-9|a-z|A-Z]), 下划线 (`_`), 或正斜杠 (`/`) 

例外: 基本名称（如下所述）中不能包含正斜杠（/）或波浪号(`~`)

##### 4.1.2 解决

ROS中有四种类型的“图形资源名称”：*base*, *relative*, *global*, and *private*，它们具有以下语法：

- `base` 
- `relative/name` 
- `/global/name` 
- `~private/name` 

例如，节点`/wg/node1` 具有名称空间 `/wg`，因此名称`node2`将解析为`/wg/node2`。

没有名称空间限定符的名称都是基本名称。 基本名称实际上是相对名称的子类，并且具有相同的解析规则。 基本名称最常用于初始化节点名称。

以"/" 开头的名称是全局名称，它们被认为是完全解析的。 应尽可能避免使用全局名称，因为它们会限制代码的可移植性。

以"~"开头的名称是私有的。 它们将节点的名称转换为名称空间。 例如，名称空间`/wg/`中的`node1` 具有专用名称空间`/wg/node1`。 私有名称对于通过参数服务器将参数传递到特定节点很有用。

以下是一些名称解析示例：

| **Node**    | **Relative** (default)     | **Global**               | **Private**                       |
| ----------- | -------------------------- | ------------------------ | --------------------------------- |
| `/node1`    | `bar` -> `/bar`            | `/bar` -> `/bar`         | `~bar` -> `/node1/bar`            |
| `/wg/node2` | `bar` -> `/wg/bar`         | `/bar` -> `/bar`         | `~bar` -> `/wg/node2/bar`         |
| `/wg/node3` | `foo/bar` -> `/wg/foo/bar` | `/foo/bar` -> `/foo/bar` | `~foo/bar` -> `/wg/node3/foo/bar` |

##### 4.1.3 重新映射

在命令行上启动ROS节点中的任何名称时，都可以对其进行重新映射。 有关此功能的更多信息，请参见重新映射参数。

#### 4.2 包资源名称

包资源名称在ROS中与文件系统级概念一起使用，以简化引用磁盘上文件和数据类型的过程。包资源名称非常简单：它们只是资源所在的包的名称加上资源的名称。例如，名称"std_msgs/String"是指"std_msgs"包中的"String"消息类型。

可以使用“程序包资源名称”引用的一些与ROS相关的文件，包括：

- [Message (msg) types](http://wiki.ros.org/msg) 
- [Service (srv) types](http://wiki.ros.org/srv) 
- [Node types](http://wiki.ros.org/Nodes) 

包资源名称与文件路径非常相似，只不过它们的名称要短得多。 这是由于ROS能够在磁盘上定位软件包并对其内容进行其他假设。 例如，消息描述始终存储在`msg`子目录中，并具有`.msg` 扩展名，因此`std_msgs / String`是`path / to / std_msgs / msg / String.msg`的简写。 同样，节点类型`foo / bar`等效于在具有可执行权限的程序包`foo`中搜索名为`bar`的文件。

##### 4.2.1 有效名称

包资源名称具有严格的命名规则，因为它们经常在自动生成的代码中使用。 因此，ROS软件包除下划线外不能包含特殊字符，并且必须以字母字符开头。 有效名称具有以下特征：

1. 第一个字符是字母字符 ([a-z|A-Z]) 
2. 后续字符可以是字母数字 ([0-9|a-z|A-Z]), 下划线 (`_`) 或正斜杠 (`/`) 
3. 最多有一个正斜杠 ('/'). 

#### 4.3 代码API

[roscpp::names API reference](http://docs.ros.org/indigo/api/roscpp/html/namespaceros_1_1names.html) (ROS Indigo) 

## 技术概要

本技术概述更详细地介绍了ROS的实现。 大多数ROS用户不需要了解这些详细信息，但对于希望编写自己的ROS客户端库或希望将其他系统与ROS集成的用户来说，它们非常重要。

本技术概述假定您已经熟悉ROS系统及其概念。 例如，ROS概念概述提供了计算图体系结构的概述，包括ROS主节点和节点的作用。

### 1. Master主服务器

通过XMLRPC实现Master，XMLRPC是基于HTTP的无状态协议。 之所以选择XMLRPC，主要是因为它比较轻巧，不需要有状态的连接，并且在各种编程语言中具有广泛的可用性。 例如，在Python中，您可以启动任何Python解释器并开始与ROS Master进行交互：

```python
$ python
>>> from xmlrpclib import ServerProxy
>>> import os
>>> master = ServerProxy(os.environ['ROS_MASTER_URI'])
>>> master.getSystemState('/')
[1, 'current system state', [[['/rosout_agg', ['/rosout']]], [['/time', ['/rosout']], ['/rosout', ['/rosout']], ['/clock', ['/rosout']]], [['/rosout/set_logger_level', ['/rosout']], ['/rosout/get_loggers', ['/rosout']]]]]
```

Master具有注册API，这些API允许节点注册为发布者，订阅者和服务提供商。 Master具有URI，并存储在ROS_MASTER_URI环境变量中。 此URI对应于它正在运行的XML-RPC服务器的host：port。 默认情况下，Master将绑定到端口11311。

有关更多信息，包括API列表，请参阅[Master API](http://wiki.ros.org/ROS/Master_API)。

### 2. Parameter Server参数服务器

尽管Parameter Server实际上是ROS Master的一部分，但我们将其API当作一个单独的实体进行讨论，以实现将来的分离。

与主API一样，参数服务器API也通过XMLRPC实现。使用XMLRPC可以轻松地与ROS客户端库集成，并且在存储和检索数据时还提供了更大的类型灵活性。参数服务器可以存储基本的XML-RPC标量（32位整数，布尔值，字符串，双精度数，iso8601日期），列表和以base64编码的二进制数据。参数服务器还可以存储字典（即结构），但是它们具有特殊含义。

参数服务器使用嵌套的字典表示形式表示名称空间，其中每个字典代表命名层次结构中的一个级别。这意味着字典中的每个键都代表一个名称空间。如果值是字典，则参数服务器将假定它正在存储名称空间的值。例如，如果要将参数/ ns1 / ns2 / foo设置为值1，则/ ns1 / ns2 /的值将是字典{foo：1}，而/ ns1 /的值将是字典{ ns2：{foo：1}}。

XMLRPC API使集成Parameter Server调用变得非常容易，而不必使用ROS客户端库。假设您有权访问XMLRPC客户端库，则可以直接进行调用。例如：

```python
$ python
>>> from xmlrpclib import ServerProxy
>>> import os
>>> ps = ServerProxy(os.environ['ROS_MASTER_URI'])
>>> ps.getParam('/', '/foo')
[-1, 'Parameter [/bar] is not set', 0]
>>> ps.setParam('/', '/foo', 'value')
[1, 'parameter /foo set', 0]
>>> ps.getParam('/', '/foo')
[1, 'Parameter [/foo]', 'value']
```

更详细的API列表请参见[Parameter Server API](http://wiki.ros.org/ROS/Parameter%20Server%20API).

### 3. Node节点

ROS节点具有多个API：

1. 从属API。从属API是XMLRPC API，它具有两个作用：从主控接收回调，并与其他节点协商连接。有关API的详细列表，请参见[Slave API](http://wiki.ros.org/ROS/Slave_API)。
2. 话题传输协议实现（请参阅[TCPROS](http://wiki.ros.org/ROS/TCPROS)和[UDPROS](http://wiki.ros.org/ROS/UDPROS)）。节点使用约定的协议彼此建立话题连接。最通用的协议是TCPROS，它使用持久的有状态TCP / IP套接字连接。
3. 命令行API。每个节点都应支持[命令行重映射参数](http://wiki.ros.org/Remapping%20Arguments)，从而可以在运行时配置节点内的名称。

每个节点都有一个_URI_，它对应于它正在运行的[XMLRPC服务器](http://wiki.ros.org/ROS/Master_Slave_APIs)的host：port。 XMLRPC服务器不用于传输话题或服务数据：相反，它用于与其他节点协商连接并与主服务器进行通信。该服务器是在ROS客户端库中创建和管理的，但是通常对客户端库用户不可见。 XMLRPC服务器可以绑定到节点正在运行的主机上的任何端口。

XMLRPC服务器提供了一个[从属API](http://wiki.ros.org/ROS/Slave_API)，该API使节点能够从主服务器接收发布者更新调用。这些发布者更新包含话题名称和发布该话题的节点的URI列表。 XMLRPC服务器还将接收来自subscriber的请求topic连接的呼叫。通常，当节点收到发布者更新时，它将连接到任何新发布者。

当订阅者使用发布者的XMLRPC服务器请求话题连接时，将协商话题传输。订阅者向发布者发送受支持协议的列表。然后，发布者从该列表中选择协议，例如TCPROS，并返回该协议的必要设置（例如，TCP / IP服务器套接字的IP地址和端口）。然后，订阅者使用提供的设置建立单独的连接。

### 4. Topic Transport话题传输

有多种方法可以在网络上传输数据，每种方法各有优缺点，这在很大程度上取决于应用程序。 TCP被广泛使用，因为它提供了简单，可靠的通信流。 TCP数据包始终按顺序到达，丢失的数据包将重新发送，直到它们到达为止。虽然对于有线以太网非常有用，但是当基础网络是有损的WiFi或蜂窝调制解调器连接时，这些功能就会变成错误。在这种情况下，UDP更合适。当多个订阅者分组在一个子网上时，对于发布者而言，通过UDP广播同时与所有订阅者通信可能是最有效的。

由于这些原因，ROS不会提交单个传输。给定发布者URI，订阅节点使用适当的传输，通过XMLRPC与该发布者协商连接。协商的结果是两个节点已连接，消息从发布者流向订阅者。

每个传输都有自己的协议，用于交换消息数据。例如，使用TCP，协商将涉及发布者为订阅者提供呼叫连接的IP地址和端口。然后，订阅者创建一个指向指定地址和端口的TCP / IP套接字。节点交换一个连接头，其中包括诸如消息类型的MD5总和和话题名称之类的信息，然后发布者开始直接通过套接字发送序列化的消息数据。

为了强调，节点之间通过适当的传输机制直接进行通信。数据不路由通过主服务器。数据不是通过XMLRPC发送的。 XMLRPC系统仅用于协商数据连接。

开发者链接：

* [ROS/TCPROS](http://wiki.ros.org/ROS/TCPROS)
* [ROS/UDPROS](http://wiki.ros.org/ROS/UDPROS)

### 5. 消息序列化及msg MD5总和

消息以非常紧凑的表示形式进行序列化，该表示形式大致对应于小尾数格式的消息数据的类似c结构的序列化。 紧凑的表示意味着两个通信节点必须在消息数据的布局上达成共识。

ROS中的消息类型（[msgs](http://wiki.ros.org/msg)）使用msg文本的特殊MD5总和计算进行版本控制。 通常，客户端库不会直接实现此MD5和的计算，而是使用roslib / scripts / gendeps的输出将此MD5和存储在自动生成的消息源代码中。 作为参考，此MD5总和是从.msg文件的MD5文本计算得出的，其中MD5文本是.msg文本，其中：

* 评论已删除
* 空格已删除
* 软件包的依赖项名称已删除
* 常量在其他声明之前重新排序

为了捕获嵌入式消息类型中发生的更改，按出现顺序将MD5文本与每种嵌入式类型的MD5文本串联在一起。

### 6. 建立话题连接

放在一起，两个节点开始交换消息的顺序是：

1. 订阅者启动。 它读取命令行重映射参数以解析将使用的话题名称。 （重新映射参数）
2. 发布者启动。 它读取命令行重映射参数以解析将使用的话题名称。 （重新映射参数）
3. 订阅者向主服务器注册。 （XMLRPC）
4. 发布者向主服务器注册。 （XMLRPC）
5. 主服务器将新发布者通知订阅者。 （XMLRPC）
6. 订阅者联系发布者以请求话题连接并协商传输协议。 （XMLRPC）
7. 发布者将选定的传输协议的设置发送给订阅者。 （XMLRPC）
8. 订阅者使用所选的传输协议连接到发布者。 （TCPROS等...）

XMLRPC部分如下所示：

```python
/subscriber_node → master.registerSubscriber(/subscriber_node, /example_topic, std_msgs/String, http://hostname:1234)
# Master returns that there are no active publishers.
/publisher_node → master.registerPublisher(/publisher_node, /example_topic, std_msgs/String, http://hostname:5678)
# Master notices that /subscriber_node is interested in /example_topic, so it makes a callback to the subscriber
master → subscriber.publisherUpdate(/publisher_node, /example_topic, [http://hostname:5678])
# Subscriber notices that it has not connected to http://hostname:5678 yet, so it contacts it to request a topic.
subscriber → publisher.requestTopic(/subscriber_node, /example_topic, [[TCPROS]])
# Publisher returns TCPROS as the selected protocol, so subscriber creates a new connection to the publishers TCPROS host:port.
```

#### 6.1 示例

![master-node-example.png](http://wiki.ros.org/ROS/Technical%20Overview?action=AttachFile&do=get&target=master-node-example.png)

要控制Hokuyo激光测距仪，我们启动[hokuyo_node](http://wiki.ros.org/hokuyo_node)节点，该节点与激光对话并在扫描话题上发布sensor_msgs / LaserScan消息。为了可视化激光扫描数据，我们启动rviz节点并订阅扫描话题。订阅后，rviz节点开始接收LaserScan消息，并将其呈现到屏幕上。

请注意两侧如何分离。 hokuyo_node节点所做的只是发布扫描，而不知道是否有人订阅。 rviz所做的只是订阅扫描，而不知道是否有人在发布扫描。可以以任何顺序启动，终止和重新启动两个节点，而不会引起任何错误情况。

在上面的示例中，laser_viewer和hokuyo_node节点如何找到彼此？他们使用由称为主节点的特殊节点提供的名称服务。主服务器具有一个众所周知的XMLRPC URI，所有节点都可以访问。在首次发布话题之前，节点会发布其发布该话题的意图。该广告通过XMLRPC将有关发布的信息发送到主服务器，包括消息类型，话题名称和发布节点的URI。主服务器将这些信息维护在发布者表中。

当节点订阅话题时，它通过XMLRPC与主节点通信，发送相同的信息（消息类型，话题名称和节点URI）。主服务器在订阅者表中维护此信息。作为回报，为订阅者提供了发布者URI的当前列表。随着发布者列表的更改，订阅者还将收到来自主服务器的更新。给定发布者列表后，预订节点即可启动特定于传输的连接。

注意：消息数据不会流经主服务器。它仅提供名称服务，将订阅者与发布者联系起来。

### 7. 建立一个服务连接

在本概述中，我们没有对服务进行过多讨论，但是可以将它们视为话题的简化版本。话题可以有许多发布者，而只能有一个服务提供商。向主服务器注册的最新节点被视为当前服务提供商。这允许使用更简单的设置协议——实际上，服务客户端不必是ROS节点。

1. 向主服务器注册
2. 服务客户端在主服务器上查找服务
3. 服务客户端为服务创建TCP / IP
4. 服务客户端和服务[交换连接头](http://wiki.ros.org/ROS/Connection%20Header)
5. 服务客户端发送序列化的请求消息
6. 服务使用序列化响应消息进行回复。

如果最后几步看起来很熟悉，那是因为它们是TCPROS协议的扩展。实际上，rospy和roscpp都使用相同的TCP / IP服务器套接字来接收话题和服务连接。

由于在注册新服务时没有来自主服务器的回调，因此许多客户端库提供了一种“等待服务” API方法，该方法仅轮询主服务器，直到出现服务注册为止。

#### 7.1 持久的服务连接

默认情况下，服务连接是无状态的。 对于客户希望进行的每个呼叫，它都会重复以下步骤：在主服务器上查找服务，并通过新的连接交换请求/响应数据。

无状态方法通常更健壮，因为它允许重新启动服务节点，但是如果对同一服务频繁重复调用，则此开销可能会很高。

ROS允许与服务的持久连接，该连接提供了非常高的吞吐量的连接，用于重复调用服务。 通过这些持久连接，客户端和服务之间的连接将保持打开状态，以便服务客户端可以继续通过该连接发送请求。

持久连接应格外小心。 如果出现新的服务提供商，则不会中断正在进行的连接。 同样，如果持久连接失败，则不会尝试重新连接。

## 核心部件

虽然我们无法提供ROS生态系统的详尽清单，但我们可以确定ROS的一些核心部分，并讨论其功能，技术规格和质量，以使您更好地了解ROS。

------

### 1. 通讯基础设施

在最低级别，ROS提供了一个消息传递接口，该接口提供了进程间通信，通常称为中间件。

ROS中间件提供以下功能：

* 发布/订阅匿名消息传递
* 记录和播放信息
* 请求/响应远程过程调用
* 分布式参数系统

#### 消息传递

当实现新的机器人应用程序时，通信系统通常是最先出现的需求之一。 ROS的内置且经过良好测试的消息传递系统通过匿名发布/订阅机制管理分布式节点之间的通信详细信息，从而节省了您的时间。使用消息传递系统的另一个好处是，它迫使您在系统中的节点之间实现清晰的接口，从而改善了封装并促进了代码重用。这些消息接口的结构在消息IDL（接口描述语言）中定义。

#### 记录和播放信息

由于发布/订阅系统是匿名且异步的，因此可以轻松捕获和重播数据，而无需更改代码。假设您有任务A从传感器读取数据，并且您正在开发任务B处理由任务A生成的数据。ROS使捕获任务A发布的数据到文件，然后从服务器重新发布该数据变得容易。稍后再归档。消息传递抽象允许任务B相对于数据源（可以是任务A或日志文件）是不可知的。这是一种功能强大的设计模式，可以大大减少您的开发工作并提高系统的灵活性和模块化性。

#### 远程过程调用

发布/订阅消息传递的异步特性适用于机器人技术中的许多通信需求，但是有时您希望进程之间进行同步的请求/响应交互。 ROS中间件使用服务来提供此功能。像主题一样，使用相同的简单消息IDL定义服务调用中进程之间发送的数据。

#### 分布式参数系统

ROS中间件还为任务提供了一种通过全局键值存储共享配置信息的方式。该系统使您可以轻松地修改任务设置，甚至允许任务更改其他任务的配置。

-----

### 特定于机器人的功能

除了核心中间件组件之外，ROS还提供了特定于机器人的通用库和工具，这些资源将使您的机器人快速启动并运行。以下是ROS提供的一些特定于机器人的功能：

* 机器人的标准消息定义
* 机器人几何库
* 机器人描述语言
* 可抢占的远程过程调用
* 诊断程序
* 姿势估计
* 本土化
* 制图
* 导航

#### 标准机器人消息

社区的多年讨论和发展导致了一系列标准消息格式，这些格式涵盖了机器人技术中的大多数常见用例。对于诸如姿势，变换和向量之类的几何概念，存在消息定义。用于相机，IMU和激光等传感器；以及用于导航数据，如里程表，路径和地图；等等。通过在应用程序中使用这些标准消息，您的代码将与ROS生态系统的其余部分（从开发工具到功能库）无缝地互操作。

#### 机器人几何库

![TF可视化](https://www.ros.org/wp-content/uploads/2013/12/tf.jpg)

在许多机器人项目中，一个共同的挑战是跟踪机器人不同部分之间的相对位置。例如，如果要将照相机的数据与激光的数据合并，则需要在某个公共参考系中知道每个传感器的位置。对于具有许多运动部件的类人机器人而言，这个问题尤为重要。我们使用tf（转换）库在ROS中解决了此问题，该库将跟踪机器人系统中所有内容的位置。

设计tf库时出于效率考虑，已用于管理具有一百多个自由度和数百赫兹的更新率的机器人的坐标变换数据。使用tf库，您既可以定义静态变换（例如固定在移动基座上的摄像机），也可以定义动态变换（例如，机械臂中的关节）。您可以在系统中的任何一对坐标系之间转换传感器数据。 tf库处理了该信息的生产者和消费者可以分布在整个网络中的事实，以及该信息以不同的速率更新的事实。

#### 机器人描述语言

ROS为您解决的另一个常见机器人问题是如何以机器可读的方式描述您的机器人。 ROS提供了一组用于描述和建模机器人的工具，以便其他ROS系统（包括tf，[robot_state_publisher](http://wiki.ros.org/robot_state_publisher)和rviz）可以理解它。在ROS中用于描述机器人的格式为URDF（统一机器人描述格式），该格式由一个XML文档组成，您可以在其中描述机器人的物理属性，从肢体的长度和轮子的大小到传感器的位置以及机器人各部分的视觉外观。

一旦以这种方式定义，您的机器人就可以轻松地与tf库一起使用，以三个维度进行渲染以实现[良好的可视化效果](http://wiki.ros.org/rviz)，并与[模拟器](http://gazebosim.org/)和[运动计划器](http://moveit.ros.org/)一起使用。

#### 可抢占的远程过程调用

虽然主题（匿名发布/订阅）和服务（远程过程调用）涵盖了机器人技术中的大多数通信用例，但有时您需要启动寻求目标的行为，监控其进度，能够一路抢先，以及完成后收到通知。 ROS为此提供了行动。动作与服务类似，只是它们可以在返回最终响应之前报告进度，并且可以被调用者抢占。因此，例如，您可以指示机器人导航到某个位置，在其尝试到达该位置时监视其进度，沿途停止或重定向它，并告知它何时成功（或失败）。动作是在整个ROS生态系统中使用的强大概念。

#### 诊断程序

![诊断截图](https://www.ros.org/wp-content/uploads/2013/12/diagnostics.jpg)

ROS提供了一种标准的方法来生成，收集和汇总有关机器人的[诊断信息](http://wiki.ros.org/diagnostics)，以便您一眼就能快速看到机器人的状态并确定如何解决出现的问题。

#### 姿势估计，本地化和导航

![导航屏幕截图](https://www.ros.org/wp-content/uploads/2013/12/navigation_ros.jpg)

ROS还提供了一些“包含电池”功能，可帮助您开始进行机器人项目。 有ROS软件包可以解决基本的机器人技术问题，例如[姿态估计](http://wiki.ros.org/robot_pose_ekf)，[在地图中的定位](http://wiki.ros.org/amcl)，[构建地图](http://wiki.ros.org/gmapping)，甚至是[移动导航](http://wiki.ros.org/navigation)。

无论您是想进行快速研究和开发的工程师，还是想及时完成研究的机器人研究人员，还是想了解更多有关机器人技术的业余爱好者，这些现成的功能都将为您提供事半功倍的帮助。

-----

### 工具

ROS最强大的功能之一就是强大的开发工具集。这些工具支持自省，调试，绘图和可视化所开发系统的状态。底层的发布/订阅机制使您可以自发地对流经系统的数据进行内部检查，从而在出现问题时很容易理解和调试问题。 ROS工具通过广泛的图形和命令行实用程序集合利用了这种自省功能，从而简化了开发和调试。

#### 命令行工具

您是否将所有时间都花在远程登录机器人上？可以在没有GUI的情况下100％使用ROS。所有核心功能和自省工具都可以通过我们的45多种命令行工具之一进行访问。有用于启动节点组的命令。反思主题，服务和行动；记录和回放数据；和许多其他情况。如果您更喜欢使用图形工具，则rviz和rqt提供类似（和扩展）的功能。

#### Rviz

![rviz屏幕截图](https://www.ros.org/wp-content/uploads/2013/12/cart_pushing_rviz_holonomic.jpg)

[rviz](http://wiki.ros.org/rviz)可能是ROS中最著名的工具，它提供了多种传感器数据类型和任何URDF描述的机器人的通用三维可视化。

rviz可以可视化ROS中提供的许多常见消息类型，例如激光扫描，三维点云和相机图像。它还使用tf库中的信息在您选择的公共坐标系中显示所有传感器数据，以及对机器人进行三维渲染。在同一应用程序中可视化所有数据不仅令人印象深刻，而且还使您能够快速查看机器人的视线，并识别诸如传感器未对准或机器人模型不正确之类的问题。

#### rqt

![rqt](https://www.ros.org/wp-content/uploads/2013/12/rqt.jpg)

ROS提供了基于Qt的框架[rqt](http://wiki.ros.org/rqt)，用于为您的机器人开发图形界面。您可以通过将大量内置的rqt插件库进行配置并将其配置为选项卡式，分屏式和其他布局，来创建自定义界面。您还可以通过编写自己的rqt插件来引入新的接口组件。

![rqt_graph屏幕截图](https://www.ros.org/wp-content/uploads/2013/12/rqt_graph.jpg)

[rqt_graph插件](http://wiki.ros.org/rqt_graph)提供对实时ROS系统的自检和可视化，显示节点及其之间的连接，并允许您轻松调试和了解正在运行的系统以及其结构。

![rqt_plot屏幕截图](https://www.ros.org/wp-content/uploads/2013/12/rqt_plot.jpg)

使用[rqt_plot插件](http://wiki.ros.org/rqt_plot)，您可以监视编码器，电压或任何可以表示为随时间变化的数字的东西。使用rqt_plot插件，您可以选择最适合您的绘图后端（例如matplotlib，Qwt，pyqtgraph）。

![rqt_publisher屏幕截图](https://www.ros.org/wp-content/uploads/2013/12/rqt_publisher.jpg)

为了监视和使用主题，您拥有[rqt_topic](http://wiki.ros.org/rqt_topic)和[rqt_publisher](http://wiki.ros.org/rqt_publisher)插件。前者使您可以监视和内省系统中正在发布的任意数量的主题。后者使您可以将自己的消息发布到任何主题，从而促进系统的临时试验。

![rqt_bag截图](https://www.ros.org/wp-content/uploads/2013/12/rqt_bag.png)

对于数据记录和回放，ROS使用[bag格式](http://wiki.ros.org/Bags/Format)。 Bag文件可以通过[rqt_bag插件](http://wiki.ros.org/rqt_bag)创建和图形化访问。该插件可以将数据记录到袋子中，回放袋子中选定的主题，并可视化袋子中的内容，包括图像的显示和随时间推移绘制的数值。