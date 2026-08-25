# JetArm

[English](README.md) | 中文

<p align="center">
  <img src="./sources/images/jetarm.png" alt="JetArm Logo" width="600"/>
</p>

## 产品概述

### 关于JetArm

您是否曾想象过为您的机器人项目配备一双能够真正"看见"三维世界的眼睛？对于桌面级机械臂来说，从二维平面的限制跃升到真正的深度感知空间抓取，一直是一个重大的技术挑战。

幻尔JetArm正是为此而生。它不仅仅是一个机械臂，更是一个基于ROS框架构建的开放研究平台，深度集成3D视觉和边缘AI计算。我们相信突破始于感知——通过配备高性能3D深度相机，JetArm捕获环境的RGB图像和深度点云数据。这使它能够精确理解物体的形状、姿态和空间关系。摆脱平面的限制，它可以在真实的3D空间中实现自由抓取和复杂的任务规划。

这就是JetArm的起点：您的AI算法在物理世界中与3D物体精确交互的终极伙伴。无论是机器人教育、前沿具身AI研究，还是构建令人印象深刻的创意AI项目，JetArm都提供了强大的、完全开源的基础。

### 核心：为空间智能设计的软硬件系统

JetArm的架构经过精心设计，将视觉、控制和计算能力分离，以实现最佳性能和灵活性。

**视觉的维度跃升**：其核心是顶部安装的3D深度相机。它克服了传统2D相机的局限，提供空间深度信息——这是3D场景理解和操作的基础数据。

**主从控制**：它使用NVIDIA Jetson模块作为主控制器进行视觉处理和AI推理，配合专用的STM32控制板进行精确的舵机驱动。这种"决策-执行分离"的设计释放了主控制器的计算能力用于视觉和AI应用，同时确保实时、流畅的机器人运动。它还支持未来灵活升级主计算模块。

**强劲执行**：由六个35KG大扭矩智能串行总线舵机驱动，提供强大的刚性和高达450g的有效负载能力，配备可更换的末端执行器。

### 软件栈：为开发者构建的开放生态系统

JetArm预装了深度集成ROS开发框架的Linux系统，让您从底层到应用层拥有完全控制权。

**开源框架**：系统完全使用ROS（机器人操作系统）开发。这意味着您可以访问所有核心算法的源代码，并直接利用ROS丰富的工具链进行开发、调试和仿真。

**全栈技术体验**：通过JetArm项目，您可以获得一系列前沿机器人技术的实践经验：ROS编程、3D视觉（点云处理）、OpenCV、YOLO/MediaPipe等深度学习模型、逆运动学、MoveIt!、Gazebo仿真等。

**多模态AI集成**：得益于Jetson平台强大的计算能力，JetArm可以支持和运行多模态大语言模型。结合其3D视觉和麦克风阵列，它能够实现复杂的、基于自然语言的任务理解和交互，将具身AI的概念变为可触摸的实践实验。

### 可扩展性与学习：您的创意画布

JetArm被设计为一个拥有无限可能的平台。

**灵活扩展**：机身配备多个扩展接口，可轻松连接各种传感器模块（如超声波、力矩传感器）或功能模块，探索语音交互或力反馈控制等高级应用。

**全面教程**：我们提供详细的分步视频教程和学习材料，涵盖18个以上主题——从基础运动控制和相机校准到高级AI应用开发——帮助您快速从初学者成长为专家。

## 演示视频

### 主要教程
- **JetArm 3D视觉机械臂：深入了解AI ROS机器人**: [观看](https://www.youtube.com/watch?v=wYL03uhiER4)
- **JetArm机械臂真的能运行ChatGPT吗？我们测试了！**: [观看](https://www.youtube.com/watch?v=G9IH4B9-BPw)

### 你的创意，付诸实践
- **想要混入其中？想都别想！JetArm的锐利眼睛会立即挑出任何高个子冒充者！**: [观看](https://www.youtube.com/shorts/vY0HlBzOgBY)
- **由3D深度相机驱动，这个机械臂重新定义多任务处理 🤖💡**: [观看](https://www.youtube.com/shorts/crg2_PT5pfQ)

## Hackster项目

- **为JetArm机器人构建多模态AI大脑**: [阅读](https://www.hackster.io/HiwonderRobot/building-a-multimodal-ai-brain-for-the-jetarm-robot-8accdc)
- **JetArm Pro：可扩展的移动操作ROS平台**: [阅读](https://www.hackster.io/HiwonderRobot/jetarm-pro-expandable-ros-platform-for-mobile-manipulation-aff995)
- **JetArm Pro：下一代机器人的移动操作器**: [阅读](https://www.hackster.io/HiwonderRobot/jetarm-pro-mobile-manipulator-for-next-gen-robotics-b4be45)

## 官方资源

### 幻尔科技官方

- **官方网站**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **产品页面**: [https://www.hiwonder.com/products/jetarm](https://www.hiwonder.com/products/jetarm)
- **官方文档**: [https://docs.hiwonder.com/projects/JetArm/en/latest/](https://docs.hiwonder.com/projects/JetArm/en/latest/)
- **技术支持**: support@hiwonder.com

## 快速开始

### 安装

请参考[官方文档](https://docs.hiwonder.com/projects/JetArm/en/latest/)获取针对您硬件平台的详细安装说明。

## 社区与支持

- **GitHub Issues**: 报告问题和请求功能
- **邮件支持**: support@hiwonder.com
- **文档资料**: 全面的指南和教程

## 许可证

本项目开源，可用于教育和研究目的。

---

**幻尔科技** - 赋能机器人教育创新
