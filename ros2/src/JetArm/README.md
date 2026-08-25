# JetArm

English | [中文](README_cn.md)

<p align="center">
  <img src="./sources/images/jetarm.png" alt="JetArm Logo" width="600"/>
</p>

## Product Overview

### About JetArm

Have you ever imagined giving your robotics projects a pair of eyes that can truly "see" the world in 3 dimensions? For desktop-level robotic arms, the leap from the constraints of a 2D plane to true, depth-aware spatial grasping has long been a significant technical challenge.

The Hiwonder JetArm was born from this very need. It is more than a robotic arm; it is an open research platform built on the ROS framework, deeply integrating 3D vision and edge AI computing. We believe breakthroughs begin with perception — by equipping a high-performance 3D depth camera, JetArm captures RGB images and depth point cloud data of its environment. This allows it to precisely understand an object's shape, pose, and spatial relationships. Freed from the limitations of a flat plane, it can achieve free-form grasping and complex task planning in real 3D space.

This is JetArm's starting point: the ultimate partner for your AI algorithms to interact precisely with 3D objects in the physical world. Whether for robotics education, cutting-edge embodied AI research, or building impressive creative AI projects, JetArm provides a powerful, fully open-source foundation.

### The Core: A Hardware-Software System Engineered for Spatial Intelligence

JetArm's architecture is thoughtfully designed, separating vision, control, and computing power for optimal performance and flexibility.

**A Dimensional Leap in Vision**: At its heart is a top-mounted 3D depth camera. It overcomes the limitations of traditional 2D cameras by providing spatial depth information — the foundational data for 3D scene understanding and manipulation.

**Master-Slave Control**: It utilizes an NVIDIA Jetson module as the main controller for vision processing and AI inference, paired with a dedicated STM32 control board for precise servo actuation. This "decision-action separation" design liberates the main controller's computational power for vision and AI applications while ensuring real-time, smooth robotic motion. It also enables flexible future upgrades of the main computing module.

**Robust Execution**: Powered by six 35KG high-torque intelligent serial bus servos, it offers substantial rigidity and a payload capacity of up to 450g, complete with interchangeable end effectors.

### The Software Stack: An Open Ecosystem Built for Developers

JetArm comes pre-installed with a Linux-based system deeply integrated with the ROS development framework, giving you full control from the ground up to the application level.

**Open-Source Framework**: The system is fully developed using ROS (Robot Operating System). This means you have access to the source code of all core algorithms and can directly utilize ROS's rich toolchain for development, debugging, and simulation.

**Full-Stack Technology Experience**: Through JetArm projects, you can gain hands-on experience with a suite of cutting-edge robotics technologies: ROS programming, 3D vision (point cloud processing), OpenCV, deep learning models like YOLO/MediaPipe, inverse kinematics, MoveIt!, Gazebo simulation, and more.

**Multimodal AI Integration**: Thanks to the powerful computing capabilities of the Jetson platform, JetArm can support and run multimodal large language models. Combined with its 3D vision and microphone array, it enables complex, natural language-based task understanding and interaction, turning the concept of Embodied AI into a tangible, hands-on experiment.

### Extensibility & Learning: Your Canvas for Creativity

JetArm is designed as a platform of infinite possibilities.

**Flexible Expansion**: The body features multiple expansion ports, allowing easy connection to various sensor modules (e.g., ultrasonic, force-torque) or functional modules for exploring advanced applications like voice interaction or force-feedback control.

**Comprehensive Tutorials**: We provide detailed, step-by-step video tutorials and learning materials covering over 18 topics—from basic motion control and camera calibration to advanced AI application development—helping you progress quickly from beginner to expert.

## Demo Videos

### Main Tutorials
- **JetArm 3D Vision Robot Arm: A Deep Dive into AI ROS Robot**: [Watch](https://www.youtube.com/watch?v=wYL03uhiER4)
- **Can JetArm Robot Arm Really Run ChatGPT? We Tested It!**: [Watch](https://www.youtube.com/watch?v=G9IH4B9-BPw)

### Your Ideas, In Action
- **Trying to Blend In? Think Again! JetArm's Sharp Eyes will Instantly Pick Out Any Tall Imposters!**: [Watch](https://www.youtube.com/shorts/vY0HlBzOgBY)
- **Powered by a 3D Depth Camera, This Robot Arm Redefines Multitasking 🤖💡**: [Watch](https://www.youtube.com/shorts/crg2_PT5pfQ)

## Hackster Projects

- **Building a Multimodal AI Brain for the JetArm Robot**: [Read](https://www.hackster.io/HiwonderRobot/building-a-multimodal-ai-brain-for-the-jetarm-robot-8accdc)
- **JetArm Pro: Expandable ROS Platform for Mobile Manipulation**: [Read](https://www.hackster.io/HiwonderRobot/jetarm-pro-expandable-ros-platform-for-mobile-manipulation-aff995)
- **JetArm Pro: Mobile Manipulator for Next-Gen Robotics**: [Read](https://www.hackster.io/HiwonderRobot/jetarm-pro-mobile-manipulator-for-next-gen-robotics-b4be45)

## Official Resources

### Official Hiwonder

- **Official Website**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **Product Page**: [https://www.hiwonder.com/products/jetarm](https://www.hiwonder.com/products/jetarm)
- **Official Documentation**: [https://docs.hiwonder.com/projects/JetArm/en/latest/](https://docs.hiwonder.com/projects/JetArm/en/latest/)
- **Technical Support**: support@hiwonder.com

## Getting Started

### Installation

Refer to the [official documentation](https://docs.hiwonder.com/projects/JetArm/en/latest/) for detailed installation instructions specific to your hardware platform.

## Community & Support

- **GitHub Issues**: Report bugs and request features
- **Email Support**: support@hiwonder.com
- **Documentation**: Comprehensive guides and tutorials

## License

This project is open-source and available for educational and research purposes.

---

**Hiwonder** - Empowering Innovation in Robotics Education
