---
title: 镜神的知识网络｜ROS2、机器人导航与计算机视觉
description: 面向机器人开发的技术知识库，整理 ROS2、Nav2、Livox Mid-360、FAST-LIO、SLAM、OpenCV、YOLO、Linux、Python、C/C++ 等学习笔记与工程实践。
hide:
  - toc
---

<div class="jingshen-hero" markdown>

<span class="jingshen-eyebrow">QINGYAOZHUOZHANG · SECOND BRAIN</span>

# 镜神的知识网络

**把零散的踩坑记录，整理成可以反复检索、直接复用的工程知识。**

这里重点沉淀 **ROS 2、机器人导航、Livox Mid-360、FAST-LIO、Nav2、SLAM、OpenCV、YOLO、Linux 与编程基础**。内容来自学习、调试和实际项目过程，并持续更新。

[开始浏览 ROS 2 :material-arrow-right:](navigation/ROS2/导航_主线/ROS2主流程.md){ .md-button .md-button--primary }
[Mid-360 配置教程](navigation/lidar/Mid-360激光雷达配置教程.md){ .md-button }
[查看 GitHub](https://github.com/qingyaozhuozhang/qingyaozhuozhang.github.io){ .md-button }

</div>

## 从这里开始

<div class="grid cards" markdown>

-   :material-robot-outline:{ .lg .middle } **ROS 2 与机器人导航**

    ---

    ROS 2 通信、TF、Launch、Nav2、BehaviorTree、代价地图与完整导航流程。

    [:octicons-arrow-right-24: ROS2 主流程](navigation/ROS2/导航_主线/ROS2主流程.md)

-   :material-radar:{ .lg .middle } **Livox Mid-360 与 SLAM**

    ---

    Mid-360 驱动、网络配置、点云、PCD、FAST-LIO、定位与重定位实践。

    [:octicons-arrow-right-24: Mid-360 配置教程](navigation/lidar/Mid-360激光雷达配置教程.md)

-   :material-eye-outline:{ .lg .middle } **计算机视觉**

    ---

    OpenCV、YOLOv8、深度学习基础、数据集标注、模型训练与调用。

    [:octicons-arrow-right-24: OpenCV 笔记](视觉/opencv/opencv知识点.md)

-   :material-linux:{ .lg .middle } **Linux 与开发工具**

    ---

    Ubuntu / Linux、Git、Markdown 与常用开发工具的配置和使用记录。

    [:octicons-arrow-right-24: Linux 知识点](视觉/Linux/Linux知识点.md)

-   :material-code-braces:{ .lg .middle } **Python / C / C++**

    ---

    编程基础、工程规范、数据结构与算法，作为项目开发的基础知识索引。

    [:octicons-arrow-right-24: Python 基础](编程/python/核心基础语法.md)

-   :material-tools:{ .lg .middle } **可复用例程**

    ---

    摄像头调用、YOLO 数据集、模型训练、仿真和实车操作等可直接复用流程。

    [:octicons-arrow-right-24: 常用例程](常用例程/编程常用例程/基础操作.md)

</div>

## 推荐阅读

<div class="grid cards" markdown>

- **Livox Mid-360：Ubuntu 驱动、ROS2 与 FAST-LIO 建图**  
  从环境依赖、网络配置到点云与 FAST-LIO，适合第一次把 Mid-360 跑通。  
  [:material-book-open-page-variant: 阅读教程](navigation/lidar/Mid-360激光雷达配置教程.md)

- **Navigation2 导航框架**  
  梳理 Nav2 行为树、全局/局部代价地图和导航框架中的关键组件。  
  [:material-book-open-page-variant: 阅读笔记](navigation/ROS2/导航_主线/Navigation2导航框架.md)

- **BehaviorTree.ROS2 开发者笔记**  
  面向机器人任务逻辑的行为树结构、节点与工程实践。  
  [:material-book-open-page-variant: 阅读笔记](navigation/ROS2/导航_主线/决策树.md)

- **YOLOv8 部署与训练结果分析**  
  从环境配置到训练结果，为视觉检测项目提供快速入口。  
  [:material-book-open-page-variant: 阅读实践](视觉/深度学习/yolo实践.md)

</div>

## 知识地图

| 方向 | 关键词 |
| --- | --- |
| 机器人导航 | ROS2、Nav2、BehaviorTree、TF、Launch、Costmap |
| 激光雷达 | Livox Mid-360、FAST-LIO、点云、PCD、SLAM、重定位 |
| 计算机视觉 | OpenCV、YOLO、深度学习、目标检测 |
| 系统与工具 | Linux、Ubuntu、Git、Markdown |
| 编程 | Python、C、C++、数据结构与算法 |

!!! tip "使用建议"
    不知道内容在哪时，直接点击页面顶部的搜索框，输入 `Mid-360`、`Nav2`、`TF`、`YOLO`、`OpenCV` 等关键词。

## 近期整理

<!-- AUTO_RECENT_UPDATES_START -->
> 此区域会在 MkDocs 构建时根据 Git 提交记录自动生成，无需手动维护。
<!-- AUTO_RECENT_UPDATES_END -->
