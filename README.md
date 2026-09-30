<div align="center">

# 镜神的知识网络

**ROS 2 · 机器人导航 · Livox Mid-360 · FAST-LIO · Nav2 · 计算机视觉 · Linux**

把零散的学习记录、配置流程和项目踩坑，整理成一套可以反复检索、快速复用的个人技术知识库。

[![Website](https://img.shields.io/badge/Website-qingyaozhuozhang.github.io-4f46e5?logo=githubpages&logoColor=white)](https://qingyaozhuozhang.github.io/)
[![GitHub Pages](https://img.shields.io/github/deployments/qingyaozhuozhang/qingyaozhuozhang.github.io/github-pages?label=GitHub%20Pages&logo=github)](https://qingyaozhuozhang.github.io/)
[![Stars](https://img.shields.io/github/stars/qingyaozhuozhang/qingyaozhuozhang.github.io?style=flat&logo=github)](https://github.com/qingyaozhuozhang/qingyaozhuozhang.github.io/stargazers)
[![Last Commit](https://img.shields.io/github/last-commit/qingyaozhuozhang/qingyaozhuozhang.github.io?logo=git)](https://github.com/qingyaozhuozhang/qingyaozhuozhang.github.io/commits/main)

**[进入知识库](https://qingyaozhuozhang.github.io/) · [ROS 2 导航](https://qingyaozhuozhang.github.io/navigation/ROS2/导航_主线/ROS2主流程/) · [Livox Mid-360](https://qingyaozhuozhang.github.io/navigation/lidar/Mid-360激光雷达配置教程/)**

</div>

---

## 这个仓库是什么

这是一个面向 **机器人开发与工程实践** 的个人第二大脑，重点整理真实学习和项目过程中会反复用到的内容：

- **ROS 2 / Nav2 / BehaviorTree**：导航框架、行为树、Launch、TF、常用开发工具
- **Livox Mid-360 / FAST-LIO / SLAM**：激光雷达配置、点云、定位、建图与重定位
- **计算机视觉**：OpenCV、YOLO、深度学习基础与实践
- **Linux / Git / Ubuntu**：开发环境、版本控制和常见问题
- **Python / C / C++**：基础语法、数据结构、算法与工程规范
- **常用例程**：摄像头调用、YOLO 数据集、模型训练、仿真与实车操作

> [!TIP]
> 如果你正在搜索 **ROS2 导航、Livox Mid-360 配置、FAST-LIO 建图、Nav2、OpenCV、YOLO 或 Linux 开发笔记**，建议直接进入在线知识库使用站内搜索。

## 推荐入口

| 主题 | 内容 |
| --- | --- |
| 🚗 ROS 2 导航 | [ROS2 主流程](https://qingyaozhuozhang.github.io/navigation/ROS2/导航_主线/ROS2主流程/) · [Navigation2 导航框架](https://qingyaozhuozhang.github.io/navigation/ROS2/导航_主线/Navigation2导航框架/) |
| 📡 Livox Mid-360 | [Mid-360 配置教程](https://qingyaozhuozhang.github.io/navigation/lidar/Mid-360激光雷达配置教程/) · [雷达定位信息](https://qingyaozhuozhang.github.io/navigation/lidar/雷达定位信息/) |
| 🌳 BehaviorTree | [BehaviorTree.ROS2 开发笔记](https://qingyaozhuozhang.github.io/navigation/ROS2/导航_主线/决策树/) |
| 👁️ 计算机视觉 | [OpenCV 学习笔记](https://qingyaozhuozhang.github.io/视觉/opencv/opencv知识点/) · [YOLOv8 实践](https://qingyaozhuozhang.github.io/视觉/深度学习/yolo实践/) |
| 🐧 Linux / Git | [Linux 知识点](https://qingyaozhuozhang.github.io/视觉/Linux/Linux知识点/) · [Git 学习笔记](https://qingyaozhuozhang.github.io/视觉/Linux/git知识点/) |
| 💻 编程基础 | Python、C、C++、算法与数据结构 |

## 仓库结构

```text
.
├── docs/                  # 网站正文与图片
│   ├── navigation/        # ROS2、Nav2、激光雷达、SLAM
│   ├── 视觉/              # 深度学习、YOLO、OpenCV、Linux
│   ├── 编程/              # Python、C、C++
│   ├── 常用例程/          # 可直接复用的操作与代码流程
│   └── 日常工具扩展/      # 工具、资料与 Markdown
├── mkdocs.yml             # MkDocs 站点配置与导航
├── requirements.txt       # Python 依赖
└── README.md
```

## 本地预览

首次克隆：

```bash
git clone git@github.com:qingyaozhuozhang/qingyaozhuozhang.github.io.git
cd qingyaozhuozhang.github.io

python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt

mkdocs serve
```

浏览器打开终端提示的本地地址即可预览。

## 日常更新流程

已经克隆过仓库后，不需要再次执行 `git init` 或 `git remote add`。每次编辑前先同步：

```bash
git pull --rebase origin main
```

在 `docs/` 中编辑或新增 Markdown 文件；如果新增页面需要出现在导航栏，在 `mkdocs.yml` 的 `nav` 中加入对应路径。

编辑完成后：

```bash
git add .
git commit -m "docs: update notes"
git push origin main
```

GitHub Actions / GitHub Pages 会按照仓库现有部署流程更新网站。

## 技术栈

`MkDocs` · `Material for MkDocs` · `Markdown` · `GitHub Pages`

---

<div align="center">

如果这些笔记对你有帮助，可以给仓库点一个 ⭐，方便以后再次找到。

</div>
