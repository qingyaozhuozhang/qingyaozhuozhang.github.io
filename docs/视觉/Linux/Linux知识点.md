---
title: Linux 学习笔记｜Ubuntu 开发环境与常用命令
description: Linux 与 Ubuntu 开发学习笔记，整理系统基础、环境配置、常用命令及工程开发相关知识。
tags:
  - Linux
  - Ubuntu
  - 开发环境
---
# Linux / Ubuntu 开发学习笔记

## 01. Linux 基础概念 & 安装 💿

### 一. Linux 的应用领域？

- **个人桌面领域的应用**
- **服务器领域**（最强的）
- **嵌入式领域**

### 二. Linux 简介

- Linux 是一个开源、免费的操作系统。
- 常见的操作系统：windows, IOS, Android, MacOS, Linux, Unix。
- **源码下载网址**：`https://kernel.org/`
- **Linux 的主要发行版**：
  - Ubuntu (乌班图)、RedHat (红帽)、CentOS、Debain (蝶变)、Fedora、SuSE、OpenSUSE。
- Linux 主要是一个内核，其他人来包装它做成一个工具。

### 三. Unix 是怎么来的？

- (此处原文为空，保留结构)

### 四. 安装 Linux

- 学习 Linux 需要一个环境，我们需要创建一个虚拟机，然后再虚拟机上安装一个 Centos 系统来学习。
- 具体看基础操作的文件。

### 五. VM 和 Linux 的关系

- 主机/母机 (windows) `[` VM15.5虚拟机软件 `[` Linux虚拟机系统 `]` `]`

## 02. 网络连接的三种方式 🌐

1. **桥接模式**：
   - 虚拟系统可以和外部系统通讯，但是容易造成 ip 冲突。
   - `192.168.0.20`，如果前面两位相同则在同一个网段。同一个网段的 ip 最多有 255 个。
2. **NAT 模式**：
   - 网络地址转换模式，新系统可以和外部系统通讯，不造成 IP 冲突。
3. **主机模式**：
   - 独立的系统。

## 03. 虚拟机克隆 🐑

你已经安装了一台 linux 操作系统：

1. **直接拷贝一份安装好的虚拟机文件**：
   - 点击 vmware 左上角文件中的打开，打开后缀名为 `.vmx` 的文件。
2. **使用 vmware 的克隆操作**：
   - 右键点击操作系统名称，找到管理，点击克隆。克隆方法有两种，一种是打开同一个系统，另一种是开一个新的副本。

> **注意**：使用第二种方法，克隆时，需要先关闭 linux 系统（右上角有关机键）。

## 04. 虚拟机快照 📸

- 如果你在使用虚拟机系统的时候，你想回到原先的某一个状态 -> **快照管理**。
- 右键点击系统名称，找到快照。要回复到哪个状态的时候，看快照管理器，点击哪个状态，然后点击转到就可以。
- 但是每个快照会占用空间。

## 05. 虚拟机迁移和删除 📦

- 虚拟系统安装好了，它的本质就是文件（放在文件夹里的）。你可以把安装好的虚拟系统这个文件整体拷贝或者剪切到另外位置使用。
- 如果在文件夹里删除，并没有真的删掉，要到 vmware 中点击移除才行。

## 06. 安装 VMTools 🛠️

1. vmtools 安装后，可以让我们在 windows 下更好的管理 vm 虚拟机。
2. 可以设置 windows 和 centos 的共享文件夹。
   - 安装看基础操作文件。

## 07. Linux 目录结构 📂

### 一. 结构特点

1. linux 的文件系统是采用级层式的**树状目录结构**，在此结构中的最上层是根目录 `/`，然后在此目录下再创建其他的目录（全是目录）。
2. 在 linux 的世界中，**一切皆文件**。对主体的文件夹需要背下来。

## 08. Linux 远程登录 📡

1. **远程登录 - Xshell**：
   - 下载网站 `https://www.xshell.com/zh/free-for-home-school/`。
   - Xshell 是一个强大的安全终端模拟软件（需要 ip）。
2. **远程传输文件 - Xftp**：

**连接步骤：**

1. 在终端中输入 `ifconfig` -> `ens33` (linux虚拟机的) -> `inet` 后面的是它的网卡。
2. **ping**：
   - ping 使用来测试两台机器是否能够连通的。
   - 指令：`ping 目标网卡`。
3. 使用 Xshell 的流程看基础操作文件（Xftp 也是一样的流程）。

## 09. vi 和 vim 编辑器 📝

Linux 系统会内置 vi 文本编辑器，vim 是 vi 的增强版。

### 一. vi 和 vim 常用的三种模式

1. **正常模式**：
   - 以 vim 打开一个档案就直接进入一般模式了（这是默认的模式），在这个模式中，你可以使用【上下左右】按键来移动光标，你可以使用 [删除字符] 或【删除整行】来处理档案内容，也可以使用【复制、粘贴】来处理你的文件数据。
2. **插入模式**：
   - 按下 `i`, `I`, `o`, `O`, `a`, `A`, `r`, `R` 等任何一个字母之后才会进入编辑模式，一般来说按 `i` 即可。
3. **命令行模式**：
   - 在这个模式当中，可以提供你相关指令，完成读取、存盘、替换、离开 vim、显示行号等的动作。

**操作流程：**

- 在终端中输入 `vim` 语句进入正常模式。
- 输入 `i`，进入插入模式。
- 内容编辑完成之后，点击 `Esc`，进入命令行模式，然后输入 `:wq`（w 代表 write，q 代表退出）。

## 10. 开机、重启和用户登录注销 🔌

### 一. 关机和重启命令

1. **基本介绍**：
   - `shutdown -h now`：立刻进行关机（h 是 halt 的缩写）。
   - `shutdown -h 1`：1分钟后会关机 (h 应该是 halt，原文写 r 是 reboot 的缩写，此处修正为 h 对应关机)。
   - `shutdown -r now`：现在重新启动计算机。
   - `halt`：关机。
   - `reboot`：现在重新启动计算机。
   - `sync`：把内存数据同步到磁盘。
2. **注意细节**：
   - 不管是重启下系统还是关闭系统，首先要运行 `sync` 命令，把内存中的数据写到磁盘中。
   - 如果直接输入 `shutdown`，默认 `shutdown -h 1`。

### 二. 用户登录和注销

1. **基本介绍**：
   - 登录时尽量少用 root 账号登录，因为它是系统管理员，最大的权限，避免操作失误，可以利用普通用户登录，“登录后再用 `su - 用户名` 命令来切换成系统管理员身份”。
   - 在提示符下输入 `logout` 即可注销用户。
2. **使用细节**：
   - `logout` 注销指令在图形运行级别无效（就是直接鼠标右键点击终端打开的界面），在运行级别 3 下有效。
   - 如果登录非管理员用户，然后再切换到管理员用户，此时使用 `logout`，只会从管理员用户退出，然后回到非管理员用户中，如果在非管理员用户中使用 `logout`，会直接退出登录。

## 11. 用户管理 👤

Linux 是一个多用户多任务的操作系统，任何一个要使用系统资源的用户，都必须首先像系统管理员申请一个账号，然后以这个账号的身份进入系统。

1. **添加用户**：
   - 基本语法：`useradd 用户名`。
   - 当创建用户成功后，会自动的创建和用户同名的家目录（默认家目录在 `/home/新建的目录名`）。
   - 也可以通过 `useradd -d 指定目录 新建的用户名`，给新创建的用户指定家目录。
2. **指定/修改密码**：
   - 基本语法：`passwd 用户名`。
   - 注意：普通用户无法登录，需要设密码。
   - `pwd` 是用于显示当前你在哪个目录下。
3. **删除用户**：
   - `userdel 用户名`：用于删除用户，但保留用户家目录。
   - `userdel -r 用户名`：用于删除用户以及用户家目录（一般是建议保留家目录）。
4. **查询用户信息指令**：
   - 基本语法：`id 用户名`。
5. **切换用户**：
   - `su - 切换用户`（如果当前用户权限不够，先切换到最高权限目录，再切换到对应用户）。
   - 从权限高的用户切换到权限低的用户，不需要输入密码，反之需要。
   - 退出命令为 `exit` / `logout`。
6. **查看当前用户/登录用户**：
   - 基本语法：`whoami`（只会显示谁登录了）/ `who am I`（会显示登录时间和谁）。
   - 显示的是你第一次登录到这个用户的信息（屏蔽掉切换的用户）。
7. **用户组**：
   - 类似于角色，系统可以对有共性/权限的多个用户进行统一的管理。
   - **新增组**：`groupadd 组名`。
     - 如果创建用户时没有其他指令，会直接创建一个以用户名来命名的组。
   - **删除组**：`groupdel 组名`。
   - **增加用户时直接加上组**：`useradd -g 用户组 用户名`。
   - **修改用户的组**：`usermod -g 用户组 用户名`。
8. **用户和组相关的文件**：
   - `/etc/passwd` 文件：用户 (user) 的配置文件，记录用户的各种信息。
     - 每行的含义：`用户名：口令：用户标识号（id）：注释性描述：主目录：登录Shell`。
   - `/etc/shadow` 文件：口令配置文件。
     - 每行的含义：`登录名：加密口令：最后一次修改时间：最小时间间隔：最大时间间隔：警告时间：不活动时间：失效时间：标志`。
   - `/etc/group` 文件：组的配置文件，记录 Linux 包含的组的信息。
     - 每行含义：`组名：口令：组标识号：组内用户列表`。
   - 可以使用 `vim` 来查看文件内容。加密口令其实就是加密用户的密码。

## 12. 找回 root 密码 🔑

1. 启动系统，进入开机界面，在界面中按 `e` 进入编辑界面。
2. 使用键盘上的上下键光标往下移动，找到以 “Linux16” 开头内容所在的行数。
3. 在行的最后面输入 `:init=/bin/sh`。
4. 输入完成后，直接按快捷键：`ctrl+x` 进入单用户模式。
5. 在鼠标闪烁的位置输入：`mount -o remount,rw /`，完成后按回车键。
6. 在新的一行最后面输入：`passwd root`，完成后按回车键，输入密码，再次确认密码。
   - 密码修改成功后，会显示 `passwd...` 的样式。
7. 继续在光标闪烁的位置上输入 `touch /.autorelabel`。
8. 输入 `exec /sbin/init`，完成后按回车键。
9. 等待系统自动修改密码，完成后，系统会自动重启，新的密码生效。

## 13. Linux 帮助指令 ❓

1. **man 获得帮助信息**：
   - 基本语法：`man [命令或配置文件]`（功能描述：获得帮助信息）。
   - 示例：`man ls`。
2. **help 指令**：
   - 基本语法：`help 指令`（功能描述：获得 shell 内置命令的帮助信息）。
   - 如果没有全部显示出来可以按空格键，使其继续显示。

## 14. 文件目录类 📁

1. **pwd 指令**：显示当前工作目录的绝对路径。
2. **ls 指令**：`ls [选项][目录或是文件]`。
   - `-a`: 显示当前目录所有的文件和目录，包括隐藏的。
   - `-l`: 以列表的方式显示信息。
3. **cd 指令**：切换到指定目录。
   - `cd ~` 或者 `cd`：回到自己的家目录。
   - `cd ..`：回到当前目录的上一级目录（可以这样写 `cd ../../..`）。
4. **mkdir 指令**：用于创建目录。
   - `-p`: 创建多级目录。
   - 示例：`mkdir -p /home/animal/tiger`。
5. **rmdir 指令**：删除空目录。
   - **注意**：`rmdir` 删除的是空目录，如果目录下有内容时是无法删除的。
   - 提示：如果需要删除非空目录，需要使用 `rm -rf 要删除的目录`。
6. **touch 指令**：创建空文件。
7. **cp 指令**：拷贝文件到指定目录。
   - 语法：`cp [选项] source dest`。
   - `-r`：递归复制整个文件夹。
   - 强制覆盖不提示的方法：`\cp`。
8. **rm 指令**：移除文件或目录。
   - `-r`: 递归删除整个文件夹。
   - `-f`: 强制删除不提示。
9. **mv 指令**：移动文件与目录或重命名。
   - `mv oldName newName` (重命名)。
   - `mv /temp/movefile /targetFolder` (移动文件)。
10. **cat 指令**：查看文件内容。
    - `-n`: 显示行号。
    - **细节**：cat 只能浏览文件，一般会带上管道命令 `| more`（enter 多显示一行，空格键是换页）。
11. **more 指令**：基于 Vi 编辑器的文本过滤器，按页显示。
12. **less 指令**：分屏查看，比 more 强大。不是一次加载整个文件，适合大型文件。
13. **echo 指令**：输出内容到控制台。
14. **head 指令**：显示文件的开头部分（默认前 10 行）。
    - `head -n 5 文件`（查看前 5 行）。
15. **tail 指令**：输出文件中尾部的内容（默认后 10 行）。
    - `tail -n 5 文件`（查看后 5 行）。
    - `tail -f 文件`（实时追踪该文档的所有更新）。
16. **> 和 >> 指令**：
    - `>` 输出重定向（覆盖写）。示例：`ls -l > a.txt`。
    - `>>` 追加。示例：`ls -al >> aa.txt`。
17. **ln 指令**：软链接，类似于 windows 里的快捷方式。
    - `ln -s [原文件或目录] [软链接名]`。
    - 删除软链接：`rm myroot`。
18. **history 指令**：查看已执行过的历史命令。
    - `history 10`：查看最近 10 个。
    - `!5`：将序号为 5 的命令重新执行一遍。

## 15. 时间日期类 📅

1. **date 指令 - 显示当前日期**：
   - `date`（显示当前时间）。
   - `date +%Y`（显示当前年份）。
   - `date +%m`（显示当前月份）。
   - `date +%d`（显示当前是哪一天）。
   - `date "+%Y-%m-%d %H:%M:%S"`（显示年月日时分秒）。
2. **date 指令 - 设置日期**：
   - `date -s 字符串时间`。
3. **cal 指令**：查看日历。
   - `cal 2020`（显示 2020 年的日历）。

## 16. 搜索查找类 🔍

1. **find 指令**：从指定目录向下递归地遍历。
   - `-name <查询文件名>`：按照指定的文件名查找。
   - `-user <用户名>`：查找属于指定用户名所有文件。
   - `-size <文件大小>`：按大小查找（+n 大于，-n 小于，n 等于，单位 k, M, G）。
2. **locate 指令**：快速定位文件路径，基于数据库。
   - **特别说明**：第一次运行前，必须使用 `updatedb` 指令创建 locate 数据库。
   - `which` 指令：查看某个指令在哪个目录下。
3. **grep 指令和管道符号 |**：
   - grep 过滤查找，管道符 `|` 表示将前一个命令的处理结果传递给后面的命令处理。
   - `-n`: 显示匹配行及行号。
   - `-i`: 忽略字母大小写。
   - 示例：`cat 路径 | grep "yes"` 或 `grep -n "yes" 文件路径`。

## 17. 压缩和解压类 📦

1. **gzip / gunzip 指令**：
   - `gzip 文件路径`：压缩文件，只能将文件压缩为 `*.gz` 文件。
   - `gunzip 文件.gz`：解压缩文件。
2. **zip / unzip 指令**：
   - `zip [选项] xxx.zip 将要压缩的内容`。
     - `-r`: 递归压缩，即压缩目录。
   - `unzip [选项] xxx.zip`。
     - `-d <目录>`: 指定解压后文件的存放目录。
3. **tar 指令**：打包指令，最后打包的文件是 `.tar.gz`。
   - `-c`: 产生 .tar 打包文件。
   - `-v`: 显示详细信息。
   - `-f`: 指定压缩后的文件名。
   - `-z`: 打包同时压缩。
   - `-x`: 解包 .tar 文件。
   - 示例（压缩）：`tar -zcvf xxx.tar.gz 文件路径`。
   - 示例（解压）：`tar -zxvf /home/myhome.tar.gz -C /opt/tmp2`。

## 18. 组管理和权限管理 🔐

1. **基本概念**：每个用户必须属于一个组。每个文件有所有者、所在组、其他组的概念。
2. **所有者**：
   - 查看指令：`ls -ahl`（第一个是所有者，第二个是所在组）。
   - 修改所有者：`chown 用户名 文件名`。
3. **所在组**：
   - 修改所在组：`chgrp 组名 文件名`。
4. **其他组**：除所有者和所在组外的用户。
5. **权限介绍** (`-rwxrm-r--`)：
   - **第 0 位**：文件类型（`l` 链接, `d` 目录, `c` 字符设备, `b` 块设备, `-` 普通文件）。
   - **第 1-3 位 (User)**：所有者权限。
   - **第 4-6 位 (Group)**：所属组权限。
   - **第 7-9 位 (Other)**：其他用户权限。
   - **rwx 含义**：
     - `r` (4): 读。
     - `w` (2): 写（对目录来说是创建/删除/重命名）。
     - `x` (1): 执行（对目录来说是进入）。
6. **修改权限 - chmod**：
   - 方式一：`chmod u=rwx,g=rx,o=x 文件名`。
   - 方式二：`chmod 751 文件名` (4+2+1=7)。
7. **修改所有者/所在组**：
   - `chown newowner:newgroup 文件`。
   - `-R`：递归生效。

## 19. 权限管理应用实例 👮

- **场景**：police, bandit 组；jack, jerry (警察)；xh, xq (土匪)。
- **流程**：
  1. 创建组。
  2. 创建用户。
  3. jack 创建文件，自己 r+w，本组 r，其他组无权限。
  4. 修改权限测试读写。

## 20. 定时任务调度 ⏰

### 一. crond 任务调度

- `crontab` 进行定时任务的设置。
- **基本语法**：
  - `crontab -e`: 编辑定时任务。
  - `crontab -l`: 查询任务。
  - `crontab -r`: 删除当前用户所有任务。
- `service crond restart`：重启任务调度。

## 21. 任务调度机制 (at) ⏲️

### 一. at 定时任务

1. **基本介绍**：
   - `at` 是**一次性**定时计划任务，由 `atd` 守护进程执行。
   - `atd` 每 60 秒检查一次队列。
   - 查看进程：`ps -ef | grep atd`。
2. **命令格式**：`at [选项] [时间]`。
   - 结束输入：`ctrl + d`。
3. **时间定义**：
   - `hh:mm` (如 04:00)。
   - `midnight`, `noon`, `teatime`。
   - `12pm`, `AM/PM`。
   - `now + 5 minutes`。
   - `tomorrow`, `today`。
4. **常用指令**：
   - `atq`: 查看系统中的任务。
   - `atrm 序号`: 删除已经设置的任务。

## 22. 磁盘分区、挂载 💾

### 一. Linux 分区

1. **原理**：Linux 只有一个根目录，采用“载入”方法，将一个分区和一个目录联系起来。
2. **查看挂载**：`lsblk` 或者 `lsblk -f`。
3. **硬盘说明**：
   - IDE 硬盘标识为 `hdx~`。
   - SCSI 硬盘标识为 `sdx~`。

### 二. 挂载经典案例

- 增加硬盘步骤：虚拟机添加硬盘 -> 分区 -> 格式化 -> 挂载 -> 设置自动挂载。

## 23. 磁盘情况查询 📊

1. **查询系统整体磁盘使用情况**：
   - `df -h`。
2. **查询指定目录的磁盘占用情况**：
   - `du -h /目录`。
   - `-s`: 汇总。
   - `-a`: 含文件。
   - `--max-depth=1`: 子目录深度。

## 24. 磁盘情况 - 工作实用指令 🧮

需要结合过滤查找 (`grep`) 和 统计 (`wc -l`)。

1. 统计 `/opt` 文件夹下文件的个数：`ls -l /opt | grep "^-" | wc -l`。
2. 统计 `/opt` 文件夹下目录的个数：`ls -l /opt | grep "^d" | wc -l`。
3. 统计包括子文件夹的文件个数：`ls -lR /opt | grep "^-" | wc -l`。
4. 以树状显示目录结构：`tree 目录`（需安装 `yum install tree`）。

## 25. 网络配置 🌐

### 一. 查看和配置

1. **查看 IP 和网关**：
   - Windows: `ipconfig`。
   - Linux: `ifconfig`。
2. **ping 测试**：`ping 目的主机`。
3. **网络环境配置**：
   - **自动获取**：特点是启动后自动获取，缺点是 IP 可能变。
   - **指定 IP (静态)**：修改 `/etc/sysconfig/network-scripts/ifcfg-ens33`。

**静态 IP 配置示例：**

Bash

```
BOOTPROTO=static
IPADDR=192.168.200.130
GATEWAY=192.168.200.2
DNS1=192.168.200.2
```

- 修改后重启服务：`service network restart` 或 `reboot`。

## 26. 设置主机名和 hosts 映射 📝

### 一. 设置主机名

- 查看：`hostname`。
- 修改：`/etc/hostname`。

### 二. 设置 hosts 映射

- **Windows**: `C:\Windows\System32\drivers\etc\hosts`。
- **Linux**: `/etc/hosts`。

### 三. DNS 机制

- **Hosts**: 记录 IP 和 Hostname 映射的文本文件。
- **DNS**: 域名系统，分布式数据库。
- 顺序：本地解析器缓存 -> DNS -> Hosts -> 互联网 (原文顺序如此)。

## 27. 进程管理 📈

### 一. 基本介绍

- 每个进程都有一个 ID 号 (pid)。
- 进程分为**前台**和**后台**。

### 二. 显示系统执行的进程 (ps)

- `ps -aux | grep xxx`: 查看进程。
- `ps -ef`: 全格式显示，可以查看**父进程**。
- **参数详解**：
  - `USER`: 用户。
  - `PID`: 进程号。
  - `%CPU` / `%MEM`: 占用率。
  - `STAT`: 状态 (S-睡眠, R-运行, Z-僵死)。

### 三. 终止进程

- `kill [选项] 进程号`。
  - `-9`: 强迫立即停止。
- `killall 进程名称` (支持通配符)。

### 四. 查看进程树

- `pstree [选项]`。
  - `-p`: 显示 PID。
  - `-u`: 显示所属用户。

## 28. 服务管理 ⚙️

### 一. 介绍

- 服务本质是运行在后台的进程，监听端口，称为**守护进程**。
- **service 指令** (CentOS 7.0 之前常用):
  - `service 服务名 [start|stop|restart|reload|status]`。
  - 位置：`/etc/init.d`。

### 二. 服务的运行级别

- **常用级别**：`3` (多用户命令行) 和 `5` (图形界面)。
- `0`: 停机。
- `6`: 重启。
- CentOS 7 简化为 `multi-user.target` (3) 和 `graphical.target` (5)。

### 三. systemctl 管理指令 (CentOS 7+)

- 语法：`systemctl [start|stop|restart|status] 服务名`。
- **设置自启动**：
  - `systemctl list-unit-files`: 查看状态。
  - `systemctl enable 服务名`: 设置开机启动。
  - `systemctl disable 服务名`: 关闭开机启动。

### 四. 打开/关闭指定端口 (Firewall)

- 打开端口：`firewall-cmd --permanent --add-port=端口号/协议`。
- 关闭端口：`firewall-cmd --permanent --remove-port=端口号/协议`。
- **重新载入生效**：`firewall-cmd --reload`。
- 查询端口：`firewall-cmd --query-port=端口/协议`。

## 29. 动态监控系统 🖥️

### 一. top 指令

- `top` 可以实时更新正在运行的进程。
- `-d 秒数`: 指定更新间隔。
- **交互操作**：
  - `P`: 以 CPU 使用率排序。
  - `M`: 以内存使用率排序。
  - `k`: 终止指定进程。
  - `u`: 监视特定用户。

### 二. 监控网络状态

- `netstat -anp`: 查看系统网络情况。
  - `-an`: 按顺序排列。
  - `-p`: 显示进程。

## 30. RPM 管理 📦

### 一. 介绍

- RPM 是 RedHat 软件包管理工具，用于下载包打包及安装。

### 二. 查询指令

- `rpm -qa`: 查询所有安装的 rpm 软件包。
- `rpm -qa | grep xx`: 查询特定软件。
- `rpm -q 软件包名`: 查询是否安装。
- `rpm -qi`: 查询信息。
- `rpm -qf 文件全路径`: 查询文件所属的软件包。

### 三. 管理指令

- **卸载**：`rpm -e RPM包名称`。
  - `--nodeps`: 强制删除，忽略依赖。
- **安装**：`rpm -ivh RPM包全路径`。
  - `i`=install, `v`=verbose, `h`=hash (进度条)。

## 31. YUM 🥕

### 一. 介绍

- Yum 是 Shell 前端软件包管理器，基于 RPM，能自动下载并**处理依赖性关系**。

### 二. 指令

- **查询**：`yum list | grep xx`。
- **安装**：`yum install xxx`。

---

## 32. Ubuntu 工程实战：从命令到问题定位（扩展）

> 这一部分补足前面 01—31 节中的操作链条，所有示例以 Ubuntu 22.04/24.04 为主。命令前的 `$` 是提示符，不需要输入。会修改系统配置的命令，请在测试环境验证后使用。

### 32.1 终端、Shell、路径和进程究竟是什么？

- **终端**（Terminal）负责显示与输入；**Shell**（如 Bash）解析命令、管道、重定向；**内核**负责进程调度、文件系统和硬件管理。
- **绝对路径**从 `/` 开始；**相对路径**以当前目录为参照；`~` 是当前用户家目录；`.` 是当前目录；`..` 是上级目录。
- Shell 展开顺序会影响命令：`"$name"` 保留参数整体，裸 `$name` 可能按空白拆词；路径中有空格时必须加引号。

```bash
pwd                              # 打印当前工作目录
printf 'user=%s home=%s\n' "$USER" "$HOME"
mkdir -p ~/linux-demo/{src,logs}  # -p: 父目录不存在时一并创建
cd ~/linux-demo
printf '%s\n' 'hello linux' > src/hello.txt
cat src/hello.txt
find . -maxdepth 2 -type f -print
```

**输出**应包含 `./src/hello.txt`。`>` 是覆盖写，`>>` 是追加写；如果误将 `>` 用在重要文件，旧内容可能被覆盖。

### 32.2 文件权限：三组 rwx 与权限数字如何对应？

`ls -l` 结果类似 `-rwxr-x---`：第一位表示普通文件 `-` / 目录 `d`，接下来依次是所有者、所属组、其他用户的读写执行权限。`r=4`、`w=2`、`x=1`，所以 `750` 表示 `rwxr-x---`。

```bash
ls -l src/hello.txt
chmod 640 src/hello.txt       # 所有者读写、组可读、其他人无权限
chmod u+x ./run.sh           # 若 run.sh 存在，给所有者增加执行权限
stat -c '%a %U:%G %n' src/hello.txt
id
```

**常见错误**：`./run.sh: Permission denied` 不一定需要 `sudo`，优先检查文件的 `x` 权限以及挂载点是否 `noexec`。`chmod 777` 会扩大不必要的权限，不应作为默认修复办法。复制到非 POSIX 文件系统后权限语义可能发生变化。

### 32.3 管道、退出状态、grep/find/awk/sed 的协作

管道 `|` 把前一个进程的标准输出接到后一个进程的标准输入；`&&` 只有前一个成功才执行后续，`||` 表示失败时执行后续。

```bash
printf 'alpha\nbeta\nalpine\n' > src/names.txt
grep -n '^al' src/names.txt        # 查找以 al 开头的行，同时显示行号
awk '{print NR ":" $0}' src/names.txt
sed -n '2p' src/names.txt         # 仅打印第二行
find ./src -type f -name '*.txt' -print
false || echo 'previous command failed'
```

脚本里建议添加 `set -euo pipefail`（理解它的限制后使用），可让未定义变量、明显失败与管道中间步骤失败更容易暴露；条件判断内的命令仍需按 Bash 规则处理。

### 32.4 进程和服务：为什么终端关闭 Flask 服务就停？

前台执行的 Python 进程通常关联交互式终端；关闭终端可能导致 SIGHUP 或会话结束。真正的长期服务应由 systemd 等监督进程管理：负责启动、自动重启、日志和开机自启。

```bash
ps aux | grep '[p]ython'
pgrep -af 'python|flask'
ss -lntp | grep ':5000' || true
systemctl --version | head -n 1
journalctl -u ssh --since '1 hour ago' --no-pager | tail -n 20
```

自建 Python 服务的 systemd **示例**（先替换用户名与绝对路径，不能直接照抄部署）：

```ini
# /etc/systemd/system/my-python-app.service
[Unit]
Description=My Python application
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/myapp
ExecStart=/home/ubuntu/myapp/.venv/bin/python /home/ubuntu/myapp/app.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now my-python-app.service
systemctl status my-python-app.service
journalctl -u my-python-app.service -n 80 --no-pager
```

对外提供登录服务时应配置 HTTPS、反向代理、认证和秘密管理；不要在生产中直接暴露 Flask 的开发服务器。

### 32.5 硬盘空间不足：区分 df、du 和 inode

```bash
df -hT / /home                 # 文件系统剩余空间，注意 /home 是否独立分区
df -i /                       # inode 用量；inode 耗尽也会无法创建文件
sudo du -xhd1 /home 2>/dev/null | sort -h
find ~/.cache -maxdepth 2 -type d -print | head
lsblk -f
findmnt -T /home
```

**解释**：`df` 反映文件系统总量；`du` 按目录累计可访问文件大小；删除已经被进程打开的文件，空间可能要等进程释放句柄才归还；`du` 和 `df` 不一致时可用 `sudo lsof +L1` 排查。

清理顺序优先是确认大目录与用途 → 删除可重建缓存 → 核验释放空间。不要为了释放空间执行 `sudo rm -rf /`、无差别清空系统目录或删除项目数据库。

### 32.6 apt、dpkg、Python venv 与动态库搜索路径

```bash
sudo apt update                      # 只刷新软件包索引
apt policy python3                  # 查来源及可安装版本
dpkg -l | grep '^ii' | head
python3 -m venv .venv
source .venv/bin/activate
python -m pip --version             # 确认 pip 指向虚拟环境
which python
python -m pip list
```

Ubuntu 系统级 Python 包与 `venv` 应区分管理；安装第三方依赖推荐 `python -m pip install ...`，不要随意对系统 Python 使用 `sudo pip`。

```bash
ldd /path/to/executable | grep 'not found'
find /usr/local/lib -name '*.so*' -maxdepth 2 -print | head
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
```

`LD_LIBRARY_PATH` 赋值的等号左右**不允许有空格**。使用 `ldd` 时应避免运行来源不可信的可执行文件；生产环境更推荐 `ldconfig`/RPATH 等清晰的依赖管理方案。

### 32.7 网络诊断：设备能 ping 通不代表应用能访问

区分 DNS → 路由 → TCP 端口 → HTTP 应用 → 防火墙 → TLS：

```bash
ip -br address
ip route
getent hosts github.com
ping -c 3 1.1.1.1
ss -lntp
curl -I --max-time 5 https://github.com
curl -v --max-time 5 http://127.0.0.1:5000/
```

服务器监听 `127.0.0.1:5000` 时通常仅本机可连；若客户端要跨设备访问，还需监听合适的接口并检查主机/云安全组防火墙规则，不能只凭 `ping` 成功判断服务端口已开放。内外网部署还需明确 HTTPS 和认证策略。

### 32.8 编写一个可测试的 Bash 运维脚本

```bash
#!/usr/bin/env bash
set -euo pipefail

log_file="${1:-./machine-status.log}"
{
  echo "===== $(date -Is) ====="
  echo '--- uptime ---'
  uptime
  echo '--- disk ---'
  df -h /
  echo '--- memory ---'
  free -h
  echo '--- network ---'
  ip -br address
} >> "$log_file"
printf 'log saved to %s\n' "$log_file"
```

保存为 `system_report.sh`，执行 `chmod +x system_report.sh && ./system_report.sh`；命令行传参 `./system_report.sh /tmp/report.log` 会将输出写到自定义位置。脚本中的 `"${1:-默认值}"` 表示第一个位置参数为空或未提供时使用默认值。

### 32.9 cron、日志与磁盘空间协同

`crontab -e` 的例子：`0 2 * * * /home/ubuntu/scripts/backup.sh >> /home/ubuntu/backup.log 2>&1`，意为每天凌晨 2 点执行。cron 的 PATH 通常不同于交互式 Shell，所以脚本中应尽量写绝对路径；cron 是否运行、日志是否写出、脚本是否成功，需要分开验证。

**对比**：cron 适合定时启动，systemd 适合长期服务；`at` 适合一次性任务。日志长时间累计应配合 logrotate 或 journald 的容量限制。

### 32.10 综合练习：ROS 2 环境自检

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
ros2 node list
ros2 topic list -t | grep -E '(/tf|/odom|/Odometry|/livox)' || true
ros2 doctor --report | head -n 50
```

该脚本只能做运行环境快速检查，不证明定位或导航质量正常。对时间敏感传感器，还需确认 ROS 时间/系统时钟、TF 树与数据延迟。
