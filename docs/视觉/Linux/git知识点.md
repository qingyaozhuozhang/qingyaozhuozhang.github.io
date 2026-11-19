---
title: Git 学习笔记｜版本控制、分支与 GitHub 协作
description: Git 版本控制学习笔记，整理仓库、提交、分支、远程仓库与 GitHub 协作的常用流程。
tags:
  - Git
  - GitHub
  - 版本控制
---
# Git 与 GitHub 版本控制学习笔记

## 01. Git 简介与配置 ⚙️

### 一. 什么是 git?

- git 是一个免费开源的分布式版本控制系统。
- 它使用一个特殊的叫做仓库的数据库来记录文件的变化。
- 仓库中的每个文件都有一个完整的版本历史记录。

### 二. 版本控制系统

1. **集中式**: SVN
2. **分布式**: git

### 三. git 的使用方式

1. 命令行
2. 图形化界面 (GUI)
3. IDE 插件/扩展

### 四. 配置

1. **使用 `git config` 命令配置一下用户名和邮箱**。示例：

   Bash

```
git config --global user.name "Jasper Yang" # 如果用户名中间没有空格，双引号可以省略
git config --global user.email geekhall.cn@gmail.com
```

   - `省略 (Local)`: 本地配置，只对本地仓库有效
   - `--global`: 全部配置，所有仓库生效
   - `--system`: 系统配置，对所有用户生效

2. **保存用户名和邮箱**: 使用 `git config --global credential.helper store`，这样就可以不用每次都输入了。

3. **查看配置**: 使用 `git config --global --list`。

## 02. 新建仓库与克隆 📦

### 一. 新建仓库

1. **`git init`**:
   - 首先创建一个目录 `mkdir 目录名称`，然后再输入 `git init`，接着就创建好了一个被 git 管理的仓库。
   - **注意**:
     - `.git` 是一个隐藏目录，需要输入指令符才能查看。
     - 如果在 init 后面加上名称，示例：`git init ab`，就会在该目录下创建一个 ab 目录，然后在 ab 目录下创建 git 仓库。
2. **`git clone`**

### 二. 关于 git init 的相关东西

1. 输入 `ls -a` 来显示所有文件。如果显示有 `.git` 的文件，说明仓库已经新建成功了。
   - `.git` 这个目录存放了 Git 仓库的所有数据。
   - **注意**: 输入 `ls -a` 和 `ls -al` 是不同的，`ls -a` 是单纯就是显示目录内文件或目录的名称，而 `ls -al` 则是再多显示其他东西。
2. 想看到 `.git` 目录里的文件和目录，先输入 `cd .git`，然后再输入 `ls -altr`。
3. **删除 .git 目录**: 输入 `rm -rf .git`（删除完以后，不会显示，删除信息，返回删除的路径）。

### 三. 关于 git clone 的相关东西

1. 直接在 clone 后面输入远程仓库的地址，示例：

   Bash

```
git clone https://github.com/geekhall-l # 但是这个网址已经没有用了
```

2. 使用 `ls -ltr` 也可以看到当前目录下的仓库 (但是看不到隐藏的目录（比 `ls -altr` 看到的少）)。

## 03. 工作区域和文件状态 🗂️

### 一. 工作区域

1. **git 的本地数据管理分为三个区域**：
   - **工作区**: 文件夹，也叫做工作目录或者本地工作目录。
   - **暂存区**: 用于保存即将提交到 Git 仓库的修改内容。
   - **本地仓库**: 包含了完整的项目历史和元数据，是 git 存储代码和版本信息的主要位置。

### 二. 文件状态

1. 未跟踪
2. 未修改
3. 已修改
4. 已暂存

## 04. 添加和提交文件 📝

### 一. 命令

1. **`git init`**: 创建仓库
2. **`git status`**: 查看仓库的状态
   - `echo 内容 > 文件名.后缀名`，示例：`echo "这是第一个文件" > file1.txt`（使用 `git status` 查看状态的时候，是红色的）。
   - `cat` 用于查看文件里面的内容。`cat 文件名.后缀名` 示例：`cat file1.txt`。
   - `mv` 用于更改文件名。`mv 原文件名.后缀名 新文件名.后缀名`，示例：`mv file.txt file4.txt`。
   - `-s` 表示查看状态这个命令的简略模式：`git status -s`。
     - 结果为 `(1)?? temp/`（第一列是暂存区的状态，第二列是工作区的状态）
     - `(2)M .gitignore` 中的 M 表示 .gitignore 文件被修改过。
3. **`git add`**: 添加到暂存区
   - `git add 文件名.后缀名`，示例：`git add file1.txt`（使用 `git status` 查看状态的时候，是绿色的）。
   - 可以使用通配符 `*` 来添加多个文件到暂存区，示例：`git add *.txt`。
   - 可以使用 `.` 来添加当前文件夹下的所有文件，示例：`git add .` (`.` 表示当前目录)。
4. **`git commit`**: 提交
   - `git commit -m "提交指示信息"`，示例：`git commit -m "第一次提交"`。
   - 不输入 `-m` 参数，会进入到交互式界面 (默认使用 vim 来编辑文本信息（vim 是一种文本编辑器）)。
     - 进入交互模式时，可以使用方向键来移动光标，使用 `i` 键进入编辑模式。
     - 然后按 `Esc` 退出编辑模式，在非编辑模式下直接输入 `:wq`（光标会自动跳转到最底下），保存退出。
   - `git commit -am "提交指示信息"`（`git commit -a -m "提交指示信息"`）：同时完成添加至暂存区和提交至仓库的两个操作。
5. **`git log`**: 可以用于查看提交记录（可以看到 id, 作者和邮箱，日期，编辑的注释信息）。
   - 可以使用 `git log --oneline` 来查看简洁的提交记录，只显示 id 和注释信息。
   - 然后按 `Q` 键退出，回到命令行。

**注意**:

- 直接输入 `ls` 可以看到当前目录下的文件，`ls -a` 可以看到更加完整的目录，`ls -al` 可以看到完整的目录状态。
- `git commit` 提交的是暂存区的文件，而不是工作区的文件。
- 小括号内有命令提示，可以跟着操作。
- 如果有报错 `E325: ATTENTION`，可以删除，可以先输入 `:qa` 退出，然后再删除交换文件（示例：`rm /d/git/learn_git/.git/.COMMIT_EDITMSG.swp`），接着就可以正常进入交互式界面。

## 05. 版本回退 (git reset) ⏪

### 一. git reset

- 用于回退版本，可以退回到之前提交的某一个状态。

### 二. 三种模式

1. **`git reset --soft`**:
   - 保留工作区和暂存区的所有修改内容。
   - `git reset --soft id` (oneline 中的 id)。示例：`git reset --soft 5af80b8`。
2. **`git reset --hard`**:
   - 丢弃工作区和暂存区的所有修改内容。
3. **`git reset --mixed`**:
   - 保留工作区的修改内容，而且丢弃暂存区的修改内容。
   - `git reset HEAD^` 也行（没添加任何参数，默认就是 `--mixed`）。

**注意**:

- `cp -rf 原目录名 新目录名`，可以用于将目录复制到新的目录中，示例：`cp -rf repo repo-soft`。
- 路径只能使用斜杠，不能使用反斜杠。
- `git ls-files` 命令可以用于查看暂存区的内容。
- `HEAD^` 表示上一个版本。

### 三. git 中所有的操作都是可以回溯的

- `git reflog` 来查看操作的 id（需要找到误操作之前的版本号），然后使用 `git reset --hard id` 来回溯（但是 reflog 只能看到对仓库进行的提交等等操作，而转换目录等却看不到）。

## 06. 查看差异 (git diff) 🔍

### 一. git diff 功能

1. 用于查看文件在工作区，暂存区以及版本库之间的差异。
2. 可以查看文件在两个特定版本之间的差异。
3. 可以查看文件在两个分支之间的差异。

### 二. 常用命令

1. **`git diff`**: 如果后面什么都没加，会默认比较的是**工作区和暂存区**之间的差异内容，它会显示发生更改的文件以及更改的详细信息（红色表示删除的内容，绿色表示修改后的内容。如果没有任何内容表示工作区和暂存你去的内容是相同的）。
2. **`git diff HEAD`**: 比较**工作区和版本库**之间的差异。
3. **`git diff --cached`**: 比较**暂存区和版本库**之间的差异。
4. **`git diff id1 id2` (HEAD)**: 比较两个版本之间的差异。
5. **`git diff id1 id2 文件名.后缀名`**: 比较两个版本之间同一个文件的差异。
6. **`git diff branch1 branch2`**: 比较两个分支之间的差异。

**注意**:

- `vi 文件名.后缀名`，可以用于更改文件内容，然后会进入交互式界面。
- `100644` 表示的是文件权限，`index` 后面的数值是通过哈希算法得出来的 40 位哈希值。
- `HEAD` 指向分支的最新提交节点，`HEAD~` (`HEAD^`) 表示最新版本的上一个版本，`HEAD~n` 表示最新版本之前的第 n 个版本。

## 07. 删除文件 (git rm) 🗑️

### 一. git rm

- 从版本库中删除文件。

### 二. 删除方式

1. **直接删除文件** (总共两步): `rm 文件名.后缀名`。
2. **git rm** (一步就行): 把文件从工作区和暂存区同时删除。`git rm 文件名.后缀名`。
3. 最后要注意的是要记得**提交**，不然在版本库中还是存在的。

**注意**:

- `rm` 并不是 git 的命令而是 Linux 操作系统的命令。
- 但是这个只是删除了工作区的内容，没有删除暂存区的内容，所以要再执行 `git add .` (`<file>`) 来提交到暂存区。
- `git rm --cached <file>` 把文件从暂存区删除，但保留在当前工作区中（但都是从版本库中删除了）。
- `git rm -r *` 递归删除某个目录下的所有子目录和文件。

## 08. 忽略文件 (.gitignore) 🙈

### 一. .gitignore

- 这个文件的作用也就是可以让我们忽略掉一些不应该被加入到版本库中的文件。

### 二. 应该忽略哪些文件

1. 系统或者软件自动生成的文件。
2. 编译产生的中间文件和结果文件。
3. 运行时生成日志文件、缓存文件、临时文件。
4. 涉及、密码、口令、密钥等敏感信息文件。

### 三. 操作

- `echo access.log > .gitignore`: 然后暂存区中的 `access.log` 文件就会被替换成 `.gitignore` 文件。
- 可以使用 `*.log` 这样的通配符来匹配所有以 log 结尾的文件（将这个文件放在 `.gitignore` 文件里面就可以将所有的后缀名为 .log 的文件都忽略掉）。
- `>>` 表示追加到这个文件的后面。`echo "1" >> other.log`（直接在版本库中产生效果）。

**注意**:

- `.gitignore` 生效需要有一个前提就是这个文件不能是已经被添加到版本库中文件。
- git 默认是不会将空的文件夹添加到仓库里面的。
- `echo "hello" > temp/hello.txt`。
- 文件夹的格式是以 `/` 结尾的，示例：`temp/`。

### 四. .gitignore 文件的匹配规则

从上到下逐行匹配，每一行表示一个忽略模式。

1. 空行或者以 `#` 开头的行会被 Git 忽略。一般空行用于可读性的分隔，`#` 一般用作注释。
2. 使用标准的 blob 模式匹配，例如：
   - 星号 `*` 通配任意个字符。
   - 问号 `?` 匹配单个字符。
   - 中括号 `[]` 表示匹配列表中的单个字符，比如: `[abc]` 表示 a/b/c。
3. 两个星号 `**` 表示匹配任意的中间目录。
4. 中括号用短中线连接，比如：`[0-9]` 表示任意一位数字，`[a-z]` 表示任意一位小写字母。
5. 感叹号 `!` 表示取反。

**示例**:

代码段

```
# 忽略所有的 .a文件
*.a
# 但跟踪所遇的lib.a，即便你在前面忽略了， .a文件
!lib.a
# 只忽略当前目录下的TODO文件，而不忽略 subdir/TODO
/TODO
# 忽略任何目录名下为build的文件夹
build/
# 忽略doc/notes.txt,但不忽略 doc/sever/arch.txt
doc/*.txt
# 忽略 doc/ 目录及其所有子目录下的 .pdf文件
doc/**/*.pdf
```

## 09. GitHub 与远程仓库同步 ☁️

### 一. 同步本地仓库和远程仓库

- `git pull`
- `git push`

**注意**:

- 要先将文件 commit 之后执行这些命令才有效。
- `git push` 如果后面没有加上任何参数的话，会直接默认关联 GitHub。

## 10. 关联远程仓库 🔗

### 一. 添加与拉取

1. **添加远程仓库**:
   - `git remote add 别名 git@github.com:qingyaozhuozhang/python-basis.git`
   - `git push -u 别名 master` (第一条命令取的别名，就只写本地仓库的 master)
2. **查看远程仓库**:
   - 使用 `git remote -v` 来查看一下。
   - 可以查看当前仓库所对应的远程仓库的别名和地址。
3. **拉取远程仓库的内容**:
   - `git pull 远程仓库名 本地分支名：远程分支名`

**注意**: 需要提前有 add 和 commit 的步骤不然会报错。

### 二. README 文件

- README 文件是一个 Markdown 格式的文件（能够显示在仓库首页）。

### 三. git fetch

- fetch 命令只是获取远程仓库的修改。

## 11. Gitee 与 GitLab 🦊

- **Gitee (码云)**: SSH 密钥配置跟 GitHub 一样，优点在于是国内的，比较快。
- **GitLab (极狐)**: SSH 密钥配置也一样，优点在于是私有化部署（但是需要钱）。

## 12. GUI 工具 🖥️

- 看基本操作文档。

## 13. 在 VSCode 中使用 Git 🆚

### 一. code .

- 表示以 VSCode 的方式打开当前目录（要有这一步源码管理器才能生效）。

### 二. 源码管理器

- 有 git 支持。

1. 将有 git 管理的文件更改保存后，图标上会显示数字。
2. 点击文件（不是点击文件的图标）就会在界面中显示出修改前和修改后的两个界面（相当于 git diff）。
3. **四个图标**: (1) 打开文件 (2) 放弃更改 (3) 添加到暂存区（点击之后，文件会从更改变成暂存的更改）(4) 状态。
4. 提交的方框里面就是填入提交的注释信息，点击提交按钮就可以 commit。同步更改就是是否需要把这个提交推送到远程仓库中。

## 14. 分支管理 (git branch) 🌿

### 一. 分支的使用场景

- (无内容)

### 二. git branch

- **查看**: `git branch` 查看当前仓库的所有分支 (master 默认分支)。
- **创建**: `git branch 分支名` 用于创建新的分支。
- **删除**: `git branch -d 分支名` 用于删除已经被用于合并的分支。
- **强制删除**: `git branch -D 分支名` 用于强制删除分支。

### 三. git checkout / switch

1. `git checkout 分支名称`: 用于切换到不同的分支（默认此功能）。
2. 可以用于恢复到修改之前的状态: `git checkout -b 分支名称 id`。
3. `git switch 分支名称`: 专门用于切换分支的。

### 四. git merge

- `git merge 要合入的分支名称`: 将不同的分支合并到当前分支中。
- 会进入交互式界面，直接保存就行，然后就能成功合并（但是合并并不会使分支消失）。

### 五. 查看分支图

- `git log --graph --oneline --decorate --all`: 可以查看分支图。

## 15. 解决合并冲突 💥

### 一. 什么时候发生冲突？

- 当两个分支修改同一个文件的内容的时候，需要自己动手进行修改（会将两部分的内容都进行保留）。

### 二. 怎样修改？

- 通过 `vi 文件名.后缀名`。
- 进入文件，将多余的格式符删掉就行，然后暂存提交。

## 16. 回退和 Rebase 🔄

### 一. rebase

- `git rebase 文件名.后缀名`: 会在两条分支的节点将另一条分支的合并到该分支的最新提交记录之后。

### 二. alias

- 将某个命令定义为一个别名: `alias 命令名称="命令"`。
- **注意**: `cd ..` 表示返回上一级目录。

### 三. rebase 和 merge 的区别

1. **merge (团队)**:
   - **优点**: 不会破坏原分支的提交历史，方便回溯和查看。
   - **缺点**: 会产生额外的提交节点，分支图比较复杂。
2. **rebase (自己)**:
   - **优点**: 不会新增额外的提交记录，形成线性历史，比较直观和干净。
   - **缺点**: 会改变提交历史，改变了当前分支 branch out 的节点。避免在共享分支使用。

## 17. 分支管理和工作流模型 🌊

### 一. git 分支工作流模型

1. **gitFlow 模型**:
   - **(1) main 主线分支**:
     - 版本号规则:
       - 主版本: 主要的功能变化或重大更新。
       - 次版本: 一些新的功能、改进和更新，通常不会影响现有功能。
       - 修订版本: 一些小的 bug 修复，安全漏洞补丁等。通常不会更改现有功能和接口。
   - **(2) hotfix 问题修复分支**
   - **(3) develop 开发分支**
   - **(4) feature 功能分支**
   - **(5) release 预发布分支**
   - **按生命周期分为**:
     - 主要分支: main 和 develop
     - 辅助分支: hotfix、feature 和 release（可以删除）
2. **GitHub Flow 模型**:
   - **(1) master 主分支**: 长期存在，一般会设置分支保护。
   - **(2) feature 分支**: PR 合并请求 -> Review 评审。

### 二. 分支命名

- 推荐使用带有意义的描述性名称来命名分支。
  - 版本发布分支/Tag 示例：`v1.0.0`
  - 功能修复分支示例：`feature-login-page`
  - 修复分支示例：`hotfix-@issueid-desc`

### 三. 分支管理

- 定期合并已经成功验证的分支，及时删除已经合并的分支。
- 保持合适的分支数量。
- 为分支设置合适的管理权限。

---

## 18. 实战式理解 Git：从工作区到远程协作（扩展）

### 18.1 先建立正确的心智模型

Git 不只是“上传文件到 GitHub”。它保存提交快照及提交之间的父子关系。可用以下三层理解：

- **工作区**：编辑器里正在修改的文件。
- **暂存区（index）**：下一次提交准备包含的具体内容。
- **本地仓库（`.git`）**：已提交的历史对象与分支引用。

`git status` 是最先应检查的命令。`git add` 不是上传 GitHub，而是将当前内容加入暂存区；`git commit` 创建本地提交；`git push` 才是将本地提交推到远程。

### 18.2 完整可复现实验：初始化 → 修改 → 暂存 → 提交

```bash
mkdir -p ~/git-lab && cd ~/git-lab
git init -b main
# 仅本实验仓库配置身份；不覆盖全局身份
git config user.name "Example Developer"
git config user.email "example@example.com"
printf '# Git lab\n' > README.md
git status --short
git add README.md
git diff --cached
git commit -m 'docs: initialize git lab'
printf '\nAdd one more sentence.\n' >> README.md
git diff
git add README.md
git commit -m 'docs: extend README'
git log --oneline --graph --decorate -5
```

这里 `git diff` 是工作区与暂存区之间的差异，`git diff --cached` 是暂存区与当前 HEAD 的差异，`git log` 用来阅读提交历史。

### 18.3 reset、restore、revert 三种操作不要混淆

| 目标 | 推荐命令 | 是否改历史 |
|:--|:--|:--|
| 撤销尚未暂存的单文件修改 | `git restore file` | 否 |
| 取消文件暂存 | `git restore --staged file` | 否 |
| 撤销一次已经共享的提交 | `git revert <hash>` | 追加新提交 |
| 改写本地最近提交 | `git reset --soft HEAD~1` | 是 |
| 丢弃最近提交与工作区变更 | `git reset --hard HEAD~1` | 是，危险 |

`reset --hard` 可能永久丢失未保存的工作区更改。执行之前，先 `git status`、`git log`，必要时用临时分支或 `git stash push -u` 保留现场。

**实验**：创建一个单独的练习提交后执行 `git revert HEAD`，再查看 `git log --oneline`。你会看到“原提交”和“抵消原提交的新提交”都保留在历史中。

### 18.4 分支、合并与冲突：完整的安全练习

```bash
# 在之前的练习仓库里操作
git switch -c feature/topic-note
printf 'feature branch\n' > note.txt
git add note.txt
git commit -m 'feat: add note'
git switch main
git merge --no-ff feature/topic-note
git log --oneline --graph --all
```

`git merge` 将两条分支历史接到一起；若两条分支修改了同一文本区域，Git 可能生成：

```text
<<<<<<< HEAD
main branch text
=======
feature branch text
>>>>>>> feature/topic-note
```

冲突标记只出现在尚未解决的文件中。人工选择或合并内容，移除标记，然后：

```bash
git status
git add note.txt
git commit    # 普通 merge 冲突解决完成后创建合并提交
```

若正在 rebase，解决冲突后应该 `git rebase --continue`，不是盲目执行普通 merge 提交。

### 18.5 fetch、pull、push、rebase 的关系

- `git fetch origin` 只更新远程跟踪引用，不自动改你的工作区。
- `git pull` 大致等于 fetch 后再按配置 merge 或 rebase；因此应了解自己用的是哪一种策略。
- `git push -u origin main` 上传并建立上游分支关联。
- `git rebase main` 将当前分支的本地提交重新应用到新基底，提交 SHA 通常会变化。**已经被别人基于其继续工作的共享分支，不要随意 rebase 后强推。**

```bash
git remote -v
git fetch origin
git branch -vv
git log --oneline --decorate --graph --all -15
git status
```

如果远端发生了你不知道的修改，不要直接 `git push --force`。确需覆盖个人分支时，理解风险后优先使用 `--force-with-lease`，它也不是不会覆盖他人工作。

### 18.6 reflog、stash 与误操作恢复

```bash
git reflog --date=iso | head -n 15
# 找到要找回的提交 SHA 后，先新建救援分支
git branch rescue/old-state <commit-hash>
git stash push -u -m 'work in progress'
git stash list
git stash pop
```

`reflog` 是本地引用变动记录，不是远程备份；其记录会过期。`stash pop` 可能发生冲突，因此恢复前也应检查当前工作区是否干净。

### 18.7 .gitignore 与秘密文件管理

```gitignore
# Python
.venv/
__pycache__/
*.pyc
# 构建产物
build/
dist/
site/
# 本地秘密和环境
.env
*.key
# 但允许上传一个不含凭据的格式说明（如果项目确实需要）
!.env.example
```

`.gitignore` 只影响**未被 Git 跟踪**的文件。已经提交过的文件不能靠添加 `.gitignore` 自动从历史中消失；至少需要 `git rm --cached path` 并重新提交。如果密钥已经推送到公共仓库，必须先吊销/轮换，再考虑历史清理。不要把密钥直接写在 Markdown 示例里。

### 18.8 GitHub Markdown：为什么代码块和公式会显示错？

1. **A\* 的星号**：在普通文本里写作 `A\*`；在 fenced C++/Python 代码里不需要这种转义。
2. **多行代码**：围栏 ` ```cpp ` 和结束的 ` ``` ` 各占一行，最好与前后内容空一行，避免复杂列表嵌套。
3. **数学公式**：Typora `Ctrl + Shift + M` 插入的显示数学块通常使用 `$$`，GitHub 也支持这种写法。
4. **表格风格**：GitHub 对仓库原始 `.md` 的默认表格主题由 GitHub 控制，本地 MkDocs `extra.css` 只会影响静态网站，不能改变 GitHub 的仓库预览页面。

下面这一段可以直接放进 `.md` 文件测试：

````markdown
### A\* 搜索

$$
f(n)=g(n)+h(n)
$$

```cpp
int add(int a, int b) {
    return a + b;
}
```

| 术语 | 含义 |
|:--|:--|
| $g(n)$ | 已知代价 |
| $h(n)$ | 估计代价 |
````

### 18.9 GitHub Pages + MkDocs：构建错误怎么排查？

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install mkdocs-material mkdocs-git-revision-date-localized-plugin mkdocs-minify-plugin
mkdocs build --strict
mkdocs serve
```

上面只是常见依赖的示例。如果仓库已有 `requirements.txt`，应优先安装锁定的依赖，而不是使用不受约束的最新版本。站点构建时应检查 `mkdocs.yml` 导航路径、图片相对路径、Markdown 数学扩展、主题静态资源。

### 18.10 Git 学习路线的验收标准

应能独立回答：`git add` 和 `git commit` 各改了什么？怎样撤销“未提交的更改”和“已经推送的错误提交”？如何从 `reflog` 找回本地提交？如何在不覆盖队友工作的情况下处理冲突？如何用 `git diff` 证明自己实际改了哪些文件？能用一套小型仓库演示这些问题，就不再只是记命令。
