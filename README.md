# 欢迎来到镜神的知识网络 👋

这里是我存放第二大脑的地方。

!!! tip "网站说明"
    本站点基于 MkDocs 构建，部署于 GitHub Pages。

# 如何编辑

注意：每次编辑前请先执行`git pull`保证当前仓库为最新状态以防止产生冲突。（`git pull origin main`）

1. 使用`git clone git@github.com:qingyaozhuozhang/qingyaozhuozhang.github.io.git`将仓库克隆到本地。
2. 确保安装了`Python`，输入`pip install -r requirements.txt`安装Python依赖（如不需要本地使用`mkdosc`，此步骤忽略）。
3. 在`.\docs\`路径下的打开/创建文档进行编辑，如有创建文件请在`mkdosc.yaml`文件的`nav`选项下添加文件路径。
4. 编辑完成后提交并**push到main分支**。(`git push origin main`)

```
完整示例:
git init
git add .
git commit -m ""
git remote add origin git@github.com:qingyaozhuozhang/qingyaozhuozhang.github.io.git
git branch -M main
git push origin main
```

