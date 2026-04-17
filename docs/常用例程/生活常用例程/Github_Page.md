### 创建一个新的Github Page完整流程

**使用MkDocs + Material Theme静态文档方案**

（1）正常流程

1.初始化项目结构

`mkdocs new .`

实现结果：

```
MyKnowledge/
  ├── mkdocs.yml    # 核心配置文件
  └── docs/         # 存放你所有 markdown 文档的地方
      └── index.md  # 首页
```

2.编辑mkdocs.yaml文件（美化外观）

https://github.com/qingyaozhuozhang/jingshen/tree/main/Github_Page

（需要下载requirements.txt里面的依赖）

3.本地预览

`mkdocs serve`

然后浏览器打开http://127.0.0.1:8000

4.在Github上新建一个链接仓库（`.github.io`）

在Github上新建一个名称跟Github名字一样+`.github.io`的仓库

![](./picture/repository.png)

5.上传仓库后部署配置

在根目录中创建目录和文件：`.github/workflows/deploy.yml`

```
你的项目文件夹/
├── mkdocs.yml
├── requirements.txt      <-- 刚才建的
└── .github/
    └── workflows/
        └── deploy.yml    <-- 现在要建的
```

https://github.com/qingyaozhuozhang/jingshen/tree/main/Github_Page

6.切换Pages分支

![](./picture/deploy.png)

- Source：保持选`Deploy from a branch`
- Branch：选择`gh-pages`分支
  - 文件夹保持`/(root)`

7.验证

直接访问https://qingyaozhuozhang.github.io/



**可能遇到的问题分析处理**

- 编辑mkdocs.yaml
  - 部署地址需要对应
  - `site_url: https://qingyaozhuozhang.github.io/`
  - 编辑内容在`nav`模块中
  
    - （冒号后面不能放空白）
    - ```
      编程常用例程:    （不行）
      编程常用例程:    （行）
        待更新：常用例程/编程常用例程/index.md
      ```
  - 路径从`docs/`之后开始`docs/`不需要
- 上传遇到语法错误而无法显示
  - 到`Actions`选项中查看具体报错
