### Git提交远程仓库流程

- **正常流程**

```
git init
git add .
git commit -m "说明"
git remote add origin git@github.com:qingyaozhuozhang/qingyaozhuozhang.github.io.git
git push origin main
```

- **非正常问题说明**

1.如果之前已经有在这个仓库创建过链接，需要换到另一个仓库需要使用`set-url`

`git remote set-url origin git@github.com:qingyaozhuozhang/qingyaozhuozhang.github.io.git`

2.如果推上去的内容与原先内容有冲突

（1）直接使用`-f`参数

`git push -f origin main`

（2）第一次推送得使用`-u`参数

`git push -u origin main`

3.需要使用`git pull`的情况

（1）正常

`git pull --rebase origin main`（`--rebase`保持提交历史是一条干净的支线，而不是充满复杂的"Merge branch.."的记录）

（2）遇到`refusing to merge unrelated histories`问题

`git pull origin main --allow-unrelated-histories`（合并两个完全独立、没有共同祖先的仓库）

4.在整个文件夹的根目录下，想要上传单个文件夹

- `git add docs/`
- 如果没有之前仓库里面的东西，只有单一一个文件夹，就必须`git clone`下来，然后上传

5.克隆仓库

`git clone git@github.com:qingyaozhuozhang/qingyaozhuozhang.github.io.git`

6.github强制覆盖本地

`git fetch --all`

`git reset --hard origin/main`

7.修改仓库错误，需要回退

- 打开github仓库主页右侧的`Commits`找到之前的`Commit Hash`
  - 或者在终端中输入`git reflog`
- `git reset --hard <你的Hash>`
- 回退到之前的内容，但是本地新增加的那个文件夹不会删除

