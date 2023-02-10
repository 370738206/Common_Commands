# Git

## Git基础

### 框架

* 工作区
* 暂存区（add 命令）
* 版本库（commit 命令）

### 配置

```shell
git config --global user.name "liyemin"
git config --global user.email "370738206@qq.com"
git config --global --list
```

### 初始化本地仓库

```shell
git init
```

### 修改并提交

```shell
git add <file> 
git add <floder>
git status
git commit -m 'change comment' 
```

### 回退文件状态 **restore**

* 撤回工作区修改（还没有使用add命令）

```shell
git restore <file>
```

* 撤回暂存区修改（已经使用add命令）

```shell
git restore --staged <file>
```

### 对比不同

```shell
git diff <file>
```

### 查看历史版本

```shell
git log <可选 --pretty=oneline>
git reflog
```

### 回退历史版本 **reset**

```shell
git reset --hard <commit_id>
```

### 删除文件

* 只从工作区删除

```shell
rm <file>
```

* 同时从工作区和版本库删除

```shell
git rm <file>
```

* 删除文件夹及其包含的全部文件

```shell
git rm -r <file>
```

## Git远程仓库

### 配置远程仓库

创建公钥

```shell
ssh-keygen -t rsa -C "370738206@qq.com"
```

### 本地仓库链接远程仓库

```shell
git remote add origin <repositorie_link>
```

### 本地仓库内容推送到远程

使用<-u>参数，意味着本地<master>分支的<推送 push><拉取 pull>命令与远程仓库绑定

```shell
git push <可选 -u> origin master 
```

### 解除与远程仓库的联系

```shell
git remote -v # 查看信息
git remote rm origin
```

### 从远程仓库拉取

* 直接与本地仓库合并

```shell
git pull origin master
```

* 查看不同后手动选择合并

```shell
git fetch origin master
git log -p master..origin/master
git merge origin/master
```

### 从远程仓库克隆到本地

```shell
git clone <repositorie_link>
```

## Git分支

### 创建分支

```shell
git bransh <branch_name>
```

### 切换分支

* 若有对应分支

```shell
git switch  <branch_name>
```

* 若没有对应分支

```shell
git switch -c  <branch_name>
```

### 列出分支

* 列出本地仓库所有分支

```shell
git branch
```

* 列出远程仓库所有分支

```shell
git branch -r
```

* 列出所有本地和远程分支

```shell
git branch -a
```

### 删除分支

```shell
git branch -d <branch_name>
```

### 合并分支

在当前分支下，合并另一分支内容

```shell
git merge <branch_name>
```

## 参考链接

[安装教程](https://blog.csdn.net/mukes/article/details/115693833?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167600213516800182125777%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=167600213516800182125777&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-115693833-null-null.142^v73^control,201^v4^add_ask,239^v1^insert_chatgpt&utm_term=git%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187)

[使用教程](https://so.csdn.net/so/search?q=git&t=blog&u=qq_52596258)

[推送报错解决](https://blog.csdn.net/liulei952413829/article/details/117553977)

[远程合并分支](https://www.cnblogs.com/xiaomaomao/p/13886095.html#:~:text=Github%E8%BF%9C%E7%A8%8B%E5%88%9B%E5%BB%BA%E5%88%86%E6%94%AF%E3%80%81%E5%90%88%E5%B9%B6%E5%88%86%E6%94%AF%E5%88%B0%E4%B8%BB%E5%88%86%E6%94%AF%201%E3%80%81%E6%89%BE%E5%88%B0%20github%20%E4%B8%8A%E7%9A%84%E4%BB%93%E5%BA%93----%3E%E7%82%B9%E5%87%BB%20main%20%E5%88%86%E6%94%AF----%3E%E5%BC%B9%E5%87%BA%E4%B8%80%E4%B8%AA%E8%BE%93%E5%85%A5%E6%A1%86----%3E%E8%BE%93%E5%85%A5%E6%A1%86%E5%86%85%E5%A1%AB%E5%86%99%E8%BF%9C%E7%A8%8B%E5%88%86%E6%94%AF%E5%90%8D%E7%A7%B0----%3E%E7%82%B9%E5%87%BB%20Create,branch%3Aremote_feature01%20from%20main%20----%3E%E8%BF%99%E6%A0%B7%E5%B0%B1%E4%BB%8E%20main%20%E5%88%86%E6%94%AF%E4%B8%8A%E5%88%87%E6%8D%A2%E5%87%BA%E4%BA%86%E4%B8%80%E4%B8%AA%20remote_feature01%20%E5%88%86%E6%94%AF)
