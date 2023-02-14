# Pytorch

## Anaconda 命令

> 查看 Anaconda 版本

conda --version

> 查看当前存在哪些虚拟环境

conda env list
conda info -e

> 创建虚拟环境

conda create -n your_env_name python=x.x

> 复制虚拟环境

conda create -n <new_name> --clone <existing_env>

> 删除虚拟环境

conda remove -n your_env_name --all

> 激活或者切换虚拟环境

Linux:  source activate your_env_nam
Windows: activate your_env_name

> 关闭虚拟环境

deactivate env_name

> 导出环境/导入环境

* conda 包
  导出
  conda env export > <name.yaml>
  安装
  conda env create -f <name.yaml>

* pip包
  导出
  pip freeze > <name.txt>
  安装
  pip install -r <name.txt>

> 查看环境已安装包

conda list

## Pycharm 使用 Anaconda 环境

File -> New Project
Location -> Python Interpreter -> Previously configured interpreter
Add Interpreter -> Conda Environment ->Interpreter -> OK
Location -> Python Interpreter -> Previously configured interpreter -> Interpreter

## Anaconda 包安装

> 安装 Pytorch

查看 NVIDIA_CUDA 版本： nvidia-smi
前往 Pytorch官网： <https://pytorch.org/>
获取安装命令

> 安装 TensorBoard (其他类似)
pip install tensorboard

> 添加国内镜像网址
pip install tensorboard -i <https://pypi.tuna.tsinghua.edu.cn/simple>

> 国内镜像源集合

阿里云 <http://mirrors.aliyun.com/pypi/simple/>
中国科技大学 <https://pypi.mirrors.ustc.edu.cn/simple/>
豆瓣 <http://pypi.douban.com/simple/>
清华大学 <https://pypi.tuna.tsinghua.edu.cn/simple/>
中国科学技术大学 <http://pypi.mirrors.ustc.edu.cn/simple/>

## TensorBoard 命令

### 可视化类型

> 图片  

writer.add_image()  
writer.add_images()

> 统计图  

writer.add_scalar()  
writer.add_scalars()

> 网络结构  

writer.add_graph()

### 可视化框架

```python

from torch.utils.tensorboard import SummaryWriter

writer = SummaryWriter("../logs")

writer.add_images("input", imgs, step)

tensorboard --logdir=logs

```

## 数据集操作

### 加载数据集

```python

import torchvision
from torch.utils.data import DataLoader

test_transformer = torchvision.transforms.Compose([
    torchvision.transforms.ToTensor()
])

test_data = torchvision.datasets.CIFAR10(root="../datasets", train=False, transform=test_transformer, download=True)
test_loader = DataLoader(dataset=test_data, batch_size=64, shuffle=True, num_workers=0, drop_last=False)

```
