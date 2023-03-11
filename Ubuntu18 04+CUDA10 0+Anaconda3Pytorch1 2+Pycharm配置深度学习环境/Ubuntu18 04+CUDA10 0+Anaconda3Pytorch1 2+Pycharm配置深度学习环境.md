# Ubuntu18.04+CUDA10.0+Anaconda3Pytorch1.2+Pycharm配置深度学习环境

## NVIDIA显卡驱动安装

- 终端输入

```bash
ubuntu-drivers devices
```

![](https://pic4.zhimg.com/80/v2-ecf667909236076dce06472da87407c7.png)

最后有recommended的就是推荐的安装，这里需要注意的一点是，显卡驱动版本必须和后续要安装的cuda版本匹配

![](https://pic4.zhimg.com/80/v2-490af999379dfe15c11efc070ebcaef2.png)

- 如果安装推荐版本直接在终端执行

```bash
sudo ubuntu-drivers autoinstall
```

- 安装完成后，在终端执行`sudo reboot`或者手动重启
- 重启后，在终端执行下面命令查看驱动是否安装成功

```bash
nvidia-smi
```

![](https://pic4.zhimg.com/80/v2-473d3a0822fc600f9ac6812b5977927e.png)

右上角显示的是能支持的cuda的最高版本，不是已经安装的cuda版本

## CUDA10.0安装

cuda10.0已经是比较老的版本了，由于本人还要使用autoware.ai，autoware.ai对cuda的版本要求是10.0，比较逆天，因此还是选择安装了10.0版本

- 安装完显卡驱动是安装cuda的前提，cuda官方链接：****[CUDA Toolkit 10.0 Download](https://developer.nvidia.com/cuda-10.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=runfilelocal)****
- 依次选择操作系统，系统架构，发行版本，安装方式等等，最后点击Download下载安装文件

![Image](https://pic4.zhimg.com/80/v2-905280728aa9b14c2156ff7f083ba4cc.png)

- cd到下载安装文件的位置，执行

```bash
sudo sh ./cuda_10.0.130_410.48_linux.run
```

- 然后会出来一个专利书来阅读，长按CTRL+F（CTRL +Ｂ是往前翻一页）即可，最后会出现一堆问题，注意有些地方是打字，不是按回车hhh

```bash
1): Do you accept the previously read EULA?
accept
2): Install NVIDIA Accelerated Graphics Driver for Linux-x86_64 **?
n(已经安装了显卡驱动)
3): Install the CUDA 10.0 Toolkit?
y
4): Enter Toolkit Location
（直接回车，默认就行）
5): Do you want to install a symbolic link at /usr/local/cuda?
y
6): Install the CUDA 10.0 Samples?
y
7): Enter CUDA Samples Location
（直接回车，默认就行）
```

- 设置环境变量，在终端执行

```bash
gedit ~/.bashrc
```

- 在文件末尾追加

```bash
export PATH=/usr/local/cuda-10.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH
```

- 测试一下是否安装成功，在终端执行

```bash
cd /usr/local/cuda-10.0/samples/1_Utilities/deviceQuery
sudo make -j4
./deviceQuery
```

![](https://pic4.zhimg.com/80/v2-91e4a017d8262c8e6996438218533867.png)

这里显示的是实际安装的cuda的版本

## 安装CUDNN

- 进入[官网下载](https://developer.nvidia.com/cudnn)cudnn，需要注册登录才能下载，找到与cuda10.0对应的版本，这两个应该都可以
![](https://pic4.zhimg.com/80/v2-4358535f34c8e775ee6f237c6f537f3b.png)
![](https://pic4.zhimg.com/80/v2-9e8c07eb26fee952dfca7e6992fc747b.png)
- cd 到下载安装包的位置，解压安装包，解压完成后进入安装包文件夹，在终端执行

```bash
sudo cp cuda/include/cudnn.h /usr/local/cuda/include

sudo cp cuda/lib64/libcudnn* /usr/local/cuda/lib64

sudo chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*
```

- 测试一下是否安装成功，在终端执行

```bash
cat /usr/local/cuda/include/cudnn.h | grep CUDNN_MAJOR -A 2
```

![](https://pic4.zhimg.com/80/v2-e36ccd2ca426b40083d45685321ee678.png)

## 安装****Anaconda3****

- 到清华镜像源下载安装文件，[https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/](https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/)
- cd到下载安装文件的位置，在终端执行，一直全选yes即可，也可以更改安装路径（没试过）

```bash
bash Anaconda3-2021.05-Linux-x86_64.sh
```

- 打开condarc切换conda清华源，在终端执行

```bash
sudo gedit ~/.condarc
```

- 把里面的内容替换成下面内容，也可能根本没有这个文件，直接新建就行

```bash
channels:
  - defaults
show_channel_urls: true
default_channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/msys2
custom_channels:
  conda-forge: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  msys2: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  bioconda: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  menpo: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  pytorch: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  simpleitk: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
report_errors: false
auto_activate_base: false
```

## 安装Pytorch

重头戏来了

- 在conda中创建一个虚拟环境，用来安装pytorch，在终端执行

```bash
conda create -n pytorch -python=3.6
```

pytorch是我起的名字，可以改成自己起的名字

-python=3.6 同样的3,6是我自己的版本号，改成自己的即可，这个参数可以不加，但是在后面进入python3时要写python3（血与泪的教训，在创建环境的时候没指定python3,进入python时又直接输入了python，结果进了python2，torch库导不进来，弄了好半天）

- 激活环境，在终端执行

```bash
conda activate pytorch #(改成自己起的名字)
```

- 在虚拟环境里输入命令安装pytorch，对于cuda10.0，下面这个版本亲测可行，1.2.0版本应该暂时也够用，执行

```bash
conda install pytorch==1.2.0 torchvision==0.4.0 cudatoolkit=10.0 -c pytorch
```

可能会因为网速的原因会中断，多执行几次，不用担心，已经安装好的不会重复安装

我这里已经安装过了，正常会有很多进度条，安装的过程一般比较快

![](https://pic4.zhimg.com/80/v2-db5e5ec7d65e777d23a6070ad2d6db27.png)

- 测试一下，输入python3进入py3，如果上面在创建环境时已经指定了python=3.x，现在直接输入python也行

```python
import torch
import torchvision

torch.__version__
torchvision.__version__
print(torch.cuda.is_available())
```

![](https://pic4.zhimg.com/80/v2-3a0d194397d70cdff0451913da56f831.png)

到这里已经成功百分之九十五了！

## 安装Pycharm

- 进入**[官方下载地址](https://www.jetbrains.com/pycharm/download/other.html)**，选择自己想要的版本下载安装包
- 解压安装包，cd到解压的安装包的bin目录下，执行命令进行安装

```bash
./pycharm.sh
```

- 创建快捷方式，在/usr/share/applications创建一个文件

```bash
cd /usr/share/applications
sudo gedit pycharm.desktop
```

- 向文件中写入以下内容

```python
[Desktop Entry]
Version=1.0
Type=Application
Name=Pycharm
Icon= /xxxxx/pycharm-community-2022.3.3/bin/pycharm.png #用自己的路径
Exec=sh /xxxxx/pycharm-community-2022.3.3/bin/pycharm.sh #用自己的路径
MimeType=application/x-py;
Name[en_US]=pycharm
```

- 可以把软件复制到其他地方，避免在Downloads里被清除
- 打开pycharm，新建项目，使用之前创建的环境，也可以先随便创建后在设置里改

![](https://pic4.zhimg.com/80/v2-87a205a4ab63b953066e2c22dba0987f.png)
![](https://pic4.zhimg.com/80/v2-d9219c117c4897b9cd5ccd94da7ac393.png)
![](https://pic4.zhimg.com/80/v2-0dc9b8d3b337bc2bbc5156bb2a6f8d1b.png)
OK，大功告成！

> 参考:
> 

> [https://blog.csdn.net/qq_39537898/article/details/120928365#t4](https://blog.csdn.net/qq_39537898/article/details/120928365#t4)
> 

> [https://zhuanlan.zhihu.com/p/487691706](https://zhuanlan.zhihu.com/p/487691706)
> 

> [https://blog.csdn.net/TU_Dresden/article/details/121049141](https://blog.csdn.net/TU_Dresden/article/details/121049141)
> 

> [https://blog.csdn.net/beautifulback/article/details/122395333](https://blog.csdn.net/beautifulback/article/details/122395333)
> 

> 感谢前人走过的路
>