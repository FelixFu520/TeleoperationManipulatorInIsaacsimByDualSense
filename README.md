# TeleoperationManipulatorInIsaacsimByDualSense
使用DualSense手柄遥操isaacsim的机械臂

## 硬件连接
我的连接链路: 手柄 --> Windows11电脑 --> 无影客户端 --> IsaacSim(无影云电脑中)

### 手柄连接Windows11电脑
手柄型号是PS5的DualSence, 手柄连接电脑有两种方式，一种是USB，一种是蓝牙

#### 蓝牙连接
[连接教程](https://www.playstation.com/zh-hans-cn/support/hardware/pair-dualsense-controller-bluetooth/)

<img src="docs/images/dualsence.png" width="500" alt="描述文本">

<img src="docs/images/dualsence02.png" width="500" alt="描述文本">

<img src="docs/images/dualsence03.png" width="500" alt="描述文本">


为了验证windows11是否成功识别到DualSence手柄, 需要找个可视化界面观察下各个按键是否正常，
我找到的是[RemotePlayInstaller.exe](https://remoteplay.dl.playstation.net/remoteplay/lang/cs/index.html), 但是公司电脑不让装， 所以改用[DS4Windows](https://ds4-windows.com/about/), 安装后测试结果如下图

![](docs/images/dualsencedemo.gif)


但是无影似乎不支持蓝牙连接的映射, 所以后面使用USB连接

#### USB连接
直接用USB线连接即可，连接后可以在设备管理器中看到设备信息

![](docs/images/dualsence04.png)

### DualSense手柄连接到无影机器
上面已经通过USB将DualSence手柄连接到Windows电脑上， 现在要把这个手柄重定向到无影云电脑中，[官方教程](https://help.aliyun.com/zh/wtc/user-guide/use-game-controllers)

<img src="docs/images/wuying.png" width="500" alt="描述文本">

#### 连接
首先， 需要为这台无影云电脑配置策略， 可以找管理员或无影的工作人员。

然后，就可以在外设中看到DualSence手柄了，

![](docs/images/dualsence06.gif)

#### 验证
##### lsusb
```

lsusb | grep Sony
# 或者查看输入设备列表
cat /proc/bus/input/devices | grep -i "Sony"
```
##### jstest-gtk
```
sudo apt install jstest-gtk

jstest-gtk
```
![](docs/images/dualsence07.gif)
##### jstest
```
sudo apt install joystick

jstest /dev/input/js1
```
![](docs/images/dualsence08.gif)

##### evtest
```
sudo apt install evtest

evtest
```
![](docs/images/dualsence09.gif)

 
