# ROSで SegwayRMP200 を動かす。
動作環境はUbuntu18.04のOSが載っているPC
## ROSのインストール
以下のROSの公式サイトに[ROSのインストール方法](http://wiki.ros.org/melodic/Installation/Ubuntu)が載っている。

http://wiki.ros.org/melodic/Installation/Ubuntu

公式サイトによると次の10行のコマンドでインストールできる。

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash
$ sudo apt install curl
```
```bash
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```bash
$ sudo apt update
```
```bash
$ sudo apt install ros-melodic-desktop
```
```bash
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
```bash
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
```bash
$ sudo apt install python-rosdep
```
```bash
$ sudo rosdep init
```
```bash
$ rosdep update
```
## コンパイル方法
```bash
~/segway-main $ cd libsegwayrmp
```
```bash
~/segway-main/libsegwayrmp $ make
```
```bash
~/segway-main/libsegwayrmp $ cd ..
```
```sh
~/segway-main $ catkin_make
```
```c
int main(void) {
return 0
}
```
