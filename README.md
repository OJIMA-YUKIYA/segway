# ROSで SegwayRMP200 を動かす。
動作環境はUbuntu18.04のOSが載っているPC
## ROSのインストール
以下のROSの公式サイトに[ROSのインストール方法](http://wiki.ros.org/melodic/Installation/Ubuntu)が載っている。

http://wiki.ros.org/melodic/Installation/Ubuntu

公式サイトによると次の10行のコマンドでインストールできる。

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash
sudo apt install curl
```
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```bash
sudo apt update
```
```bash
sudo apt install ros-melodic-desktop
```
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
```bash
sudo apt install python-rosdep
```
```bash
sudo rosdep init
```
```bash
rosdep update
```
## コンパイル方法
初めてコンパイルす場合、segway-main下のlibsegwayrmpをmakeコマンドで必ずコンパイルする。2回目以降は、libsegwayrmpのソースコードを変更しない限り、libsegwayrmpをコンパイルする必要はないので、2回目以降はスルーする。
```bash
~/segway-main/libsegwayrmp $ make
```
segway-main下で catkin_make によりコンパイルを行う。
```bash
~/segway-main $ catkin_make
```
