# task switch
bebop2で充電＋フィールド内stay、持続的被覆制御、物体検知＋監視の動作を
切り替えつつ行う実験およびシミュレーション用ROSパッケージ。
# Description
CBFの制約として書かれた各タスクをフラグに応じて切り替え実行する実験。
- 必ず守りたい制約
	- 充電
	- フィールド内stay

	この2つはつねに守らなければならない。
- やりたいタスク
	- 持続的被覆
	- 物体検出

	上の制約が守られている条件下で、この2つの制約を物体検知の結果にしたがって切り替えていく。

# Dependency
- rotors_simulator  
bebop2 の urdf モデルは rotors_simulator のモデルを使っている。以下、git リポジトリからのクローン方法を示す。


	```
	$ sudo apt install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox  
	$ git clone https://github.com/ayushgaud/rotors_simulator.git  
	$ git clone https://github.com/ethz-asl/mav_comm.git  
	```


- mqtt_bridge  
https://qiita.com/ledmonster/items/1b46fabf0415d7c0abae  
を参照に入れる。ただし
ただし、ビルド、もしくは launch しようとするといろいろ Import Error で怒られる（inject 等）ため、pipで色々入れる必要がある。  


  ```
  sudo pip install bson==0.5.2 inject==3.3.1 paho-mqtt==1.2 msgpack-python==0.4.8
  ```
  ↑だとエラーを吐く。bson だけ
  `pip uninstall bson`
  でアンインストールしておくととりあえずは動く？（要検証）

  <!-- つまったら -->
  <!-- ` -->
  <!-- sudo pip install --upgrade pip setuptools -->
  <!-- ` -->
  <!-- で解決するかも。 -->


- vrpn_client  
ROS wiki に vrpn_client についてのページがあるためそちらを参照。
基本的には
	```
	$ git clone https://github.com/ros-drivers/vrpn_client_ros.git
	```
すればオーケーなはず。怒られたら
`rosdep install vrpn_client_ros`を叩いておく

- jsk plugins  
東大のjskが作ったrviz用のプラグイン。
	```
	$ sudo apt install -y ros-kinetic-jsk-visualization
	```

- tensorflow_object_detector  
https://github.com/osrf/tensorflow_object_detector このリポジトリのREADMEに従えばいい．  

  ROS kineticで動かす場合はpython2である必要があるため、tensorflowを入れる際には
  関連するパッケージも含めてバージョンに注意する必要がある。
  20200318時点では
  numpyだけ先に`pip install numpy==1.13.3, mock==3.0.5`で入れておくこと。

  tensorflowのpython パッケージのバージョンは1.13.1でやる．  
  ただし，上記のREADME通りにわざわざtensorflow用の仮想環境を作るとlaunchするときに面倒なので，もとの環境に普通に$ pip install tensorflow==1.13.1 でインストールすればいい．  
  その際，pipやsetuptoolsなどのバージョンが古いと怒られる．その時はpip install --upgrade pip setuptools を実行してからインストールすると大丈夫．  
  注意点として，aptでとってきたpipを上の手順でupgradeするとpipがぶっ壊れるので
  pipを入れなおす必要がある．（ぶっ壊れたら入れなおせばOK）  
  手順は https://qiita.com/Suzukaze31/items/e6d15ddd9ffcd5e6c246 を参照

  依存関係で怒られたら、
  ```
  rosdep install task_switch
  ```
  してみる。
# Usage

- シミュレーション
	1. task_switchをlaunch. joyコンが刺さっていることを確認すること。
		``` 
		$ roslaunch task_switch task_switch.launch
		```
	2. matlabでmain.m内でreel = 0に設定したうえで実行。


- 実験
  - 注意  
  モーションキャプチャ上でのbebopの名前は、「bebop10*」を想定している。
  *部分の番号は、bebop.launchの引数numberとmatlab内のagent numberと連動している。
  もし名前を変える場合は、bebop.launch内のgroup nsの部分を変更、更にはmqtt関連の設定を見なおす必要がある。

  - 手順
  1. `` roslaunch task_switch task_switch.launch real:="true"``
  2. 同じROSネットワーク上に接続された各PCでbebopにwifi接続
  3. 各PC上で``bebop.launch number:="*"`` 
  4. matlabでmain.m内でreal = 1に設定したうえで実行
- 共通
  - matlab内のパラメータmatlab_plot=1とすると、matlab上で情報信頼度やエージェントの位置がプロットされる。重い場合は0にしてプロットを切る。
  - rviz_info_plot = 1でrviz上に情報信頼度をプロットする。重かったら切る。

