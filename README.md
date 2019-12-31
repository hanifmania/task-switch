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
mqtt_bridge と検索して上の方に出てくる「mqtt_bridge を作ったよ！」というタイトルの記事に書いてあるとおりに git からクローンしたりすればオーケー
ただし、ビルド、もしくは launch しようとするといろいろ Import Error で怒られる（inject 等）ため、随時pip等で入れていけば大丈夫。


- vrpn_client  
ROS wiki に vrpn_client についてのページがあるためそちらを参照。
基本的には
	```
	$ git clone https://github.com/ros-drivers/vrpn_client_ros.gitすればオーケーなはず。
	```

- jsk plugins  
東大のjskが作ったrviz用のプラグイン。
	```
	$ sudo apt install -y ros-kinetic-jsk-visualization
	```

- tensorflow_object_detector  
https://github.com/osrf/tensorflow_object_detector このリポジトリのREADMEに従えばいい．  
tensorflowのpython パッケージのバージョンは1.13.1でやる．  
ただし，上記のREADME通りにわざわざtensorflow用の仮想環境を作るとlaunchするときに面倒なので，もとの環境に普通に$ pip install tensorflow==1.13.1 でインストールすればいい．  
その際，pipやsetuptoolsなどのバージョンが古いと怒られる．その時はpip install --upgrade pip setuptools を実行してからインストールすると大丈夫．

# Usage

- シミュレーション
	1. task_switchをlaunch. joyコンが刺さっていることを確認すること。
		``` 
		$ roslaunch task_switch task_switch.launch
		```
	2. matlabでmain.m内でreel = 0に設定したうえで実行。


- 実験
hogehoge 
