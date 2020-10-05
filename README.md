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
  1. multi_sim.launchをlaunch. joyコンが刺さっていることを確認すること。
      niwaya上で
      ```
      $ roslaunch task_switch multi_sim.launch
      ```
      numpyのバージョンが新しいとエラーになる場合がある．動作確認はnumpy==1.13.1でしか行っていないので注意．

- 実験
  - 注意  
  モーションキャプチャ上でのbebopの名前は、「bebop10*」を想定している。
  この名前は、vrpnを通して配信されるトピック名に影響する．
  bebop10*という名前を想定して，controller等もベタ書きしているので，名前を変える際は注意する．

  - 手順
  1. niwayaにjoyコンが刺さっていることを確認すること。
      niwaya上で
      ```
      $ roslaunch task_switch py_central.launch agentNum:="3"
      ```
      agentNumは使用するbebopの台数を指定
  2. 同じROSネットワーク上に接続された各PCでbebopにwifi接続
  3. 各PC上で
      ```
      $ roslaunch task_switch py_bebop.launch number:="1"
      ``` 
      ``number:="*"``はそれぞれのPCで接続しているbebopの，モーションキャプチャ上での番号と一致させること．
      （例：モーキャプ上で「bebop102」としているならnumber:="2"）
  4. configureウィンドウがniwayaに表示されているはずなので，調整する．  
  5. joyコンの×ボタンを押して，入力ソースを切り替えると実験が動く．

  - configについて
    - umaxを大きくし過ぎるとドローンがとんでもない速さで飛んでしまうので気を付ける．具体的には0.3程度にしておくと安心．また，umaxのチェックボックスを外さないこと．
    - 各cbfはチェックボックスで入/切できる．soft制約にしたときのslack変数の重さをフェーダーで調節することで，動きの微調整が可能
      hard制約にしたいときはこの重さを0にすればhardとして扱われる．
    - field cbfを切ると，ネットにぶつからないようにするcbfが切れてしまうので，基本はチェックを外さないこと．

