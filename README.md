# task switch
bebop2で充電＋フィールド内stay、持続的被覆制御、物体検知＋監視の動作を
切り替えつつ行う実験およびシミュレーション用ROSパッケージ。

# branchについて
- master branch
  - pythonオンリーで書かれた実験＋シミュレーション
- matlab branch
  - pythonとmatlabで書かれた実験＋シミュレーション（2020年9月までmasterだったもの）
  
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

  **tensor flow object detectorのdetect_ros.pyは，cloneしたままの状態だと，物体検出結果のxy座標が逆転してしまっているバグがあるため直すこと．具体的にはdetect_ros.pyの145,146行目のimage_heightとimage_width変数を入れ替ればOK．**

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


# 各configの設定

## cbf_param_server
CBFのオンオフを司る．
- activate_cbf
  オフにするとすべてのCBFが解除される
- activate_fieldcbf
  フィールドから逸脱しない用CBFのオンオフ
- activate_chargecbf
  充電基地へ戻るCBFのオンオフ．オフにするとバッテリーの値が固定値になる．
- activate_pcccbf
  persistent coverage control用CBFのオンオフ
- activate_staycbf
  物体を見つけた際に留まるCBFのオンオフ
- activate_collisioncbf
  ドローン同士の衝突を回避するCBFのオンオフ
- activate_umax
  速度制限をオンオフ
- fieldcbf_slack_weight
  フィールドから離脱しない用のCBFについて，soft constraintとしたときのスラック変数に対する最適化問題におけるweight．
  大きくするほど制約を破りにくくなる．**0にするとhard constraint**
- chargecbf_slack_weight
  充電基地に戻るCBFについて，soft constraintとしたときのスラック変数に対する最適化問題におけるweight．
  大きくするほど制約を破りにくくなる．**0にするとhard constraint**
- pcccbf_slack_weight
  persistent coverage control用CBFについて，soft constraintとしたときのスラック変数に対する最適化問題におけるweight．
  大きくするほど制約を破りにくくなる．**0にするとhard constraint**
- staycbf_slack_weight
  物体を見つけた際に留まるCBFについて，soft constraintとしたときのスラック変数に対する最適化問題におけるweight．
  大きくするほど制約を破りにくくなる．**0にするとhard constraint**
- gamma
  persistent coverage controlにおける，目標被覆性能値
- pcc_CBF_h_gain_k
  pccでCBFとしているh = J - k*J_goalのkの値
- umax
  速度制限値


## charge_param_server
- maxEnergy
  仮想バッテリーの最大容量（一度充電基地に戻ったらこの値まで充電される）
- minEnergy
  仮想バッテリーが下回ってはいけない値（この値になる前に充電基地に戻ってくる）
- Kd
  バッテリーの減少速度（per second）
  0にするとバッテリーは減らない．
- k_charge
  充電基地に戻ってくる際のドローンの速度
  
## pcc_param_server
- controller_gain
  普通のcoverage(非CBF)の際の制御器のゲイン．pcccbfを実験の際は特に気にしなくて良い
- agent_R
  エージェントの被覆できる最大半径
- collisionR
  エージェントの衝突半径．この半径内に他のドローンが入らないようにCBFが働く
- agent_b_
  被覆制御用のパラメータ（CCTAや杉本さん論文参照）
- delta_decrease
  情報信頼度が低下していくスピード
- delta_increase
  監視によって情報信頼度が上昇していくスピード


# その他
- フィールドの大きさはpy_central.launchのx_min～y_maxで設定する．ドローンがネットにぶつかってしまう場合はここを狭くすることも検討する
- ボロノイのメッシュの細かさもpy_central.launch内で設定できる．
- 実験におけるcharging stationの位置や大きさ，各ドローンの初期充電値の設定は，config/bebop_all.yaml内を変更する．bebop_all.yamlはpy_central.launch内で呼ばれているので，変更後に一々各PCでプルしたりしなくても良い，（bebop101.yamlとかはシミュレーション用で残ってしまっていますが，実験では使いません．）
