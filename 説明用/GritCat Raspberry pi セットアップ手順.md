
# 用意するもの
- Raspberry pi 4B 8G (4Gでも可)
- PC（LANポートがついているものが好ましい．なければ外付けのものを用意）
- SDカード
- SDカードリーダー
- ネットワーク環境
- 知識

1. Raspberry pi Imager のインストール
	https://www.raspberrypi.com/software/
	このドキュメントではVer2.0.0を持ちいます．
2. Imagerを使いOSを焼きこむ
	1. ハードの選択<br>
	   ![](images/Pasted%20image%2020251127174110.png)
	3. OSの選択
	   Ubuntu22.04のServerを選択する．<br>
	   ![](images/Pasted%20image%2020251127174319.png)
	   ![](images/Pasted%20image%2020251127174338.png)
	   ![](images/Pasted%20image%2020251127174411.png)
	4. SDカードが差し込まれていれば，OSを焼きこむSDカードが項目に現れるので，それを選択<br>
	   ![](images/Pasted%20image%2020251127174431.png)<br>
	   デバイス名を記入．
	   デバイス名に混合が発生するとエラーが起こるため，被りがないようにする．
	   （ロボットリストを参考に名前を付ける）<br>
	   ![](images/Pasted%20image%2020251127174514.png)<br>
	   ユーザー名は「gritcat」で統一
	   パスワードは「ekagaku」で統一
	   ![](images/Pasted%20image%2020251127174629.png)<br>
	5. SSHの有効化のトグルを必ずONに
	   認証方法はパスワードでOK<br>
	   ![](images/Pasted%20image%2020251127174752.png)<br>
	  完了したら書き込み開始

3. PCについているLANポートのアドレスを変更
	1. コントロールパネル⇒ネットワークとインターネット⇒ネットワークと共有センター⇒アダプター設定の変更⇒Raspberrypiを接続する予定のイーサネットポートを右クリック⇒プロパティ⇒インターネットプロトコルバージョン4
	2. 以下のように記入<br>
	   ![](images/Pasted%20image%2020251127200438.png)<br>
	3. OK⇒閉じるを押す
	4. コマンドプロンプトで「ipconfig」 アドレスが設定したとおりになっているか確認<br>
	   ![](images/Pasted%20image%2020251127200619.png)<br>
4. Raspberry piを起動
	1. USBケーブルを差し，Raspberry piを起動
	2. Raspberry piにPing（ノック）が通るか確認
	```bash
	ping "ロボットの名前".local
	```
	以下のような表示があり，応答が確認できればOK<br>
	![](images/Pasted%20image%2020251203210633.png)<br>
	3. 
5. SSH接続
	1. 先ほどのホスト名にダイレクトにSSH接続
	```bash
	ssh gritcat@"ロボットの名前".local
	```
	以下のようにユーザー名が表示されればOK<br>
	![](images/Pasted%20image%2020251203212021.png)<br>
	2. ネットプランを追加し，イーサネットに固定IPを与える
	```
	 sudo nano /etc/netplan/99_network_config.yaml
	```
	以下のように書き込む.
	```sh
	network:
	  version: 2
	  renderer: networkd
	  ethernets:
	    eth0:
	      dhcp4: false
	      dhcp6: false
	      addresses: [192.168.147."割り当てられたアドレス"/24]
	      routes:
	        - to: default
	          via: 192.168.147.1
	      nameservers:
	        addresses: [192.168.147.1, 8.8.8.8, 8.8.4.4]
	```
	書き込んだネットプランを適応する．
	```
	sudo chmod /etc/netplam/99_network_config.yaml
	sudo netplan apply
	```
	アドレスが変わっていることを確認
	```
	ip a
	```
 	<br>
	
	![](images/Pasted%20image%2020251204091935.png)<br>
	再起動
6. VSCodeでSSH接続
	1. 拡張機能のRemote Developementをインストールしておく
	2. Remote ExplorerタブからRaspberry piに接続<br>
	   ![](images/Pasted%20image%2020251204100408.png)<br>
	   Connectを押して接続
	   ![](images/Pasted%20image%2020251204100440.png)<br>
	3. パスワード入力　左下に「SSH：アドレス」が表示されれば接続完了
	4. ホームディレクトリを開く<br>
	   ![](images/Pasted%20image%2020251204100658.png)<br>
	5. ターミナルを開く
7. ソフトウェアセットアップ
	1. アップデート
	```bash
	sudo apt update
	```
	2. githubからリポジトリをクローン
	```bash
	git clone https://github.com/foqhn/ekagaku_gritcat.git -b ws_client_ver
	```
	3. ROSのインストール
	```bash
	source ~/ekagaku_gritcat/app/install_ros.sh
	```
	途中でパスワードの入力や確認画面が出るので，適宜対応<br>
	4. パッケージのインストール
	```bash
	source ~/ekagaku_gritcat/app/install_ros_packages-.sh
	python3 -m pip install --upgrade pip
	pip install -r ~/ekagaku_gritcat/app/requirment.txt 
	```
	5. rosdepのセットアップ
	```bash
    cd ./ekagaku_gritcat/grit_ws/
    sudo rosdep init
    rosdep update
	```
	6. bno055(IMU)のパッケージをダウンロード
	```bash
    cd ~/ekagaku_gritcat/grit_ws/src
    git clone https://github.com/flynneva/bno055.git
    cd ..
    colcon build
    source ~/.bashrc
	```
	7. bno055のパッケージを少し書き換える
		1. bno055/launch/bno055.launch.pyを編集<br>
			![](images/Pasted%20image%2020251206195642.png)<br>
		2. bno055/bno055/params/bno055_params_i2c.yamlを編集<br>
			![](images/Pasted%20image%2020251206195822.png)
	8. GPSのセットアップ
	以下のコマンドを打ち，GPSのセットアップファイルを開く
	```bash
	sudo nano /etc/default/gpsd
	```
	以下のように編集
	```sh
	# Default settings for the gpsd init script and the hotplug wrapper.
	# Start the gpsd daemon automatically at boot time
	START_DAEMON="true"
	# Use USB hotplugging to add new USB devices automatically to the daemon
	USBAUTO="true"
	# Devices gpsd should collect to at boot time.
	# They need to be read/writeable, either by user gpsd or the group dialout.
	DEVICES="/dev/ttyUSB0"
	# Other options you want to pass to gpsd
	GPSD_OPTIONS="-F /var/run/gpsd.sock -b -n -r"
	```
	9. 
8. スタートアップ設定
	1. サービスファイルを作成
	```bash
	sudo nano /etc/systemd/system/gritcat-system.service
	```
	内容を以下のように編集
	```bash
	[Unit]
	# サービスの説明
	Description=My Robot System Service
	# ネットワークがオンラインになってから起動する
	After=network-online.target
	Wants=network-online.target
	
	[Service]
	
	User=gritcat
	# 実行するグループを指定 (通常はユーザーと同じ)
	Group=gritcat
	
	# スクリプトがあるディレクトリの絶対パスを指定
	# '~' は使えないため、/home/ユーザー名 のようにフルパスで記述
	WorkingDirectory=/home/gritcat/ekagaku_gritcat
	
	# 実行するコマンドを絶対パスで指定
	ExecStart=/home/gritcat/ekagaku_gritcat/app/run_system.sh
	
	
	# エラーなどでプロセスが終了した場合、10秒後に自動で再起動する
	Restart=on-failure
	RestartSec=10
	
	# 標準出力をjournaldに送る
	StandardOutput=journal
	StandardError=journal
	SyslogIdentifier=my-robot
	
	[Install]
	# システム起動時に自動起動するように設定
	WantedBy=multi-user.target
	```
	2. サービスの有効化
	```bash
	# 権限を与える
	sudo chmod +x /home/gritcat/ekagaku_gritcat/app/run_system.sh
	# systemdに新しいサービスファイルを認識させる
	sudo systemctl daemon-reload
	# OS起動時にサービスが自動で起動するように設定（有効化）
	sudo systemctl enable gritcat-system.service
	# 今すぐサービスを起動する
	sudo systemctl start gritcat-system.service
	# サービスの状態を確認する
	sudo systemctl status gritcat-system.service
	```
	3. 
9. 
	
