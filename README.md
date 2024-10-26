## 使用手順
以下のコマンドを入力

1.ディレクトリに移動
```bash
cd ~/raspberry_ws/src
```
```
git clone https://github.com/omasahiro/encoder_odom.git
```

2.pigpioのインストール
```
sudo apt-get update
```
```
sudo apt-get install pigpio
```

3.pigpioデーモンの設定と起動
```
sudo systemctl enable pigpiod
```
```
sudo systemctl start pigpiod
```

4.ビルドして実行
```
cd ~/ros2_ws
```
```
colcon build --packages-select encoder_odom
```
```
source install/setup.bash
```
```
ros2 run encoder_odom encoder_odom_node
```
