## 使用手順
以下のコマンドを入力
```bash
# ディレクトリに移動
cd ~/raspberry_ws/src
git clone https://github.com/omasahiro/encoder_odom.git

# pigpioのインストール
sudo apt-get update
sudo apt-get install pigpio

# pigpioデーモンの設定と起動
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# ワークスペースのルートディレクトリに移動
cd ~/ros2_ws

# ビルド
colcon build --packages-select encoder_odom

# 環境のセットアップ
source install/setup.bash

# 実行
ros2 run encoder_odom encoder_odom_node

