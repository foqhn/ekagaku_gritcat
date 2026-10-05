#!/bin/bash
# grit_ws の外部 ROS パッケージを取得してビルドする (新しいロボットのセットアップ時・更新時に実行)
#   bash ~/ekagaku_gritcat/grit_ws/setup_ros_packages.sh
#
# - bno055 (https://github.com/flynneva/bno055) は Git 管理外 (.gitignore 済み) なので、ここでクローンする。
#   動作確認済みのバージョンに固定し、リポジトリ側のファイルは一切書き換えない。
#   GritCat 用の起動ファイル・パラメータは gritcat_utils パッケージにある:
#     ros2 launch gritcat_utils bno055.launch.py namespace:=<ロボットID>
# - その後 GritCat のパッケージ (bno055, gritcat_utils, gpsd_driver) をビルドする。
#
# オプション:
#   --reset-bno055   bno055 に手作業の変更があれば破棄して元に戻す
#                    (以前は bno055 の launch/params を直接書き換えていた。今は使わないので戻してよい)
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BNO055_REPO="https://github.com/flynneva/bno055.git"
BNO055_COMMIT="45e1ff16936101711260c9fda63fbad99376ce3b"   # v0.5.0 (GritCat で動作確認済み)
RESET_BNO055=0
[ "${1:-}" = "--reset-bno055" ] && RESET_BNO055=1

# ROS 2 の環境 (colcon/ros2 コマンド)
set +u
source /opt/ros/humble/setup.bash
set -u

# bno055 が使う Python ライブラリ (package.xml の依存)
for pkg in python3-smbus python3-serial; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
        echo "必要なパッケージ $pkg がありません。インストールします (sudo パスワードを求められることがあります)"
        sudo apt-get install -y "$pkg"
    fi
done

cd "$WS/src"
if [ ! -d bno055/.git ]; then
    echo "=== bno055 をクローンします ==="
    git clone "$BNO055_REPO" bno055
fi

if ! git -C bno055 diff --quiet; then
    if [ "$RESET_BNO055" = "1" ]; then
        echo "=== bno055 の手作業の変更を破棄します ==="
        git -C bno055 diff --stat
        git -C bno055 checkout -- .
    else
        echo "注意: bno055 に手作業の変更があります (GritCat では使っていません。戻すには --reset-bno055 を付けて再実行):"
        git -C bno055 diff --stat
    fi
fi

# 固定したバージョンにそろえる (未取得なら取得)
if ! git -C bno055 cat-file -e "${BNO055_COMMIT}^{commit}" 2>/dev/null; then
    git -C bno055 fetch origin
fi
git -C bno055 checkout --quiet "$BNO055_COMMIT"
echo "bno055: $(git -C bno055 describe --tags --always)"

echo "=== ビルドします ==="
cd "$WS"
colcon build --packages-select bno055 gritcat_utils gpsd_driver

echo
echo "完了しました。ロボットのプログラムを再起動してください: sudo systemctl restart gritcat-system"
