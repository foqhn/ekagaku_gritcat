#!/bin/bash
# GritCat Wi-Fi 設定ヘルパーのインストール (ロボット上で一度だけ実行)
#   sudo bash app/tools/install_wifi_helper.sh
#
# 1. ヘルパーを root 所有の /usr/local/sbin/gritcat-wifi-helper にコピーする
#    (リポジトリ内のファイルは gritcat ユーザーが書き換えられるため、そのまま sudo 許可してはいけない)
# 2. gritcat ユーザーがそのヘルパーだけをパスワードなしで実行できるよう sudoers に登録する
# ヘルパーを更新したときも、このスクリプトを再実行してコピーし直すこと。
set -euo pipefail

ROBOT_USER="${ROBOT_USER:-gritcat}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/gritcat_wifi_helper.py"
DEST="/usr/local/sbin/gritcat-wifi-helper"
SUDOERS="/etc/sudoers.d/gritcat-wifi"

if [ "$(id -u)" -ne 0 ]; then
    echo "sudo で実行してください: sudo bash $0" >&2
    exit 1
fi

# スキャンに iw を使う
if ! command -v iw >/dev/null 2>&1; then
    apt-get install -y iw
fi

install -o root -g root -m 755 "$SRC" "$DEST"
echo "Installed: $DEST"

TMP="$(mktemp)"
echo "${ROBOT_USER} ALL=(root) NOPASSWD: ${DEST}" > "$TMP"
# 構文チェックしてから配置 (壊れた sudoers で sudo が使えなくなるのを防ぐ)
visudo -cf "$TMP"
install -o root -g root -m 440 "$TMP" "$SUDOERS"
rm -f "$TMP"
echo "Installed: $SUDOERS"

echo "完了しました。動作確認: sudo -u ${ROBOT_USER} sudo -n ${DEST} list"
