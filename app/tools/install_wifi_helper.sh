#!/bin/bash
# GritCat のローカルダッシュボードが使う root 権限まわりのセットアップ (ロボット上で一度だけ実行)
#   sudo bash app/tools/install_wifi_helper.sh
#
# 1. Wi-Fi 設定ヘルパーを root 所有の /usr/local/sbin/gritcat-wifi-helper にコピーする
#    (リポジトリ内のファイルは gritcat ユーザーが書き換えられるため、そのまま sudo 許可してはいけない)
# 2. gritcat ユーザーがそのヘルパーだけをパスワードなしで実行できるよう sudoers に登録する
# 3. ロボット本体の再起動/シャットダウン用に、次の2コマンドだけをパスワードなしで実行できるよう登録する
#      /usr/bin/systemctl reboot, /usr/bin/systemctl poweroff
# ヘルパーを更新したときも、このスクリプトを再実行してコピーし直すこと。
set -euo pipefail

ROBOT_USER="${ROBOT_USER:-gritcat}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/gritcat_wifi_helper.py"
DEST="/usr/local/sbin/gritcat-wifi-helper"
SUDOERS="/etc/sudoers.d/gritcat-wifi"
SUDOERS_POWER="/etc/sudoers.d/gritcat-power"

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

# sudoers を構文チェックしてから配置する (壊れた sudoers で sudo が使えなくなるのを防ぐ)
install_sudoers() {
    local dest="$1" content="$2" tmp
    tmp="$(mktemp)"
    printf '%s\n' "$content" > "$tmp"
    visudo -cf "$tmp"
    install -o root -g root -m 440 "$tmp" "$dest"
    rm -f "$tmp"
    echo "Installed: $dest"
}

install_sudoers "$SUDOERS" "${ROBOT_USER} ALL=(root) NOPASSWD: ${DEST}"
install_sudoers "$SUDOERS_POWER" "${ROBOT_USER} ALL=(root) NOPASSWD: /usr/bin/systemctl reboot, /usr/bin/systemctl poweroff"

echo "完了しました。動作確認:"
echo "  sudo -u ${ROBOT_USER} sudo -n ${DEST} list"
echo "  sudo -u ${ROBOT_USER} sudo -n -l /usr/bin/systemctl reboot   (コマンドのパスが表示されれば OK。実行はされません)"
