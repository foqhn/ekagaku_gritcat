#!/usr/bin/env python3
"""
GritCat Wi-Fi 設定ヘルパー (root で実行される)

ローカルダッシュボード (main.py, ユーザー gritcat) から sudo 経由で呼ばれ、
GritCat 専用の Netplan ファイルにアクセスポイントを追加/削除して反映する。
既存の Netplan ファイル (例: 99_network_config) には一切触れない。

インストール先: /usr/local/sbin/gritcat-wifi-helper (root 所有, 755)
  → app/tools/install_wifi_helper.sh を参照。
  リポジトリ内のこのファイルを直接 sudo 許可してはいけない
  (gritcat ユーザーが書き換えられる場所のスクリプトに sudo を許すと root を奪われるため)。

使い方 (入力はすべて標準入力の JSON。パスワード類をコマンドライン引数に載せない):
  gritcat-wifi-helper list            登録済み SSID 一覧 (GritCat 管理分 + その他の設定分)
  gritcat-wifi-helper scan            周辺のアクセスポイントをスキャン
  gritcat-wifi-helper add    < {"ssid": "...", "psk": "<64桁hex>"}
  gritcat-wifi-helper remove < {"ssid": "..."}
出力: JSON 1行 {"ok": true/false, ...}
"""
import json
import os
import re
import shutil
import subprocess
import sys
import time

INTERFACE = "wlan0"
STATE_DIR = "/etc/gritcat"
STATE_FILE = os.path.join(STATE_DIR, "wifi-networks.json")     # 正本 (SSID と PSK)
NETPLAN_FILE = "/etc/netplan/90-gritcat-wifi.yaml"             # STATE_FILE から生成
BACKUP_SUFFIX = ".gritcat-bak"
APPLY_DELAY_SEC = 2          # HTTP 応答を返してから反映するための待ち時間
ROLLBACK_CHECK_SEC = 90      # 反映後この秒数たっても Wi-Fi がつながらなければ元に戻す
NETPLAN = "/usr/sbin/netplan"
MAX_NETWORKS = 20

SSID_MAX_BYTES = 32
PSK_RE = re.compile(r"^[0-9a-f]{64}$")


def out(ok, **kw):
    print(json.dumps({"ok": ok, **kw}, ensure_ascii=False))
    sys.exit(0 if ok else 1)


def validate_ssid(ssid):
    if not isinstance(ssid, str) or not ssid:
        raise ValueError("SSID が空です")
    if len(ssid.encode("utf-8")) > SSID_MAX_BYTES:
        raise ValueError("SSID が長すぎます (32バイトまで)")
    if any(ord(c) < 0x20 or ord(c) == 0x7f for c in ssid):
        raise ValueError("SSID に制御文字は使えません")
    return ssid


def load_state():
    try:
        with open(STATE_FILE, encoding="utf-8") as f:
            data = json.load(f)
        return [n for n in data.get("networks", []) if isinstance(n, dict) and "ssid" in n and "psk" in n]
    except FileNotFoundError:
        return []


def render_netplan(networks):
    """Netplan YAML を生成する。文字列は JSON 形式で書く (YAML のダブルクォート文字列として有効)"""
    lines = [
        "# GritCat ローカルダッシュボードが管理するファイルです。手で編集しないでください。",
        "# 既存の Netplan 設定とマージされ、ここに書いたアクセスポイントが追加されます。",
        "network:",
        "  version: 2",
        "  wifis:",
        f"    {INTERFACE}:",
        "      dhcp4: true",
        "      access-points:",
    ]
    for n in networks:
        lines.append(f"        {json.dumps(n['ssid'], ensure_ascii=False)}:")
        lines.append(f"          password: {json.dumps(n['psk'])}")
    return "\n".join(lines) + "\n"


def write_secure(path, text):
    """root のみ読み書きできる権限 (600) で書く"""
    tmp = path + ".tmp"
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)


def backup(path):
    if os.path.exists(path):
        shutil.copy2(path, path + BACKUP_SUFFIX)
    elif os.path.exists(path + BACKUP_SUFFIX):
        os.remove(path + BACKUP_SUFFIX)  # 「元はファイルなし」を表す


def restore(path):
    if os.path.exists(path + BACKUP_SUFFIX):
        os.replace(path + BACKUP_SUFFIX, path)
    elif os.path.exists(path):
        os.remove(path)


def wifi_connected():
    try:
        r = subprocess.run(["iw", "dev", INTERFACE, "link"], capture_output=True, text=True, timeout=5)
        return r.returncode == 0 and r.stdout.startswith("Connected")
    except Exception:
        return False


def save_and_apply(networks):
    """状態を保存→Netplan 生成→構文チェック。成功したら反映は別プロセスで少し後に行う"""
    os.makedirs(STATE_DIR, mode=0o700, exist_ok=True)
    backup(STATE_FILE)
    backup(NETPLAN_FILE)
    write_secure(STATE_FILE, json.dumps({"networks": networks}, ensure_ascii=False, indent=2))
    if networks:
        write_secure(NETPLAN_FILE, render_netplan(networks))
    elif os.path.exists(NETPLAN_FILE):
        os.remove(NETPLAN_FILE)

    r = subprocess.run([NETPLAN, "generate"], capture_output=True, text=True, timeout=30)
    if r.returncode != 0:
        restore(STATE_FILE)
        restore(NETPLAN_FILE)
        subprocess.run([NETPLAN, "generate"], capture_output=True, timeout=30)
        raise RuntimeError(f"netplan generate に失敗したため元に戻しました: {(r.stderr or r.stdout).strip()[:300]}")

    # 呼び出し元 (ローカルダッシュボード) が応答を返せるよう、反映は切り離したプロセスで行う
    was_connected = "1" if wifi_connected() else "0"
    subprocess.Popen([sys.executable, os.path.abspath(__file__), "_apply", was_connected],
                     start_new_session=True, stdin=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def cmd_apply_and_watch(was_connected):
    """(内部用) 反映し、つながらなければ直前の設定に戻す"""
    time.sleep(APPLY_DELAY_SEC)
    subprocess.run([NETPLAN, "apply"], capture_output=True, timeout=120)
    if was_connected != "1":
        return  # 元々つながっていなかった場合は比較できないので戻さない
    deadline = time.time() + ROLLBACK_CHECK_SEC
    while time.time() < deadline:
        if wifi_connected():
            return
        time.sleep(5)
    # つながらなかった → 直前の設定に戻して再反映
    restore(STATE_FILE)
    restore(NETPLAN_FILE)
    subprocess.run([NETPLAN, "apply"], capture_output=True, timeout=120)


def list_other_ssids():
    """GritCat 管理外 (既存の Netplan ファイル) で設定されている SSID を返す (取得できなければ空)"""
    try:
        r = subprocess.run([NETPLAN, "get", f"wifis.{INTERFACE}.access-points"],
                           capture_output=True, text=True, timeout=15)
        if r.returncode != 0:
            return []
        ssids = []
        for line in r.stdout.splitlines():
            # 最上位のキー行のみ (例: 'kashimoba-051:' / '"my ssid":')
            m = re.match(r'^(?:"((?:[^"\\]|\\.)*)"|\'([^\']*)\'|([^\s#][^:]*)):\s*$', line)
            if m:
                ssids.append(json.loads(f'"{m.group(1)}"') if m.group(1) is not None else (m.group(2) or m.group(3)).strip())
        return ssids
    except Exception:
        return []


def cmd_scan():
    try:
        r = subprocess.run(["iw", "dev", INTERFACE, "scan"], capture_output=True, text=True, timeout=20)
    except FileNotFoundError:
        out(False, error="iw コマンドがありません (sudo apt install iw)")
    if r.returncode != 0:
        out(False, error=f"スキャンに失敗しました: {r.stderr.strip()[:200]}")
    found = {}
    signal = None
    for line in r.stdout.splitlines():
        line = line.strip()
        if line.startswith("BSS "):
            signal = None
        elif line.startswith("signal:"):
            try:
                signal = float(line.split()[1])
            except (IndexError, ValueError):
                signal = None
        elif line.startswith("SSID:"):
            ssid = line[5:].strip()
            if ssid and (ssid not in found or (signal or -999) > (found[ssid] or -999)):
                found[ssid] = signal
    nets = sorted(({"ssid": s, "signal_dbm": v} for s, v in found.items()),
                  key=lambda n: n["signal_dbm"] if n["signal_dbm"] is not None else -999, reverse=True)
    out(True, networks=nets)


def main():
    if os.geteuid() != 0:
        out(False, error="root 権限で実行してください")
    if len(sys.argv) < 2:
        out(False, error="usage: gritcat-wifi-helper list|scan|add|remove")
    cmd = sys.argv[1]
    try:
        if cmd == "_apply":
            cmd_apply_and_watch(sys.argv[2] if len(sys.argv) > 2 else "0")
            return
        if cmd == "list":
            managed = [n["ssid"] for n in load_state()]
            others = [s for s in list_other_ssids() if s not in managed]
            out(True, managed=managed, others=others)
        if cmd == "scan":
            cmd_scan()

        req = json.load(sys.stdin)
        ssid = validate_ssid(req.get("ssid"))
        networks = load_state()
        if cmd == "add":
            psk = req.get("psk", "")
            if not isinstance(psk, str) or not PSK_RE.match(psk):
                raise ValueError("PSK の形式が不正です")
            networks = [n for n in networks if n["ssid"] != ssid]  # 同じ SSID は上書き
            if len(networks) >= MAX_NETWORKS:
                raise ValueError(f"登録できるのは {MAX_NETWORKS} 件までです")
            networks.append({"ssid": ssid, "psk": psk})
            save_and_apply(networks)
            out(True, message=f"「{ssid}」を追加しました。数秒後に反映されます")
        elif cmd == "remove":
            if not any(n["ssid"] == ssid for n in networks):
                raise ValueError(f"「{ssid}」はローカルダッシュボードで追加したアクセスポイントではありません")
            save_and_apply([n for n in networks if n["ssid"] != ssid])
            out(True, message=f"「{ssid}」を削除しました。数秒後に反映されます")
        else:
            out(False, error=f"unknown command: {cmd}")
    except (ValueError, RuntimeError, json.JSONDecodeError) as e:
        out(False, error=str(e))


if __name__ == "__main__":
    main()
