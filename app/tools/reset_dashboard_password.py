#!/usr/bin/env python3
"""
ローカルダッシュボードのパスワードを再設定する (パスワードを忘れたとき用。ロボット上で gritcat ユーザーで実行)

  python3 ~/ekagaku_gritcat/app/tools/reset_dashboard_password.py          # 新しいパスワードを設定
  python3 ~/ekagaku_gritcat/app/tools/reset_dashboard_password.py --clear  # パスワードを削除 (認証なしに戻す)

実行後、ロボットのプログラムを再起動しなくても次のログインから反映される (ログイン中の人は再ログインが必要)。
"""
import getpass
import os
import sys

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(APP_DIR)
sys.path.insert(0, APP_DIR)
os.chdir(REPO_DIR)  # config.json はリポジトリ直下 (ロボットのプログラムと同じ場所)

from src import dashboard_auth  # noqa: E402


def main():
    if "--clear" in sys.argv[1:]:
        dashboard_auth.clear_password()
        print("パスワードを削除しました。ローカルダッシュボードは認証なしで開けます。")
        return
    password = getpass.getpass("新しいパスワード: ")
    if password != getpass.getpass("もう一度入力: "):
        sys.exit("入力が一致しません。")
    try:
        dashboard_auth.set_password(password)
    except ValueError as e:
        sys.exit(str(e))
    print("パスワードを設定しました。")


if __name__ == "__main__":
    main()
