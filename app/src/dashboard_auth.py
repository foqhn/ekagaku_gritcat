"""
ローカルダッシュボードのパスワード認証 (main.py と app/tools/reset_dashboard_password.py から使う)

- パスワードは config.json の "dashboard_auth" に、ソルト付き PBKDF2-SHA256 のハッシュとして保存する (平文は保存しない)
- ログインに成功するとセッショントークンを発行する (メモリ上のみ。プログラムを再起動すると再ログインが必要)
- 連続して間違えたら一定時間ログインを受け付けない (総当たり対策)
"""
import hashlib
import hmac
import secrets
import threading
import time

from src.config_manager import ConfigManager

AUTH_KEY = "dashboard_auth"
PBKDF2_ITERATIONS = 200_000
MIN_PASSWORD_LENGTH = 8
SESSION_TTL_SEC = 12 * 3600
MAX_FAILURES = 5
LOCK_SEC = 30

_lock = threading.Lock()
# token -> (有効期限 time.time(), 発行時のパスワードハッシュ)
# パスワードが変わったら (別プロセスのリセット用スクリプトで変えた場合も) 古いセッションは無効になる
_sessions = {}
_failures = {"count": 0, "locked_until": 0.0}


def hash_password(password, salt_hex=None, iterations=PBKDF2_ITERATIONS):
    salt = bytes.fromhex(salt_hex) if salt_hex else secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, iterations)
    return {"salt": salt.hex(), "hash": digest.hex(), "iterations": iterations}


def get_auth():
    auth = ConfigManager.load_config().get(AUTH_KEY)
    return auth if isinstance(auth, dict) and auth.get("hash") and auth.get("salt") else None


def is_password_set():
    return get_auth() is not None


def validate_new_password(password):
    if not isinstance(password, str) or len(password) < MIN_PASSWORD_LENGTH:
        raise ValueError(f"パスワードは {MIN_PASSWORD_LENGTH} 文字以上にしてください")
    if len(password) > 128:
        raise ValueError("パスワードが長すぎます (128文字まで)")


def set_password(password):
    """パスワードを設定/変更する (既存のセッションはすべて無効にする)"""
    validate_new_password(password)
    conf = ConfigManager.load_config()
    conf[AUTH_KEY] = hash_password(password)
    if not ConfigManager.save_config(conf):
        raise RuntimeError("config.json に保存できませんでした")
    with _lock:
        _sessions.clear()


def clear_password():
    """パスワードを削除して認証を無効にする (リセット用)"""
    conf = ConfigManager.load_config()
    conf.pop(AUTH_KEY, None)
    ConfigManager.save_config(conf)
    with _lock:
        _sessions.clear()


def check_password(password):
    auth = get_auth()
    if not auth or not isinstance(password, str):
        return False
    computed = hash_password(password, auth["salt"], int(auth.get("iterations", PBKDF2_ITERATIONS)))
    return hmac.compare_digest(computed["hash"], auth["hash"])


def login(password):
    """
    ログインを試みる。戻り値: (token or None, エラーメッセージ or None)
    連続で間違えると LOCK_SEC 秒間ロックする
    """
    with _lock:
        wait = _failures["locked_until"] - time.time()
        if wait > 0:
            return None, f"続けて間違えたため、{int(wait) + 1} 秒後にもう一度試してください"
    if not check_password(password):
        with _lock:
            _failures["count"] += 1
            if _failures["count"] >= MAX_FAILURES:
                _failures["count"] = 0
                _failures["locked_until"] = time.time() + LOCK_SEC
        return None, "パスワードが違います"
    with _lock:
        _failures["count"] = 0
    return new_session(), None


def new_session():
    auth = get_auth()
    with _lock:
        token = secrets.token_urlsafe(32)
        _sessions[token] = (time.time() + SESSION_TTL_SEC, auth["hash"] if auth else None)
    return token


def is_valid_session(token):
    if not token:
        return False
    auth = get_auth()
    with _lock:
        entry = _sessions.get(token)
        if entry and entry[0] > time.time() and auth and entry[1] == auth["hash"]:
            return True
        _sessions.pop(token, None)
        return False


def logout(token):
    with _lock:
        _sessions.pop(token, None)


def strip_secret(conf):
    """API で返す設定からパスワードのハッシュを除く"""
    return {k: v for k, v in conf.items() if k != AUTH_KEY}
