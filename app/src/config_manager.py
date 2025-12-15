import json
import os

# 設定ファイルのパス（main.pyと同じ階層に作成されます）
CONFIG_FILE = "config.json"

# デフォルト設定
DEFAULT_CONFIG = {
    "robot_id": "robot03",
    "server_ip": "192.168.11.14",  # 必要ならIPもここに移せます
    "server_port": 8000
}

class ConfigManager:
    @staticmethod
    def load_config():
        """設定ファイルを読み込む。なければデフォルトを作成して返す"""
        if not os.path.exists(CONFIG_FILE):
            ConfigManager.save_config(DEFAULT_CONFIG)
            return DEFAULT_CONFIG
        
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            print(f"Config load error: {e}. Using default.")
            return DEFAULT_CONFIG

    @staticmethod
    def save_config(config_data):
        """設定をファイルに保存する"""
        try:
            with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4)
            return True
        except Exception as e:
            print(f"Config save error: {e}")
            return False

    @staticmethod
    def update_robot_id(new_id):
        """IDだけを更新する便利メソッド"""
        config = ConfigManager.load_config()
        config["robot_id"] = new_id
        return ConfigManager.save_config(config)