import React from 'react';
import './WifiSignal.css'; // スタイルを定義するCSSファイルをインポート

/**
 * Wifiの電波強度を可視化するコンポーネント
 * @param {string} props.wifi - WiFiデータ
 */
const WifiSignal = ({ wifi }) => {
    // wifi="Connecting..."という状態を考慮 
    if (wifi === "Connecting..." || wifi === "No Data") {
        return <div className="wifi-container">No WiFi Data</div>;
    }
    // stringのwifiデータをパース　wifi={ssid: "MyNetwork", signal_strength: "-65"}
    const wifiData = JSON.parse(wifi);
    const { ssid, signal_strength } = wifiData;
    console.log('Parsed WiFi data:', { ssid, signal_strength });
    // 電波強度を数値に変換
    const signalStrengthNumber = parseInt(signal_strength, 10);

    // 電波強度に基づいてバーの数を決定する
    const getSignalBars = (strength) => {
        if (strength > -50) {
            return 4; // 非常に強い
        } else if (strength > -60) {
            return 3; // 強い
        } else if (strength > -70) {
            return 2; // 普通
        } else {
            return 1; // 弱い
        }
    };

    const signalBars = getSignalBars(signalStrengthNumber);

    return (
        <div className="wifi-container">
            <div className="wifi-ssid">{ssid}</div>
            <div className="wifi-signal-strength">
                <div className={`bar ${signalBars >= 1 ? 'active' : ''}`}></div>
                <div className={`bar ${signalBars >= 2 ? 'active' : ''}`}></div>
                <div className={`bar ${signalBars >= 3 ? 'active' : ''}`}></div>
                <div className={`bar ${signalBars >= 4 ? 'active' : ''}`}></div>
            </div>
        </div>
    );
};

export default WifiSignal;