import React from 'react';
import './WifiSignal.css'; // スタイルを定義するCSSファイルをインポート

/**
 * Wifiの電波強度を可視化するコンポーネント
 * @param {object} props.wifi - WiFiデータ
 * @param {string} props.wifi.ssid - SSID
 * @param {string} props.wifi.signal_strength - 電波強度
 */
const WifiSignal = ({ wifi }) => {
    const { ssid, signal_strength } = wifi;

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