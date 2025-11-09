import React from 'react';

/**
 * 複数のセンサーデータを表示するコンポーネント
 * @param {object} props
 * @param {string} props.imu - IMUデータ文字列
 * @param {string} props.mag - Magnetometerデータ文字列
 * @param {string} props.wifi - WiFiデータ文字列
 */
const SensorData = ({ imu, mag, wifi }) => {
    return (
        <div id="data" style={{ margin: '20px' }}>
        <h2>IMU Data</h2>
        <pre>{imu}</pre>
        
        <h2>Magnetometer Data</h2>
        <pre>{mag}</pre>
        
        <h2>WiFi Data</h2>
        <pre>{wifi}</pre>
        </div>
    );
};

export default SensorData;