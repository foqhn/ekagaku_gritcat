import React from 'react';

/**
 * センサーデータをカード形式で表示するコンポーネント
 * @param {object} props
 * @param {string|object} props.imu - IMU データ（JSON 文字列またはオブジェクト）
 * @param {string|object} props.mag - 磁気データ（JSON 文字列またはオブジェクト）
 * @param {string} props.wifi - WiFi データ文字列
 * @param {string} props.gps - GPS データ文字列
 */
const SensorData = ({ imu, mag, wifi, gps }) => {
    // 安全に JSON をパースするヘルパー関数
    const safeParse = (data) => {
        if (!data) return null;
        if (typeof data === 'object') return data;
        try {
            return JSON.parse(data);
        } catch (e) {
            console.warn('SensorData: JSON parse error', e);
            return null;
        }
    };

    const imuData = safeParse(imu);
    const magData = safeParse(mag);

    // テーブル描画ユーティリティ
    const renderTable = (obj) => (
        <table className="sensor-table">
            <tbody>
                {Object.entries(obj).map(([key, value]) => (
                    <tr key={key}>
                        <td className="sensor-key">{key}</td>
                        <td className="sensor-value">
                            {typeof value === 'object' ? JSON.stringify(value) : value}
                        </td>
                    </tr>
                ))}
            </tbody>
        </table>
    );

    return (
        <div id="data" className="sensor-data-container">
            {/* IMU */}
            <div className="sensor-card">
                <h2 className="sensor-title">IMU Data</h2>
                {imuData ? (
                    <div className="sensor-section">
                        <h3 className="sensor-subtitle">Orientation</h3>
                        {renderTable(imuData.orientation)}
                        <h3 className="sensor-subtitle">Angular Velocity</h3>
                        {renderTable(imuData.angular_velocity)}
                        <h3 className="sensor-subtitle">Linear Acceleration</h3>
                        {renderTable(imuData.linear_acceleration)}
                    </div>
                ) : (
                    <p>Connecting…</p>
                )}
            </div>

            {/* Magnetometer */}
            <div className="sensor-card">
                <h2 className="sensor-title">Magnetometer Data</h2>
                {magData ? (
                    <div className="sensor-section">
                        <h3 className="sensor-subtitle">Magnetic Field</h3>
                        {renderTable(magData.magnetic_field)}
                    </div>
                ) : (
                    <p>Connecting…</p>
                )}
            </div>

            {/* GPS */}
            <div className="sensor-card">
                <h2 className="sensor-title">GPS Data</h2>
                <pre className="sensor-pre">{gps || 'Connecting…'}</pre>
            </div>

            {/* WiFi */}
            <div className="sensor-card">
                <h2 className="sensor-title">WiFi Data</h2>
                <pre className="sensor-pre">{wifi || 'Connecting…'}</pre>
            </div>
        </div>
    );
};

export default SensorData;