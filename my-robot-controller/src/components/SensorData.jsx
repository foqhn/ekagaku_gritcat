import React from 'react';

/**
 * センサーデータをカード形式で表示するコンポーネント
 * @param {object} props
 * @param {string|object} props.imu - IMU データ（JSON 文字列またはオブジェクト）
 * @param {string|object} props.mag - 磁気データ（JSON 文字列またはオブジェクト）
 * @param {string|object} props.bme - BME280 データ（JSON 文字列またはオブジェクト）
 * @param {string|number} props.compass - コンパスデータ（0-360の数値）
 * @param {string} props.wifi - WiFi データ文字列
 * @param {string} props.gps - GPS データ文字列
 */
const SensorData = ({ imu, mag, wifi, gps, bme, compass }) => {
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
    const bmeData = safeParse(bme);
    const gpsData = safeParse(gps);

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
            {/* IMU & Compass */}
            <div className="sensor-card">
                <h2 className="sensor-title">IMU & Compass Data</h2>
                {(imuData || magData || compass !== undefined) ? (
                    <div className="sensor-section">
                        {compass !== undefined && compass !== null && (
                            <>
                                <h3 className="sensor-subtitle">Compass</h3>
                                <table className="sensor-table">
                                    <tbody>
                                        <tr>
                                            <td className="sensor-key">Heading</td>
                                            <td className="sensor-value">{Number(compass).toFixed(2)}°</td>
                                        </tr>
                                    </tbody>
                                </table>
                            </>
                        )}
                        {imuData && (
                            <>
                                <h3 className="sensor-subtitle">Angular Velocity</h3>
                                {renderTable(imuData.angular_velocity)}
                                <h3 className="sensor-subtitle">Linear Acceleration</h3>
                                {renderTable(imuData.linear_acceleration)}
                            </>
                        )}
                        {magData && (
                            <>
                                <h3 className="sensor-subtitle">Magnetic Field</h3>
                                {renderTable(magData.magnetic_field)}
                            </>
                        )}
                    </div>
                ) : (
                    <p>Connecting…</p>
                )}
            </div>

            {/* GPS */}
            <div className="sensor-card">
                <h2 className="sensor-title">GPS Data</h2>
                {gpsData ? (
                    <div className="sensor-section">
                        <table className="sensor-table">
                            <tbody>
                                <tr>
                                    <td className="sensor-key">Latitude</td>
                                    <td className="sensor-value">{gpsData.latitude?.toFixed(6)}</td>
                                </tr>
                                <tr>
                                    <td className="sensor-key">Longitude</td>
                                    <td className="sensor-value">{gpsData.longitude?.toFixed(6)}</td>
                                </tr>
                                <tr>
                                    <td className="sensor-key">Altitude</td>
                                    <td className="sensor-value">{gpsData.altitude?.toFixed(2)} m</td>
                                </tr>
                                <tr>
                                    <td className="sensor-key">Status</td>
                                    <td className="sensor-value">{gpsData.status?.status === 0 ? 'Fix' : 'No Fix'}</td>
                                </tr>
                                {gpsData.position_covariance && (
                                    <>
                                        <tr>
                                            <td className="sensor-key">Horizontal Error</td>
                                            <td className="sensor-value">
                                                {Math.sqrt(gpsData.position_covariance[0] + gpsData.position_covariance[4]).toFixed(2)} m
                                            </td>
                                        </tr>
                                        <tr>
                                            <td className="sensor-key">Vertical Error</td>
                                            <td className="sensor-value">
                                                {Math.sqrt(gpsData.position_covariance[8]).toFixed(2)} m
                                            </td>
                                        </tr>
                                    </>
                                )}
                            </tbody>
                        </table>
                    </div>
                ) : (
                    <p>Connecting…</p>
                )}
            </div>

            {/* Environment (BME280) */}
            <div className="sensor-card">
                <h2 className="sensor-title">Environment Data</h2>
                {bmeData ? (
                    <div className="sensor-section">
                        <table className="sensor-table">
                            <tbody>
                                <tr>
                                    <td className="sensor-key">Temperature</td>
                                    <td className="sensor-value">{bmeData.temperature_celsius} °C</td>
                                </tr>
                                <tr>
                                    <td className="sensor-key">Humidity</td>
                                    <td className="sensor-value">{bmeData.humidity_percent} %</td>
                                </tr>
                                <tr>
                                    <td className="sensor-key">Pressure</td>
                                    <td className="sensor-value">{bmeData.pressure_hpa} hPa</td>
                                </tr>
                            </tbody>
                        </table>
                    </div>
                ) : (
                    <p>Connecting…</p>
                )}
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