import React from 'react';
import './SensorControls.css';

const SensorControls = ({ onToggleSensor, onToggleLog, sensorStates }) => {
    const { cam = false, imu = false, gps = false, bme = false } = sensorStates || {};
    return (
        <div className="sensor-controls">
            <h3>Sensor Controls</h3>
            <div className="control-group">
                <div className="control-item">
                    <span>Camera</span>
                    <label className="toggle-switch">
                        <input
                            type="checkbox"
                            checked={cam}
                            onChange={(e) => onToggleSensor('cam', e.target.checked)}
                        />
                        <span className="slider round"></span>
                    </label>
                </div>
                <div className="control-item">
                    <span>IMU</span>
                    <label className="toggle-switch">
                        <input
                            type="checkbox"
                            checked={imu}
                            onChange={(e) => onToggleSensor('imu', e.target.checked)}
                        />
                        <span className="slider round"></span>
                    </label>
                </div>
                <div className="control-item">
                    <span>GPS</span>
                    <label className="toggle-switch">
                        <input
                            type="checkbox"
                            checked={gps}
                            onChange={(e) => onToggleSensor('gps', e.target.checked)}
                        />
                        <span className="slider round"></span>
                    </label>
                </div>
                <div className="control-item">
                    <span>Environment</span>
                    <label className="toggle-switch">
                        <input
                            type="checkbox"
                            checked={bme}
                            onChange={(e) => onToggleSensor('bme', e.target.checked)}
                        />
                        <span className="slider round"></span>
                    </label>
                </div>
            </div>

            <hr className="divider" />

            <div className="control-group">
                <div className="control-item">
                    <span>LOG Recording</span>
                    <label className="toggle-switch">
                        <input
                            type="checkbox"
                            onChange={(e) => onToggleLog(e.target.checked)}
                        />
                        <span className="slider round log-slider"></span>
                    </label>
                </div>
            </div>
        </div>
    );
};

export default SensorControls;
