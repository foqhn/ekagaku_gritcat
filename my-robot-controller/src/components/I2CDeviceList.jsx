import React from 'react';

const I2CDeviceList = ({ devices }) => {
    if (!devices || devices.length === 0) {
        return (
            <div className="panel-container" style={{ padding: '15px', marginTop: '20px' }}>
                <h3 style={{ margin: '0 0 10px 0', color: '#94a3b8', fontSize: '1rem' }}>Detected I2C Devices</h3>
                <p style={{ color: '#64748b', fontSize: '0.9rem' }}>No devices detected or scanning...</p>
            </div>
        );
    }

    return (
        <div className="panel-container" style={{ padding: '15px', marginTop: '20px' }}>
            <h3 style={{ margin: '0 0 10px 0', color: '#94a3b8', fontSize: '1rem' }}>Detected I2C Devices</h3>
            <ul style={{ listStyle: 'none', padding: 0, margin: 0 }}>
                {devices.map((dev, index) => (
                    <li key={index} style={{
                        display: 'flex',
                        justifyContent: 'space-between',
                        padding: '8px',
                        borderBottom: index < devices.length - 1 ? '1px solid #334155' : 'none',
                        color: '#e2e8f0',
                        fontSize: '0.9rem'
                    }}>
                        <span style={{ fontFamily: 'monospace', color: '#a5b4fc' }}>{dev.address}</span>
                        <span>{dev.name}</span>
                    </li>
                ))}
            </ul>
        </div>
    );
};

export default I2CDeviceList;
