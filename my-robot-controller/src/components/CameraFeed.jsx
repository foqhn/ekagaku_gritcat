import React from 'react';

/**
 * カメラ映像を表示するコンポーネント
 * @param {object} props
 * @param {string} props.src - Base64エンコードされた画像データURL
 */
const CameraFeed = ({ src }) => {
    return (
        <div>
        <h2>Camera</h2>
        {src ? (
            <img 
            id="camera" 
            src={src} 
            alt="Camera Stream" 
            style={{ minWidth: '640px', minHeight: '480px', backgroundColor: '#333', border: '1px solid black' }}
            />
        ) : (
            <div 
            id="camera-placeholder"
            style={{ 
                minWidth: '640px', 
                minHeight: '480px', 
                backgroundColor: '#333', 
                border: '1px solid black',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: 'white'
            }}
            >
            No Image Stream
            </div>
        )}
        </div>
    );
};

export default CameraFeed;