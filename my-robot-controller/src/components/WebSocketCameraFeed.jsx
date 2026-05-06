import React from 'react';

/**
 * 映像を表示するだけのコンポーネント
 * @param {string} props.src - 表示する画像のURL (Blob URL)
 * @param {boolean} props.isConnected - 映像ソケットの接続状態
 */
const CameraFeed = ({ src, isConnected }) => {
    const containerStyle = {
        width: '100%',
        maxWidth: '640px',
        minHeight: '480px',
        backgroundColor: '#333',
        border: '1px solid black',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        color: 'white',
        position: 'relative',
        overflow: 'hidden'
    };

    return (
        <div>
            <h2>Robot Camera Feed</h2>
            <div style={containerStyle}>
                {src ? (
                    <img
                        src={src}
                        alt="Robot View"
                        style={{ width: '100%', height: 'auto', display: 'block' }}
                    />
                ) : (
                    <div>{isConnected ? "Loading Frames..." : "Video Disconnected"}</div>
                )}

                <div style={{
                    position: 'absolute', top: '10px', right: '10px',
                    fontSize: '12px', padding: '4px 8px', borderRadius: '4px',
                    backgroundColor: isConnected ? 'green' : 'red'
                }}>
                    {isConnected ? 'LIVE' : 'OFFLINE'}
                </div>
            </div>
        </div>
    );
};

export default CameraFeed;