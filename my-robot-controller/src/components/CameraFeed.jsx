import React, { useRef, useEffect } from 'react';

/**
 * カメラ映像を表示するコンポーネント (WebRTC対応版)
 * @param {object} props
 * @param {MediaStream} props.stream - WebRTC経由で受け取ったメディアストリーム
 */
const CameraFeed = ({ stream }) => {
    const videoRef = useRef(null);

    // ストリームが更新されたときにビデオ要素に紐付ける
    useEffect(() => {
        if (videoRef.current && stream) {
            videoRef.current.srcObject = stream;
        }
    }, [stream]);

    return (
        <div>
            <h2>Camera (WebRTC)</h2>
            {stream ? (
                <video
                    ref={videoRef}
                    id="camera"
                    autoPlay       // 自動再生
                    playsInline    // iOSなどで全画面表示されないようにする
                    muted          // 自動再生を許可するために通常はミュートにする
                    style={{
                        width: '100%',
                        maxWidth: '640px',
                        minHeight: '480px',
                        backgroundColor: '#333',
                        border: '1px solid black'
                    }}
                />
            ) : (
                <div
                    id="camera-placeholder"
                    style={{
                        width: '100%',
                        maxWidth: '640px',
                        minHeight: '480px',
                        backgroundColor: '#333',
                        border: '1px solid black',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        color: 'white'
                    }}
                >
                    Waiting for WebRTC Stream...
                </div>
            )}
        </div>
    );
};

export default CameraFeed;