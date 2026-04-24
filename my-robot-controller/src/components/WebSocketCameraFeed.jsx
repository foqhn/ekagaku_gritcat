import React, { useState, useEffect, useRef } from 'react';

/**
 * カメラ映像を表示するコンポーネント (WebSocketバイナリ版)
 * @param {string} props.robotId - 接続するロボットのID
 * @param {string} props.serverIp - サーバーのIPとポート (例: "192.168.11.127:8000")
 */
const WebSocketCameraFeed = ({ robotId, serverIp }) => {
    const [imageUrl, setImageUrl] = useState(null);
    const [isConnected, setIsConnected] = useState(false);
    const prevUrlRef = useRef(null);
    const socketRef = useRef(null);

    useEffect(() => {
        if (!robotId || !serverIp) return;

        // 映像専用のWebSocketに接続
        const wsUrl = `ws://${serverIp}/ws/frontend/video/${robotId}`;
        const ws = new WebSocket(wsUrl);
        ws.binaryType = 'blob'; // バイナリ（Blob）として受け取る設定
        socketRef.current = ws;

        ws.onopen = () => setIsConnected(true);
        ws.onclose = () => {
            setIsConnected(false);
            setImageUrl(null);
        };

        ws.onmessage = (event) => {
            if (event.data instanceof Blob) {
                // 受信したバイナリデータをURLに変換
                const newUrl = URL.createObjectURL(event.data);

                setImageUrl(newUrl);

                // メモリリーク防止：古いURLオブジェクトを解放
                if (prevUrlRef.current) {
                    URL.revokeObjectURL(prevUrlRef.current);
                }
                prevUrlRef.current = newUrl;
            }
        };

        // クリーンアップ：コンポーネント終了時にソケットを閉じる
        return () => {
            if (socketRef.current) socketRef.current.close();
            if (prevUrlRef.current) URL.revokeObjectURL(prevUrlRef.current);
        };
    }, [robotId, serverIp]);

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
            <h2>Camera (WebSocket Stream)</h2>
            <div style={containerStyle}>
                {imageUrl ? (
                    <img
                        src={imageUrl}
                        alt="Robot Feed"
                        style={{
                            width: '100%',
                            height: 'auto',
                            display: 'block'
                        }}
                    />
                ) : (
                    <div>
                        {isConnected ? "Loading Frames..." : "Waiting for Video Socket..."}
                    </div>
                )}

                {/* 接続状態のインジケーター（任意） */}
                <div style={{
                    position: 'absolute',
                    top: '10px',
                    right: '10px',
                    fontSize: '12px',
                    padding: '4px 8px',
                    borderRadius: '4px',
                    backgroundColor: isConnected ? 'green' : 'red'
                }}>
                    {isConnected ? 'LIVE' : 'OFFLINE'}
                </div>
            </div>
        </div>
    );
};

export default WebSocketCameraFeed;