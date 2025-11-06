import { useState, useEffect } from 'react';
import CameraFeed from './components/CameraFeed'; // 分割した場合
import SensorData from './components/SensorData'; // 分割した場合
import ConnectionManager from './components/ConnectionManager'; // 分割した場合
import Joystick from './components/Joystick';
import './App.css';
import './joystick.css';


function App() {
    // アプリケーション全体で管理する状態
  const [robotList, setRobotList] = useState([]); // ロボットのリスト
  const [selectedRobot, setSelectedRobot] = useState(''); // 選択されたロボットID
  const [connectionStatus, setConnectionStatus] = useState('Not Connected'); // 接続状態
  const [ws, setWs] = useState(null); // WebSocketインスタンス

  // センサーデータ用の状態
  const [cameraSrc, setCameraSrc] = useState(''); // カメラ映像のBase64文字列
  const [imuData, setImuData] = useState('Connecting...');
  const [magData, setMagData] = useState('Connecting...');
  const [wifiData, setWifiData] = useState('Connecting...');
  
  // 制御用の状態
  const [speed, setSpeed] = useState(50);

    // --- 副作用の管理 ---

  // 1. ロボットリストの定期的な取得
  useEffect(() => {
    const fetchRobotList = async () => {
      try {
        const resp = await fetch(`/api/robots`);
        const list = await resp.json();
        setRobotList(list);
        // 初回取得時にリストの先頭を選択状態にする
        if (list.length > 0 && selectedRobot === '') {
          setSelectedRobot(list[0]);
        }
      } catch (e) {
        console.error('Failed to fetch robot list:', e);
      }
    };

    fetchRobotList(); // 初回実行
    const intervalId = setInterval(fetchRobotList, 5000); // 5秒ごとにポーリング

    // クリーンアップ関数: コンポーネントが不要になったらタイマーを止める
    return () => clearInterval(intervalId);
  }, [selectedRobot]); // selectedRobotの初期値設定のために依存配列に含める

  // 2. WebSocketのメッセージハンドリング
  useEffect(() => {
    if (!ws) return; // wsインスタンスがなければ何もしない

    ws.onmessage = (event) => {
      try {
        const payload = JSON.parse(event.data);
        if (payload.type === 'sensor_data' && payload.data) {
          const data = payload.data;
          if (data.image) {
            setCameraSrc('data:image/jpeg;base64,' + data.image);
          }
          if (data.imu) {
            setImuData(JSON.stringify(data.imu, null, 2));
          }
          if (data.mag) {
            setMagData(JSON.stringify(data.mag, null, 2));
          }
          if (data.wifi) {
            setWifiData(JSON.stringify(data.wifi, null, 2));
          }
        }
      } catch (e) {
        console.error('Failed to process message:', e);
      }
    };
    
    // 他のイベントハンドラもここに設定...
    ws.onclose = () => {
        setConnectionStatus('Not Connected');
        setWs(null);
    };
    ws.onerror = () => {
        console.error('WebSocket Error:', error);
        setConnectionStatus('Connection Error');
    };

    // wsインスタンスが変わった時だけこのeffectを再実行
  }, [ws]);


  // --- イベントハンドラ ---
  const handleConnect = () => {
    if (ws) {
      ws.close();
    }
    if (!selectedRobot) return;

    const url = `ws://${window.location.host}/ws/frontend/${selectedRobot}`;
    const newWs = new WebSocket(url);
    setConnectionStatus('Connecting...');
    newWs.onopen = () => {
      setConnectionStatus(`Connected: ${selectedRobot}`);
      setWs(newWs); // WebSocketインスタンスをstateに保存
    };
  };

    // コマンド送信関数
  const sendCommand = (command, left, right) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      const payload = { command, left, right };
      ws.send(JSON.stringify(payload));
    }
  };

  const handleJoystickMove = (left, right) => {
    sendCommand('move', left, right);
  };
  
  const handleStop = () => {
    sendCommand('move', 0, 0);
  };

  return (
    <div style={{ display: 'flex' }}>
      <CameraFeed src={cameraSrc} />
      
      <div id="sidebar">
        <SensorData imu={imuData} mag={magData} wifi={wifiData} />
        <ConnectionManager
          robotList={robotList}
          selectedRobot={selectedRobot}
          onSelectChange={setSelectedRobot}
          onToggleConnection={handleToggleConnection}
          status={connectionStatus}
          isConnected={isConnected}
        />
        <div style={{ margin: '20px' }}>
          <h2>Controls</h2>
          <div>
            <label htmlFor="speed-slider">Speed: </label>
            <input
              type="range"
              min="0"
              max="100"
              value={speed}
              onChange={(e) => setSpeed(e.target.value)}
            />
            <span>{speed}</span>
          </div>
          <Joystick onMove={handleJoystickMove} onStop={handleStop} speed={speed} />
        </div>
      </div>
    </div>
  );
}

export default App;