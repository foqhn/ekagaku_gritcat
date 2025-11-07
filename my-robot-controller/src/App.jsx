import { useState, useEffect } from 'react';
import CameraFeed from './components/CameraFeed'; // 分割した場合
import SensorData from './components/SensorData'; // 分割した場合
import ConnectionManager from './components/ConnectionManager'; // 分割した場合
import Joystick from './components/Joystick';
import './App.css';
import './joystick.css';


function App() {
  // which tool/view is active: 'connect' | 'camera' | 'tools'
  const [activeTool, setActiveTool] = useState('connect');
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

  const isConnected = ws?.readyState === WebSocket.OPEN;

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
    if (!ws) return;

    ws.onmessage = (event) => {
      // event.data は常にテキストとして受信される
      const receivedText = event.data;

      // 1. try...catchブロックで囲む
      try {
        // 2. 受け取ったテキストをJSONとしてパース（変換）を試みる
        const payload = JSON.parse(receivedText);

        // 3. パースに成功した場合のみ、データ構造を検証して処理を続ける
        // これにより、JSONではあるが期待した形式ではないデータも無視できる
        if (payload && typeof payload === 'object' && payload.type === 'sensor_data' && payload.data) {
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
        // 期待した形式のJSONでなければ、何もせず次のメッセージを待つ

      } catch (error) {
        // 4. JSON.parse() が失敗した場合（データがJSON形式でなかった場合）の処理
        // アプリケーションをクラッシュさせずに、エラーをコンソールに表示する
        console.warn("Received non-JSON message, ignoring:", receivedText);
      }
    };

    ws.onclose = () => {
      setConnectionStatus('Not Connected');
      setWs(null);
    };
    ws.onerror = (error) => {
      console.error('WebSocket Error:', error);
      setConnectionStatus('Connection Error');
    };
    
  }, [ws]);


  // --- イベントハンドラ ---
  const handleToggleConnection = () => {
    if (isConnected) {
      ws.close();
    } else {
      if (!selectedRobot) return;
      const url = `ws://${window.location.host}/ws/frontend/${selectedRobot}`;
      const newWs = new WebSocket(url);
      setConnectionStatus('Connecting...');
      newWs.onopen = () => {
        setConnectionStatus(`Connected: ${selectedRobot}`);
        setWs(newWs);
      };
    }
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
    <div className="app-root">
      <aside className="sidebar" aria-label="tool sidebar">
        <button
          className={`tool-btn ${activeTool === 'connect' ? 'active' : ''}`}
          onClick={() => setActiveTool('connect')}
          aria-label="ロボットへの接続"
          title="ロボットへの接続"
        >
          ⛓
          <span className="tooltip">ロボットへの接続</span>
        </button>

        <button
          className={`tool-btn ${activeTool === 'camera' ? 'active' : ''}`}
          onClick={() => setActiveTool('camera')}
          aria-label="カメラとセンサーデータの表示"
          title="カメラ／センサーデータ"
        >
          🎥
          <span className="tooltip">カメラ・センサー</span>
        </button>

        <button
          className={`tool-btn ${activeTool === 'controls' ? 'active' : ''}`}
          onClick={() => setActiveTool('controls')}
          aria-label="コントロール"
          title="コントロール／ジョイスティック"
        >
          🎛
          <span className="tooltip">コントロール</span>
        </button>
      </aside>

      <main className="main-area" >
        {activeTool === 'connect' && (
          <section>
            <h2>ロボットへの接続</h2>
            <ConnectionManager
              robotList={robotList}
              selectedRobot={selectedRobot}
              onSelectChange={setSelectedRobot}
              onToggleConnection={handleToggleConnection}
              status={connectionStatus}
              isConnected={isConnected}
            />
          </section>
        )}

        {activeTool === 'camera' && (
          <section>
            <h2>カメラとセンサーデータ</h2>
            <div style={{ display: 'flex', gap: 16, alignItems: 'flex-start' }}>
              <div style={{ flex: 1 }}>
                <CameraFeed src={cameraSrc} />
              </div>
              <div style={{ width: 360 }}>
                <SensorData imu={imuData} mag={magData} wifi={wifiData} />
              </div>
            </div>
            <h3>ジョイスティックによる操作</h3>
            <div style={{ display: 'flex', gap: 24, alignItems: 'flex-start' }}>
              <div>
                <label htmlFor="speed-slider">Speed: </label>
                <input
                  id="speed-slider"
                  type="range"
                  min="0"
                  max="100"
                  value={speed}
                  onChange={(e) => setSpeed(e.target.value)}
                />
                <span style={{ marginLeft: 8 }}>{speed}</span>
              </div>

              <Joystick onMove={handleJoystickMove} onStop={handleStop} speed={speed} />
            </div>
          </section>

        )}

        {activeTool === 'controls' && (
          <section>
            
          </section>
        )}
      </main>
    </div>
  );
}

export default App;