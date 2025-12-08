import { useState, useEffect, useRef } from 'react';
import CameraFeed from './components/CameraFeed';
import SensorData from './components/SensorData';
import ConnectionManager from './components/ConnectionManager';
import Joystick from './components/Joystick';
import WifiSignal from './components/WifiSignal';
import BlockEditor from './components/BlockEditor';
import SensorControls from './components/SensorControls';
import LogManager from './components/LogManager';
import './App.css';

function App() {
  // UI navigation state
  const [activeTool, setActiveTool] = useState('connect'); // 'connect' | 'camera' | 'controls'

  // Robot selection and connection state
  const [robotList, setRobotList] = useState([]);
  const [selectedRobot, setSelectedRobot] = useState('');
  const [connectionStatus, setConnectionStatus] = useState('Not Connected');
  const [ws, setWs] = useState(null);
  const isConnected = ws?.readyState === WebSocket.OPEN;

  // Sensor data state
  const [cameraSrc, setCameraSrc] = useState('');
  const [imuData, setImuData] = useState('Connecting...');
  const [magData, setMagData] = useState('Connecting...');
  const [wifiData, setWifiData] = useState('Connecting...');
  const [gpsData, setGpsData] = useState('Connecting...');
  const [bmeData, setBmeData] = useState('Connecting...');

  // Sensor ON/OFF tracking (optimistic UI)
  const [sensorStates, setSensorStates] = useState({ cam: false, imu: false, gps: false, bme: false });

  // Tracks if user manually turned OFF a sensor to prevent auto-on
  // Using useRef to avoid stale closure issues in ws.onmessage
  const manualOverrideRef = useRef({ cam: false, imu: false, gps: false, bme: false });

  // Ref to store timeout IDs for each sensor to reset state after inactivity
  const timersRef = useRef({});

  // Log management state
  const [logFiles, setLogFiles] = useState([]);

  // Control state
  const [speed, setSpeed] = useState(50);

  // System logs
  const [consoleLogs, setConsoleLogs] = useState([
    { msg: 'System initialized.', type: 'info' },
    { msg: 'Ready to connect.', type: 'info' }
  ]);
  const addLog = (msg, type = 'info') => {
    setConsoleLogs(prev => [...prev.slice(-4), { msg, type }]);
  };

  // Block editor generated code
  const [generatedCode, setGeneratedCode] = useState('');
  const [blocklyState, setBlocklyState] = useState(null);

  // Periodically fetch robot list
  useEffect(() => {
    const fetchRobotList = async () => {
      try {
        // User requested to use the remote base URL directly
        const resp = await fetch('http://192.168.11.14:8000/api/robots');
        if (!resp.ok) {
          throw new Error(`API Error: ${resp.status}`);
        }
        const list = await resp.json();

        if (Array.isArray(list)) {
          setRobotList(list);
          if (list.length > 0 && selectedRobot === '') {
            setSelectedRobot(list[0]);
          }
        } else {
          console.warn('Invalid robot list format:', list);
          addLog('Received invalid robot list format.', 'error');
          setRobotList([]); // Fallback to empty array to prevent crash
        }
      } catch (e) {
        console.error('Failed to fetch robot list:', e);
        // Do not spam logs on every interval failure, just warn in console
      }
    };
    fetchRobotList();
    const intervalId = setInterval(fetchRobotList, 5000);
    return () => clearInterval(intervalId);
  }, [selectedRobot]);

  // WebSocket message handling
  useEffect(() => {
    if (!ws) return;
    ws.onmessage = (event) => {
      try {
        const payload = JSON.parse(event.data);
        if (payload && typeof payload === 'object') {
          if (payload.type === 'sensor_data' && payload.data) {
            const data = payload.data;

            // Helper to handle sensor data updates
            const handleSensorUpdate = (type, updateFn) => {
              updateFn();
              // Only turn ON if NOT manually overridden to OFF
              if (!manualOverrideRef.current[type]) {
                setSensorStates(prev => ({ ...prev, [type]: true }));

                // Reset auto-off timer
                if (timersRef.current[type]) clearTimeout(timersRef.current[type]);
                timersRef.current[type] = setTimeout(() => {
                  setSensorStates(prev => ({ ...prev, [type]: false }));
                  delete timersRef.current[type];
                }, 10000);
              }
            };

            if (data.image) {
              handleSensorUpdate('cam', () => setCameraSrc('data:image/jpeg;base64,' + data.image));
            }
            if (data.imu) {
              handleSensorUpdate('imu', () => setImuData(JSON.stringify(data.imu, null, 2)));
            }
            if (data.mag) {
              setMagData(JSON.stringify(data.mag, null, 2));
            }
            if (data.wifi) {
              setWifiData(JSON.stringify(data.wifi, null, 2));
            }
            if (data.gps) {
              handleSensorUpdate('gps', () => setGpsData(JSON.stringify(data.gps, null, 2)));
            }
            if (data.bme280) {
              handleSensorUpdate('bme', () => setBmeData(JSON.stringify(data.bme280, null, 2)));
            }
          } else if (payload.type === 'log_file_list' && payload.files) {
            setLogFiles(payload.files);
          } else if (payload.type === 'log_file_content' && payload.filename && payload.data) {
            triggerFileDownload(payload.filename, payload.data);
          }
        }
      } catch (error) {
        console.warn('Received non-JSON message, ignoring:', event.data);
      }
    };
    ws.onclose = () => {
      setConnectionStatus('Not Connected');
      setWs(null);
      setLogFiles([]);
      // Do NOT reset sensor states or overrides on disconnect to preserve user intent
    };
    ws.onerror = (error) => {
      console.error('WebSocket Error:', error);
      setConnectionStatus('Connection Error');
      addLog('WebSocket Connection Error', 'error');
    };
  }, [ws]);

  // Cleanup timers on component unmount
  useEffect(() => {
    return () => {
      Object.values(timersRef.current).forEach(id => clearTimeout(id));
    };
  }, []);

  // Event handlers
  const handleToggleConnection = () => {
    if (isConnected) {
      ws.close();
    } else {
      if (!selectedRobot) return;
      // User requested to use the remote base URL directly
      //const url = `wss://ekagaku-robot.onrender.com/ws/frontend/${selectedRobot}`;//リモート
      const url = `ws://192.168.11.14:8000/ws/frontend/${selectedRobot}`;//ローカル
      const newWs = new WebSocket(url);
      setConnectionStatus('Connecting...');
      newWs.onopen = () => {
        setConnectionStatus(`Connected: ${selectedRobot}`);
        addLog(`Connected to ${selectedRobot}`, 'success');
        setWs(newWs);
        // Do NOT reset sensor states or overrides on connect to preserve user intent
      };
    }
  };

  const sendCommand = (command, left, right) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command, left, right }));
    }
  };

  const handleJoystickMove = (left, right) => sendCommand('move', left, right);
  const handleStop = () => sendCommand('move', 0, 0);

  // Optimistic UI update for sensor toggle (manual override)
  const handleToggleSensor = (type, isChecked) => {
    setSensorStates(prev => ({ ...prev, [type]: isChecked }));

    if (isChecked) {
      // User turned ON: Allow auto-off logic to work, so clear override
      manualOverrideRef.current[type] = false;
      // Start auto-off timer in case no data comes
      if (timersRef.current[type]) clearTimeout(timersRef.current[type]);
      timersRef.current[type] = setTimeout(() => {
        setSensorStates(prev => ({ ...prev, [type]: false }));
        delete timersRef.current[type];
      }, 10000);
    } else {
      // User turned OFF: Set override to TRUE to prevent data from turning it back on
      manualOverrideRef.current[type] = true;
      if (timersRef.current[type]) {
        clearTimeout(timersRef.current[type]);
        delete timersRef.current[type];
      }
    }

    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command: 'sensor', sensor_type: type, bin: isChecked ? 1 : 0 }));
    }
  };

  const handleToggleLog = (isChecked) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command: 'log', msg: isChecked ? 1 : 0 }));
    }
  };

  const handleReloadLogs = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command: 'list_log_files' }));
    } else {
      alert('Not connected to robot.');
    }
  };

  const handleDownloadLog = (filename) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command: 'get_log_file', filename }));
    }
  };

  const triggerFileDownload = (filename, csvContent) => {
    const bom = new Uint8Array([0xef, 0xbb, 0xbf]);
    const blob = new Blob([bom, csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    const url = URL.createObjectURL(blob);
    link.setAttribute('href', url);
    link.setAttribute('download', filename);
    link.style.visibility = 'hidden';
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  const handleSavePython = () => {
    const blob = new Blob([generatedCode], { type: 'text/plain;charset=utf-8' });
    const link = document.createElement('a');
    const url = URL.createObjectURL(blob);
    link.setAttribute('href', url);
    link.setAttribute('download', 'robot_program.py');
    link.style.visibility = 'hidden';
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  const handleSaveCode = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command: 'save_code', code: generatedCode }));
      addLog('Program saved to robot.', 'success');
    } else {
      alert('Not connected to robot.');
    }
  };

  const handleRunProgram = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command: 'start_program' }));
      addLog('Program started.', 'success');
    } else {
      alert('Not connected to robot.');
    }
  };

  const handleStopProgram = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ command: 'stop_program' }));
      addLog('Program stopped.', 'warning');
    } else {
      alert('Not connected to robot.');
    }
  };

  // UI rendering
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

      <main className="main-area">
        <div style={{ flex: '1', overflow: 'hidden', display: 'flex', flexDirection: 'column' }}>
          {activeTool === 'connect' && (
            <section style={{ height: '100%', overflow: 'hidden' }}>
              <div className="robot-ctl">
                <div className="programming">
                  <div className="prog-menu">
                    <button onClick={handleSaveCode} style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                      <span>💾</span> 書き込み
                    </button>
                    <button onClick={handleSavePython} style={{ display: 'flex', alignItems: 'center', gap: '8px', backgroundColor: '#3b82f6', color: '#fff' }}>
                      <span>🐍</span> Python保存
                    </button>
                    <button onClick={handleRunProgram} style={{ display: 'flex', alignItems: 'center', gap: '8px', backgroundColor: '#22c55e', color: '#fff' }}>
                      <span>▶</span> 実行
                    </button>
                    <button onClick={handleStopProgram} style={{ display: 'flex', alignItems: 'center', gap: '8px', backgroundColor: '#ef4444', color: '#fff' }}>
                      <span>⏹</span> 停止
                    </button>
                  </div>
                  <div className="prog-editzone">
                    <div style={{ flex: 1, position: 'relative' }}>
                      <BlockEditor
                        onCodeChange={setGeneratedCode}
                        initialState={blocklyState}
                        onStateChange={setBlocklyState}
                      />
                    </div>
                    <div style={{ height: '120px', marginTop: '12px', display: 'flex', flexDirection: 'column' }}>
                      <h3 style={{ fontSize: '14px', marginBottom: '8px', color: '#94a3b8' }}>Generated Python Code</h3>
                      <textarea
                        value={generatedCode}
                        readOnly
                        style={{ flex: 1, width: '100%', boxSizing: 'border-box' }}
                      />
                    </div>
                  </div>
                </div>
                <div className="robot-info">
                  <div className="connection-instructions">
                    <p>
                      ロボットに接続するには、まずロボットの電源を入れ、WiFiネットワークに接続してください。
                      次に、以下のドロップダウンメニューから接続したいロボットのIDを選択し、「Connect」ボタンをクリックします。
                    </p>
                  </div>
                  <ConnectionManager
                    robotList={robotList}
                    selectedRobot={selectedRobot}
                    onSelectChange={setSelectedRobot}
                    onToggleConnection={handleToggleConnection}
                    status={connectionStatus}
                    isConnected={isConnected}
                  />
                  {isConnected && (
                    <section style={{ marginTop: 20, padding: 0 }}>
                      <div className="wifi-signal">
                        <h3>WiFi Signal Strength</h3>
                        <WifiSignal wifi={wifiData} />
                      </div>
                    </section>
                  )}
                </div>
              </div>
            </section>
          )}

          {activeTool === 'camera' && (
            <section style={{ height: '100%', overflowY: 'auto', padding: '24px' }}>
              <div style={{ display: 'grid', gridTemplateColumns: '2fr 1fr', gap: '24px', alignItems: 'start' }}>
                {/* Left Column */}
                <div style={{ display: 'flex', flexDirection: 'column', gap: '24px' }}>
                  <div className="panel-container">
                    <CameraFeed src={cameraSrc} />
                  </div>
                  <div className="panel-container" style={{ padding: '20px' }}>
                    <h3 style={{ marginTop: 0, marginBottom: '16px', color: '#94a3b8' }}>Manual Control</h3>
                    <div style={{ display: 'flex', gap: 24, alignItems: 'center' }}>
                      <div style={{ flex: 1 }}>
                        <label htmlFor="speed-slider" style={{ display: 'block', marginBottom: '8px', color: '#e2e8f0' }}>Speed: {speed}</label>
                        <input
                          id="speed-slider"
                          type="range"
                          min="0"
                          max="100"
                          value={speed}
                          onChange={(e) => setSpeed(e.target.value)}
                          style={{ width: '100%' }}
                        />
                      </div>
                      <Joystick onMove={handleJoystickMove} onStop={handleStop} speed={speed} />
                    </div>
                  </div>
                </div>
                {/* Right Column */}
                <div style={{ display: 'flex', flexDirection: 'column', gap: '24px' }}>
                  <SensorControls
                    onToggleSensor={handleToggleSensor}
                    onToggleLog={handleToggleLog}
                    sensorStates={sensorStates}
                  />
                  <LogManager
                    files={logFiles}
                    onReload={handleReloadLogs}
                    onDownload={handleDownloadLog}
                  />
                  <div className="panel-container">
                    <SensorData imu={imuData} mag={magData} wifi={wifiData} gps={gpsData} bme={bmeData} />
                  </div>
                </div>
              </div>
            </section>
          )}

          {activeTool === 'controls' && (
            <section>
              <h2>コントロール</h2>
              <p>コントロール機能は現在開発中です。</p>
            </section>
          )}
        </div>
        <div className="console-window">
          {consoleLogs.map((log, i) => (
            <div key={i} className={`log-entry log-${log.type}`}>&gt; {log.msg}</div>
          ))}
        </div>
      </main>
    </div>
  );
}

export default App;
