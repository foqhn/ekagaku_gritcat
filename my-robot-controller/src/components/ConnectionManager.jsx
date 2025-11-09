import React from 'react';

/**
 * 接続管理UIコンポーネント
 * @param {object} props
 * @param {string[]} props.robotList - ロボットIDの配列
 * @param {string} props.selectedRobot - 現在選択中のロボットID
 * @param {function} props.onSelectChange - select要素の変更時に呼ばれる関数
 * @param {function} props.onToggleConnection - 接続/切断ボタンクリック時に呼ばれる関数
 * @param {string} props.status - 接続状態を示す文字列
 * @param {boolean} props.isConnected - 接続されているかどうか
 */
const ConnectionManager = ({ robotList, selectedRobot, onSelectChange, onToggleConnection, status, isConnected }) => {

    const statusColor = isConnected ? 'green' : 'red';

    return (
        <div style={{ margin: '20px' }}>
        <h3>Connection</h3>
        <div>
            <label htmlFor="robot-select">Robot ID:</label>
            <select 
            id="robot-select" 
            value={selectedRobot} 
            onChange={e => onSelectChange(e.target.value)}
            disabled={isConnected} // 接続中は変更不可にする
            >
            {robotList.length === 0 ? (
                <option value="">-- no robots --</option>
            ) : (
                robotList.map(id => <option key={id} value={id}>{id}</option>)
            )}
            </select>
            
            <button onClick={onToggleConnection}>
            {isConnected ? 'Disconnect' : 'Connect'}
            </button>
            
            <span style={{ color: statusColor, marginLeft: '10px' }}>
            {status}
            </span>
        </div>
        </div>
    );
};

export default ConnectionManager;