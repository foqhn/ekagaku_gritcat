import React from 'react';
import './LogManager.css';

const LogManager = ({ files, onReload, onDownload }) => {
    const [selectedFile, setSelectedFile] = React.useState('');

    const handleDownload = () => {
        if (selectedFile) {
            onDownload(selectedFile);
        }
    };

    return (
        <div className="log-manager">
            <h3>Log File Manager</h3>
            <div className="log-actions">
                <button onClick={onReload} className="btn-reload">
                    ↻ Reload List
                </button>
                <select
                    value={selectedFile}
                    onChange={(e) => setSelectedFile(e.target.value)}
                    className="file-select"
                >
                    <option value="" disabled>-- Select a file --</option>
                    {files.map((file) => (
                        <option key={file} value={file}>{file}</option>
                    ))}
                </select>
                <button
                    onClick={handleDownload}
                    disabled={!selectedFile}
                    className="btn-download"
                >
                    ⬇ Download
                </button>
            </div>
        </div>
    );
};

export default LogManager;
