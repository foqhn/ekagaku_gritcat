import React from 'react';
import './LogManager.css';

const LogManager = ({ files, onReload, onDownload }) => {
    const [selectedFile, setSelectedFile] = React.useState('');

    const collator = React.useMemo(() => {
        // "一般的なファイル名の自然順ソート"に寄せる:
        // - numeric: "file2" < "file10" のように数字を数値として扱う
        // - sensitivity: 'base' 大文字小文字等の差を弱める
        return new Intl.Collator(undefined, { numeric: true, sensitivity: 'base' });
    }, []);

    const sortedFiles = React.useMemo(() => {
        const list = Array.isArray(files) ? files : [];
        const decorated = list.map((file, idx) => ({ file, idx }));

        decorated.sort((a, b) => {
            // 名前で降順（新しそうなものが上に来やすい）に自然順ソート
            // 例: 2026... > 2025..., log_9 > log_10 にならない（numericで解消）
            const byName = collator.compare(String(b.file), String(a.file));
            return byName !== 0 ? byName : a.idx - b.idx; // 安定ソート
        });

        return decorated.map((d) => d.file);
    }, [files, collator]);

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
                    {sortedFiles.map((file) => (
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
