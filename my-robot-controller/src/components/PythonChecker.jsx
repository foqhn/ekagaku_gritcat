import React, { useState, useCallback, useEffect } from 'react';

// 初期表示用のサンプルコード
const initialCode = `import math

def calculate_circle_area(radius):
    # 半径が負の場合はエラー
    if radius < 0:
        return "Error: Radius cannot be negative"
    return math.pi * radius ** 2

# 正しい例
print(calculate_circle_area(5))

# わざと構文エラーを起こした例（インデントが間違っている）
  print("This line has incorrect indentation")
`;

function PythonChecker() {
    const [code, setCode] = useState(initialCode);
    const [error, setError] = useState(null);
    // Skulptライブラリがロードされたかを管理するstateを追加
    const [isSkulptReady, setIsSkulptReady] = useState(false);

    // --- ↓↓↓ ここから追加 ↓↓↓ ---
    // コンポーネントのマウント時にSkulptのロード状態を監視する
    useEffect(() => {
        // window.Sk が存在すれば準備完了
        if (window.Sk) {
            setIsSkulptReady(true);
            return;
        }

        // まだロードされていない場合、短い間隔で存在をチェックする
        const intervalId = setInterval(() => {
            if (window.Sk) {
                setIsSkulptReady(true);
                clearInterval(intervalId); // 見つかったら監視を終了
            }
        }, 100); // 100ミリ秒ごとにチェック

        // コンポーネントがアンマウントされる際にクリーンアップ
        return () => clearInterval(intervalId);
    }, []); // 空の依存配列で、初回レンダリング時に一度だけ実行
    // --- ↑↑↑ ここまで追加 ↑↑↑ ---


    // Skulptを使って構文をチェックする関数 (変更なし)
    const checkSyntax = useCallback((sourceCode) => {
        if (!window.Sk) {
            setError("Skulptライブラリがロードされていません。");
            return;
        }

        window.Sk.configure({
            read: (file) => {
                if (window.Sk.builtinFiles === undefined || window.Sk.builtinFiles["files"][file] === undefined) {
                    throw new Error("File not found: '" + file + "'");
                }
                return window.Sk.builtinFiles["files"][file];
            }
        });

        try {
            window.Sk.compile(sourceCode, "your_code.py", "exec");
            setError('');
        } catch (err) {
            setError(err.toString());
        }
    }, []);


    const handleCodeChange = (e) => {
        setCode(e.target.value);
    };

    const handleCheckClick = () => {
        checkSyntax(code);
    };

    const renderResult = () => {
        if (error === null) {
            return (
                <p style={{ color: '#555' }}>
                    「チェック実行」ボタンを押して、コードの構文を確認してください。
                </p>
            );
        }
        if (error) {
            return (
                <pre style={{ color: 'red', backgroundColor: '#fff0f0', padding: '15px', borderRadius: '5px', whiteSpace: 'pre-wrap', wordWrap: 'break-word' }}>
                    {error}
                </pre>
            );
        }
        return (
            <p style={{ color: 'green', fontWeight: 'bold' }}>
                ✅ 構文エラーは見つかりませんでした。
            </p>
        );
    };

    return (
        <div style={{ width: '95%', margin: 'auto', fontFamily: 'sans-serif' }}>
            <textarea
                value={code}
                onChange={handleCodeChange}
                rows={45}
                style={{ height: '100%', width: '100%', overflowY: 'scroll', fontFamily: 'monospace', fontSize: '16px', border: '1px solid #ccc', padding: '10px' }}
                spellCheck="false"
            />
            <button
                onClick={handleCheckClick}
                // isSkulptReadyがfalseの間はボタンを無効化する
                disabled={!isSkulptReady}
                style={{
                    marginTop: '1rem',
                    padding: '10px 20px',
                    fontSize: '16px',
                    color: 'white',
                    border: 'none',
                    borderRadius: '5px'
                }}
            >
                {'チェック実行'}
            </button>
            <div style={{ marginTop: '1rem' }}>
                {renderResult()}
            </div>
        </div>
    );
}

export default PythonChecker;