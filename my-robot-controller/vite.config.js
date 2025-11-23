import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      // '/api' という文字列で始まるリクエストをプロキシの対象にする
      '/api': {
        // 転送先のバックエンドサーバーのURL
        target: 'wss://ekagaku-robot.onrender.com/ws/robot',
        // オリジンを書き換える
        changeOrigin: true,
      },
      // WebSocketのリクエストもプロキシする場合
      '/ws': {
        target: 'wss://ekagaku-robot.onrender.com/ws/robot',
        ws: true,
      },
    }
  }
})
