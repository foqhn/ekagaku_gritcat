import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
    plugins: [react()],
    server: {
        proxy: {
            '/api': {
                //target: 'https://ekagaku-robot.onrender.com',
                target: 'http://192.168.11.14:8000',
                changeOrigin: true,
                secure: false,
            },
            '/ws': {
                //target: 'wss://ekagaku-robot.onrender.com',
                target: 'ws://192.168.11.14:8000',
                ws: true,
                changeOrigin: true,
                secure: false,
            },
        },
    },
});
