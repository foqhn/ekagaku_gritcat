import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
    plugins: [react()],
    server: {
        proxy: {
            '/api': {
                target: 'https://ekagaku-robot.onrender.com',
                changeOrigin: true,
                secure: false,
            },
            '/ws': {
                target: 'wss://ekagaku-robot.onrender.com',
                ws: true,
                changeOrigin: true,
                secure: false,
            },
        },
    },
});
