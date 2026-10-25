import {defineConfig} from 'vite';
import react from '@vitejs/plugin-react';
export default defineConfig({plugins:[react()],server:{proxy:{'/api':'http://127.0.0.1:8765','/meshes':'http://127.0.0.1:8765','/ws':{target:'ws://127.0.0.1:8765',ws:true}}},build:{rollupOptions:{output:{manualChunks:{three:['three'],charts:['recharts'],editor:['@uiw/react-codemirror','@codemirror/lang-cpp']}}},chunkSizeWarningLimit:1200}});
