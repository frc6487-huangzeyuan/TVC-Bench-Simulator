import {defineConfig} from 'vite';
import react from '@vitejs/plugin-react';
export default defineConfig({plugins:[react()],server:{proxy:{'/api':'http://127.0.0.1:18767','/meshes':'http://127.0.0.1:18767','/device-console':'http://127.0.0.1:18767','/ws':{target:'ws://127.0.0.1:18767',ws:true}}},build:{rollupOptions:{output:{manualChunks:{three:['three'],charts:['recharts'],editor:['@uiw/react-codemirror','@codemirror/lang-cpp']}}},chunkSizeWarningLimit:1200}});
