import React from 'react';
import ReactDOM from 'react-dom/client';
import './index.css';
import LiveStream from './components/LiveStream/index';
import reportWebVitals from './reportWebVitals';

const root = ReactDOM.createRoot(
  document.getElementById('root') as HTMLElement
);
root.render(
  <React.StrictMode>
    <div style={{width:"100%", height:"100%", display:"flex", alignItems:"center"}}>
    <LiveStream />
    </div>
  </React.StrictMode>
);

reportWebVitals();
