// One-port HTTP + WebSocket signaling + static client
// Run: PORT=8082 node server.js
const http = require('http');
const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');

const PORT = process.env.PORT || 8082;
const publicDir = path.join(__dirname, 'public');

const server = http.createServer((req, res) => {
  if (req.url === '/health') {
    res.writeHead(200, { 'Content-Type': 'text/plain' });
    return res.end('ok');
  }
  // Serve index.html and static files
  let filePath = req.url === '/' ? path.join(publicDir, 'index.html') : path.join(publicDir, req.url);
  // Prevent directory traversal
  if (!filePath.startsWith(publicDir)) {
    res.writeHead(403); return res.end('Forbidden');
  }
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404); return res.end('Not found');
    }
    const ext = path.extname(filePath);
    const contentType = ({
      '.html': 'text/html',
      '.js': 'application/javascript',
      '.css': 'text/css',
      '.map': 'application/json'
    })[ext] || 'application/octet-stream';
    res.writeHead(200, {
      'Content-Type': contentType,
      'X-Content-Type-Options': 'nosniff'
    });
    res.end(data);
  });
});

const wss = new WebSocket.Server({ server });

wss.on('connection', (ws) => {
  console.log('WS client connected, total:', wss.clients.size);

  // Notify 'ready' to client other to resend offer
  wss.clients.forEach((client) => {
    if (client !== ws && client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify({ type: 'ready' }));
    }
  });

  ws.on('message', (msg) => {
    // Force package relay to text - websocketpp ROS
    const txt = (typeof msg === 'string') ? msg : msg.toString('utf8');
    try {
      const obj = JSON.parse(txt);
      console.log('[relay]', obj.type || 'unknown');
    } catch {
      console.log('[relay] non-json');
    }
    // Relay to others client
    wss.clients.forEach((client) => {
      if (client !== ws && client.readyState === WebSocket.OPEN) {
        client.send(txt);
      }
    });
  });

  ws.on('close', () => console.log('WS client disconnected'));
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`HTTP+WS signaling server on http://0.0.0.0:${PORT}`);
  console.log(`Open from browser: http://<PI_IP>:${PORT}/`);
});
