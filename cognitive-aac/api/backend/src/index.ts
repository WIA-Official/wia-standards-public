/**
 * WIA Cognitive AAC - Backend API Server
 * 케어기버/전문가 통합 백엔드 서버
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import rateLimit from 'express-rate-limit';
import { Server as WebSocketServer } from 'ws';
import { createServer } from 'http';
import routes from './routes';

// ============================================================================
// Configuration
// ============================================================================

const PORT = process.env.PORT ?? 3001;
const NODE_ENV = process.env.NODE_ENV ?? 'development';

// ============================================================================
// Express App Setup
// ============================================================================

const app = express();

// Security middleware
app.use(helmet());
app.use(cors({
  origin: NODE_ENV === 'production'
    ? ['https://your-domain.com']
    : ['http://localhost:3000', 'http://localhost:3001'],
  credentials: true,
}));

// Rate limiting
const limiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15 minutes
  max: 100, // limit each IP to 100 requests per windowMs
  message: { error: '요청이 너무 많습니다. 잠시 후 다시 시도해 주세요.' },
});
app.use(limiter);

// Body parsing
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true }));

// Request logging
app.use((req, res, next) => {
  console.log(`[${new Date().toISOString()}] ${req.method} ${req.path}`);
  next();
});

// API routes
app.use('/api/v1', routes);

// Error handling
app.use((err: Error, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error('Error:', err.message);
  res.status(500).json({
    error: NODE_ENV === 'production' ? '서버 오류가 발생했습니다' : err.message,
  });
});

// 404 handler
app.use((req, res) => {
  res.status(404).json({ error: '요청한 리소스를 찾을 수 없습니다' });
});

// ============================================================================
// HTTP & WebSocket Server
// ============================================================================

const server = createServer(app);
const wss = new WebSocketServer({ server, path: '/ws' });

// WebSocket connection handling
wss.on('connection', (ws, req) => {
  const clientId = new URL(req.url ?? '', 'http://localhost').searchParams.get('clientId');
  console.log(`WebSocket connected: ${clientId}`);

  ws.on('message', (data) => {
    try {
      const message = JSON.parse(data.toString());
      handleWebSocketMessage(ws, clientId, message);
    } catch (error) {
      ws.send(JSON.stringify({ type: 'error', message: '잘못된 메시지 형식입니다' }));
    }
  });

  ws.on('close', () => {
    console.log(`WebSocket disconnected: ${clientId}`);
  });

  // Welcome message
  ws.send(JSON.stringify({
    type: 'connected',
    clientId,
    timestamp: new Date().toISOString(),
  }));
});

// WebSocket message handler
function handleWebSocketMessage(
  ws: import('ws').WebSocket,
  clientId: string | null,
  message: { type: string; payload?: unknown }
): void {
  switch (message.type) {
    case 'subscribe':
      // Subscribe to client updates
      console.log(`Subscribed to client: ${clientId}`);
      break;

    case 'activity':
      // Broadcast activity to caregivers
      broadcastToSubscribers(clientId, {
        type: 'activity',
        clientId,
        payload: message.payload,
        timestamp: new Date().toISOString(),
      });
      break;

    case 'mood_change':
      // Broadcast mood change
      broadcastToSubscribers(clientId, {
        type: 'mood_change',
        clientId,
        payload: message.payload,
        timestamp: new Date().toISOString(),
      });
      break;

    case 'alert':
      // Broadcast alert
      broadcastToSubscribers(clientId, {
        type: 'alert',
        clientId,
        payload: message.payload,
        timestamp: new Date().toISOString(),
      });
      break;

    default:
      ws.send(JSON.stringify({ type: 'error', message: '알 수 없는 메시지 타입입니다' }));
  }
}

// Broadcast to all subscribers
function broadcastToSubscribers(clientId: string | null, message: object): void {
  wss.clients.forEach((client) => {
    if (client.readyState === 1) { // WebSocket.OPEN
      client.send(JSON.stringify(message));
    }
  });
}

// ============================================================================
// Server Start
// ============================================================================

server.listen(PORT, () => {
  console.log(`
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║   WIA Cognitive AAC Backend Server                        ║
║   홍익인간 (弘益人間) - 널리 인간을 이롭게 하라           ║
║                                                           ║
║   Server running on port ${PORT}                            ║
║   Environment: ${NODE_ENV}                                 ║
║                                                           ║
║   REST API: http://localhost:${PORT}/api/v1                 ║
║   WebSocket: ws://localhost:${PORT}/ws                      ║
║   Health: http://localhost:${PORT}/api/v1/health            ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝
  `);
});

// Graceful shutdown
process.on('SIGTERM', () => {
  console.log('SIGTERM received. Shutting down gracefully...');
  server.close(() => {
    console.log('Server closed');
    process.exit(0);
  });
});

export { app, server, wss };
