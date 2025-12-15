/**
 * WIA AAC Protocol Module
 */

// Message types - export with renamed ErrorCode to avoid conflict
export {
  PROTOCOL_NAME,
  PROTOCOL_VERSION,
  DEFAULT_PORT,
  SUB_PROTOCOL,
  MessageType,
  WiaAacMessage,
  ConnectPayload,
  ConnectAckPayload,
  DisconnectPayload,
  SignalPayload,
  SubscribePayload,
  SubscribeAckPayload,
  UnsubscribePayload,
  CommandPayload,
  CommandAckPayload,
  ProtocolError,
  PingPayload,
  PongPayload,
  ErrorCodes,
  ErrorCode as ProtocolErrorCode,
  ConnectMessage,
  ConnectAckMessage,
  DisconnectMessage,
  SignalMessage,
  SubscribeMessage,
  SubscribeAckMessage,
  CommandMessage,
  CommandAckMessage,
  ErrorMessage,
  PingMessage,
  PongMessage,
} from './message';

export * from './MessageBuilder';
export * from './MessageParser';
export * from './ProtocolHandler';
export { SimpleEventEmitter } from './SimpleEventEmitter';
