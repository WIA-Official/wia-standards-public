/**
 * WIA Cognitive AAC - Family Service
 * 가족 참여 서비스
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import { v4 as uuidv4 } from 'uuid';
import {
  Photo,
  Event,
  Reminder,
  LifeStoryEntry,
  ReminiscenceSession,
  Symbol,
} from '../types';

// ============================================================================
// Types
// ============================================================================

export interface MediaUpload {
  id: string;
  type: 'photo' | 'video' | 'audio';
  url: string;
  caption?: string;
  uploadedBy: string;
  uploadedAt: Date;
  clientId: string;
}

export interface FamilyMessage {
  id: string;
  clientId: string;
  senderId: string;
  senderName: string;
  type: 'text' | 'voice' | 'video';
  content: string;
  mediaUrl?: string;
  timestamp: Date;
  read: boolean;
}

export interface SharedCalendar {
  clientId: string;
  events: Event[];
  upcomingReminders: Reminder[];
}

export interface FamilyAlbum {
  clientId: string;
  photos: Photo[];
  totalCount: number;
}

// ============================================================================
// Family Service
// ============================================================================

export class FamilyService {
  // In-memory stores
  private photos: Map<string, Photo[]> = new Map();
  private events: Map<string, Event[]> = new Map();
  private messages: Map<string, FamilyMessage[]> = new Map();
  private lifeStoryEntries: Map<string, LifeStoryEntry[]> = new Map();
  private reminiscenceSessions: Map<string, ReminiscenceSession[]> = new Map();

  // ============================================================================
  // Media Sharing
  // ============================================================================

  /**
   * 사진 업로드
   */
  uploadPhoto(
    clientId: string,
    photoData: Omit<Photo, 'id' | 'uploadedAt'>
  ): Photo {
    const photo: Photo = {
      ...photoData,
      id: uuidv4(),
      uploadedAt: new Date(),
    };

    const clientPhotos = this.photos.get(clientId) ?? [];
    clientPhotos.push(photo);
    this.photos.set(clientId, clientPhotos);

    return photo;
  }

  /**
   * 사진으로 심볼 생성
   */
  createSymbolFromPhoto(
    clientId: string,
    photoId: string,
    label: string,
    category: string
  ): Symbol | null {
    const clientPhotos = this.photos.get(clientId) ?? [];
    const photo = clientPhotos.find((p) => p.id === photoId);

    if (!photo) return null;

    const symbol: Symbol = {
      id: uuidv4(),
      label,
      imageUrl: photo.url,
      category,
      isCore: false,
      customFor: clientId,
      createdAt: new Date(),
    };

    return symbol;
  }

  /**
   * 가족 앨범 조회
   */
  getFamilyAlbum(clientId: string): FamilyAlbum {
    const clientPhotos = this.photos.get(clientId) ?? [];

    return {
      clientId,
      photos: clientPhotos.sort(
        (a, b) => b.uploadedAt.getTime() - a.uploadedAt.getTime()
      ),
      totalCount: clientPhotos.length,
    };
  }

  // ============================================================================
  // Messaging
  // ============================================================================

  /**
   * 메시지 전송
   */
  sendMessage(
    clientId: string,
    senderId: string,
    senderName: string,
    content: string,
    type: 'text' | 'voice' | 'video' = 'text',
    mediaUrl?: string
  ): FamilyMessage {
    const message: FamilyMessage = {
      id: uuidv4(),
      clientId,
      senderId,
      senderName,
      type,
      content,
      mediaUrl,
      timestamp: new Date(),
      read: false,
    };

    const clientMessages = this.messages.get(clientId) ?? [];
    clientMessages.push(message);
    this.messages.set(clientId, clientMessages);

    return message;
  }

  /**
   * 메시지 목록 조회
   */
  getMessages(
    clientId: string,
    limit: number = 50,
    before?: Date
  ): FamilyMessage[] {
    const clientMessages = this.messages.get(clientId) ?? [];

    let filtered = clientMessages;
    if (before) {
      filtered = clientMessages.filter((m) => m.timestamp < before);
    }

    return filtered
      .sort((a, b) => b.timestamp.getTime() - a.timestamp.getTime())
      .slice(0, limit);
  }

  /**
   * 메시지 읽음 처리
   */
  markMessagesAsRead(clientId: string, messageIds: string[]): void {
    const clientMessages = this.messages.get(clientId) ?? [];

    for (const message of clientMessages) {
      if (messageIds.includes(message.id)) {
        message.read = true;
      }
    }
  }

  // ============================================================================
  // Calendar
  // ============================================================================

  /**
   * 이벤트 추가
   */
  addEvent(clientId: string, eventData: Omit<Event, 'id'>): Event {
    const event: Event = {
      ...eventData,
      id: uuidv4(),
    };

    const clientEvents = this.events.get(clientId) ?? [];
    clientEvents.push(event);
    this.events.set(clientId, clientEvents);

    return event;
  }

  /**
   * 공유 캘린더 조회
   */
  getSharedCalendar(clientId: string): SharedCalendar {
    const clientEvents = this.events.get(clientId) ?? [];
    const now = new Date();

    // 다가오는 이벤트 필터링
    const upcomingEvents = clientEvents
      .filter((e) => e.startTime > now)
      .sort((a, b) => a.startTime.getTime() - b.startTime.getTime());

    // 다가오는 리마인더 수집
    const upcomingReminders: Reminder[] = [];
    for (const event of upcomingEvents) {
      for (const reminder of event.reminders) {
        if (!reminder.sent) {
          const reminderTime = new Date(
            event.startTime.getTime() - reminder.beforeMinutes * 60 * 1000
          );
          if (reminderTime > now) {
            upcomingReminders.push(reminder);
          }
        }
      }
    }

    return {
      clientId,
      events: upcomingEvents,
      upcomingReminders,
    };
  }

  /**
   * 이벤트 리마인더 전송
   */
  sendEventReminder(eventId: string, reminderId: string): boolean {
    for (const [, clientEvents] of this.events) {
      const event = clientEvents.find((e) => e.id === eventId);
      if (event) {
        const reminder = event.reminders.find((r) => r.id === reminderId);
        if (reminder) {
          reminder.sent = true;
          reminder.sentAt = new Date();
          return true;
        }
      }
    }
    return false;
  }

  // ============================================================================
  // Reminiscence (치매 특화)
  // ============================================================================

  /**
   * 인생 이야기 추가
   */
  addLifeStoryEntry(
    clientId: string,
    entryData: Omit<LifeStoryEntry, 'id' | 'createdAt'>
  ): LifeStoryEntry {
    const entry: LifeStoryEntry = {
      ...entryData,
      id: uuidv4(),
      createdAt: new Date(),
    };

    const clientEntries = this.lifeStoryEntries.get(clientId) ?? [];
    clientEntries.push(entry);
    this.lifeStoryEntries.set(clientId, clientEntries);

    return entry;
  }

  /**
   * 인생 이야기 목록 조회
   */
  getLifeStoryBook(clientId: string): LifeStoryEntry[] {
    return this.lifeStoryEntries.get(clientId) ?? [];
  }

  /**
   * 회상 세션 시작
   */
  startReminiscenceSession(
    clientId: string,
    topic: string
  ): ReminiscenceSession {
    const clientEntries = this.lifeStoryEntries.get(clientId) ?? [];

    // 주제와 관련된 인생 이야기 찾기
    const relatedEntries = clientEntries.filter(
      (entry) =>
        entry.title.toLowerCase().includes(topic.toLowerCase()) ||
        entry.content.toLowerCase().includes(topic.toLowerCase()) ||
        entry.era?.toLowerCase().includes(topic.toLowerCase())
    );

    const session: ReminiscenceSession = {
      id: uuidv4(),
      clientId,
      topic,
      entries: relatedEntries,
      startedAt: new Date(),
    };

    const clientSessions = this.reminiscenceSessions.get(clientId) ?? [];
    clientSessions.push(session);
    this.reminiscenceSessions.set(clientId, clientSessions);

    return session;
  }

  /**
   * 회상 세션 종료
   */
  endReminiscenceSession(sessionId: string, notes?: string): boolean {
    for (const [, clientSessions] of this.reminiscenceSessions) {
      const session = clientSessions.find((s) => s.id === sessionId);
      if (session) {
        session.endedAt = new Date();
        session.notes = notes;
        return true;
      }
    }
    return false;
  }

  /**
   * 회상 주제 추천
   */
  suggestReminiscenceTopics(clientId: string): string[] {
    const clientEntries = this.lifeStoryEntries.get(clientId) ?? [];
    const topics = new Set<string>();

    for (const entry of clientEntries) {
      if (entry.era) topics.add(entry.era);
      if (entry.people) entry.people.forEach((p) => topics.add(p));
    }

    // 기본 주제 추가
    const defaultTopics = [
      '어린 시절',
      '결혼',
      '첫 직장',
      '가족',
      '좋아하는 음악',
      '고향',
      '휴가',
      '명절',
    ];

    for (const topic of defaultTopics) {
      if (topics.size < 10) topics.add(topic);
    }

    return Array.from(topics).slice(0, 10);
  }

  /**
   * 오늘의 기념일 확인
   */
  getTodaysAnniversaries(clientId: string): LifeStoryEntry[] {
    const clientEntries = this.lifeStoryEntries.get(clientId) ?? [];
    const today = new Date();
    const todayMonth = today.getMonth();
    const todayDate = today.getDate();

    // 날짜 관련 엔트리 찾기 (실제로는 더 정교한 매칭 필요)
    return clientEntries.filter((entry) => {
      const titleMatch = entry.title.match(/(\d{1,2})월\s*(\d{1,2})일/);
      if (titleMatch) {
        const month = parseInt(titleMatch[1], 10) - 1;
        const date = parseInt(titleMatch[2], 10);
        return month === todayMonth && date === todayDate;
      }
      return false;
    });
  }
}

export default FamilyService;
