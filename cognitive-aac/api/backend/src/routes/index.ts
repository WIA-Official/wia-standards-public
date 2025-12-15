/**
 * WIA Cognitive AAC - API Routes
 * 케어기버/전문가 API 라우트 정의
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import { Router, Request, Response, NextFunction } from 'express';
import { CaregiverService } from '../services/CaregiverService';
import { ProfessionalService } from '../services/ProfessionalService';
import { FamilyService } from '../services/FamilyService';
import { ComplianceService } from '../compliance/ComplianceService';

// ============================================================================
// Service Instances
// ============================================================================

const caregiverService = new CaregiverService();
const professionalService = new ProfessionalService();
const familyService = new FamilyService();
const complianceService = new ComplianceService();

// ============================================================================
// Middleware
// ============================================================================

// 인증 미들웨어 (간략화)
const authenticate = (req: Request, res: Response, next: NextFunction): void => {
  const authHeader = req.headers.authorization;
  if (!authHeader) {
    res.status(401).json({ error: '인증이 필요합니다' });
    return;
  }
  // 실제로는 JWT 검증 등
  (req as any).userId = 'user-123';
  (req as any).userRole = 'caregiver';
  next();
};

// 감사 로깅 미들웨어
const auditLog = (req: Request, res: Response, next: NextFunction): void => {
  const userId = (req as any).userId ?? 'anonymous';
  const userRole = (req as any).userRole ?? 'user';

  complianceService.logAccess(
    userId,
    userRole,
    'view',
    req.path,
    req.params.id ?? '',
    `${req.method} ${req.originalUrl}`,
    req.ip
  );

  next();
};

// ============================================================================
// Router Setup
// ============================================================================

const router = Router();

// ============================================================================
// Caregiver Routes
// ============================================================================

const caregiverRouter = Router();

// 실시간 모니터링
caregiverRouter.get('/clients/:clientId/live', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const data = caregiverService.getLiveMonitoring(clientId);
    res.json(data);
  } catch (error) {
    res.status(500).json({ error: '모니터링 데이터 조회 실패' });
  }
});

// 일간 요약
caregiverRouter.get('/clients/:clientId/daily-summary', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const { date } = req.query;
    const targetDate = date ? new Date(date as string) : new Date();
    const summary = caregiverService.getDailySummary(clientId, targetDate);
    res.json(summary);
  } catch (error) {
    res.status(500).json({ error: '일간 요약 조회 실패' });
  }
});

// 주간 진행상황
caregiverRouter.get('/clients/:clientId/weekly-progress', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const { weekStart } = req.query;
    const startDate = weekStart ? new Date(weekStart as string) : new Date();
    const progress = caregiverService.getWeeklyProgress(clientId, startDate);
    res.json(progress);
  } catch (error) {
    res.status(500).json({ error: '주간 진행상황 조회 실패' });
  }
});

// 의사소통 로그
caregiverRouter.get('/clients/:clientId/communication-log', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const { startDate, endDate } = req.query;
    const start = startDate ? new Date(startDate as string) : new Date(Date.now() - 7 * 24 * 60 * 60 * 1000);
    const end = endDate ? new Date(endDate as string) : new Date();
    const log = caregiverService.getCommunicationLog(clientId, start, end);
    res.json(log);
  } catch (error) {
    res.status(500).json({ error: '의사소통 로그 조회 실패' });
  }
});

// 활동 기록
caregiverRouter.post('/clients/:clientId/activities', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const activity = caregiverService.recordActivity(clientId, {
      ...req.body,
      clientId,
      timestamp: new Date(),
    });
    res.status(201).json(activity);
  } catch (error) {
    res.status(500).json({ error: '활동 기록 실패' });
  }
});

// 메시지 기록
caregiverRouter.post('/clients/:clientId/messages', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const message = caregiverService.recordMessage(clientId, {
      ...req.body,
      clientId,
      timestamp: new Date(),
    });
    res.status(201).json(message);
  } catch (error) {
    res.status(500).json({ error: '메시지 기록 실패' });
  }
});

// 알림 확인
caregiverRouter.post('/alerts/:alertId/acknowledge', authenticate, auditLog, (req, res) => {
  try {
    const { alertId } = req.params;
    const userId = (req as any).userId;
    const success = caregiverService.acknowledgeAlert(alertId, userId);
    res.json({ success });
  } catch (error) {
    res.status(500).json({ error: '알림 확인 실패' });
  }
});

// 원격 UI 조정
caregiverRouter.post('/clients/:clientId/adjust-ui', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const result = caregiverService.adjustUIRemotely(clientId, req.body);
    res.json(result);
  } catch (error) {
    res.status(500).json({ error: 'UI 조정 실패' });
  }
});

// 기기 잠금
caregiverRouter.post('/clients/:clientId/lock', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const { locked } = req.body;
    const result = caregiverService.setDeviceLock(clientId, locked);
    res.json(result);
  } catch (error) {
    res.status(500).json({ error: '기기 잠금 실패' });
  }
});

// ============================================================================
// Professional Routes
// ============================================================================

const professionalRouter = Router();

// 담당 클라이언트 목록
professionalRouter.get('/clients', authenticate, auditLog, (req, res) => {
  try {
    const userId = (req as any).userId;
    const clients = professionalService.getClientList(userId);
    res.json(clients);
  } catch (error) {
    res.status(500).json({ error: '클라이언트 목록 조회 실패' });
  }
});

// 클라이언트 상세
professionalRouter.get('/clients/:clientId', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const client = professionalService.getClientDetail(clientId);
    if (!client) {
      res.status(404).json({ error: '클라이언트를 찾을 수 없습니다' });
      return;
    }
    res.json(client);
  } catch (error) {
    res.status(500).json({ error: '클라이언트 조회 실패' });
  }
});

// 평가 실시
professionalRouter.post('/clients/:clientId/assessments', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const userId = (req as any).userId;
    const { type, scores, notes } = req.body;
    const result = professionalService.conductAssessment(
      clientId,
      type,
      userId,
      scores,
      notes
    );
    res.status(201).json(result);
  } catch (error) {
    res.status(500).json({ error: '평가 실시 실패' });
  }
});

// 평가 이력
professionalRouter.get('/clients/:clientId/assessments', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const history = professionalService.getAssessmentHistory(clientId);
    res.json(history);
  } catch (error) {
    res.status(500).json({ error: '평가 이력 조회 실패' });
  }
});

// 진행 비교
professionalRouter.get('/clients/:clientId/progress-comparison', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const { startDate, endDate } = req.query;
    const comparison = professionalService.compareProgress(
      clientId,
      new Date(startDate as string),
      new Date(endDate as string)
    );
    res.json(comparison);
  } catch (error) {
    res.status(500).json({ error: '진행 비교 실패' });
  }
});

// 목표 설정
professionalRouter.post('/clients/:clientId/goals', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const userId = (req as any).userId;
    const goal = professionalService.setGoal(clientId, {
      ...req.body,
      clientId,
      createdBy: userId,
      status: 'not_started',
      progress: 0,
    });
    res.status(201).json(goal);
  } catch (error) {
    res.status(500).json({ error: '목표 설정 실패' });
  }
});

// 현재 목표
professionalRouter.get('/clients/:clientId/goals', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const goals = professionalService.getCurrentGoals(clientId);
    res.json(goals);
  } catch (error) {
    res.status(500).json({ error: '목표 조회 실패' });
  }
});

// 목표 진행 업데이트
professionalRouter.patch('/goals/:goalId/progress', authenticate, auditLog, (req, res) => {
  try {
    const { goalId } = req.params;
    const { progress } = req.body;
    const result = professionalService.trackGoalProgress(goalId, progress);
    if (!result) {
      res.status(404).json({ error: '목표를 찾을 수 없습니다' });
      return;
    }
    res.json(result);
  } catch (error) {
    res.status(500).json({ error: '목표 진행 업데이트 실패' });
  }
});

// 어휘 분석
professionalRouter.get('/clients/:clientId/vocabulary-analysis', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const analysis = professionalService.analyzeVocabulary(clientId);
    res.json(analysis);
  } catch (error) {
    res.status(500).json({ error: '어휘 분석 실패' });
  }
});

// 의사소통 패턴 분석
professionalRouter.get('/clients/:clientId/communication-patterns', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const patterns = professionalService.analyzeCommunicationPatterns(clientId);
    res.json(patterns);
  } catch (error) {
    res.status(500).json({ error: '패턴 분석 실패' });
  }
});

// 임상 보고서 생성
professionalRouter.post('/clients/:clientId/reports', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const userId = (req as any).userId;
    const { type, period } = req.body;
    const report = professionalService.generateReport(
      clientId,
      type,
      userId,
      {
        start: new Date(period.start),
        end: new Date(period.end),
      }
    );
    res.status(201).json(report);
  } catch (error) {
    res.status(500).json({ error: '보고서 생성 실패' });
  }
});

// ============================================================================
// Family Routes
// ============================================================================

const familyRouter = Router();

// 사진 업로드
familyRouter.post('/clients/:clientId/photos', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const userId = (req as any).userId;
    const photo = familyService.uploadPhoto(clientId, {
      ...req.body,
      uploadedBy: userId,
    });
    res.status(201).json(photo);
  } catch (error) {
    res.status(500).json({ error: '사진 업로드 실패' });
  }
});

// 가족 앨범
familyRouter.get('/clients/:clientId/album', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const album = familyService.getFamilyAlbum(clientId);
    res.json(album);
  } catch (error) {
    res.status(500).json({ error: '앨범 조회 실패' });
  }
});

// 메시지 전송
familyRouter.post('/clients/:clientId/messages', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const userId = (req as any).userId;
    const { content, type, mediaUrl, senderName } = req.body;
    const message = familyService.sendMessage(
      clientId,
      userId,
      senderName,
      content,
      type,
      mediaUrl
    );
    res.status(201).json(message);
  } catch (error) {
    res.status(500).json({ error: '메시지 전송 실패' });
  }
});

// 메시지 목록
familyRouter.get('/clients/:clientId/messages', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const { limit, before } = req.query;
    const messages = familyService.getMessages(
      clientId,
      limit ? parseInt(limit as string, 10) : 50,
      before ? new Date(before as string) : undefined
    );
    res.json(messages);
  } catch (error) {
    res.status(500).json({ error: '메시지 조회 실패' });
  }
});

// 이벤트 추가
familyRouter.post('/clients/:clientId/events', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const userId = (req as any).userId;
    const event = familyService.addEvent(clientId, {
      ...req.body,
      createdBy: userId,
      startTime: new Date(req.body.startTime),
      endTime: req.body.endTime ? new Date(req.body.endTime) : undefined,
    });
    res.status(201).json(event);
  } catch (error) {
    res.status(500).json({ error: '이벤트 추가 실패' });
  }
});

// 공유 캘린더
familyRouter.get('/clients/:clientId/calendar', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const calendar = familyService.getSharedCalendar(clientId);
    res.json(calendar);
  } catch (error) {
    res.status(500).json({ error: '캘린더 조회 실패' });
  }
});

// 인생 이야기 추가 (치매)
familyRouter.post('/clients/:clientId/life-story', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const userId = (req as any).userId;
    const entry = familyService.addLifeStoryEntry(clientId, {
      ...req.body,
      clientId,
      createdBy: userId,
    });
    res.status(201).json(entry);
  } catch (error) {
    res.status(500).json({ error: '인생 이야기 추가 실패' });
  }
});

// 인생 이야기 목록
familyRouter.get('/clients/:clientId/life-story', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const entries = familyService.getLifeStoryBook(clientId);
    res.json(entries);
  } catch (error) {
    res.status(500).json({ error: '인생 이야기 조회 실패' });
  }
});

// 회상 세션 시작
familyRouter.post('/clients/:clientId/reminiscence', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const { topic } = req.body;
    const session = familyService.startReminiscenceSession(clientId, topic);
    res.status(201).json(session);
  } catch (error) {
    res.status(500).json({ error: '회상 세션 시작 실패' });
  }
});

// 회상 주제 추천
familyRouter.get('/clients/:clientId/reminiscence/topics', authenticate, auditLog, (req, res) => {
  try {
    const { clientId } = req.params;
    const topics = familyService.suggestReminiscenceTopics(clientId);
    res.json(topics);
  } catch (error) {
    res.status(500).json({ error: '주제 추천 실패' });
  }
});

// ============================================================================
// Compliance Routes
// ============================================================================

const complianceRouter = Router();

// 동의 기록
complianceRouter.post('/consents', authenticate, (req, res) => {
  try {
    const userId = (req as any).userId;
    const consent = complianceService.recordConsent(userId, req.body);
    res.status(201).json(consent);
  } catch (error) {
    res.status(500).json({ error: '동의 기록 실패' });
  }
});

// 동의 이력
complianceRouter.get('/consents', authenticate, (req, res) => {
  try {
    const userId = (req as any).userId;
    const history = complianceService.getConsentHistory(userId);
    res.json(history);
  } catch (error) {
    res.status(500).json({ error: '동의 이력 조회 실패' });
  }
});

// GDPR - 데이터 접근
complianceRouter.get('/gdpr/access', authenticate, (req, res) => {
  try {
    const userId = (req as any).userId;
    const data = complianceService.requestDataAccess(userId);
    res.json(data);
  } catch (error) {
    res.status(500).json({ error: '데이터 접근 실패' });
  }
});

// GDPR - 데이터 삭제
complianceRouter.delete('/gdpr/erasure', authenticate, (req, res) => {
  try {
    const userId = (req as any).userId;
    const result = complianceService.requestDataErasure(userId);
    res.json(result);
  } catch (error) {
    res.status(500).json({ error: '데이터 삭제 실패' });
  }
});

// GDPR - 데이터 이동
complianceRouter.post('/gdpr/portability', authenticate, (req, res) => {
  try {
    const userId = (req as any).userId;
    const { format } = req.body;
    const request = complianceService.requestDataPortability(userId, format);
    res.status(202).json(request);
  } catch (error) {
    res.status(500).json({ error: '데이터 내보내기 실패' });
  }
});

// 데이터 내보내기 상태
complianceRouter.get('/exports/:exportId', authenticate, (req, res) => {
  try {
    const { exportId } = req.params;
    const status = complianceService.checkExportStatus(exportId);
    if (!status) {
      res.status(404).json({ error: '내보내기 요청을 찾을 수 없습니다' });
      return;
    }
    res.json(status);
  } catch (error) {
    res.status(500).json({ error: '상태 조회 실패' });
  }
});

// 규정 준수 보고서
complianceRouter.get('/reports', authenticate, (req, res) => {
  try {
    const { type, startDate, endDate } = req.query;
    const report = complianceService.generateComplianceReport(
      (type as 'hipaa' | 'gdpr' | 'combined') ?? 'combined',
      {
        start: startDate ? new Date(startDate as string) : new Date(Date.now() - 30 * 24 * 60 * 60 * 1000),
        end: endDate ? new Date(endDate as string) : new Date(),
      }
    );
    res.json(report);
  } catch (error) {
    res.status(500).json({ error: '보고서 생성 실패' });
  }
});

// ============================================================================
// Health Check
// ============================================================================

router.get('/health', (req, res) => {
  res.json({
    status: 'healthy',
    timestamp: new Date().toISOString(),
    version: '1.0.0',
  });
});

// ============================================================================
// Mount Routers
// ============================================================================

router.use('/caregiver', caregiverRouter);
router.use('/professional', professionalRouter);
router.use('/family', familyRouter);
router.use('/compliance', complianceRouter);

export default router;
