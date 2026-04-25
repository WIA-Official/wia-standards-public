# 07. 보안
## Security - Encryption, Access Control, Audit Logging

**Version**: 1.0.0
**Last Updated**: 2026-01-11

---

## 목차

1. [보안 개요](#보안-개요)
2. [데이터 암호화](#데이터-암호화)
3. [접근 제어 (RBAC)](#접근-제어-rbac)
4. [인증 및 권한 부여](#인증-및-권한-부여)
5. [감사 로깅](#감사-로깅)
6. [네트워크 보안](#네트워크-보안)
7. [한국 규제 준수](#한국-규제-준수)
8. [보안 모범 사례](#보안-모범-사례)

---

## 보안 개요

### 보안 프레임워크

```typescript
/**
 * WIA Cryo Monitoring 보안 프레임워크
 */
interface SecurityFramework {
  // CIA Triad (기밀성, 무결성, 가용성)
  confidentiality: {
    encryption: {
      atRest: "AES-256";
      inTransit: "TLS 1.3";
      keyManagement: "AWS KMS" | "Azure Key Vault" | "HashiCorp Vault";
      encryptionKr: string;
    };
    accessControl: {
      model: "RBAC" | "ABAC";
      modelKr: string;
      enforcement: "Mandatory" | "Discretionary";
      enforcementKr: string;
    };
    confidentialityKr: string;
  };

  integrity: {
    dataValidation: {
      input: boolean;
      output: boolean;
      schema: "Zod" | "Joi";
      validationKr: string;
    };
    checksums: {
      algorithm: "SHA-256" | "SHA-512";
      verification: boolean;
      checksumKr: string;
    };
    digitalSignatures: {
      enabled: boolean;
      algorithm: "RSA-2048" | "ECDSA";
      signatureKr: string;
    };
    integrityKr: string;
  };

  availability: {
    redundancy: {
      servers: number;
      databases: "Master-Slave" | "Multi-Master";
      backups: "Daily" | "Hourly" | "Real-time";
      redundancyKr: string;
    };
    disasterRecovery: {
      rpo: number;                // Recovery Point Objective (분)
      rto: number;                // Recovery Time Objective (분)
      backupSites: number;
      drKr: string;
    };
    availabilityKr: string;
  };

  // 한국 법규 준수
  compliance: {
    regulations: string[];
    certifications: string[];
    complianceKr: string;
  };

  frameworkKr: string;
}

const securityFramework: SecurityFramework = {
  confidentiality: {
    encryption: {
      atRest: "AES-256",
      inTransit: "TLS 1.3",
      keyManagement: "AWS KMS",
      encryptionKr: "AES-256 저장 암호화, TLS 1.3 전송 암호화"
    },
    accessControl: {
      model: "RBAC",
      modelKr: "역할 기반 접근 제어 (RBAC)",
      enforcement: "Mandatory",
      enforcementKr: "강제 접근 제어"
    },
    confidentialityKr: "기밀성 - 데이터 암호화 및 접근 제어"
  },

  integrity: {
    dataValidation: {
      input: true,
      output: true,
      schema: "Zod",
      validationKr: "Zod 스키마 기반 입출력 검증"
    },
    checksums: {
      algorithm: "SHA-256",
      verification: true,
      checksumKr: "SHA-256 체크섬 검증"
    },
    digitalSignatures: {
      enabled: true,
      algorithm: "RSA-2048",
      signatureKr: "RSA-2048 디지털 서명"
    },
    integrityKr: "무결성 - 데이터 검증 및 서명"
  },

  availability: {
    redundancy: {
      servers: 3,
      databases: "Master-Slave",
      backups: "Hourly",
      redundancyKr: "3대 서버, 마스터-슬레이브 DB, 시간별 백업"
    },
    disasterRecovery: {
      rpo: 60,                    // 1시간
      rto: 30,                    // 30분
      backupSites: 2,
      drKr: "RPO 1시간, RTO 30분, 2개 백업 사이트"
    },
    availabilityKr: "가용성 - 이중화 및 재해 복구"
  },

  compliance: {
    regulations: [
      "개인정보보호법 (PIPA)",
      "의료기기법",
      "생명윤리 및 안전에 관한 법률",
      "정보통신망법"
    ],
    certifications: [
      "ISO 27001 (정보보안)",
      "ISO 27017 (클라우드 보안)",
      "ISO 27701 (개인정보 관리)",
      "ISMS-P (정보보호 및 개인정보보호 관리체계)"
    ],
    complianceKr: "한국 법규 및 국제 표준 준수"
  },

  frameworkKr: "CIA Triad 기반 보안 프레임워크"
};
```

---

## 데이터 암호화

### 저장 데이터 암호화 (Encryption at Rest)

```typescript
/**
 * 저장 데이터 암호화 시스템
 */
import crypto from "crypto";

class DataEncryptionAtRest {
  private algorithm: string = "aes-256-gcm";
  private keyLength: number = 32;        // 256 bits
  private ivLength: number = 16;         // 128 bits
  private tagLength: number = 16;        // 128 bits

  /**
   * 데이터 암호화
   */
  encrypt(plaintext: string, key: Buffer): {
    ciphertext: string;
    iv: string;
    tag: string;
    encryptedKr: string;
  } {
    // IV (Initialization Vector) 생성
    const iv = crypto.randomBytes(this.ivLength);

    // 암호화
    const cipher = crypto.createCipheriv(this.algorithm, key, iv);
    let encrypted = cipher.update(plaintext, "utf8", "hex");
    encrypted += cipher.final("hex");

    // 인증 태그
    const tag = cipher.getAuthTag();

    console.log(`[암호화] 데이터 암호화 완료 (${plaintext.length}바이트)`);

    return {
      ciphertext: encrypted,
      iv: iv.toString("hex"),
      tag: tag.toString("hex"),
      encryptedKr: "AES-256-GCM으로 암호화됨"
    };
  }

  /**
   * 데이터 복호화
   */
  decrypt(
    ciphertext: string,
    key: Buffer,
    iv: string,
    tag: string
  ): {
    plaintext: string;
    decryptedKr: string;
  } {
    // 복호화
    const decipher = crypto.createDecipheriv(
      this.algorithm,
      key,
      Buffer.from(iv, "hex")
    );

    // 인증 태그 설정
    decipher.setAuthTag(Buffer.from(tag, "hex"));

    let decrypted = decipher.update(ciphertext, "hex", "utf8");
    decrypted += decipher.final("utf8");

    console.log(`[복호화] 데이터 복호화 완료 (${decrypted.length}바이트)`);

    return {
      plaintext: decrypted,
      decryptedKr: "복호화 성공"
    };
  }

  /**
   * 암호화 키 생성
   */
  generateKey(): {
    key: Buffer;
    keyHex: string;
    keyKr: string;
  } {
    const key = crypto.randomBytes(this.keyLength);

    console.log(`[키 생성] 256비트 암호화 키 생성 완료`);

    return {
      key,
      keyHex: key.toString("hex"),
      keyKr: "AES-256 암호화 키 (안전하게 보관하세요!)"
    };
  }

  /**
   * 필드 레벨 암호화
   * 민감한 필드만 선택적으로 암호화
   */
  encryptFields<T extends Record<string, any>>(
    data: T,
    fieldsToEncrypt: (keyof T)[],
    key: Buffer
  ): {
    encrypted: T;
    metadata: {
      encryptedFields: string[];
      algorithm: string;
      metadataKr: string;
    };
  } {
    const encrypted = { ...data };
    const encryptedFields: string[] = [];

    for (const field of fieldsToEncrypt) {
      if (data[field] !== undefined) {
        const value = JSON.stringify(data[field]);
        const result = this.encrypt(value, key);

        encrypted[field] = {
          _encrypted: true,
          ciphertext: result.ciphertext,
          iv: result.iv,
          tag: result.tag
        } as any;

        encryptedFields.push(field as string);
      }
    }

    console.log(`[필드 암호화] ${encryptedFields.length}개 필드 암호화 완료`);

    return {
      encrypted,
      metadata: {
        encryptedFields,
        algorithm: this.algorithm,
        metadataKr: `${encryptedFields.join(", ")} 필드 암호화됨`
      }
    };
  }

  /**
   * 필드 복호화
   */
  decryptFields<T extends Record<string, any>>(
    data: T,
    key: Buffer
  ): {
    decrypted: T;
    decryptedKr: string;
  } {
    const decrypted = { ...data };

    for (const field in data) {
      const value = data[field];

      if (
        typeof value === "object" &&
        value !== null &&
        value._encrypted === true
      ) {
        const result = this.decrypt(
          value.ciphertext,
          key,
          value.iv,
          value.tag
        );

        decrypted[field] = JSON.parse(result.plaintext);
      }
    }

    return {
      decrypted,
      decryptedKr: "필드 복호화 완료"
    };
  }
}

// 사용 예시
const encryption = new DataEncryptionAtRest();

// 암호화 키 생성
const { key } = encryption.generateKey();

// 민감한 데이터
const sensitiveData = {
  userId: "USER-001",
  name: "김영희",
  email: "kim@example.com",
  ssn: "123456-1234567",             // 주민등록번호 (민감)
  medicalRecord: {                   // 의료 기록 (민감)
    diagnosis: "제대혈 보관",
    sampleId: "SAMPLE-001"
  }
};

// 민감한 필드만 암호화
const { encrypted } = encryption.encryptFields(
  sensitiveData,
  ["ssn", "medicalRecord"],
  key
);

console.log("암호화된 데이터:", encrypted);

// 복호화
const { decrypted } = encryption.decryptFields(encrypted, key);
console.log("복호화된 데이터:", decrypted);
```

### 전송 데이터 암호화 (Encryption in Transit)

```typescript
/**
 * TLS/SSL 설정
 */
interface TLSConfiguration {
  // TLS 버전
  minVersion: "TLSv1.2" | "TLSv1.3";
  maxVersion: "TLSv1.3";
  tlsVersionKr: string;

  // 암호화 스위트
  cipherSuites: string[];
  cipherSuitesKr: string;

  // 인증서
  certificate: {
    type: "Self-Signed" | "CA-Signed";
    typeKr: string;
    issuer: string;
    issuerKr: string;
    validFrom: Date;
    validUntil: Date;
    commonName: string;
    subjectAltNames: string[];
    certKr: string;
  };

  // HSTS (HTTP Strict Transport Security)
  hsts: {
    enabled: boolean;
    maxAge: number;               // 초
    includeSubDomains: boolean;
    preload: boolean;
    hstsKr: string;
  };

  // Certificate Pinning
  certificatePinning: {
    enabled: boolean;
    pins: string[];               // SHA-256 해시
    pinningKr: string;
  };

  tlsKr: string;
}

const tlsConfig: TLSConfiguration = {
  minVersion: "TLSv1.2",
  maxVersion: "TLSv1.3",
  tlsVersionKr: "TLS 1.2 이상 (TLS 1.3 권장)",

  cipherSuites: [
    "TLS_AES_256_GCM_SHA384",
    "TLS_CHACHA20_POLY1305_SHA256",
    "TLS_AES_128_GCM_SHA256",
    "ECDHE-RSA-AES256-GCM-SHA384",
    "ECDHE-RSA-AES128-GCM-SHA256"
  ],
  cipherSuitesKr: "강력한 암호화 스위트 (Forward Secrecy 지원)",

  certificate: {
    type: "CA-Signed",
    typeKr: "인증 기관 서명 인증서",
    issuer: "Let's Encrypt",
    issuerKr: "Let's Encrypt (무료 SSL)",
    validFrom: new Date("2026-01-01"),
    validUntil: new Date("2026-12-31"),
    commonName: "api.cryo-monitor.wia.org",
    subjectAltNames: [
      "api.cryo-monitor.wia.org",
      "*.cryo-monitor.wia.org",
      "stream.cryo-monitor.wia.org"
    ],
    certKr: "와일드카드 SSL 인증서"
  },

  hsts: {
    enabled: true,
    maxAge: 31536000,             // 1년
    includeSubDomains: true,
    preload: true,
    hstsKr: "HSTS 활성화 (1년, 서브도메인 포함, Preload)"
  },

  certificatePinning: {
    enabled: true,
    pins: [
      "sha256/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=",
      "sha256/BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB="
    ],
    pinningKr: "인증서 피닝 활성화 (MITM 공격 방지)"
  },

  tlsKr: "TLS 1.3 기반 안전한 통신"
};
```

---

## 접근 제어 (RBAC)

### 역할 기반 접근 제어

```typescript
/**
 * 역할 및 권한 정의
 */
enum Permission {
  // 센서
  SENSOR_VIEW = "sensor:view",
  SENSOR_CREATE = "sensor:create",
  SENSOR_UPDATE = "sensor:update",
  SENSOR_DELETE = "sensor:delete",
  SENSOR_CALIBRATE = "sensor:calibrate",

  // 탱크
  TANK_VIEW = "tank:view",
  TANK_CREATE = "tank:create",
  TANK_UPDATE = "tank:update",
  TANK_DELETE = "tank:delete",
  TANK_REFILL = "tank:refill",

  // 알림
  ALERT_VIEW = "alert:view",
  ALERT_ACKNOWLEDGE = "alert:acknowledge",
  ALERT_RESOLVE = "alert:resolve",
  ALERT_CONFIGURE = "alert:configure",

  // 사용자
  USER_VIEW = "user:view",
  USER_CREATE = "user:create",
  USER_UPDATE = "user:update",
  USER_DELETE = "user:delete",

  // 시설
  FACILITY_VIEW = "facility:view",
  FACILITY_CONFIGURE = "facility:configure",

  // 보고서
  REPORT_VIEW = "report:view",
  REPORT_EXPORT = "report:export",

  // 감사
  AUDIT_VIEW = "audit:view"
}

const PermissionKr: Record<Permission, string> = {
  [Permission.SENSOR_VIEW]: "센서 조회",
  [Permission.SENSOR_CREATE]: "센서 생성",
  [Permission.SENSOR_UPDATE]: "센서 수정",
  [Permission.SENSOR_DELETE]: "센서 삭제",
  [Permission.SENSOR_CALIBRATE]: "센서 교정",
  [Permission.TANK_VIEW]: "탱크 조회",
  [Permission.TANK_CREATE]: "탱크 생성",
  [Permission.TANK_UPDATE]: "탱크 수정",
  [Permission.TANK_DELETE]: "탱크 삭제",
  [Permission.TANK_REFILL]: "탱크 보충 기록",
  [Permission.ALERT_VIEW]: "알림 조회",
  [Permission.ALERT_ACKNOWLEDGE]: "알림 확인",
  [Permission.ALERT_RESOLVE]: "알림 해결",
  [Permission.ALERT_CONFIGURE]: "알림 규칙 설정",
  [Permission.USER_VIEW]: "사용자 조회",
  [Permission.USER_CREATE]: "사용자 생성",
  [Permission.USER_UPDATE]: "사용자 수정",
  [Permission.USER_DELETE]: "사용자 삭제",
  [Permission.FACILITY_VIEW]: "시설 조회",
  [Permission.FACILITY_CONFIGURE]: "시설 설정",
  [Permission.REPORT_VIEW]: "보고서 조회",
  [Permission.REPORT_EXPORT]: "보고서 내보내기",
  [Permission.AUDIT_VIEW]: "감사 로그 조회"
};

/**
 * 역할 정의
 */
interface Role {
  roleId: string;
  name: string;
  nameKr: string;
  description: string;
  descriptionKr: string;
  permissions: Permission[];
  permissionsKr: string[];
}

const roles: Role[] = [
  {
    roleId: "ROLE-SUPER-ADMIN",
    name: "Super Administrator",
    nameKr: "슈퍼 관리자",
    description: "Full system access",
    descriptionKr: "모든 권한 보유",
    permissions: Object.values(Permission),
    permissionsKr: Object.values(PermissionKr)
  },
  {
    roleId: "ROLE-FACILITY-ADMIN",
    name: "Facility Administrator",
    nameKr: "시설 관리자",
    description: "Manage facility operations",
    descriptionKr: "시설 운영 관리",
    permissions: [
      Permission.SENSOR_VIEW,
      Permission.SENSOR_UPDATE,
      Permission.SENSOR_CALIBRATE,
      Permission.TANK_VIEW,
      Permission.TANK_UPDATE,
      Permission.TANK_REFILL,
      Permission.ALERT_VIEW,
      Permission.ALERT_ACKNOWLEDGE,
      Permission.ALERT_RESOLVE,
      Permission.ALERT_CONFIGURE,
      Permission.USER_VIEW,
      Permission.FACILITY_VIEW,
      Permission.FACILITY_CONFIGURE,
      Permission.REPORT_VIEW,
      Permission.REPORT_EXPORT
    ],
    permissionsKr: [
      "센서 조회/수정/교정",
      "탱크 조회/수정/보충",
      "알림 전체 관리",
      "사용자 조회",
      "시설 전체 관리",
      "보고서 조회/내보내기"
    ]
  },
  {
    roleId: "ROLE-ENGINEER",
    name: "Engineer",
    nameKr: "엔지니어",
    description: "Technical operations",
    descriptionKr: "기술 운영",
    permissions: [
      Permission.SENSOR_VIEW,
      Permission.SENSOR_UPDATE,
      Permission.SENSOR_CALIBRATE,
      Permission.TANK_VIEW,
      Permission.TANK_UPDATE,
      Permission.TANK_REFILL,
      Permission.ALERT_VIEW,
      Permission.ALERT_ACKNOWLEDGE,
      Permission.ALERT_RESOLVE,
      Permission.REPORT_VIEW
    ],
    permissionsKr: [
      "센서 조회/수정/교정",
      "탱크 조회/수정/보충",
      "알림 조회/확인/해결",
      "보고서 조회"
    ]
  },
  {
    roleId: "ROLE-TECHNICIAN",
    name: "Technician",
    nameKr: "기술자",
    description: "Monitoring and maintenance",
    descriptionKr: "모니터링 및 유지보수",
    permissions: [
      Permission.SENSOR_VIEW,
      Permission.TANK_VIEW,
      Permission.TANK_REFILL,
      Permission.ALERT_VIEW,
      Permission.ALERT_ACKNOWLEDGE,
      Permission.REPORT_VIEW
    ],
    permissionsKr: [
      "센서 조회",
      "탱크 조회/보충",
      "알림 조회/확인",
      "보고서 조회"
    ]
  },
  {
    roleId: "ROLE-VIEWER",
    name: "Viewer",
    nameKr: "조회자",
    description: "Read-only access",
    descriptionKr: "조회 전용",
    permissions: [
      Permission.SENSOR_VIEW,
      Permission.TANK_VIEW,
      Permission.ALERT_VIEW,
      Permission.REPORT_VIEW
    ],
    permissionsKr: [
      "센서 조회",
      "탱크 조회",
      "알림 조회",
      "보고서 조회"
    ]
  },
  {
    roleId: "ROLE-AUDITOR",
    name: "Auditor",
    nameKr: "감사자",
    description: "Audit and compliance",
    descriptionKr: "감사 및 규정 준수",
    permissions: [
      Permission.SENSOR_VIEW,
      Permission.TANK_VIEW,
      Permission.ALERT_VIEW,
      Permission.USER_VIEW,
      Permission.FACILITY_VIEW,
      Permission.REPORT_VIEW,
      Permission.REPORT_EXPORT,
      Permission.AUDIT_VIEW
    ],
    permissionsKr: [
      "모든 데이터 조회",
      "보고서 내보내기",
      "감사 로그 조회"
    ]
  }
];

/**
 * 접근 제어 관리자
 */
class AccessControlManager {
  private userRoles: Map<string, string[]> = new Map();

  /**
   * 권한 확인
   */
  hasPermission(userId: string, permission: Permission): boolean {
    const userRoleIds = this.userRoles.get(userId);
    if (!userRoleIds) return false;

    for (const roleId of userRoleIds) {
      const role = roles.find(r => r.roleId === roleId);
      if (role && role.permissions.includes(permission)) {
        return true;
      }
    }

    return false;
  }

  /**
   * 권한 검증 (미들웨어)
   */
  requirePermission(permission: Permission): (userId: string) => void {
    return (userId: string) => {
      if (!this.hasPermission(userId, permission)) {
        throw new Error(
          `권한 없음: ${PermissionKr[permission]} 권한이 필요합니다`
        );
      }
    };
  }

  /**
   * 사용자 역할 할당
   */
  assignRole(userId: string, roleId: string): void {
    if (!this.userRoles.has(userId)) {
      this.userRoles.set(userId, []);
    }

    const userRoleIds = this.userRoles.get(userId)!;
    if (!userRoleIds.includes(roleId)) {
      userRoleIds.push(roleId);

      const role = roles.find(r => r.roleId === roleId);
      console.log(`[역할 할당] 사용자 ${userId}에게 ${role?.nameKr} 역할 부여`);
    }
  }

  /**
   * 사용자 역할 제거
   */
  revokeRole(userId: string, roleId: string): void {
    const userRoleIds = this.userRoles.get(userId);
    if (!userRoleIds) return;

    const index = userRoleIds.indexOf(roleId);
    if (index !== -1) {
      userRoleIds.splice(index, 1);

      const role = roles.find(r => r.roleId === roleId);
      console.log(`[역할 제거] 사용자 ${userId}의 ${role?.nameKr} 역할 제거`);
    }
  }

  /**
   * 사용자 권한 목록
   */
  getUserPermissions(userId: string): {
    permissions: Permission[];
    permissionsKr: string[];
  } {
    const userRoleIds = this.userRoles.get(userId);
    if (!userRoleIds) {
      return { permissions: [], permissionsKr: [] };
    }

    const permissionsSet = new Set<Permission>();
    const permissionsKrSet = new Set<string>();

    for (const roleId of userRoleIds) {
      const role = roles.find(r => r.roleId === roleId);
      if (role) {
        role.permissions.forEach(p => permissionsSet.add(p));
        role.permissionsKr.forEach(pk => permissionsKrSet.add(pk));
      }
    }

    return {
      permissions: Array.from(permissionsSet),
      permissionsKr: Array.from(permissionsKrSet)
    };
  }
}

// 사용 예시
const acl = new AccessControlManager();

// 사용자에게 역할 할당
acl.assignRole("USER-001", "ROLE-ENGINEER");

// 권한 확인
const canCalibrate = acl.hasPermission("USER-001", Permission.SENSOR_CALIBRATE);
console.log(`센서 교정 권한: ${canCalibrate ? "있음" : "없음"}`);

// 권한 검증 미들웨어
const requireCalibration = acl.requirePermission(Permission.SENSOR_CALIBRATE);
try {
  requireCalibration("USER-001");
  console.log("권한 확인 완료 - 센서 교정 가능");
} catch (error) {
  console.error(error.message);
}
```

---

## 인증 및 권한 부여

### JWT 기반 인증

```typescript
/**
 * JWT 인증 시스템
 */
import jwt from "jsonwebtoken";

class JWTAuthenticationSystem {
  private secretKey: string;
  private issuer: string = "WIA Cryo Monitor";
  private audience: string = "cryo-monitor.wia.org";

  constructor(secretKey: string) {
    this.secretKey = secretKey;
  }

  /**
   * 액세스 토큰 생성
   */
  generateAccessToken(user: {
    userId: string;
    email: string;
    roles: string[];
    facilityId: string;
  }): {
    token: string;
    expiresIn: number;
    tokenKr: string;
  } {
    const expiresIn = 3600;               // 1시간

    const payload = {
      sub: user.userId,
      email: user.email,
      roles: user.roles,
      facilityId: user.facilityId,
      iss: this.issuer,
      aud: this.audience,
      iat: Math.floor(Date.now() / 1000),
      exp: Math.floor(Date.now() / 1000) + expiresIn
    };

    const token = jwt.sign(payload, this.secretKey, {
      algorithm: "HS256"
    });

    console.log(`[토큰 생성] 사용자 ${user.userId}의 액세스 토큰 발급 (1시간 유효)`);

    return {
      token,
      expiresIn,
      tokenKr: "액세스 토큰 (1시간 유효)"
    };
  }

  /**
   * 리프레시 토큰 생성
   */
  generateRefreshToken(userId: string): {
    token: string;
    expiresIn: number;
    tokenKr: string;
  } {
    const expiresIn = 604800;             // 7일

    const payload = {
      sub: userId,
      type: "refresh",
      iss: this.issuer,
      aud: this.audience,
      iat: Math.floor(Date.now() / 1000),
      exp: Math.floor(Date.now() / 1000) + expiresIn
    };

    const token = jwt.sign(payload, this.secretKey, {
      algorithm: "HS256"
    });

    console.log(`[토큰 생성] 사용자 ${userId}의 리프레시 토큰 발급 (7일 유효)`);

    return {
      token,
      expiresIn,
      tokenKr: "리프레시 토큰 (7일 유효)"
    };
  }

  /**
   * 토큰 검증
   */
  verifyToken(token: string): {
    valid: boolean;
    payload?: any;
    error?: string;
    errorKr?: string;
  } {
    try {
      const payload = jwt.verify(token, this.secretKey, {
        issuer: this.issuer,
        audience: this.audience
      });

      return {
        valid: true,
        payload
      };
    } catch (error) {
      if (error.name === "TokenExpiredError") {
        return {
          valid: false,
          error: "Token expired",
          errorKr: "토큰 만료"
        };
      } else if (error.name === "JsonWebTokenError") {
        return {
          valid: false,
          error: "Invalid token",
          errorKr: "유효하지 않은 토큰"
        };
      } else {
        return {
          valid: false,
          error: "Token verification failed",
          errorKr: "토큰 검증 실패"
        };
      }
    }
  }

  /**
   * 토큰 갱신
   */
  refreshAccessToken(refreshToken: string): {
    accessToken: string;
    expiresIn: number;
    refreshKr: string;
  } {
    const verification = this.verifyToken(refreshToken);

    if (!verification.valid) {
      throw new Error(`토큰 갱신 실패: ${verification.errorKr}`);
    }

    if (verification.payload.type !== "refresh") {
      throw new Error("리프레시 토큰이 아닙니다");
    }

    // 새 액세스 토큰 발급
    const { token, expiresIn } = this.generateAccessToken({
      userId: verification.payload.sub,
      email: "",
      roles: [],
      facilityId: ""
    });

    console.log(`[토큰 갱신] 사용자 ${verification.payload.sub}의 액세스 토큰 갱신`);

    return {
      accessToken: token,
      expiresIn,
      refreshKr: "액세스 토큰 갱신 완료"
    };
  }
}
```

---

## 감사 로깅

### 포괄적 감사 시스템

```typescript
/**
 * 감사 로깅 시스템
 */
class AuditLoggingSystem {
  /**
   * 감사 로그 기록
   */
  async log(entry: {
    action: AuditAction;
    actionKr: string;
    user: {
      userId: string;
      username: string;
      email: string;
      role: UserRole;
      roleKr: string;
    };
    target?: {
      type: string;
      typeKr: string;
      id: string;
      name?: string;
    };
    changes?: {
      before?: any;
      after?: any;
    };
    request: {
      ipAddress: string;
      userAgent?: string;
      method?: string;
      endpoint?: string;
    };
    result: {
      success: boolean;
      statusCode?: number;
      errorMessage?: string;
    };
    metadata?: {
      facilityId?: string;
      sessionId?: string;
      traceId?: string;
      duration?: number;
      tags?: string[];
    };
  }): Promise<{
    logId: string;
    timestamp: Date;
    logKr: string;
  }> {
    const logId = this.generateLogId();
    const timestamp = new Date();

    const auditLog: AuditLog = {
      logId,
      action: entry.action,
      actionKr: entry.actionKr,
      user: {
        ...entry.user,
        userKr: `${entry.user.username} (${entry.user.roleKr})`
      },
      target: entry.target ? {
        ...entry.target,
        targetKr: `${entry.target.typeKr}: ${entry.target.name || entry.target.id}`
      } : undefined,
      changes: entry.changes ? {
        ...entry.changes,
        changesKr: this.generateChangeSummary(entry.changes)
      } : undefined,
      request: {
        ...entry.request,
        requestKr: `${entry.request.ipAddress}에서 요청`
      },
      result: {
        ...entry.result,
        resultKr: entry.result.success ? "성공" : `실패: ${entry.result.errorMessage}`
      },
      metadata: {
        ...entry.metadata,
        tags: entry.metadata?.tags || [],
        metadataKr: this.generateMetadataSummary(entry.metadata)
      },
      timestamp
    };

    // 감사 로그 저장
    await this.saveAuditLog(auditLog);

    // 중요한 작업은 실시간 알림
    if (this.isCriticalAction(entry.action)) {
      await this.notifyCriticalAction(auditLog);
    }

    console.log(`[감사 로그] ${entry.actionKr} - ${entry.user.username}`);

    return {
      logId,
      timestamp,
      logKr: `감사 로그 기록 완료 - ${entry.actionKr}`
    };
  }

  /**
   * 감사 로그 조회
   */
  async query(params: {
    startDate: Date;
    endDate: Date;
    userId?: string;
    action?: AuditAction;
    facilityId?: string;
    success?: boolean;
    page?: number;
    limit?: number;
  }): Promise<{
    logs: AuditLog[];
    total: number;
    page: number;
    limit: number;
    queryKr: string;
  }> {
    // 데이터베이스 쿼리 (실제 구현)
    const logs: AuditLog[] = [];
    const total = 0;

    console.log(`[감사 로그 조회] ${params.startDate.toLocaleDateString("ko-KR")} ~ ${params.endDate.toLocaleDateString("ko-KR")}`);

    return {
      logs,
      total,
      page: params.page || 1,
      limit: params.limit || 100,
      queryKr: `${total}개 감사 로그 조회`
    };
  }

  /**
   * 감사 보고서 생성
   */
  async generateReport(params: {
    startDate: Date;
    endDate: Date;
    facilityId?: string;
    format: "pdf" | "xlsx" | "json";
  }): Promise<{
    reportId: string;
    url: string;
    reportKr: string;
  }> {
    const reportId = this.generateReportId();

    console.log(`[감사 보고서] ${params.format.toUpperCase()} 형식 생성 중...`);

    // 감사 로그 조회
    const { logs } = await this.query({
      startDate: params.startDate,
      endDate: params.endDate,
      facilityId: params.facilityId
    });

    // 보고서 생성 (실제 구현)
    const url = `/reports/${reportId}.${params.format}`;

    console.log(`[감사 보고서] 생성 완료 - ${logs.length}개 로그`);

    return {
      reportId,
      url,
      reportKr: `감사 보고서 생성 완료 (${logs.length}개 로그)`
    };
  }

  private generateChangeSummary(changes: any): string {
    if (!changes.before && !changes.after) return "변경 없음";
    if (!changes.before) return "새로 생성됨";
    if (!changes.after) return "삭제됨";

    const changedFields = Object.keys(changes.after).filter(
      key => JSON.stringify(changes.before[key]) !== JSON.stringify(changes.after[key])
    );

    return `${changedFields.length}개 필드 변경: ${changedFields.join(", ")}`;
  }

  private generateMetadataSummary(metadata: any): string {
    if (!metadata) return "메타데이터 없음";

    const parts: string[] = [];
    if (metadata.duration) parts.push(`처리 시간: ${metadata.duration}ms`);
    if (metadata.tags && metadata.tags.length > 0) parts.push(`태그: ${metadata.tags.join(", ")}`);

    return parts.length > 0 ? parts.join(", ") : "메타데이터 없음";
  }

  private isCriticalAction(action: AuditAction): boolean {
    const criticalActions: AuditAction[] = [
      "sensor.delete" as AuditAction,
      "tank.delete" as AuditAction,
      "user.delete" as AuditAction,
      "user.role_change" as AuditAction,
      "config.update" as AuditAction,
      "data.delete" as AuditAction
    ];

    return criticalActions.includes(action);
  }

  private async notifyCriticalAction(log: AuditLog): Promise<void> {
    console.log(`[중요 작업 알림] ${log.actionKr} - ${log.user.userKr}`);
    // 관리자에게 알림 전송
  }

  private async saveAuditLog(log: AuditLog): Promise<void> {
    // 데이터베이스 저장
  }

  private generateLogId(): string {
    return `AUDIT-${Date.now()}`;
  }

  private generateReportId(): string {
    return `REPORT-${Date.now()}`;
  }
}
```

---

## 결론

WIA Cryo Monitoring Standard는 엔터프라이즈급 보안을 제공합니다.

### 핵심 특징

1. **데이터 암호화**: AES-256 (저장), TLS 1.3 (전송)
2. **접근 제어**: RBAC 기반 세밀한 권한 관리
3. **인증**: JWT 기반 토큰 인증
4. **감사**: 모든 작업 로깅 및 추적

### 다음 장 예고

다음 장에서는 **구현 및 배포**를 다룹니다:
- Docker 컨테이너화
- Kubernetes 배포
- CI/CD 파이프라인
- 성능 최적화

---

**© 2026 WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) - Benefit All Humanity**
