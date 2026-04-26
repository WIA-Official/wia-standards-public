# WIA-DIGITAL_WILL PHASE 3 — Protocol Specification

**Standard:** WIA-DIGITAL_WILL
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

)

# Add contingent beneficiary
manager.add_contingent_beneficiary(
    primary_beneficiary_id=spouse["beneficiaryId"],
    identity={
        "fullName": "Robert Smith",
        "dateOfBirth": "1955-12-05",
        "relationship": "parent"
    },
    relationship="parent"
)

print(f"Added {len(will_doc['beneficiaries'])} beneficiaries")
```

### 4.3 Implementing Execution Triggers

```typescript
import { EventEmitter } from 'events';
import * as crypto from 'crypto';

interface ExecutionTrigger {
  triggerId: string;
  triggerType: string;
  configuration: any;
  verificationRequirements: {
    minimumProofs: number;
    proofTypes: string[];
    verificationPeriod: number;
  };
  delayPeriod: number;
  notificationList: string[];
  revocationPeriod: number;
  priority: number;
  status?: 'pending' | 'active' | 'verified' | 'executed' | 'revoked';
  activatedAt?: string;
  verifications?: Verification[];
}

interface Verification {
  verificationId: string;
  proofType: string;
  providedBy: string;
  timestamp: string;
  verificationData: any;
  digitalSignature: string;
}

class ExecutionTriggerManager extends EventEmitter {
  private triggers: Map<string, ExecutionTrigger>;
  private verifications: Map<string, Verification[]>;

  constructor() {
    super();
    this.triggers = new Map();
    this.verifications = new Map();
  }

  createDeathVerificationTrigger(
    notificationList: string[],
    acceptedSources: string[]
  ): ExecutionTrigger {
    const trigger: ExecutionTrigger = {
      triggerId: this.generateId(),
      triggerType: 'death-verification',
      configuration: {
        acceptedSources: acceptedSources,
        jurisdictionRequirements: {}
      },
      verificationRequirements: {
        minimumProofs: 2,
        proofTypes: ['death-certificate', 'medical-examiner'],
        verificationPeriod: 72 // hours
      },
      delayPeriod: 168, // 7 days
      notificationList: notificationList,
      revocationPeriod: 720, // 30 days
      priority: 10,
      status: 'pending'
    };

    this.triggers.set(trigger.triggerId, trigger);
    this.verifications.set(trigger.triggerId, []);

    return trigger;
  }

  createInactivityTrigger(
    inactivityPeriodDays: number,
    warningPeriodDays: number,
    checkInMethods: string[],
    escalationContacts: string[]
  ): ExecutionTrigger {
    const trigger: ExecutionTrigger = {
      triggerId: this.generateId(),
      triggerType: 'inactivity-period',
      configuration: {
        inactivityPeriodDays: inactivityPeriodDays,
        checkInMethods: checkInMethods,
        warningPeriodDays: warningPeriodDays,
        escalationContacts: escalationContacts,
        lastCheckIn: new Date().toISOString()
      },
      verificationRequirements: {
        minimumProofs: 0,
        proofTypes: [],
        verificationPeriod: 24
      },
      delayPeriod: 336, // 14 days
      notificationList: escalationContacts,
      revocationPeriod: 336,
      priority: 6,
      status: 'active'
    };

    this.triggers.set(trigger.triggerId, trigger);
    this.verifications.set(trigger.triggerId, []);

    // Start monitoring inactivity
    this.startInactivityMonitoring(trigger);

    return trigger;
  }

  createTimeLockTrigger(
    unlockDate: Date,
    notificationList: string[]
  ): ExecutionTrigger {
    const trigger: ExecutionTrigger = {
      triggerId: this.generateId(),
      triggerType: 'time-lock',
      configuration: {
        unlockDate: unlockDate.toISOString(),
        allowEarlyUnlock: false,
        earlyUnlockConditions: []
      },
      verificationRequirements: {
        minimumProofs: 0,
        proofTypes: [],
        verificationPeriod: 0
      },
      delayPeriod: 0,
      notificationList: notificationList,
      revocationPeriod: 168, // 7 days
      priority: 5,
      status: 'active'
    };

    this.triggers.set(trigger.triggerId, trigger);
    this.verifications.set(trigger.triggerId, []);

    // Schedule automatic execution
    this.scheduleTimeLockExecution(trigger);

    return trigger;
  }

  async submitVerification(
    triggerId: string,
    proofType: string,
    verificationData: any,
    providedBy: string,
    privateKey: string
  ): Promise<boolean> {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) {
      throw new Error('Trigger not found');
    }

    // Verify proof type is accepted
    if (!trigger.verificationRequirements.proofTypes.includes(proofType)) {
      throw new Error('Proof type not accepted for this trigger');
    }

    // Create verification record
    const verification: Verification = {
      verificationId: this.generateId(),
      proofType: proofType,
      providedBy: providedBy,
      timestamp: new Date().toISOString(),
      verificationData: verificationData,
      digitalSignature: this.signVerification(verificationData, privateKey)
    };

    // Add verification
    const verifications = this.verifications.get(triggerId) || [];
    verifications.push(verification);
    this.verifications.set(triggerId, verifications);

    // Check if verification threshold met
    if (verifications.length >= trigger.verificationRequirements.minimumProofs) {
      await this.activateTrigger(triggerId);
      return true;
    }

    this.emit('verification-submitted', {
      triggerId,
      verification,
      remaining: trigger.verificationRequirements.minimumProofs - verifications.length
    });

    return false;
  }

  private async activateTrigger(triggerId: string): Promise<void> {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) return;

    trigger.status = 'verified';
    trigger.activatedAt = new Date().toISOString();

    // Send notifications
    await this.sendNotifications(trigger);

    // Wait for revocation period
    setTimeout(() => {
      if (trigger.status === 'verified') {
        this.executeTrigger(triggerId);
      }
    }, trigger.revocationPeriod * 60 * 60 * 1000);

    this.emit('trigger-activated', { trigger });
  }

  private async executeTrigger(triggerId: string): Promise<void> {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) return;

    // Apply delay period
    setTimeout(() => {
      trigger.status = 'executed';
      this.emit('trigger-executed', { trigger });

      // Initiate will execution process
      this.initiateWillExecution(trigger);
    }, trigger.delayPeriod * 60 * 60 * 1000);
  }

  private startInactivityMonitoring(trigger: ExecutionTrigger): void {
    const checkInterval = 24 * 60 * 60 * 1000; // Daily check

    setInterval(() => {
      const lastCheckIn = new Date(trigger.configuration.lastCheckIn);
      const daysSinceCheckIn =
        (Date.now() - lastCheckIn.getTime()) / (24 * 60 * 60 * 1000);

      if (daysSinceCheckIn >= trigger.configuration.warningPeriodDays) {
        this.emit('inactivity-warning', {
          triggerId: trigger.triggerId,
          daysSinceCheckIn: daysSinceCheckIn,
          daysUntilTrigger:
            trigger.configuration.inactivityPeriodDays - daysSinceCheckIn
        });
      }

      if (daysSinceCheckIn >= trigger.configuration.inactivityPeriodDays) {
        this.activateTrigger(trigger.triggerId);
      }
    }, checkInterval);
  }

  private scheduleTimeLockExecution(trigger: ExecutionTrigger): void {
    const unlockDate = new Date(trigger.configuration.unlockDate);
    const delay = unlockDate.getTime() - Date.now();

    if (delay > 0) {
      setTimeout(() => {
        this.activateTrigger(trigger.triggerId);
      }, delay);
    }
  }

  private async sendNotifications(trigger: ExecutionTrigger): Promise<void> {
    // Implementation would send notifications to all parties
    this.emit('notifications-sent', {
      triggerId: trigger.triggerId,
      recipients: trigger.notificationList
    });
  }

  private initiateWillExecution(trigger: ExecutionTrigger): void {
    // Implementation would begin the will execution process
    this.emit('will-execution-initiated', { trigger });
  }

  private signVerification(data: any, privateKey: string): string {
    const sign = crypto.createSign('RSA-SHA256');
    sign.update(JSON.stringify(data));
    return sign.sign(privateKey, 'base64');
  }

  private generateId(): string {
    return crypto.randomBytes(16).toString('hex');
  }

  recordCheckIn(triggerId: string): void {
    const trigger = this.triggers.get(triggerId);
    if (trigger && trigger.triggerType === 'inactivity-period') {
      trigger.configuration.lastCheckIn = new Date().toISOString();
      this.emit('check-in-recorded', { triggerId });
    }
  }

  revokeTrigger(triggerId: string): boolean {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) return false;

    if (trigger.status === 'verified' || trigger.status === 'active') {
      trigger.status = 'revoked';
      this.emit('trigger-revoked', { triggerId });
      return true;
    }

    return false;
  }
}

// Usage example
const triggerManager = new ExecutionTriggerManager();

// Create death verification trigger
const deathTrigger = triggerManager.createDeathVerificationTrigger(
  ['executor-id-1', 'beneficiary-id-1', 'beneficiary-id-2'],
  ['government-registry', 'medical-examiner', 'funeral-home']
);

// Create inactivity trigger
const inactivityTrigger = triggerManager.createInactivityTrigger(
  180, // 6 months
  150, // Warning at 5 months
  ['email-response', 'app-check-in'],
  ['trusted-contact-id-1', 'executor-id-1']
);

// Create time-lock trigger
const timeLockTrigger = triggerManager.createTimeLockTrigger(
  new Date('2030-01-01'),
  ['beneficiary-id-3']
);

// Listen for events
triggerManager.on('trigger-activated', (event) => {
  console.log('Trigger activated:', event.trigger.triggerId);
});

triggerManager.on('will-execution-initiated', (event) => {
  console.log('Will execution initiated by trigger:', event.trigger.triggerId);
});

console.log('Triggers created successfully');
```

### 4.4 Smart Contract Integration

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/access/AccessControl.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/utils/cryptography/ECDSA.sol";

/**
 * @title DigitalWillExecutor
 * @dev Smart contract for managing digital will execution on blockchain
 */
contract DigitalWillExecutor is AccessControl, ReentrancyGuard {
    using ECDSA for bytes32;

    bytes32 public constant EXECUTOR_ROLE = keccak256("EXECUTOR_ROLE");
    bytes32 public constant ORACLE_ROLE = keccak256("ORACLE_ROLE");

    struct Will {
        bytes32 willId;
        address testator;
        uint256 createdAt;
        uint256 lastModified;
        WillStatus status;
        bytes32 contentHash; // IPFS hash of will document
        uint256 executionDelay;
        uint256 revocationPeriod;
    }

    struct Beneficiary {
        address beneficiaryAddress;
        uint256 allocationPercentage; // Basis points (10000 = 100%)
        BeneficiaryType beneficiaryType;
        bool verified;
    }

    struct ExecutionTrigger {
        bytes32 triggerId;
        TriggerType triggerType;
        uint256 activatedAt;
        uint256 verificationCount;
        uint256 requiredVerifications;
        TriggerStatus status;
    }

    enum WillStatus { Draft, Active, Triggered, Executed, Revoked }
    enum TriggerStatus { Pending, Active, Verified, Executed, Revoked }
    enum TriggerType { DeathVerification, Inactivity, TimeLock, Manual }
    enum BeneficiaryType { Primary, Contingent, Trust }

    mapping(bytes32 => Will) public wills;
    mapping(bytes32 => Beneficiary[]) public willBeneficiaries;
    mapping(bytes32 => ExecutionTrigger) public executionTriggers;
    mapping(bytes32 => mapping(address => bool)) public hasVerified;
    mapping(address => bytes32[]) public testatorWills;

    event WillCreated(bytes32 indexed willId, address indexed testator);
    event WillUpdated(bytes32 indexed willId, bytes32 newContentHash);
    event BeneficiaryAdded(bytes32 indexed willId, address indexed beneficiary);
    event TriggerActivated(bytes32 indexed willId, bytes32 indexed triggerId);
    event VerificationSubmitted(bytes32 indexed triggerId, address indexed verifier);
    event WillExecuted(bytes32 indexed willId, uint256 timestamp);
    event WillRevoked(bytes32 indexed willId, uint256 timestamp);

    constructor() {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
    }

    /**
     * @dev Create a new digital will
     */
    function createWill(
        bytes32 _willId,
        bytes32 _contentHash,
        uint256 _executionDelay,
        uint256 _revocationPeriod
    ) external {
        require(wills[_willId].testator == address(0), "Will already exists");

        wills[_willId] = Will({
            willId: _willId,
            testator: msg.sender,
            createdAt: block.timestamp,
            lastModified: block.timestamp,
            status: WillStatus.Draft,
            contentHash: _contentHash,
            executionDelay: _executionDelay,
            revocationPeriod: _revocationPeriod
        });

        testatorWills[msg.sender].push(_willId);
        emit WillCreated(_willId, msg.sender);
    }

    /**
     * @dev Add beneficiary to will
     */
    function addBeneficiary(
        bytes32 _willId,
        address _beneficiary,
        uint256 _allocationPercentage,
        BeneficiaryType _beneficiaryType
    ) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(
            wills[_willId].status == WillStatus.Draft ||
            wills[_willId].status == WillStatus.Active,
            "Cannot modify will"
        );

        willBeneficiaries[_willId].push(Beneficiary({
            beneficiaryAddress: _beneficiary,
            allocationPercentage: _allocationPercentage,
            beneficiaryType: _beneficiaryType,
            verified: false
        }));

        wills[_willId].lastModified = block.timestamp;
        emit BeneficiaryAdded(_willId, _beneficiary);
    }

    /**
     * @dev Activate will (make it legally binding)
     */
    function activateWill(bytes32 _willId) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(wills[_willId].status == WillStatus.Draft, "Will not in draft");
        require(validateBeneficiaries(_willId), "Invalid beneficiary allocation");

        wills[_willId].status = WillStatus.Active;
        wills[_willId].lastModified = block.timestamp;
    }

    /**
     * @dev Create execution trigger
     */
    function createExecutionTrigger(
        bytes32 _willId,
        bytes32 _triggerId,
        TriggerType _triggerType,
        uint256 _requiredVerifications
    ) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(wills[_willId].status == WillStatus.Active, "Will not active");

        executionTriggers[_triggerId] = ExecutionTrigger({
            triggerId: _triggerId,
            triggerType: _triggerType,
            activatedAt: 0,
            verificationCount: 0,
            requiredVerifications: _requiredVerifications,
            status: TriggerStatus.Pending
        });
    }

    /**
     * @dev Submit verification for trigger (e.g., death certificate)
     */
    function submitVerification(
        bytes32 _triggerId,
        bytes32 _willId,
        bytes calldata _proof
    ) external onlyRole(ORACLE_ROLE) {
        ExecutionTrigger storage trigger = executionTriggers[_triggerId];
        require(trigger.status == TriggerStatus.Pending, "Trigger not pending");
        require(!hasVerified[_triggerId][msg.sender], "Already verified");

        // Verify proof (implementation depends on proof type)
        require(_proof.length > 0, "Invalid proof");

        hasVerified[_triggerId][msg.sender] = true;
        trigger.verificationCount++;

        emit VerificationSubmitted(_triggerId, msg.sender);

        // Check if threshold met
        if (trigger.verificationCount >= trigger.requiredVerifications) {
            activateTrigger(_triggerId, _willId);
        }
    }

    /**
     * @dev Activate trigger after verification threshold met
     */
    function activateTrigger(bytes32 _triggerId, bytes32 _willId) internal {
        ExecutionTrigger storage trigger = executionTriggers[_triggerId];
        Will storage will = wills[_willId];

        trigger.status = TriggerStatus.Active;
        trigger.activatedAt = block.timestamp;
        will.status = WillStatus.Triggered;

        emit TriggerActivated(_willId, _triggerId);

        // Schedule execution after revocation period
        scheduleExecution(_willId, _triggerId);
    }

    /**
     * @dev Schedule will execution (in practice, would use oracle or keeper)
     */
    function scheduleExecution(bytes32 _willId, bytes32 _triggerId) internal {
        // In production, this would integrate with Chainlink Keepers or similar
        // For now, we just mark it as ready for execution
    }

    /**
     * @dev Execute will and distribute assets
     */
    function executeWill(bytes32 _willId) external onlyRole(EXECUTOR_ROLE) nonReentrant {
        Will storage will = wills[_willId];
        require(will.status == WillStatus.Triggered, "Will not triggered");

        // Check if revocation period has passed
        ExecutionTrigger memory trigger = findActiveTrigger(_willId);
        require(
            block.timestamp >= trigger.activatedAt + will.revocationPeriod,
            "Revocation period not passed"
        );

        // Check if execution delay has passed
        require(
            block.timestamp >= trigger.activatedAt + will.executionDelay,
            "Execution delay not passed"
        );

        will.status = WillStatus.Executed;

        // Distribute assets to beneficiaries
        distributeAssets(_willId);

        emit WillExecuted(_willId, block.timestamp);
    }

    /**
     * @dev Distribute assets to beneficiaries
     */
    function distributeAssets(bytes32 _willId) internal {
        Beneficiary[] memory beneficiaries = willBeneficiaries[_willId];
        uint256 totalBalance = address(this).balance;

        for (uint256 i = 0; i < beneficiaries.length; i++) {
            if (beneficiaries[i].beneficiaryType == BeneficiaryType.Primary) {
                uint256 amount = (totalBalance * beneficiaries[i].allocationPercentage) / 10000;
                payable(beneficiaries[i].beneficiaryAddress).transfer(amount);
            }
        }
    }

    /**
     * @dev Revoke will
     */
    function revokeWill(bytes32 _willId) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(
            wills[_willId].status != WillStatus.Executed,
            "Cannot revoke executed will"
