# WIA-FIN-007 Phase 3: Protocol Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-01-20

## Overview

Phase 3 defines protocols for secure deployment, upgradeability, and multi-chain compatibility.

## Deployment Protocol

### CREATE2 Deterministic Deployment

All WIA contracts MUST use CREATE2 for deterministic addresses:

```solidity
function deployDeterministic(
    bytes memory bytecode,
    bytes32 salt
) public returns (address deployed) {
    assembly {
        deployed := create2(0, add(bytecode, 0x20), mload(bytecode), salt)
    }
    require(deployed != address(0), "Deployment failed");
}
```

### Pre-Deployment Checklist

- [ ] Bytecode size < 24 KB
- [ ] Constructor arguments validated
- [ ] Sufficient gas for deployment
- [ ] No existing contract at predicted address
- [ ] Security audit completed
- [ ] Multi-chain addresses computed

## Upgrade Mechanisms

### Transparent Proxy Pattern

```solidity
contract WIAProxy is TransparentUpgradeableProxy {
    uint256 public constant UPGRADE_DELAY = 48 hours;
    
    function proposeUpgrade(address newImplementation) external onlyAdmin {
        // Timelock logic
    }
    
    function executeUpgrade(address newImplementation) external onlyAdmin {
        // After timelock expires
        _upgradeTo(newImplementation);
    }
}
```

### UUPS Pattern

```solidity
contract WIAContractUUPS is UUPSUpgradeable {
    function _authorizeUpgrade(address newImplementation) 
        internal 
        override 
        onlyOwner 
    {
        require(_isValidUpgrade(newImplementation), "Invalid upgrade");
    }
}
```

## Security Verification

### Required Checks

1. **Static Analysis:** Slither, Mythril, Securify
2. **Formal Verification:** Required for high-value contracts
3. **Test Coverage:** Minimum 95%
4. **Audit:** Third-party security audit
5. **Runtime Monitoring:** Post-deployment monitoring

## Multi-Chain Compatibility

### Supported Networks

- Ethereum (Chain ID: 1)
- Polygon (Chain ID: 137)
- Arbitrum (Chain ID: 42161)
- Optimism (Chain ID: 10)
- BNB Chain (Chain ID: 56)
- Avalanche (Chain ID: 43114)

### Compatibility Testing

```bash
wia test-compatibility --chains ethereum,polygon,arbitrum
```

## Emergency Procedures

### Circuit Breaker

```solidity
contract WIACircuitBreaker is Pausable {
    function emergencyStop(string memory reason, uint256 duration) 
        external 
        onlyRole(PAUSER_ROLE) 
    {
        _pause();
        emit EmergencyStop(msg.sender, reason, duration, block.timestamp);
    }
}
```

## Compliance Requirements

- [ ] CREATE2 deployment implemented
- [ ] Upgrade mechanism with timelock
- [ ] Security verification completed
- [ ] Multi-chain compatibility tested
- [ ] Emergency stop implemented
- [ ] Multi-signature governance

**Passing Score:** 80/100

---

© 2025 SmileStory Inc. / WIA

---

## A.1 CREATE2 deterministic address protocol

Deployments use CREATE2 with a 32-byte salt derived from `keccak256(deployer || version || chainId)`. The pre-computed address allows mainnet/testnet/L2 deployments to share the same address across chains, so client-side ABIs do not have to vary by chain. Cross-chain deployments MUST verify that the bytecode hash at the deterministic address matches the expected hash before treating the contract as live.

## A.2 Upgrade timelock and authorization

Both Transparent Proxy and UUPS deployments MUST gate every upgrade through a 48-hour minimum timelock. The timelock contract emits `UpgradeProposed` on schedule and `UpgradeExecuted` on activation. Multisig-administered timelocks MUST require at least M-of-N approvals with M ≥ 3 and N ≥ 5 for any production-tier deployment.

## A.3 Static analysis pipeline

Production deployments MUST run static analysis at three points: pre-deployment (Slither + Mythril + Semgrep), per pull-request CI, and post-deployment (Securify + Echidna fuzzing harness). Findings at "high" or "critical" severity MUST block deployment until remediated and the artefact re-signed. The analyser configuration is stored at `https://wiastandards.com/smart-contract/static-analysis.json` and is versioned with the standard.

## A.4 Multi-chain compatibility regression

Compatibility regressions are covered by the conformance suite at `wia-smart-contract-conformance`, which runs the same test deck against every supported chain. Deviations from canonical opcode pricing or precompile availability are flagged; the deployer MUST adjust contract logic or document the divergence in the contract's `compatible` field of the metadata block (Phase 1 §A.1).

## A.5 Circuit breaker and emergency procedures

The circuit-breaker contract emits a single `EmergencyStop` event when triggered. Off-chain monitoring MUST route the event to operations within 30 seconds; the recovery runbook covers credential rotation, log preservation, and post-mortem requirements. Recovery MUST NOT skip the timelock — a separate `RecoveryProposed/Executed` pair governs unpause.

## A.6 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache. Cross-chain replays are mitigated by chainId binding at the EIP-712 domain layer; chain re-orgs are tolerated up to the configured finality threshold (Ethereum: 64 blocks, L2: chain-specific finality).

## A.7 Formal verification gates

High-value contracts (TVL > $50M, life-cycle > 5 years, custodial) MUST carry a formal-verification artefact. The artefact covers the core invariants (balance conservation, access-control monotonicity, timelock enforcement) using Certora, Halmos, or an equivalent symbolic-execution tool. The verification report is hashed into the metadata block and the hash is anchored on chain via `MetadataAnchored`.

## A.8 Audit lifecycle

Third-party audits cover initial deployment plus every major upgrade. The audit report is published at `audit/{version}/report.pdf` with the report hash anchored on chain. Findings at "high" or "critical" severity MUST be remediated before deployment; findings at "medium" or below MUST carry a documented justification or remediation plan in the deployment notes.

## A.9 Multi-chain parity verification

The conformance suite runs the same scenario set against every supported chain and asserts equivalent outcomes (same balance changes, same event emissions, same revert reasons). Chains where opcode pricing or precompile availability diverges (BNB pre-Hertzfork, Polygon zkEVM precompile differences) carry an exception list that the suite consumes; the exception list is reviewed at every minor version and pruned when the chain's behaviour catches up.

## A.10 MEV and ordering-fairness considerations

Production deployments that touch DeFi liquidity MUST consider MEV exposure: sandwich risk on AMM swaps, back-running of liquidations, time-bandit re-orgs in proof-of-stake settings. The protocol layer integrates with private-relay submission (Flashbots Protect, MEV-Share) and commit-reveal patterns where ordering fairness is critical. The choice between batch auctions, fair-sequencing, and threshold encryption is documented per deployment in the runbook (Phase 4 §A.9).

## A.11 Account abstraction and EIP-4337

Smart-contract wallets that follow EIP-4337 receive UserOperations through the entry-point contract rather than direct externally-owned-account calls. The protocol maps the EIP-4337 lifecycle (validate → execute → post-op) onto the WIA call envelope so off-chain indexers can correlate UserOperations with the underlying wallet logic. Paymaster integrations (sponsored gas, ERC-20 gas tokens) honor the same audit trail.

## A.12 Re-org tolerance and finality

Confirmation depth defaults: Ethereum mainnet 6 (heuristic), 64 (justified), 128 (finalised); Arbitrum 32; Optimism 32; BNB Chain 12; Polygon PoS 256. Hosts MAY tighten depth for low-value calls and MUST loosen depth for finality-critical calls (bridge unlocks, irreversible state changes). The protocol re-emits state-change events on re-org rollback so subscribers can compensate; subscribers MUST handle re-emission idempotently.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-contract/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-contract-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-contract-host:1.0.0` ships every smart-contract envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-contract.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Smart-contract deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
