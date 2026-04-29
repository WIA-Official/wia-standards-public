# WIA-ESPORTS Specification - PHASE 4
# Deployment & Operations

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Table of Contents

1. [Development Environment](#development-environment)
2. [Smart Contract Deployment](#smart-contract-deployment)
3. [Infrastructure Setup](#infrastructure-setup)
4. [Monitoring & Analytics](#monitoring--analytics)
5. [Maintenance & Upgrades](#maintenance--upgrades)
6. [Compliance & Governance](#compliance--governance)

---

## Development Environment

### Prerequisites

```bash
# Node.js 18+ and npm
node --version  # v18.0.0+
npm --version   # 9.0.0+

# Install development tools
npm install -g hardhat
npm install -g foundry
npm install -g slither-analyzer

# Install project dependencies
npm install --save-dev \
  @nomiclabs/hardhat-ethers \
  @openzeppelin/contracts \
  @openzeppelin/contracts-upgradeable \
  @uniswap/v4-core \
  hardhat-gas-reporter \
  solidity-coverage
```

### Project Structure

```
defi-protocol/
├── contracts/
│   ├── core/
│   │   ├── LiquidityPool.sol
│   │   ├── Router.sol
│   │   └── Factory.sol
│   ├── periphery/
│   │   ├── Quoter.sol
│   │   └── Multicall.sol
│   ├── interfaces/
│   │   └── ILiquidityPool.sol
│   └── libraries/
│       └── Math.sol
├── scripts/
│   ├── deploy.ts
│   ├── verify.ts
│   └── upgrade.ts
├── test/
│   ├── unit/
│   ├── integration/
│   └── fork/
├── hardhat.config.ts
├── foundry.toml
└── package.json
```

### Hardhat Configuration

```typescript
// hardhat.config.ts
import { HardhatUserConfig } from "hardhat/config";
import "@nomiclabs/hardhat-ethers";
import "@nomiclabs/hardhat-waffle";
import "hardhat-gas-reporter";
import "solidity-coverage";

const config: HardhatUserConfig = {
  solidity: {
    version: "0.8.20",
    settings: {
      optimizer: {
        enabled: true,
        runs: 1000000,  // Optimize for low deployment, high execution
      },
      viaIR: true,  // Enable IR-based optimizer
    },
  },
  networks: {
    hardhat: {
      forking: {
        url: process.env.MAINNET_RPC_URL!,
        blockNumber: 18500000,
      },
    },
    sepolia: {
      url: process.env.SEPOLIA_RPC_URL,
      accounts: [process.env.DEPLOYER_PRIVATE_KEY!],
      gasPrice: 20000000000,  // 20 gwei
    },
    mainnet: {
      url: process.env.MAINNET_RPC_URL,
      accounts: [process.env.DEPLOYER_PRIVATE_KEY!],
      gasPrice: "auto",
    },
    arbitrum: {
      url: "https://arb1.arbitrum.io/rpc",
      accounts: [process.env.DEPLOYER_PRIVATE_KEY!],
      chainId: 42161,
    },
    optimism: {
      url: "https://mainnet.optimism.io",
      accounts: [process.env.DEPLOYER_PRIVATE_KEY!],
      chainId: 10,
    },
  },
  gasReporter: {
    enabled: true,
    currency: "USD",
    coinmarketcap: process.env.CMC_API_KEY,
  },
  etherscan: {
    apiKey: {
      mainnet: process.env.ETHERSCAN_API_KEY!,
      arbitrumOne: process.env.ARBISCAN_API_KEY!,
      optimisticEthereum: process.env.OPTIMISM_API_KEY!,
    },
  },
};

export default config;
```

### Foundry Configuration

```toml
# foundry.toml
[profile.default]
src = 'contracts'
out = 'out'
libs = ['node_modules', 'lib']
solc_version = '0.8.20'
optimizer = true
optimizer_runs = 1_000_000
via_ir = true

[profile.ci]
fuzz_runs = 10000
invariant_runs = 1000

[rpc_endpoints]
mainnet = "${MAINNET_RPC_URL}"
sepolia = "${SEPOLIA_RPC_URL}"
arbitrum = "https://arb1.arbitrum.io/rpc"
optimism = "https://mainnet.optimism.io"
```

---

## Smart Contract Deployment

### Deployment Script (Hardhat)

```typescript
// scripts/deploy.ts
import { ethers } from "hardhat";

async function main() {
  const [deployer] = await ethers.getSigners();
  console.log("Deploying contracts with account:", deployer.address);
  console.log("Account balance:", (await deployer.getBalance()).toString());

  // Deploy Factory
  const Factory = await ethers.getContractFactory("Factory");
  const factory = await Factory.deploy(deployer.address);
  await factory.deployed();
  console.log("Factory deployed to:", factory.address);

  // Deploy Router
  const Router = await ethers.getContractFactory("Router");
  const router = await Router.deploy(factory.address);
  await router.deployed();
  console.log("Router deployed to:", router.address);

  // Deploy Quoter
  const Quoter = await ethers.getContractFactory("Quoter");
  const quoter = await Quoter.deploy(factory.address);
  await quoter.deployed();
  console.log("Quoter deployed to:", quoter.address);

  // Initialize factory
  await factory.setRouter(router.address);
  console.log("Factory initialized with Router");

  // Save deployment addresses
  const deployment = {
    network: await ethers.provider.getNetwork(),
    factory: factory.address,
    router: router.address,
    quoter: quoter.address,
    deployer: deployer.address,
    timestamp: new Date().toISOString(),
  };

  const fs = require("fs");
  fs.writeFileSync(
    `deployments/${deployment.network.name}.json`,
    JSON.stringify(deployment, null, 2)
  );

  console.log("\nDeployment complete!");
  console.log("Verify with:");
  console.log(`npx hardhat verify --network ${deployment.network.name} ${factory.address} ${deployer.address}`);
}

main()
  .then(() => process.exit(0))
  .catch((error) => {
    console.error(error);
    process.exit(1);
  });
```

### Deployment Script (Foundry)

```bash
#!/bin/bash
# scripts/deploy.sh

set -e

# Load environment variables
source .env

# Deploy to Sepolia testnet
echo "Deploying to Sepolia..."

forge create contracts/core/Factory.sol:Factory \
  --rpc-url $SEPOLIA_RPC_URL \
  --private-key $DEPLOYER_PRIVATE_KEY \
  --constructor-args $DEPLOYER_ADDRESS \
  --verify \
  --etherscan-api-key $ETHERSCAN_API_KEY

FACTORY_ADDRESS=$(cast logs --rpc-url $SEPOLIA_RPC_URL | grep "Factory deployed" | cut -d' ' -f4)

forge create contracts/core/Router.sol:Router \
  --rpc-url $SEPOLIA_RPC_URL \
  --private-key $DEPLOYER_PRIVATE_KEY \
  --constructor-args $FACTORY_ADDRESS \
  --verify \
  --etherscan-api-key $ETHERSCAN_API_KEY

echo "Deployment complete!"
```

### Multi-Sig Deployment

```typescript
// Use Gnosis Safe for production deployments
import { ethers } from "hardhat";
import Safe from "@safe-global/safe-core-sdk";

async function deployWithMultisig() {
  const safeAddress = "0x..."; // Gnosis Safe address
  const threshold = 4; // 4 of 7 signatures required

  // Create deployment transaction
  const Factory = await ethers.getContractFactory("Factory");
  const deployTx = Factory.getDeployTransaction(safeAddress);

  // Submit to Gnosis Safe
  const safeSdk = await Safe.create({
    ethAdapter,
    safeAddress,
  });

  const safeTransaction = await safeSdk.createTransaction({
    safeTransactionData: {
      to: ethers.constants.AddressZero,
      value: "0",
      data: deployTx.data,
    },
  });

  const txHash = await safeSdk.getTransactionHash(safeTransaction);
  const signature = await safeSdk.signTransactionHash(txHash);

  // Collect remaining signatures off-chain
  console.log("Transaction hash:", txHash);
  console.log("Share with other signers for approval");
}
```

### Verification

```bash
# Verify on Etherscan
npx hardhat verify --network mainnet \
  0x1234567890abcdef1234567890abcdef12345678 \
  "0x...constructorArg1" "0x...constructorArg2"

# Verify with Foundry
forge verify-contract \
  --chain-id 1 \
  --compiler-version v0.8.20 \
  --optimizer-runs 1000000 \
  0x1234567890abcdef1234567890abcdef12345678 \
  contracts/core/Factory.sol:Factory \
  --etherscan-api-key $ETHERSCAN_API_KEY
```

---

## Infrastructure Setup

### Node Infrastructure

**Option 1: Managed Node Services**
```bash
# Alchemy
MAINNET_RPC=https://eth-mainnet.g.alchemy.com/v2/YOUR_API_KEY

# Infura
MAINNET_RPC=https://mainnet.infura.io/v3/YOUR_PROJECT_ID

# QuickNode
MAINNET_RPC=https://YOUR_NODE.quiknode.pro/YOUR_TOKEN/
```

**Option 2: Self-Hosted Archive Node**
```yaml
# docker-compose.yml
version: '3.8'
services:
  geth:
    image: ethereum/client-go:v1.13.5
    command:
      - --mainnet
      - --syncmode=snap
      - --gcmode=archive
      - --http
      - --http.addr=0.0.0.0
      - --http.port=8545
      - --http.api=eth,net,web3,debug,txpool
      - --ws
      - --ws.addr=0.0.0.0
      - --ws.port=8546
      - --ws.api=eth,net,web3,debug,txpool
    volumes:
      - geth-data:/root/.ethereum
    ports:
      - "8545:8545"
      - "8546:8546"
    restart: unless-stopped

volumes:
  geth-data:
```

### API Backend

```yaml
# docker-compose.yml
version: '3.8'
services:
  api:
    build: ./api
    ports:
      - "3000:3000"
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/defi
      - REDIS_URL=redis://redis:6379
      - RPC_URL=http://geth:8545
    depends_on:
      - db
      - redis

  db:
    image: postgres:15
    environment:
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=pass
      - POSTGRES_DB=defi
    volumes:
      - postgres-data:/var/lib/postgresql/data
    ports:
      - "5432:5432"

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis-data:/data

  the-graph-node:
    image: graphprotocol/graph-node:latest
    ports:
      - "8000:8000"
      - "8001:8001"
      - "8020:8020"
      - "8030:8030"
      - "8040:8040"
    environment:
      - postgres_host=db
      - postgres_user=user
      - postgres_pass=pass
      - postgres_db=defi
      - ethereum=mainnet:http://geth:8545
      - GRAPH_LOG=info

volumes:
  postgres-data:
  redis-data:
```

### The Graph Subgraph

```yaml
# subgraph.yaml
specVersion: 0.0.5
schema:
  file: ./schema.graphql
dataSources:
  - kind: ethereum
    name: Factory
    network: mainnet
    source:
      address: "0x1234567890abcdef1234567890abcdef12345678"
      abi: Factory
      startBlock: 18500000
    mapping:
      kind: ethereum/events
      apiVersion: 0.0.7
      language: wasm/assemblyscript
      entities:
        - Pool
        - Token
        - Swap
      abis:
        - name: Factory
          file: ./abis/Factory.json
        - name: Pool
          file: ./abis/Pool.json
      eventHandlers:
        - event: PoolCreated(indexed address,indexed address,indexed uint24,int24,address)
          handler: handlePoolCreated
        - event: Swap(indexed address,indexed address,int256,int256,uint160,uint128,int24)
          handler: handleSwap
      file: ./src/mapping.ts
```

```typescript
// src/mapping.ts
import { PoolCreated, Swap } from "../generated/Factory/Factory";
import { Pool, Token, SwapEvent } from "../generated/schema";

export function handlePoolCreated(event: PoolCreated): void {
  let pool = new Pool(event.params.pool.toHex());
  pool.token0 = event.params.token0.toHex();
  pool.token1 = event.params.token1.toHex();
  pool.feeTier = event.params.fee;
  pool.tickSpacing = event.params.tickSpacing;
  pool.createdAt = event.block.timestamp;
  pool.save();
}

export function handleSwap(event: Swap): void {
  let swap = new SwapEvent(
    event.transaction.hash.toHex() + "-" + event.logIndex.toString()
  );
  swap.pool = event.address.toHex();
  swap.sender = event.params.sender.toHex();
  swap.recipient = event.params.recipient.toHex();
  swap.amount0 = event.params.amount0;
  swap.amount1 = event.params.amount1;
  swap.timestamp = event.block.timestamp;
  swap.save();
}
```

---

## Monitoring & Analytics

### Prometheus + Grafana Setup

```yaml
# prometheus.yml
global:
  scrape_interval: 15s

scrape_configs:
  - job_name: 'defi-api'
    static_configs:
      - targets: ['api:3000']

  - job_name: 'geth'
    static_configs:
      - targets: ['geth:6060']

  - job_name: 'postgres'
    static_configs:
      - targets: ['postgres-exporter:9187']
```

```yaml
# docker-compose.monitoring.yml
version: '3.8'
services:
  prometheus:
    image: prom/prometheus:latest
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
      - prometheus-data:/prometheus
    ports:
      - "9090:9090"

  grafana:
    image: grafana/grafana:latest
    ports:
      - "3001:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin
    volumes:
      - grafana-data:/var/lib/grafana
      - ./grafana/dashboards:/etc/grafana/provisioning/dashboards

  alertmanager:
    image: prom/alertmanager:latest
    ports:
      - "9093:9093"
    volumes:
      - ./alertmanager.yml:/etc/alertmanager/alertmanager.yml

volumes:
  prometheus-data:
  grafana-data:
```

### Custom Metrics

```typescript
// metrics.ts
import { register, Counter, Gauge, Histogram } from 'prom-client';

export const swapCounter = new Counter({
  name: 'defi_swaps_total',
  help: 'Total number of swaps executed',
  labelNames: ['pool', 'status'],
});

export const tvlGauge = new Gauge({
  name: 'defi_tvl_usd',
  help: 'Total Value Locked in USD',
  labelNames: ['protocol', 'chain'],
});

export const apiLatency = new Histogram({
  name: 'defi_api_duration_seconds',
  help: 'API request duration in seconds',
  labelNames: ['method', 'endpoint', 'status'],
  buckets: [0.1, 0.5, 1, 2, 5],
});

// Middleware
app.use((req, res, next) => {
  const start = Date.now();

  res.on('finish', () => {
    const duration = (Date.now() - start) / 1000;
    apiLatency.labels(req.method, req.path, res.statusCode.toString()).observe(duration);
  });

  next();
});
```

### Dune Analytics Integration

```sql
-- Query: Daily Protocol Volume
SELECT
  DATE_TRUNC('day', evt_block_time) AS date,
  SUM(amount0 * price_usd) AS volume_usd
FROM uniswap_v4.swap
WHERE pool = 0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640
  AND evt_block_time >= NOW() - INTERVAL '30 days'
GROUP BY 1
ORDER BY 1 DESC;

-- Query: Top Liquidity Providers
SELECT
  provider,
  SUM(liquidity) AS total_liquidity,
  COUNT(*) AS position_count
FROM uniswap_v4.positions
WHERE pool = 0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640
  AND is_active = TRUE
GROUP BY 1
ORDER BY 2 DESC
LIMIT 100;
```

---

## Maintenance & Upgrades

### Upgradeable Contracts (Proxy Pattern)

```solidity
// contracts/proxy/TransparentUpgradeableProxy.sol
import "@openzeppelin/contracts/proxy/transparent/TransparentUpgradeableProxy.sol";

// contracts/core/LiquidityPoolV1.sol
import "@openzeppelin/contracts-upgradeable/proxy/utils/Initializable.sol";
import "@openzeppelin/contracts-upgradeable/access/OwnableUpgradeable.sol";

contract LiquidityPoolV1 is Initializable, OwnableUpgradeable {
    uint256 public version;

    function initialize() public initializer {
        __Ownable_init();
        version = 1;
    }

    // Existing functions...
}

// contracts/core/LiquidityPoolV2.sol
contract LiquidityPoolV2 is LiquidityPoolV1 {
    uint256 public newFeature;

    function initializeV2(uint256 _newFeature) public reinitializer(2) {
        version = 2;
        newFeature = _newFeature;
    }

    // New functions...
}
```

### Upgrade Script

```typescript
// scripts/upgrade.ts
import { ethers, upgrades } from "hardhat";

async function main() {
  const proxyAddress = "0x..."; // Existing proxy address

  // Deploy new implementation
  const LiquidityPoolV2 = await ethers.getContractFactory("LiquidityPoolV2");

  console.log("Upgrading LiquidityPool...");
  const upgraded = await upgrades.upgradeProxy(proxyAddress, LiquidityPoolV2);

  console.log("LiquidityPool upgraded to:", upgraded.address);

  // Call new initializer if needed
  await upgraded.initializeV2(100);

  console.log("Upgrade complete!");
}

main()
  .then(() => process.exit(0))
  .catch((error) => {
    console.error(error);
    process.exit(1);
  });
```

### Emergency Procedures

```solidity
// Emergency pause contract
contract EmergencyPause is Ownable {
    bool public paused;

    event Paused(address account);
    event Unpaused(address account);

    modifier whenNotPaused() {
        require(!paused, "Contract is paused");
        _;
    }

    function pause() external onlyOwner {
        paused = true;
        emit Paused(msg.sender);
    }

    function unpause() external onlyOwner {
        paused = false;
        emit Unpaused(msg.sender);
    }
}
```

---

## Compliance & Governance

### Governance System

```solidity
// contracts/governance/GovernanceToken.sol
import "@openzeppelin/contracts/token/ERC20/extensions/ERC20Votes.sol";

contract GovernanceToken is ERC20Votes {
    constructor() ERC20("Governance Token", "GOV") ERC20Permit("Governance Token") {
        _mint(msg.sender, 1000000 * 10**18);
    }
}

// contracts/governance/Governor.sol
import "@openzeppelin/contracts/governance/Governor.sol";
import "@openzeppelin/contracts/governance/extensions/GovernorSettings.sol";
import "@openzeppelin/contracts/governance/extensions/GovernorCountingSimple.sol";
import "@openzeppelin/contracts/governance/extensions/GovernorVotes.sol";
import "@openzeppelin/contracts/governance/extensions/GovernorTimelockControl.sol";

contract Esports Industry StandardGovernor is
    Governor,
    GovernorSettings,
    GovernorCountingSimple,
    GovernorVotes,
    GovernorTimelockControl
{
    constructor(
        IVotes _token,
        TimelockController _timelock
    )
        Governor("Esports Industry Standard Governor")
        GovernorSettings(
            1, /* 1 block voting delay */
            50400, /* 1 week voting period */
            100000e18 /* 100k tokens proposal threshold */
        )
        GovernorVotes(_token)
        GovernorTimelockControl(_timelock)
    {}

    function votingDelay() public view override(IGovernor, GovernorSettings) returns (uint256) {
        return super.votingDelay();
    }

    function votingPeriod() public view override(IGovernor, GovernorSettings) returns (uint256) {
        return super.votingPeriod();
    }

    function proposalThreshold() public view override(Governor, GovernorSettings) returns (uint256) {
        return super.proposalThreshold();
    }
}
```

### Governance Process

1. **Proposal Creation**: Token holder creates proposal
2. **Voting Delay**: 1 day delay before voting starts
3. **Voting Period**: 7 days voting period
4. **Quorum**: Minimum 4% of total supply must vote
5. **Timelock**: 48-hour delay before execution
6. **Execution**: Proposal automatically executed if passed

### Compliance Monitoring

```typescript
// compliance/aml.ts
import { Address, Transaction } from './types';

async function checkAMLCompliance(address: Address): Promise<boolean> {
  // Check against OFAC sanctions list
  const sanctioned = await checkSanctionsList(address);
  if (sanctioned) return false;

  // Check transaction patterns
  const txHistory = await getTransactionHistory(address);
  const suspicious = detectSuspiciousActivity(txHistory);
  if (suspicious) {
    await flagForReview(address);
    return false;
  }

  return true;
}

function detectSuspiciousActivity(txs: Transaction[]): boolean {
  // Rapid in/out (potential mixing)
  const rapidInOut = txs.filter(tx =>
    tx.type === 'deposit' &&
    txs.find(t => t.type === 'withdrawal' && t.timestamp - tx.timestamp < 3600)
  );

  if (rapidInOut.length > 5) return true;

  // Large round number transactions (potential structuring)
  const roundNumbers = txs.filter(tx =>
    tx.amount % 1000 === 0 && tx.amount >= 10000
  );

  if (roundNumbers.length > 10) return true;

  return false;
}
```

---

## Appendix

### Deployment Checklist

- [ ] All tests passing (unit, integration, fork)
- [ ] Gas optimization reviewed
- [ ] Security audit completed (2+ firms)
- [ ] Bug bounty program launched
- [ ] Testnet deployment successful
- [ ] Multi-sig wallet configured
- [ ] Emergency pause mechanism tested
- [ ] Monitoring & alerting configured
- [ ] Documentation complete
- [ ] Governance parameters finalized
- [ ] Compliance requirements met
- [ ] Insurance coverage obtained (if applicable)
- [ ] Community notification sent
- [ ] Mainnet deployment executed
- [ ] Contracts verified on Etherscan
- [ ] Subgraph deployed
- [ ] Frontend updated
- [ ] Post-deployment monitoring active

### Useful Resources

- **Hardhat**: https://hardhat.org
- **Foundry**: https://getfoundry.sh
- **OpenZeppelin**: https://openzeppelin.com
- **The Graph**: https://thegraph.com
- **Dune Analytics**: https://dune.com
- **Etherscan**: https://etherscan.io
- **Tenderly**: https://tenderly.co

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (Hongik Ingan) - Benefit All Humanity

Licensed under MIT License
