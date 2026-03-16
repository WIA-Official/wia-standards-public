# WIA-SOC-002 PHASE 4: Integration Specification

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

This document specifies integration patterns for WIA-SOC-002 digital identity with blockchain networks, identity providers, wallets, and enterprise systems.

### 1.1 Integration Architecture

```
┌──────────────────────────────────────────────────────┐
│              Application Layer                        │
├──────────────────────────────────────────────────────┤
│   WIA-SOC-002 SDK   │   Identity Wallet   │  dApps   │
├──────────────────────────────────────────────────────┤
│   DID Resolver  │  Credential Manager  │  ZK Proofs │
├──────────────────────────────────────────────────────┤
│  Blockchain Layer (Ethereum, Polygon, Solana, etc)   │
├──────────────────────────────────────────────────────┤
│      IPFS/Arweave      │      Smart Contracts        │
└──────────────────────────────────────────────────────┘
```

---

## 2. Blockchain Integration

### 2.1 Ethereum Integration

#### Smart Contract Deployment

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

contract WIAIdentityRegistry {
    struct DIDDocument {
        string document;
        address controller;
        uint256 created;
        uint256 updated;
        bool active;
    }

    mapping(bytes32 => DIDDocument) public didDocuments;
    mapping(address => bytes32[]) public controllerDIDs;

    event DIDCreated(bytes32 indexed didHash, address indexed controller);
    event DIDUpdated(bytes32 indexed didHash, uint256 timestamp);
    event DIDDeactivated(bytes32 indexed didHash, uint256 timestamp);

    function createDID(
        bytes32 didHash,
        string memory document
    ) external {
        require(!didDocuments[didHash].active, "DID already exists");

        didDocuments[didHash] = DIDDocument({
            document: document,
            controller: msg.sender,
            created: block.timestamp,
            updated: block.timestamp,
            active: true
        });

        controllerDIDs[msg.sender].push(didHash);

        emit DIDCreated(didHash, msg.sender);
    }

    function updateDID(
        bytes32 didHash,
        string memory document
    ) external {
        require(didDocuments[didHash].active, "DID not found");
        require(
            didDocuments[didHash].controller == msg.sender,
            "Not authorized"
        );

        didDocuments[didHash].document = document;
        didDocuments[didHash].updated = block.timestamp;

        emit DIDUpdated(didHash, block.timestamp);
    }

    function deactivateDID(bytes32 didHash) external {
        require(didDocuments[didHash].active, "DID not found");
        require(
            didDocuments[didHash].controller == msg.sender,
            "Not authorized"
        );

        didDocuments[didHash].active = false;

        emit DIDDeactivated(didHash, block.timestamp);
    }

    function resolveDID(bytes32 didHash)
        external
        view
        returns (string memory)
    {
        require(didDocuments[didHash].active, "DID not found or deactivated");
        return didDocuments[didHash].document;
    }
}
```

#### Integration Example

```typescript
import { ethers } from 'ethers';
import { WIAIdentityRegistry__factory } from './typechain';

const provider = new ethers.providers.JsonRpcProvider(
  'https://mainnet.infura.io/v3/YOUR-PROJECT-ID'
);

const wallet = new ethers.Wallet(privateKey, provider);

const registry = WIAIdentityRegistry__factory.connect(
  REGISTRY_ADDRESS,
  wallet
);

// Create DID
const didDocument = {
  '@context': ['https://www.w3.org/ns/did/v1'],
  id: 'did:wia:mainnet:0x7a3f...',
  verificationMethod: [...]
};

const didHash = ethers.utils.keccak256(
  ethers.utils.toUtf8Bytes('did:wia:mainnet:0x7a3f...')
);

const tx = await registry.createDID(
  didHash,
  JSON.stringify(didDocument)
);

await tx.wait();
```

### 2.2 Polygon Integration

```typescript
import { polygon } from '@maticnetwork/maticjs';

// Connect to Polygon
const posClient = new polygon.POSClient();

await posClient.init({
  network: 'mainnet',
  version: 'v1',
  parent: {
    provider: new ethers.providers.JsonRpcProvider(ethereumRPC),
    defaultConfig: { from: senderAddress }
  },
  child: {
    provider: new ethers.providers.JsonRpcProvider(polygonRPC),
    defaultConfig: { from: senderAddress }
  }
});

// Deploy DID registry on Polygon
const registry = await deployContract(
  'WIAIdentityRegistry',
  posClient.child.provider
);
```

### 2.3 Solana Integration

```typescript
import { Connection, PublicKey, Transaction } from '@solana/web3.js';
import { Program, AnchorProvider } from '@project-serum/anchor';

// Connect to Solana
const connection = new Connection('https://api.mainnet-beta.solana.com');

// Create DID account
const createDIDInstruction = program.instruction.createDid(
  didDocument,
  {
    accounts: {
      didAccount: didAccountPubkey,
      authority: authorityPubkey,
      systemProgram: SystemProgram.programId
    }
  }
);

const transaction = new Transaction().add(createDIDInstruction);
const signature = await connection.sendTransaction(transaction, [payer]);
```

---

## 3. Identity Wallet Integration

### 3.1 MetaMask Integration

```typescript
import { MetaMaskSDK } from '@metamask/sdk';

const sdk = new MetaMaskSDK();
const provider = sdk.getProvider();

// Request account access
await provider.request({ method: 'eth_requestAccounts' });

// Sign DID authentication challenge
const accounts = await provider.request({ method: 'eth_accounts' });
const signature = await provider.request({
  method: 'personal_sign',
  params: [challenge, accounts[0]]
});

// Create DID from Ethereum address
const did = `did:ethr:${accounts[0]}`;
```

### 3.2 WalletConnect Integration

```typescript
import { WalletConnectConnector } from '@web3-react/walletconnect-connector';

const connector = new WalletConnectConnector({
  rpc: { 1: 'https://mainnet.infura.io/v3/YOUR-PROJECT-ID' },
  bridge: 'https://bridge.walletconnect.org',
  qrcode: true
});

await connector.activate();

// Sign credential
const signature = await connector.signMessage(credentialHash);
```

### 3.3 Hardware Wallet Integration

```typescript
import TransportWebUSB from '@ledgerhq/hw-transport-webusb';
import Eth from '@ledgerhq/hw-app-eth';

// Connect to Ledger
const transport = await TransportWebUSB.create();
const eth = new Eth(transport);

// Get address
const { address } = await eth.getAddress("44'/60'/0'/0/0");

// Sign credential
const signature = await eth.signPersonalMessage(
  "44'/60'/0'/0/0",
  credentialHash
);
```

---

## 4. Identity Provider Integration

### 4.1 OAuth 2.0 Bridge

```typescript
// Issue VC from OAuth token
async function issueVCFromOAuth(
  oauthToken: string,
  clientId: string
): Promise<VerifiableCredential> {
  // 1. Verify OAuth token
  const userInfo = await verifyOAuthToken(oauthToken, clientId);

  // 2. Create credential
  const credential = {
    '@context': [
      'https://www.w3.org/2018/credentials/v1',
      'https://wiastandards.com/credentials/v1'
    ],
    type: ['VerifiableCredential', 'OAuthCredential'],
    issuer: 'did:wia:mainnet:0x1234...',
    credentialSubject: {
      id: `did:wia:mainnet:${userInfo.sub}`,
      email: userInfo.email,
      name: userInfo.name,
      verified: userInfo.email_verified
    }
  };

  // 3. Sign credential
  return signCredential(credential, issuerPrivateKey);
}
```

### 4.2 SAML Integration

```typescript
import { SAML } from 'passport-saml';

// Parse SAML assertion
async function issuVCFromSAML(
  samlAssertion: string
): Promise<VerifiableCredential> {
  const saml = new SAML({
    cert: idpCertificate,
    issuer: 'wia-identity-bridge'
  });

  const profile = await saml.validatePostResponse({
    SAMLResponse: samlAssertion
  });

  return {
    '@context': ['https://www.w3.org/2018/credentials/v1'],
    type: ['VerifiableCredential', 'SAMLCredential'],
    issuer: 'did:wia:mainnet:0x1234...',
    credentialSubject: {
      id: `did:wia:mainnet:${profile.nameID}`,
      attributes: profile.attributes
    }
  };
}
```

### 4.3 OpenID Connect

```typescript
import { Issuer } from 'openid-client';

// Discover OIDC provider
const issuer = await Issuer.discover('https://accounts.google.com');

// Create client
const client = new issuer.Client({
  client_id: 'your-client-id',
  client_secret: 'your-client-secret',
  redirect_uris: ['http://localhost:3000/callback'],
  response_types: ['code']
});

// Exchange code for tokens
const tokenSet = await client.callback(
  'http://localhost:3000/callback',
  params
);

// Issue VC
const credential = await issueVCFromOIDC(tokenSet.id_token);
```

---

## 5. Enterprise Integration

### 5.1 Active Directory Integration

```typescript
import { Client } from 'ldapts';

async function syncADToVCs() {
  const client = new Client({
    url: 'ldap://ad.company.com:389'
  });

  await client.bind('CN=admin,DC=company,DC=com', 'password');

  const { searchEntries } = await client.search(
    'DC=company,DC=com',
    {
      filter: '(&(objectClass=user)(mail=*))'
    }
  );

  for (const entry of searchEntries) {
    const credential = {
      '@context': ['https://www.w3.org/2018/credentials/v1'],
      type: ['VerifiableCredential', 'EmployeeCredential'],
      credentialSubject: {
        id: `did:wia:mainnet:${entry.objectGUID}`,
        email: entry.mail,
        department: entry.department,
        title: entry.title
      }
    };

    await issueCredential(credential);
  }
}
```

### 5.2 SAP Integration

```typescript
import axios from 'axios';

// Fetch employee data from SAP
async function syncSAPToVCs() {
  const response = await axios.get(
    'https://sap-system.company.com/sap/opu/odata/sap/EMPLOYEE_SRV/Employees',
    {
      auth: {
        username: 'sap-user',
        password: 'sap-password'
      }
    }
  );

  for (const employee of response.data.d.results) {
    const credential = {
      '@context': ['https://www.w3.org/2018/credentials/v1'],
      type: ['VerifiableCredential', 'EmployeeCredential'],
      credentialSubject: {
        id: `did:wia:mainnet:${employee.EmployeeID}`,
        name: employee.Name,
        department: employee.Department,
        hireDate: employee.HireDate
      }
    };

    await issueCredential(credential);
  }
}
```

### 5.3 Salesforce Integration

```typescript
import jsforce from 'jsforce';

const conn = new jsforce.Connection({
  loginUrl: 'https://login.salesforce.com'
});

await conn.login('username', 'password');

// Query contacts
const contacts = await conn.query(
  'SELECT Id, Name, Email FROM Contact WHERE Email != null'
);

// Issue credentials
for (const contact of contacts.records) {
  const credential = {
    '@context': ['https://www.w3.org/2018/credentials/v1'],
    type: ['VerifiableCredential', 'ContactCredential'],
    credentialSubject: {
      id: `did:wia:mainnet:${contact.Id}`,
      name: contact.Name,
      email: contact.Email
    }
  };

  await issueCredential(credential);
}
```

---

## 6. Storage Integration

### 6.1 IPFS Integration

```typescript
import { create } from 'ipfs-http-client';

const ipfs = create({ url: 'https://ipfs.infura.io:5001' });

// Store DID document on IPFS
async function storeDIDOnIPFS(didDocument: any): Promise<string> {
  const { cid } = await ipfs.add(JSON.stringify(didDocument));
  return `ipfs://${cid}`;
}

// Retrieve DID document from IPFS
async function retrieveDIDFromIPFS(cid: string): Promise<any> {
  const stream = ipfs.cat(cid);
  const decoder = new TextDecoder();
  let data = '';

  for await (const chunk of stream) {
    data += decoder.decode(chunk, { stream: true });
  }

  return JSON.parse(data);
}
```

### 6.2 Arweave Integration

```typescript
import Arweave from 'arweave';

const arweave = Arweave.init({
  host: 'arweave.net',
  port: 443,
  protocol: 'https'
});

// Store credential permanently on Arweave
async function storeCredentialOnArweave(
  credential: VerifiableCredential,
  wallet: any
): Promise<string> {
  const transaction = await arweave.createTransaction(
    {
      data: JSON.stringify(credential)
    },
    wallet
  );

  transaction.addTag('Content-Type', 'application/json');
  transaction.addTag('App-Name', 'WIA-Identity');
  transaction.addTag('Type', 'VerifiableCredential');

  await arweave.transactions.sign(transaction, wallet);
  await arweave.transactions.post(transaction);

  return transaction.id;
}
```

---

## 7. Interoperability

### 7.1 DIF Universal Resolver

```typescript
import { UniversalResolver } from 'did-resolver';

const resolver = new UniversalResolver({
  url: 'https://dev.uniresolver.io/1.0/identifiers/'
});

// Resolve any DID method
const didDocument = await resolver.resolve('did:ion:EiD...');
```

### 7.2 W3C Verifiable Credentials

```typescript
import { verifyCredential } from '@digitalbazaar/vc';

// Verify W3C VC
const result = await verifyCredential({
  credential: vcFromOtherProvider,
  suite: new Ed25519Signature2020(),
  documentLoader
});

console.log('Verified:', result.verified);
```

---

## 8. Government Integration

### 8.1 eIDAS Integration

```typescript
// Issue eIDAS-compliant credential
async function issueEIDASCredential(
  citizenData: any
): Promise<VerifiableCredential> {
  return {
    '@context': [
      'https://www.w3.org/2018/credentials/v1',
      'https://eidas.europa.eu/contexts/v1'
    ],
    type: ['VerifiableCredential', 'eIDASCredential'],
    issuer: {
      id: 'did:wia:mainnet:0x1234...',
      name: 'National eID Authority',
      trustFramework: 'eIDAS'
    },
    credentialSubject: {
      id: `did:wia:mainnet:${citizenData.id}`,
      givenName: citizenData.givenName,
      familyName: citizenData.familyName,
      birthDate: citizenData.birthDate,
      nationality: citizenData.nationality,
      assuranceLevel: 'high'
    },
    termsOfUse: [{
      type: 'eIDASPolicy',
      trustFramework: 'eIDAS',
      assuranceLevel: 'high'
    }]
  };
}
```

### 8.2 Digital Identity Wallet (EU)

```typescript
// EU Digital Identity Wallet integration
class EUDIWallet {
  async requestPresentation(
    attributes: string[]
  ): Promise<VerifiablePresentation> {
    // 1. User selects credentials
    const selectedCredentials = await this.selectCredentials(attributes);

    // 2. Create presentation
    const presentation = {
      '@context': ['https://www.w3.org/2018/credentials/v1'],
      type: 'VerifiablePresentation',
      holder: this.did,
      verifiableCredential: selectedCredentials
    };

    // 3. Sign presentation
    return this.signPresentation(presentation);
  }
}
```

---

## 9. Mobile Integration

### 9.1 iOS Integration

```swift
import Web3Swift
import WIAIdentitySDK

class IdentityManager {
    let wiaClient: WIAClient

    init() {
        wiaClient = WIAClient(
            apiUrl: "https://api.wiastandards.com/v1/identity"
        )
    }

    func createDID() async throws -> DIDDocument {
        let keyPair = try generateKeyPair()
        let did = try await wiaClient.createDID(
            method: "wia",
            network: "mainnet",
            publicKey: keyPair.publicKey
        )
        return did
    }

    func presentCredential(
        credential: VerifiableCredential,
        to verifier: String
    ) async throws -> VerifiablePresentation {
        let presentation = try await wiaClient.createPresentation(
            holder: self.did,
            credentials: [credential],
            challenge: verifier
        )
        return presentation
    }
}
```

### 9.2 Android Integration

```kotlin
import com.wia.identity.WIAIdentitySDK

class IdentityManager(context: Context) {
    private val wiaClient = WIAIdentitySDK.Builder()
        .apiUrl("https://api.wiastandards.com/v1/identity")
        .build()

    suspend fun createDID(): DIDDocument {
        val keyPair = generateKeyPair()
        return wiaClient.createDID(
            method = "wia",
            network = "mainnet",
            publicKey = keyPair.publicKey
        )
    }

    suspend fun verifyCredential(
        credential: VerifiableCredential
    ): VerificationResult {
        return wiaClient.verifyCredential(credential)
    }
}
```

---

## 10. Analytics & Monitoring

### 10.1 Metrics Collection

```typescript
import { InfluxDB } from '@influxdata/influxdb-client';

const influxDB = new InfluxDB({
  url: 'http://localhost:8086',
  token: 'my-token'
});

const writeApi = influxDB.getWriteApi('wia', 'identity-metrics');

// Track DID operations
function trackDIDOperation(operation: string, success: boolean) {
  const point = new Point('did_operation')
    .tag('operation', operation)
    .tag('success', success.toString())
    .floatField('count', 1)
    .timestamp(new Date());

  writeApi.writePoint(point);
}

// Track credential issuance
function trackCredentialIssuance(type: string, issuer: string) {
  const point = new Point('credential_issued')
    .tag('type', type)
    .tag('issuer', issuer)
    .floatField('count', 1)
    .timestamp(new Date());

  writeApi.writePoint(point);
}
```

### 10.2 Error Tracking

```typescript
import * as Sentry from '@sentry/node';

Sentry.init({
  dsn: 'your-sentry-dsn',
  environment: 'production'
});

// Track verification errors
try {
  await verifyCredential(credential);
} catch (error) {
  Sentry.captureException(error, {
    tags: {
      operation: 'credential-verification',
      credentialType: credential.type
    },
    extra: {
      credentialId: credential.id,
      issuer: credential.issuer
    }
  });
}
```

---

## 11. Compliance & Auditing

### 11.1 Audit Logging

```typescript
class AuditLogger {
  async logDIDCreation(did: string, controller: string) {
    await this.log({
      event: 'DID_CREATED',
      timestamp: Date.now(),
      actor: controller,
      resource: did,
      action: 'CREATE',
      result: 'SUCCESS'
    });
  }

  async logCredentialIssuance(
    credential: VerifiableCredential,
    issuer: string
  ) {
    await this.log({
      event: 'CREDENTIAL_ISSUED',
      timestamp: Date.now(),
      actor: issuer,
      resource: credential.id,
      action: 'ISSUE',
      metadata: {
        type: credential.type,
        subject: credential.credentialSubject.id
      }
    });
  }
}
```

### 11.2 GDPR Compliance

```typescript
class GDPRManager {
  async exportUserData(did: string): Promise<UserDataExport> {
    // Right to data portability (Art. 20)
    const credentials = await getCredentials(did);
    const presentations = await getPresentations(did);

    return {
      did,
      credentials,
      presentations,
      exportDate: new Date().toISOString()
    };
  }

  async deleteUserData(did: string): Promise<void> {
    // Right to erasure (Art. 17)
    await deactivateDID(did);
    await revokeAllCredentials(did);
    await deleteOffChainData(did);
  }
}
```

---

## 12. Testing Integration

### 12.1 Testnet Configuration

```typescript
const config = {
  production: {
    network: 'mainnet',
    rpcUrl: 'https://mainnet.infura.io/v3/YOUR-PROJECT-ID',
    registryAddress: '0x1234567890abcdef...'
  },
  staging: {
    network: 'goerli',
    rpcUrl: 'https://goerli.infura.io/v3/YOUR-PROJECT-ID',
    registryAddress: '0xabcdef1234567890...'
  },
  development: {
    network: 'localhost',
    rpcUrl: 'http://localhost:8545',
    registryAddress: '0x5FbDB2315678afecb367f032d93F642f64180aa3'
  }
};
```

### 12.2 Integration Tests

```typescript
import { expect } from 'chai';

describe('WIA Identity Integration', () => {
  it('should create DID on blockchain', async () => {
    const did = await createDID({
      method: 'wia',
      network: 'goerli'
    });

    expect(did.id).to.match(/^did:wia:goerli:/);

    const resolved = await resolveDID(did.id);
    expect(resolved.id).to.equal(did.id);
  });

  it('should issue and verify credential', async () => {
    const credential = await issueCredential({
      type: ['VerifiableCredential', 'TestCredential'],
      subject: testDID
    });

    const result = await verifyCredential(credential);
    expect(result.verified).to.be.true;
  });
});
```

---

**Document Status:** Complete ✅
**Implementation:** Ready for production deployment

弘益人間 · Benefit All Humanity
