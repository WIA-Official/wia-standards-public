# Phase 4: System Integration

## WIA-SEC-009 Identity Management - Integration Specification

**Version**: 1.0.0
**Date**: 2025-12-25
**Status**: Active
**Standard ID**: WIA-SEC-009-PHASE4-004
**Primary Color**: #8B5CF6 (Purple)

---

## 1. Overview

This phase defines integration patterns with IAM systems (LDAP, Active Directory, Azure AD, Okta), authentication protocols (OAuth, OIDC, SAML), and identity federation.

---

## 2. LDAP Integration

### 2.1 LDAP Connection Configuration

```json
{
  "ldapConfig": {
    "url": "ldaps://ldap.example.com:636",
    "baseDN": "dc=example,dc=com",
    "bindDN": "cn=admin,dc=example,dc=com",
    "bindPassword": "${LDAP_BIND_PASSWORD}",
    "searchBase": "ou=users,dc=example,dc=com",
    "searchFilter": "(uid={0})",
    "groupSearchBase": "ou=groups,dc=example,dc=com",
    "groupSearchFilter": "(member={0})",
    "tlsOptions": {
      "rejectUnauthorized": true,
      "ca": "/path/to/ca-cert.pem"
    },
    "attributeMapping": {
      "username": "uid",
      "email": "mail",
      "firstName": "givenName",
      "lastName": "sn",
      "displayName": "cn",
      "groups": "memberOf"
    }
  }
}
```

### 2.2 LDAP Authentication

```javascript
const ldap = require('ldapjs');

/**
 * Authenticate user against LDAP directory
 */
async function authenticateLDAP(username, password) {
  const client = ldap.createClient({
    url: 'ldaps://ldap.example.com:636',
    tlsOptions: {
      ca: [fs.readFileSync('/path/to/ca-cert.pem')]
    }
  });

  return new Promise((resolve, reject) => {
    // Search for user
    const searchDN = 'ou=users,dc=example,dc=com';
    const searchFilter = `(uid=${username})`;

    client.search(searchDN, { filter: searchFilter, scope: 'sub' }, (err, res) => {
      if (err) return reject(err);

      let userDN = null;
      let userData = null;

      res.on('searchEntry', (entry) => {
        userDN = entry.objectName;
        userData = {
          uid: entry.object.uid,
          email: entry.object.mail,
          givenName: entry.object.givenName,
          sn: entry.object.sn,
          cn: entry.object.cn,
          memberOf: Array.isArray(entry.object.memberOf)
            ? entry.object.memberOf
            : [entry.object.memberOf]
        };
      });

      res.on('end', () => {
        if (!userDN) {
          return reject(new Error('User not found'));
        }

        // Attempt to bind with user credentials
        client.bind(userDN, password, (bindErr) => {
          client.unbind();

          if (bindErr) {
            return reject(new Error('Authentication failed'));
          }

          resolve({
            authenticated: true,
            user: userData
          });
        });
      });
    });
  });
}
```

### 2.3 LDAP User Synchronization

```javascript
/**
 * Sync users from LDAP to local identity store
 */
async function syncLDAPUsers() {
  const client = createLDAPClient();

  const opts = {
    filter: '(objectClass=inetOrgPerson)',
    scope: 'sub',
    attributes: ['uid', 'cn', 'mail', 'givenName', 'sn', 'memberOf', 'employeeNumber']
  };

  return new Promise((resolve, reject) => {
    const users = [];

    client.search('ou=users,dc=example,dc=com', opts, (err, res) => {
      if (err) return reject(err);

      res.on('searchEntry', async (entry) => {
        const ldapUser = entry.object;

        const user = {
          username: ldapUser.uid,
          email: ldapUser.mail,
          givenName: ldapUser.givenName,
          familyName: ldapUser.sn,
          displayName: ldapUser.cn,
          employeeNumber: ldapUser.employeeNumber,
          groups: extractGroupNames(ldapUser.memberOf),
          source: 'ldap',
          lastSync: new Date().toISOString()
        };

        // Upsert to local database
        await upsertUser(user);
        users.push(user);
      });

      res.on('end', () => {
        client.unbind();
        resolve({
          syncedUsers: users.length,
          timestamp: new Date().toISOString()
        });
      });
    });
  });
}

function extractGroupNames(memberOf) {
  if (!memberOf) return [];

  const groups = Array.isArray(memberOf) ? memberOf : [memberOf];

  return groups.map(dn => {
    const match = dn.match(/cn=([^,]+)/i);
    return match ? match[1] : null;
  }).filter(Boolean);
}
```

---

## 3. Active Directory Integration

### 3.1 Azure AD Configuration

```json
{
  "azureAD": {
    "tenantId": "12345678-1234-1234-1234-123456789012",
    "clientId": "abcdef01-2345-6789-abcd-ef0123456789",
    "clientSecret": "${AZURE_AD_CLIENT_SECRET}",
    "authority": "https://login.microsoftonline.com/12345678-1234-1234-1234-123456789012",
    "redirectUri": "https://app.example.com/auth/callback",
    "scopes": [
      "openid",
      "profile",
      "email",
      "User.Read",
      "Group.Read.All"
    ],
    "graphApiEndpoint": "https://graph.microsoft.com/v1.0"
  }
}
```

### 3.2 Azure AD Authentication (Node.js)

```javascript
const msal = require('@azure/msal-node');

const config = {
  auth: {
    clientId: process.env.AZURE_AD_CLIENT_ID,
    authority: `https://login.microsoftonline.com/${process.env.AZURE_AD_TENANT_ID}`,
    clientSecret: process.env.AZURE_AD_CLIENT_SECRET
  }
};

const cca = new msal.ConfidentialClientApplication(config);

/**
 * Get Azure AD access token
 */
async function getAzureADToken() {
  const tokenRequest = {
    scopes: ['https://graph.microsoft.com/.default']
  };

  try {
    const response = await cca.acquireTokenByClientCredential(tokenRequest);
    return response.accessToken;
  } catch (error) {
    console.error('Failed to acquire token:', error);
    throw error;
  }
}

/**
 * Get user from Microsoft Graph
 */
async function getAzureADUser(userId) {
  const accessToken = await getAzureADToken();

  const response = await fetch(
    `https://graph.microsoft.com/v1.0/users/${userId}`,
    {
      headers: {
        'Authorization': `Bearer ${accessToken}`
      }
    }
  );

  const user = await response.json();

  return {
    id: user.id,
    username: user.userPrincipalName,
    email: user.mail || user.userPrincipalName,
    displayName: user.displayName,
    givenName: user.givenName,
    surname: user.surname,
    jobTitle: user.jobTitle,
    department: user.department,
    officeLocation: user.officeLocation,
    mobilePhone: user.mobilePhone
  };
}

/**
 * Get user's group memberships
 */
async function getAzureADUserGroups(userId) {
  const accessToken = await getAzureADToken();

  const response = await fetch(
    `https://graph.microsoft.com/v1.0/users/${userId}/memberOf`,
    {
      headers: {
        'Authorization': `Bearer ${accessToken}`
      }
    }
  );

  const data = await response.json();

  return data.value.map(group => ({
    id: group.id,
    displayName: group.displayName,
    description: group.description
  }));
}
```

### 3.3 On-Premises Active Directory Integration

```csharp
using System.DirectoryServices;
using System.DirectoryServices.AccountManagement;

public class ActiveDirectoryService
{
    private readonly string _domain;
    private readonly string _container;

    public ActiveDirectoryService(string domain, string container)
    {
        _domain = domain;
        _container = container;
    }

    /// <summary>
    /// Authenticate user against Active Directory
    /// </summary>
    public bool AuthenticateUser(string username, string password)
    {
        using (var context = new PrincipalContext(ContextType.Domain, _domain, _container))
        {
            return context.ValidateCredentials(username, password);
        }
    }

    /// <summary>
    /// Get user details from Active Directory
    /// </summary>
    public ADUser GetUser(string username)
    {
        using (var context = new PrincipalContext(ContextType.Domain, _domain, _container))
        {
            var userPrincipal = UserPrincipal.FindByIdentity(context, username);

            if (userPrincipal == null)
                return null;

            return new ADUser
            {
                SamAccountName = userPrincipal.SamAccountName,
                UserPrincipalName = userPrincipal.UserPrincipalName,
                DisplayName = userPrincipal.DisplayName,
                GivenName = userPrincipal.GivenName,
                Surname = userPrincipal.Surname,
                EmailAddress = userPrincipal.EmailAddress,
                Description = userPrincipal.Description,
                Enabled = userPrincipal.Enabled ?? false,
                LastLogon = userPrincipal.LastLogon,
                Groups = GetUserGroups(userPrincipal)
            };
        }
    }

    /// <summary>
    /// Get user's group memberships
    /// </summary>
    private List<string> GetUserGroups(UserPrincipal user)
    {
        var groups = new List<string>();

        var principalGroups = user.GetAuthorizationGroups();

        foreach (var group in principalGroups)
        {
            if (group is GroupPrincipal groupPrincipal)
            {
                groups.Add(groupPrincipal.Name);
            }
        }

        return groups;
    }
}
```

---

## 4. Okta Integration

### 4.1 Okta Configuration

```json
{
  "okta": {
    "orgUrl": "https://dev-123456.okta.com",
    "clientId": "0oa1b2c3d4e5f6g7h8i9",
    "clientSecret": "${OKTA_CLIENT_SECRET}",
    "issuer": "https://dev-123456.okta.com/oauth2/default",
    "redirectUri": "https://app.example.com/auth/callback",
    "scopes": ["openid", "profile", "email", "groups"],
    "apiToken": "${OKTA_API_TOKEN}"
  }
}
```

### 4.2 Okta Authentication

```javascript
const OktaJwtVerifier = require('@okta/jwt-verifier');

const oktaJwtVerifier = new OktaJwtVerifier({
  issuer: 'https://dev-123456.okta.com/oauth2/default',
  clientId: process.env.OKTA_CLIENT_ID,
  assertClaims: {
    aud: 'api://default',
  },
});

/**
 * Verify Okta JWT token
 */
async function verifyOktaToken(token) {
  try {
    const jwt = await oktaJwtVerifier.verifyAccessToken(token, 'api://default');

    return {
      valid: true,
      claims: jwt.claims,
      userId: jwt.claims.sub,
      username: jwt.claims.username,
      email: jwt.claims.email,
      groups: jwt.claims.groups || []
    };
  } catch (error) {
    return {
      valid: false,
      error: error.message
    };
  }
}

/**
 * Get user from Okta
 */
async function getOktaUser(userId) {
  const response = await fetch(
    `https://dev-123456.okta.com/api/v1/users/${userId}`,
    {
      headers: {
        'Authorization': `SSWS ${process.env.OKTA_API_TOKEN}`,
        'Accept': 'application/json'
      }
    }
  );

  const user = await response.json();

  return {
    id: user.id,
    status: user.status,
    created: user.created,
    lastLogin: user.lastLogin,
    profile: {
      login: user.profile.login,
      email: user.profile.email,
      firstName: user.profile.firstName,
      lastName: user.profile.lastName,
      displayName: `${user.profile.firstName} ${user.profile.lastName}`,
      mobilePhone: user.profile.mobilePhone,
      department: user.profile.department,
      title: user.profile.title
    }
  };
}

/**
 * Provision user in Okta
 */
async function createOktaUser(userData) {
  const response = await fetch(
    'https://dev-123456.okta.com/api/v1/users?activate=true',
    {
      method: 'POST',
      headers: {
        'Authorization': `SSWS ${process.env.OKTA_API_TOKEN}`,
        'Accept': 'application/json',
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        profile: {
          firstName: userData.givenName,
          lastName: userData.familyName,
          email: userData.email,
          login: userData.email,
          mobilePhone: userData.phoneNumber
        },
        credentials: {
          password: {
            value: generateSecurePassword()
          }
        }
      })
    }
  );

  return await response.json();
}
```

---

## 5. SAML 2.0 Integration

### 5.1 SAML Service Provider Configuration

```xml
<?xml version="1.0"?>
<EntityDescriptor xmlns="urn:oasis:names:tc:SAML:2.0:metadata"
                  entityID="https://sp.example.com/metadata">

  <SPSSODescriptor protocolSupportEnumeration="urn:oasis:names:tc:SAML:2.0:protocol">

    <KeyDescriptor use="signing">
      <KeyInfo xmlns="http://www.w3.org/2000/09/xmldsig#">
        <X509Data>
          <X509Certificate>
            MIIDXTCCAkWgAwIBAgIJAKZ...
          </X509Certificate>
        </X509Data>
      </KeyInfo>
    </KeyDescriptor>

    <NameIDFormat>urn:oasis:names:tc:SAML:1.1:nameid-format:emailAddress</NameIDFormat>

    <AssertionConsumerService Binding="urn:oasis:names:tc:SAML:2.0:bindings:HTTP-POST"
                              Location="https://sp.example.com/saml/acs"
                              index="1"/>

  </SPSSODescriptor>

</EntityDescriptor>
```

### 5.2 SAML Authentication Request

```xml
<?xml version="1.0"?>
<samlp:AuthnRequest xmlns:samlp="urn:oasis:names:tc:SAML:2.0:protocol"
                    xmlns:saml="urn:oasis:names:tc:SAML:2.0:assertion"
                    ID="_a1b2c3d4e5f6"
                    Version="2.0"
                    IssueInstant="2025-01-15T10:00:00Z"
                    Destination="https://idp.example.com/saml/sso"
                    AssertionConsumerServiceURL="https://sp.example.com/saml/acs"
                    ProtocolBinding="urn:oasis:names:tc:SAML:2.0:bindings:HTTP-POST">

  <saml:Issuer>https://sp.example.com/metadata</saml:Issuer>

  <samlp:NameIDPolicy Format="urn:oasis:names:tc:SAML:1.1:nameid-format:emailAddress"
                      AllowCreate="true"/>

</samlp:AuthnRequest>
```

### 5.3 SAML Response Validation

```javascript
const saml2 = require('saml2-js');

const sp = new saml2.ServiceProvider({
  entity_id: "https://sp.example.com/metadata",
  private_key: fs.readFileSync("sp-key.pem").toString(),
  certificate: fs.readFileSync("sp-cert.pem").toString(),
  assert_endpoint: "https://sp.example.com/saml/acs"
});

const idp = new saml2.IdentityProvider({
  sso_login_url: "https://idp.example.com/saml/sso",
  sso_logout_url: "https://idp.example.com/saml/slo",
  certificates: [fs.readFileSync("idp-cert.pem").toString()]
});

/**
 * Handle SAML assertion
 */
app.post("/saml/acs", (req, res) => {
  sp.post_assert(idp, { request_body: req.body }, (err, saml_response) => {
    if (err) {
      return res.status(400).json({ error: err.message });
    }

    const user = {
      nameId: saml_response.user.name_id,
      email: saml_response.user.attributes.email?.[0],
      firstName: saml_response.user.attributes.givenName?.[0],
      lastName: saml_response.user.attributes.surname?.[0],
      groups: saml_response.user.attributes.groups || []
    };

    // Create session
    req.session.user = user;

    res.redirect("/dashboard");
  });
});
```

---

## 6. Federation & SSO

### 6.1 Identity Federation Configuration

```json
{
  "federation": {
    "federationId": "fed-550e8400-e29b-41d4-a716-446655440000",
    "name": "Partner Organization Federation",
    "protocol": "oidc",
    "identityProviders": [
      {
        "id": "idp-partner-a",
        "name": "Partner A Identity Provider",
        "protocol": "oidc",
        "discoveryUrl": "https://partner-a.com/.well-known/openid-configuration",
        "clientId": "sp-client-id",
        "clientSecret": "${PARTNER_A_CLIENT_SECRET}",
        "scopes": ["openid", "profile", "email"],
        "attributeMapping": {
          "sub": "userId",
          "email": "email",
          "given_name": "givenName",
          "family_name": "familyName"
        }
      },
      {
        "id": "idp-partner-b",
        "name": "Partner B Identity Provider",
        "protocol": "saml",
        "metadataUrl": "https://partner-b.com/saml/metadata",
        "ssoUrl": "https://partner-b.com/saml/sso",
        "x509Certificate": "MIIDXTCCAkWgAwIBAgIJAKZ...",
        "attributeMapping": {
          "urn:oid:0.9.2342.19200300.100.1.1": "userId",
          "urn:oid:0.9.2342.19200300.100.1.3": "email",
          "urn:oid:2.5.4.42": "givenName",
          "urn:oid:2.5.4.4": "familyName"
        }
      }
    ],
    "accountLinking": {
      "enabled": true,
      "strategy": "email_match",
      "createIfNotExists": true
    }
  }
}
```

### 6.2 Account Linking

```javascript
/**
 * Link federated identity to local account
 */
async function linkFederatedIdentity(localUserId, federatedIdentity) {
  const link = {
    id: generateUUID(),
    localUserId,
    federationId: federatedIdentity.federationId,
    identityProviderId: federatedIdentity.idpId,
    federatedUserId: federatedIdentity.userId,
    claims: {
      email: federatedIdentity.email,
      givenName: federatedIdentity.givenName,
      familyName: federatedIdentity.familyName
    },
    linked: new Date().toISOString(),
    lastUsed: new Date().toISOString()
  };

  await db.identityLinks.insert(link);

  return {
    linked: true,
    linkId: link.id
  };
}

/**
 * Authenticate with federated identity
 */
async function authenticateWithFederation(federatedIdentity) {
  // Look for existing link
  let link = await db.identityLinks.findOne({
    federationId: federatedIdentity.federationId,
    identityProviderId: federatedIdentity.idpId,
    federatedUserId: federatedIdentity.userId
  });

  let localUser;

  if (link) {
    // Update last used
    await db.identityLinks.update(link.id, {
      lastUsed: new Date().toISOString(),
      claims: federatedIdentity.claims
    });

    localUser = await db.users.findOne({ id: link.localUserId });
  } else {
    // Try to find user by email
    localUser = await db.users.findOne({ email: federatedIdentity.email });

    if (!localUser) {
      // Create new user
      localUser = await createUserFromFederatedIdentity(federatedIdentity);
    }

    // Create link
    await linkFederatedIdentity(localUser.id, federatedIdentity);
  }

  return {
    authenticated: true,
    user: localUser,
    federatedIdentity
  };
}
```

---

## 7. Multi-Tenancy Support

### 7.1 Tenant Configuration

```json
{
  "tenant": {
    "tenantId": "tenant-acme-corp",
    "name": "ACME Corporation",
    "domain": "acme.com",
    "settings": {
      "identityProviders": ["ldap", "azure-ad"],
      "authenticationMethods": ["password", "mfa", "sso"],
      "passwordPolicy": {
        "minLength": 12,
        "requireUppercase": true,
        "requireLowercase": true,
        "requireNumbers": true,
        "requireSpecialChars": true,
        "expiryDays": 90
      },
      "mfaPolicy": {
        "required": true,
        "methods": ["totp", "sms", "webauthn"]
      },
      "sessionPolicy": {
        "maxDuration": 3600,
        "idleTimeout": 900,
        "concurrentSessions": 3
      }
    },
    "branding": {
      "logo": "https://acme.com/logo.png",
      "primaryColor": "#007bff",
      "loginPageUrl": "https://login.acme.com"
    },
    "integrations": {
      "ldap": {
        "url": "ldaps://ldap.acme.com:636",
        "baseDN": "dc=acme,dc=com"
      },
      "azureAD": {
        "tenantId": "12345678-1234-1234-1234-123456789012",
        "clientId": "abcdef01-2345-6789-abcd-ef0123456789"
      }
    }
  }
}
```

---

## 8. Monitoring & Audit Logging

### 8.1 Audit Log Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "AuditLog",
  "type": "object",
  "required": ["id", "timestamp", "eventType", "actor", "action"],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "tenantId": {
      "type": "string"
    },
    "eventType": {
      "type": "string",
      "enum": [
        "authentication",
        "authorization",
        "user.created",
        "user.updated",
        "user.deleted",
        "credential.issued",
        "credential.verified",
        "credential.revoked",
        "session.created",
        "session.terminated",
        "permission.granted",
        "permission.revoked"
      ]
    },
    "actor": {
      "type": "object",
      "properties": {
        "userId": {"type": "string"},
        "did": {"type": "string"},
        "ipAddress": {"type": "string"},
        "userAgent": {"type": "string"}
      }
    },
    "action": {
      "type": "string"
    },
    "resource": {
      "type": "object",
      "properties": {
        "type": {"type": "string"},
        "id": {"type": "string"}
      }
    },
    "result": {
      "type": "string",
      "enum": ["success", "failure"]
    },
    "details": {
      "type": "object",
      "additionalProperties": true
    }
  }
}
```

---

**Implementation Complete!**

This completes the WIA-SEC-009 Identity Management standard specification across all four phases:

1. **Phase 1**: Foundation - DID, VC, schemas
2. **Phase 2**: Data Formats - LDAP, AD, SCIM
3. **Phase 3**: Protocols - Auth, provisioning, revocation
4. **Phase 4**: Integration - IAM systems, federation, SSO

© 2025 WIA (World Certification Industry Association)
**弘益人間 (홍익인간)** - Benefit All Humanity
