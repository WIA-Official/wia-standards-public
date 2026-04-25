# WIA-IND-013: E-Sports Platform Standard
## Phase 4: Platform Integration Specification
### 弘益人間 - Broadly Benefiting Humanity

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-15

---

## Table of Contents

1. [Introduction](#introduction)
2. [Twitch Integration](#twitch-integration)
3. [YouTube Gaming Integration](#youtube-gaming-integration)
4. [Discord Integration](#discord-integration)
5. [Game Publisher APIs](#game-publisher-apis)
6. [Analytics Platforms](#analytics-platforms)
7. [Cross-Platform Data Sync](#cross-platform-data-sync)

---

## Introduction

Phase 4 provides guidelines and reference implementations for integrating WIA-IND-013 systems with major platforms like Twitch, YouTube Gaming, Discord, and game publishers. This ensures compliant systems work seamlessly with existing e-sports infrastructure.

### Design Principles

- **Platform Agnostic:** Core standards work anywhere
- **Native Integration:** Leverage platform-specific features
- **Graceful Degradation:** Work without platform access
- **Data Portability:** Easy migration between platforms
- **Philosophy:** 弘益人間 through interoperability

---

## Twitch Integration

### Stream Metadata Sync

Push WIA-IND-013 match data to Twitch:

```javascript
const updateTwitchStream = async (matchData) => {
  const twitchAPI = 'https://api.twitch.tv/helix/channels';

  const streamData = {
    broadcaster_id: BROADCASTER_ID,
    game_id: getTwitchGameId(matchData.game),
    title: `${matchData.teams[0].name} vs ${matchData.teams[1].name} - ${matchData.tournament.name}`,
    tags: ['esports', 'competitive', matchData.game.toLowerCase()],
    content_classification_labels: ['Competitive'],
    is_branded_content: false
  };

  await fetch(twitchAPI, {
    method: 'PATCH',
    headers: {
      'Authorization': `Bearer ${TWITCH_ACCESS_TOKEN}`,
      'Client-Id': TWITCH_CLIENT_ID,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(streamData)
  });
};
```

### Twitch Extensions

WIA-IND-013 Extension configuration:

```json
{
  "extensionId": "wia-ind-013-overlay",
  "version": "1.0.0",
  "name": "WIA E-Sports Overlay",
  "description": "Real-time match statistics and player info",
  "views": {
    "panel": {
      "viewerUrl": "https://ext.platform.com/panel.html",
      "height": 300,
      "canLinkExternalContent": true
    },
    "videoOverlay": {
      "viewerUrl": "https://ext.platform.com/overlay.html",
      "canLinkExternalContent": true
    },
    "component": {
      "viewerUrl": "https://ext.platform.com/component.html",
      "aspectRatioX": 16,
      "aspectRatioY": 9,
      "zoom": true,
      "canLinkExternalContent": true
    }
  },
  "configUrl": "https://ext.platform.com/config.html"
}
```

Extension receives WIA data via EBS (Extension Backend Service):

```javascript
// Extension frontend
window.Twitch.ext.onContext((context) => {
  fetch(`https://ebs.platform.com/match/${context.channelId}`)
    .then(res => res.json())
    .then(matchData => {
      // Render WIA-IND-013 data
      displayMatchStats(matchData);
    });
});
```

### Twitch EventSub

Subscribe to Twitch events and sync with WIA:

```javascript
const subscribeToTwitchEvents = async () => {
  const subscriptions = [
    {
      type: 'stream.online',
      version: '1',
      condition: { broadcaster_user_id: BROADCASTER_ID },
      transport: {
        method: 'webhook',
        callback: 'https://platform.com/twitch/webhook',
        secret: WEBHOOK_SECRET
      }
    },
    {
      type: 'channel.update',
      version: '1',
      condition: { broadcaster_user_id: BROADCASTER_ID }
    }
  ];

  for (const sub of subscriptions) {
    await fetch('https://api.twitch.tv/helix/eventsub/subscriptions', {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${TWITCH_ACCESS_TOKEN}`,
        'Client-Id': TWITCH_CLIENT_ID,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(sub)
    });
  }
};
```

### Twitch Predictions

Create predictions based on WIA match data:

```json
{
  "broadcaster_id": "123456",
  "title": "Who will win: T1 vs Gen.G?",
  "outcomes": [
    {"title": "T1"},
    {"title": "Gen.G"}
  ],
  "prediction_window": 120
}
```

Auto-resolve from WIA match results:

```javascript
const resolveFromWIA = async (matchResult) => {
  const winnerOutcomeId = matchResult.winner === 'team-t1'
    ? prediction.outcomes[0].id
    : prediction.outcomes[1].id;

  await fetch(`https://api.twitch.tv/helix/predictions`, {
    method: 'PATCH',
    body: JSON.stringify({
      broadcaster_id: BROADCASTER_ID,
      id: prediction.id,
      status: 'RESOLVED',
      winning_outcome_id: winnerOutcomeId
    })
  });
};
```

---

## YouTube Gaming Integration

### Live Stream Setup

```javascript
const createYouTubeLiveStream = async (matchData) => {
  const youtube = google.youtube('v3');

  const broadcast = await youtube.liveBroadcasts.insert({
    part: 'snippet,contentDetails,status',
    requestBody: {
      snippet: {
        title: `${matchData.teams[0].name} vs ${matchData.teams[1].name}`,
        description: `Live e-sports match: ${matchData.tournament.name}\n\nPowered by WIA-IND-013`,
        scheduledStartTime: matchData.timestamp.scheduled,
        categoryId: '20' // Gaming
      },
      status: {
        privacyStatus: 'public',
        selfDeclaredMadeForKids: false
      },
      contentDetails: {
        enableAutoStart: true,
        enableAutoStop: true,
        enableDvr: true,
        enableContentEncryption: false,
        enableEmbed: true,
        recordFromStart: true
      }
    }
  });

  return broadcast.data;
};
```

### YouTube Live Chat

Sync WIA chat with YouTube Live Chat:

```javascript
const syncChatToYouTube = async (wiaMessage) => {
  await youtube.liveChatMessages.insert({
    part: 'snippet',
    requestBody: {
      snippet: {
        liveChatId: LIVE_CHAT_ID,
        type: 'textMessageEvent',
        textMessageDetails: {
          messageText: `[${wiaMessage.sender.username}] ${wiaMessage.content.text}`
        }
      }
    }
  });
};
```

### YouTube Analytics Integration

```javascript
const pushMatchMetrics = async (matchStats) => {
  await youtubeAnalytics.reports.query({
    ids: `channel==${CHANNEL_ID}`,
    startDate: matchStats.startDate,
    endDate: matchStats.endDate,
    metrics: 'views,likes,comments,shares',
    dimensions: 'video',
    filters: `video==${matchStats.videoId}`
  });
};
```

---

## Discord Integration

### Discord Bot Commands

```javascript
const { Client, Intents } = require('discord.js');
const client = new Client({ intents: [Intents.FLAGS.GUILDS] });

client.on('interactionCreate', async interaction => {
  if (!interaction.isCommand()) return;

  const { commandName } = interaction;

  if (commandName === 'matchstatus') {
    const matchId = interaction.options.getString('match_id');
    const matchData = await fetchWIAMatch(matchId);

    const embed = {
      color: 0xF59E0B,
      title: '🎮 Match Status',
      description: `${matchData.teams[0].name} vs ${matchData.teams[1].name}`,
      fields: [
        {
          name: 'Status',
          value: matchData.status === 'live' ? '🔴 Live' : '⏰ Scheduled',
          inline: true
        },
        {
          name: 'Score',
          value: `${matchData.score.team1} - ${matchData.score.team2}`,
          inline: true
        },
        {
          name: 'Game Time',
          value: formatGameTime(matchData.gameTime),
          inline: true
        }
      ],
      footer: { text: '弘益人間 | WIA-IND-013' },
      timestamp: new Date()
    };

    await interaction.reply({ embeds: [embed] });
  }

  if (commandName === 'playerstats') {
    const playerId = interaction.options.getString('player');
    const stats = await fetchWIAPlayerStats(playerId);

    const embed = {
      color: 0xF59E0B,
      title: `📊 ${stats.username} Statistics`,
      fields: [
        { name: 'Win Rate', value: `${stats.winRate}%`, inline: true },
        { name: 'KDA', value: stats.kda.toFixed(2), inline: true },
        { name: 'Rank', value: `#${stats.globalRank}`, inline: true }
      ]
    };

    await interaction.reply({ embeds: [embed] });
  }
});
```

### Discord Webhooks

Post match updates to Discord channels:

```javascript
const postMatchUpdate = async (matchResult) => {
  const webhook = new WebhookClient({ url: DISCORD_WEBHOOK_URL });

  const embed = {
    color: 0xF59E0B,
    title: '🏆 Match Complete',
    description: `**${matchResult.winner.name}** defeats **${matchResult.loser.name}**`,
    fields: [
      { name: 'Final Score', value: matchResult.score },
      { name: 'Duration', value: formatDuration(matchResult.duration) },
      { name: 'MVP', value: matchResult.mvp.username }
    ],
    footer: { text: '弘益人間 | Powered by WIA-IND-013' },
    timestamp: new Date()
  };

  await webhook.send({ embeds: [embed] });
};
```

### Discord Rich Presence

Enable Discord Rich Presence for players:

```javascript
const updateDiscordPresence = (playerState) => {
  return {
    state: `Playing ${playerState.game}`,
    details: `${playerState.champion} - ${playerState.role}`,
    startTimestamp: playerState.matchStartTime,
    largeImageKey: playerState.game.toLowerCase(),
    largeImageText: playerState.game,
    smallImageKey: playerState.rank.tier.toLowerCase(),
    smallImageText: `${playerState.rank.tier} ${playerState.rank.division}`,
    partySize: playerState.teamSize,
    partyMax: 5,
    joinSecret: playerState.joinSecret
  };
};
```

---

## Game Publisher APIs

### Riot Games API

Fetch and sync League of Legends data:

```javascript
const syncRiotMatchData = async (riotMatchId) => {
  const riotAPI = `https://americas.api.riotgames.com/lol/match/v5/matches/${riotMatchId}`;

  const riotMatch = await fetch(riotAPI, {
    headers: { 'X-Riot-Token': RIOT_API_KEY }
  }).then(res => res.json());

  // Transform to WIA-IND-013 format
  const wiaMatch = {
    matchId: `riot-${riotMatchId}`,
    game: 'League of Legends',
    timestamp: {
      started: new Date(riotMatch.info.gameCreation),
      ended: new Date(riotMatch.info.gameEndTimestamp)
    },
    teams: riotMatch.info.teams.map(transformRiotTeam),
    players: riotMatch.info.participants.map(transformRiotPlayer),
    statistics: extractRiotStats(riotMatch.info)
  };

  // Save to WIA platform
  await saveWIAMatch(wiaMatch);
};
```

### Valorant API Integration

```javascript
const syncValorantMatch = async (matchId) => {
  const valorantData = await fetchValorantMatch(matchId);

  const wiaMatch = {
    matchId: `valorant-${matchId}`,
    game: 'Valorant',
    teams: valorantData.teams,
    players: valorantData.players.map(p => ({
      playerId: p.puuid,
      agent: p.characterId,
      performance: {
        kills: p.stats.kills,
        deaths: p.stats.deaths,
        assists: p.stats.assists,
        score: p.stats.score
      }
    }))
  };

  return wiaMatch;
};
```

---

## Analytics Platforms

### Google Analytics Integration

```javascript
const trackMatchEvent = (matchData) => {
  gtag('event', 'match_complete', {
    event_category: 'esports',
    event_label: matchData.tournament.name,
    value: matchData.viewerCount,
    custom_map: {
      'dimension1': 'match_id',
      'dimension2': 'game',
      'metric1': 'duration'
    },
    match_id: matchData.matchId,
    game: matchData.game,
    duration: matchData.statistics.duration
  });
};
```

### Tableau Integration

Export WIA data for Tableau:

```javascript
const exportForTableau = async () => {
  const matches = await fetchAllMatches();

  const tableauData = matches.map(m => ({
    'Match ID': m.matchId,
    'Date': m.timestamp.started,
    'Game': m.game,
    'Tournament': m.tournamentId,
    'Team 1': m.teams[0].name,
    'Team 2': m.teams[1].name,
    'Winner': m.result.winner,
    'Duration': m.statistics.duration,
    'Total Kills': m.statistics.totalKills,
    'Viewer Count': m.viewerCount
  }));

  return tableauData;
};
```

---

## Cross-Platform Data Sync

### Unified Player Identity

Link player accounts across platforms:

```json
{
  "playerId": "player-faker",
  "linkedAccounts": {
    "riot": {
      "puuid": "abc123-def456-ghi789",
      "gameName": "Faker",
      "tagLine": "KR1",
      "verified": true
    },
    "twitch": {
      "userId": "123456789",
      "login": "faker",
      "displayName": "Faker",
      "verified": true
    },
    "youtube": {
      "channelId": "UC...",
      "channelName": "Faker",
      "verified": true
    },
    "discord": {
      "userId": "987654321",
      "username": "Faker#0001",
      "verified": true
    }
  }
}
```

### Data Synchronization Strategy

```javascript
class CrossPlatformSync {
  async syncMatchData(matchId) {
    const wiaMatch = await this.getWIAMatch(matchId);

    // Parallel sync to all platforms
    await Promise.all([
      this.syncToTwitch(wiaMatch),
      this.syncToYouTube(wiaMatch),
      this.syncToDiscord(wiaMatch),
      this.syncToAnalytics(wiaMatch)
    ]);
  }

  async syncPlayerStats(playerId) {
    // Aggregate data from all platforms
    const [riotStats, steamStats, epicStats] = await Promise.all([
      this.fetchRiotStats(playerId),
      this.fetchSteamStats(playerId),
      this.fetchEpicStats(playerId)
    ]);

    // Merge into unified WIA profile
    const unifiedStats = this.mergeStats([riotStats, steamStats, epicStats]);

    // Save to WIA platform
    await this.saveWIAStats(playerId, unifiedStats);
  }
}
```

---

## Philosophy: 弘益人間

Platform integration embodies "Broadly Benefiting Humanity" through:

- **Interoperability:** Work with existing platforms, not against them
- **Data Freedom:** Easy export and import across systems
- **No Lock-in:** Players own their data, not platforms
- **Fair Access:** All platforms treated equally in standards
- **Community Benefit:** Integrations raise all platforms

---

## Implementation Checklist

- [ ] OAuth flows for each platform
- [ ] Webhook receivers for events
- [ ] Rate limiting respect
- [ ] Error handling and retries
- [ ] Data transformation layers
- [ ] Sync conflict resolution
- [ ] Account linking verification
- [ ] Privacy compliance (GDPR, CCPA)
- [ ] Monitoring and logging
- [ ] Documentation for developers

---

## References

- [Twitch API Documentation](https://dev.twitch.tv/docs/api/)
- [YouTube Data API](https://developers.google.com/youtube/v3)
- [Discord Developer Portal](https://discord.com/developers/docs)
- [Riot Games API](https://developer.riotgames.com/)
- [WIA-IND-013 Main Specification](../README.md)

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-013-esports is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-IND-013-esports/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-013-esports/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-013-esports/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
