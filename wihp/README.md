# WIHP — WIA Hangul Input Protocol

> Open standard for non-keyboard text entry, gestural input, and
> sign-language-to-text bridging across operating systems and
> assistive devices.

**Philosophy**: 弘益人間 — Benefit All Humanity

---

## Why this standard exists

Text input methods on mobile and assistive devices are
fragmented across vendor-specific input method engines (IMEs).
A patient with limited mobility, a Deaf user signing rather than
typing, a user in a country whose script lacks a single canonical
on-screen keyboard — all of them deal with bespoke per-vendor
implementations that do not translate from one device to another.

WIHP defines an open wire format and a federation protocol for
gestural input, sign-language-to-text bridges, and on-screen
keyboard layouts so that an input method authored once works
across iOS, Android, Linux desktop, Windows, and assistive
hardware (eye trackers, sip-and-puff devices, switch interfaces).

---

## 4-Phase architecture

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Gesture, layout, sign mapping, accessibility binding envelopes |
| 2 — API Interface | HTTP surface for layout publication, gesture stream, federation handshake |
| 3 — Federation Protocol | Cross-platform IME registry, replay defence, accessibility binding |
| 4 — Integration | iOS, Android, Linux IBus/Fcitx, Windows TSF, ARIA, ATK, eye-tracker SDKs |

---

## Quick start

```bash
docker run -p 8080:8080 wia/wihp-host:1.0.0

# Subscribe to live gesture-stream from a paired device
curl -N "http://localhost:8080/wihp/stream?device_id=did:wia:device:..." \
     -H "Accept: text/event-stream"
```

---

## CLI

A reference CLI ships under `cli/wihp.sh` with subcommands
`validate`, `layout`, `gesture`, `sign-mapping`, `accessibility-binding`,
`info`.

---

## Companion standards

* **WIA Sign Language** — sister standard for sign-to-text bridging
* **WCAG 2.2 / WAI-ARIA** — accessibility baseline
* **Unicode CLDR** — keyboard layout interchange
* **W3C Web Speech API** — speech-to-text fallback
* **WIA-OMNI-API** — credential storage for paired devices

---

MIT License — © 2025 WIA (World Certification Industry Association)

弘益人間 — Benefit All Humanity.
