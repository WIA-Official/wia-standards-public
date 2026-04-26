var __defProp = Object.defineProperty;
var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
var __getOwnPropNames = Object.getOwnPropertyNames;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __export = (target, all) => {
  for (var name in all)
    __defProp(target, name, { get: all[name], enumerable: true });
};
var __copyProps = (to, from, except, desc) => {
  if (from && typeof from === "object" || typeof from === "function") {
    for (let key of __getOwnPropNames(from))
      if (!__hasOwnProp.call(to, key) && key !== except)
        __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
  }
  return to;
};
var __toCommonJS = (mod) => __copyProps(__defProp({}, "__esModule", { value: true }), mod);

// src/types.ts
var types_exports = {};
__export(types_exports, {
  REGISTRY_CONSTANTS: () => REGISTRY_CONSTANTS,
  RegistryError: () => RegistryError,
  RegistryErrorCode: () => RegistryErrorCode
});
module.exports = __toCommonJS(types_exports);
var RegistryErrorCode = /* @__PURE__ */ ((RegistryErrorCode2) => {
  RegistryErrorCode2["INVALID_REQUEST"] = "INVALID_REQUEST";
  RegistryErrorCode2["UNAUTHORIZED"] = "UNAUTHORIZED";
  RegistryErrorCode2["FORBIDDEN"] = "FORBIDDEN";
  RegistryErrorCode2["NOT_FOUND"] = "NOT_FOUND";
  RegistryErrorCode2["CONFLICT"] = "CONFLICT";
  RegistryErrorCode2["RATE_LIMIT_EXCEEDED"] = "RATE_LIMIT_EXCEEDED";
  RegistryErrorCode2["VALIDATION_ERROR"] = "VALIDATION_ERROR";
  RegistryErrorCode2["SYSTEM_ALREADY_REGISTERED"] = "SYSTEM_ALREADY_REGISTERED";
  RegistryErrorCode2["STANDARD_NOT_FOUND"] = "STANDARD_NOT_FOUND";
  RegistryErrorCode2["COMPLIANCE_CHECK_FAILED"] = "COMPLIANCE_CHECK_FAILED";
  RegistryErrorCode2["TEMPLATE_GENERATION_FAILED"] = "TEMPLATE_GENERATION_FAILED";
  RegistryErrorCode2["INTERNAL_ERROR"] = "INTERNAL_ERROR";
  return RegistryErrorCode2;
})(RegistryErrorCode || {});
var RegistryError = class extends Error {
  code;
  statusCode;
  details;
  constructor(code, message, statusCode = 500, details) {
    super(message);
    this.code = code;
    this.statusCode = statusCode;
    this.details = details;
    this.name = "RegistryError";
  }
};
var REGISTRY_CONSTANTS = {
  // Default registry endpoint
  DEFAULT_ENDPOINT: "https://registry.wiastandards.com",
  // API version
  API_VERSION: "v1",
  // Default pagination
  DEFAULT_PAGE_SIZE: 10,
  MAX_PAGE_SIZE: 100,
  // Compliance levels in order
  COMPLIANCE_LEVELS: [
    "documented",
    "testable",
    "compliant",
    "certified",
    "reference"
  ],
  // Standard categories
  CATEGORIES: {
    CORE: { name: "Core", color: "#6366F1", emoji: "\u{1F537}" },
    AAC: { name: "Accessibility", color: "#3B82F6", emoji: "\u267F" },
    AI: { name: "Artificial Intelligence", color: "#10B981", emoji: "\u{1F916}" },
    MED: { name: "Medical", color: "#14B8A6", emoji: "\u{1F3E5}" },
    BIO: { name: "Biotechnology", color: "#14B8A6", emoji: "\u{1F9EC}" },
    DEF: { name: "Defense", color: "#64748B", emoji: "\u{1F6E1}\uFE0F" }
    // ... add more as needed
  },
  // Timeout values (milliseconds)
  TIMEOUTS: {
    DEFAULT: 3e4,
    SEARCH: 1e4,
    REGISTRATION: 6e4,
    COMPLIANCE: 12e4
  }
};
// Annotate the CommonJS export names for ESM import in node:
0 && (module.exports = {
  REGISTRY_CONSTANTS,
  RegistryError,
  RegistryErrorCode
});
/**
 * WIA-CORE-004: Interoperability Registry - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */
