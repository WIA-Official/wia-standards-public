//! Protocol error codes

/// Error code categories
pub mod codes {
    // Connection Errors (1xxx)
    pub const CONNECTION_CLOSED: u32 = 1000;
    pub const CONNECTION_LOST: u32 = 1001;
    pub const CONNECTION_TIMEOUT: u32 = 1002;
    pub const PROTOCOL_ERROR: u32 = 1003;
    pub const VERSION_MISMATCH: u32 = 1004;

    // Backend Errors (2xxx)
    pub const BACKEND_NOT_AVAILABLE: u32 = 2001;
    pub const BACKEND_BUSY: u32 = 2002;
    pub const BACKEND_ERROR: u32 = 2003;
    pub const QUEUE_FULL: u32 = 2004;
    pub const MAINTENANCE: u32 = 2005;

    // Circuit Errors (3xxx)
    pub const CIRCUIT_TOO_DEEP: u32 = 3001;
    pub const CIRCUIT_TOO_WIDE: u32 = 3002;
    pub const INVALID_GATE: u32 = 3003;
    pub const INVALID_CIRCUIT: u32 = 3004;
    pub const TRANSPILE_ERROR: u32 = 3005;

    // Job Errors (4xxx)
    pub const JOB_NOT_FOUND: u32 = 4001;
    pub const JOB_CANCELLED: u32 = 4002;
    pub const JOB_TIMEOUT: u32 = 4003;
    pub const SHOTS_LIMIT_EXCEEDED: u32 = 4004;
    pub const QUOTA_EXCEEDED: u32 = 4005;

    // Auth Errors (5xxx)
    pub const AUTH_REQUIRED: u32 = 5001;
    pub const AUTH_FAILED: u32 = 5002;
    pub const TOKEN_EXPIRED: u32 = 5003;
    pub const PERMISSION_DENIED: u32 = 5004;
}

/// Error code information
#[derive(Debug, Clone)]
pub struct ErrorCodeInfo {
    pub code: u32,
    pub name: &'static str,
    pub description: &'static str,
    pub recoverable: bool,
}

/// Get error code info
pub fn get_error_info(code: u32) -> Option<ErrorCodeInfo> {
    match code {
        codes::CONNECTION_CLOSED => Some(ErrorCodeInfo {
            code,
            name: "CONNECTION_CLOSED",
            description: "Connection was closed normally",
            recoverable: false,
        }),
        codes::CONNECTION_LOST => Some(ErrorCodeInfo {
            code,
            name: "CONNECTION_LOST",
            description: "Connection was lost unexpectedly",
            recoverable: true,
        }),
        codes::CONNECTION_TIMEOUT => Some(ErrorCodeInfo {
            code,
            name: "CONNECTION_TIMEOUT",
            description: "Connection attempt timed out",
            recoverable: true,
        }),
        codes::PROTOCOL_ERROR => Some(ErrorCodeInfo {
            code,
            name: "PROTOCOL_ERROR",
            description: "Protocol error occurred",
            recoverable: false,
        }),
        codes::VERSION_MISMATCH => Some(ErrorCodeInfo {
            code,
            name: "VERSION_MISMATCH",
            description: "Protocol version mismatch",
            recoverable: false,
        }),
        codes::BACKEND_NOT_AVAILABLE => Some(ErrorCodeInfo {
            code,
            name: "BACKEND_NOT_AVAILABLE",
            description: "Requested backend is not available",
            recoverable: true,
        }),
        codes::BACKEND_BUSY => Some(ErrorCodeInfo {
            code,
            name: "BACKEND_BUSY",
            description: "Backend is currently busy",
            recoverable: true,
        }),
        codes::BACKEND_ERROR => Some(ErrorCodeInfo {
            code,
            name: "BACKEND_ERROR",
            description: "Backend internal error",
            recoverable: true,
        }),
        codes::QUEUE_FULL => Some(ErrorCodeInfo {
            code,
            name: "QUEUE_FULL",
            description: "Job queue is full",
            recoverable: true,
        }),
        codes::MAINTENANCE => Some(ErrorCodeInfo {
            code,
            name: "MAINTENANCE",
            description: "Backend is under maintenance",
            recoverable: true,
        }),
        codes::CIRCUIT_TOO_DEEP => Some(ErrorCodeInfo {
            code,
            name: "CIRCUIT_TOO_DEEP",
            description: "Circuit depth exceeds backend limit",
            recoverable: false,
        }),
        codes::CIRCUIT_TOO_WIDE => Some(ErrorCodeInfo {
            code,
            name: "CIRCUIT_TOO_WIDE",
            description: "Circuit requires more qubits than available",
            recoverable: false,
        }),
        codes::INVALID_GATE => Some(ErrorCodeInfo {
            code,
            name: "INVALID_GATE",
            description: "Circuit contains unsupported gate",
            recoverable: false,
        }),
        codes::INVALID_CIRCUIT => Some(ErrorCodeInfo {
            code,
            name: "INVALID_CIRCUIT",
            description: "Invalid circuit format",
            recoverable: false,
        }),
        codes::TRANSPILE_ERROR => Some(ErrorCodeInfo {
            code,
            name: "TRANSPILE_ERROR",
            description: "Circuit transpilation failed",
            recoverable: false,
        }),
        codes::JOB_NOT_FOUND => Some(ErrorCodeInfo {
            code,
            name: "JOB_NOT_FOUND",
            description: "Job not found",
            recoverable: false,
        }),
        codes::JOB_CANCELLED => Some(ErrorCodeInfo {
            code,
            name: "JOB_CANCELLED",
            description: "Job was cancelled",
            recoverable: false,
        }),
        codes::JOB_TIMEOUT => Some(ErrorCodeInfo {
            code,
            name: "JOB_TIMEOUT",
            description: "Job execution timed out",
            recoverable: true,
        }),
        codes::SHOTS_LIMIT_EXCEEDED => Some(ErrorCodeInfo {
            code,
            name: "SHOTS_LIMIT_EXCEEDED",
            description: "Number of shots exceeds limit",
            recoverable: false,
        }),
        codes::QUOTA_EXCEEDED => Some(ErrorCodeInfo {
            code,
            name: "QUOTA_EXCEEDED",
            description: "Usage quota exceeded",
            recoverable: true,
        }),
        codes::AUTH_REQUIRED => Some(ErrorCodeInfo {
            code,
            name: "AUTH_REQUIRED",
            description: "Authentication required",
            recoverable: false,
        }),
        codes::AUTH_FAILED => Some(ErrorCodeInfo {
            code,
            name: "AUTH_FAILED",
            description: "Authentication failed",
            recoverable: false,
        }),
        codes::TOKEN_EXPIRED => Some(ErrorCodeInfo {
            code,
            name: "TOKEN_EXPIRED",
            description: "Authentication token expired",
            recoverable: true,
        }),
        codes::PERMISSION_DENIED => Some(ErrorCodeInfo {
            code,
            name: "PERMISSION_DENIED",
            description: "Permission denied",
            recoverable: false,
        }),
        _ => None,
    }
}

/// Check if error code is recoverable
pub fn is_recoverable(code: u32) -> bool {
    get_error_info(code).map_or(false, |info| info.recoverable)
}
