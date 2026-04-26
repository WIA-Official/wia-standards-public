/**
 * WIA-PET-008 Pet Tracking SDK - Type Definitions
 * @module @wia/pet-tracking-sdk
 */

/**
 * Coordinate system: WGS84
 */
export interface Coordinates {
  /** Latitude in decimal degrees (-90 to 90) */
  latitude: number;
  /** Longitude in decimal degrees (-180 to 180) */
  longitude: number;
  /** Altitude in meters above sea level */
  altitude?: number;
}

/**
 * Location data with quality metrics
 */
export interface Location {
  /** Geographic coordinates */
  latitude: number;
  longitude: number;
  altitude?: number;
  /** Horizontal accuracy in meters (68% confidence) */
  accuracy: number;
  /** Altitude accuracy in meters */
  altitudeAccuracy?: number;
  /** Heading in degrees from true north (0-360) */
  heading?: number;
  /** Speed in meters per second */
  speed?: number;
}

/**
 * Location quality indicators
 */
export interface LocationQuality {
  /** Positioning method used */
  method: 'GNSS' | 'WiFi' | 'Cellular' | 'Dead_Reckoning';
  /** Number of satellites used (GNSS only) */
  satellites?: number;
  /** Horizontal Dilution of Precision */
  hdop?: number;
  /** Vertical Dilution of Precision */
  vdop?: number;
  /** GNSS constellations used */
  constellations?: Array<'GPS' | 'GLONASS' | 'Galileo' | 'BeiDou' | 'QZSS'>;
  /** Signal strength indicator */
  signalStrength?: 'strong' | 'moderate' | 'weak';
}

/**
 * Complete location event
 */
export interface LocationEvent {
  /** Location data */
  location: Location;
  /** Location quality metrics */
  quality: LocationQuality;
  /** ISO 8601 timestamp */
  timestamp: string;
  /** Device identifier */
  deviceId: string;
  /** Pet identifier */
  petId?: string;
}

/**
 * Geofence zone definition
 */
export interface Geofence {
  /** Unique geofence identifier */
  id: string;
  /** Human-readable name */
  name: string;
  /** Geofence type */
  type: 'circle' | 'polygon';
  /** Center point (for circular geofences) */
  center?: Coordinates;
  /** Radius in meters (for circular geofences) */
  radius?: number;
  /** Boundary points (for polygon geofences) */
  boundary?: Coordinates[];
  /** Alert configuration */
  alerts: {
    /** Trigger alert on zone entry */
    onEnter: boolean;
    /** Trigger alert on zone exit */
    onExit: boolean;
    /** Notification channels to use */
    notificationChannels: Array<'push' | 'sms' | 'email' | 'call'>;
  };
}

/**
 * Alert/notification event
 */
export interface Alert {
  /** Unique alert identifier */
  id: string;
  /** Alert type */
  type: 'geofence_enter' | 'geofence_exit' | 'low_battery' | 'device_offline' | 'health_anomaly' | 'lost_mode';
  /** Pet identifier */
  petId: string;
  /** Associated geofence (if applicable) */
  geofenceId?: string;
  /** Alert message */
  message: string;
  /** Alert priority level */
  priority: 'critical' | 'high' | 'medium' | 'low';
  /** Location when alert triggered */
  location?: Location;
  /** ISO 8601 timestamp */
  timestamp: string;
  /** Alert acknowledged by user */
  acknowledged: boolean;
}

/**
 * Pet profile information
 */
export interface Pet {
  /** Unique pet identifier */
  id: string;
  /** Pet name */
  name: string;
  /** Species (dog, cat, etc.) */
  species: string;
  /** Breed */
  breed?: string;
  /** Color/markings */
  color?: string;
  /** Age in years */
  age?: number;
  /** Weight in kilograms */
  weight?: number;
  /** Profile photo URL */
  photo?: string;
  /** Microchip number */
  microchip?: string;
  /** Associated device ID */
  deviceId?: string;
  /** Owner user ID */
  ownerId: string;
}

/**
 * Device/tracker information
 */
export interface Device {
  /** Unique device identifier */
  id: string;
  /** Device model */
  model: string;
  /** Firmware version */
  firmwareVersion: string;
  /** Battery level percentage (0-100) */
  batteryLevel: number;
  /** Last communication timestamp */
  lastSeen: string;
  /** Device status */
  status: 'active' | 'offline' | 'low_battery' | 'emergency';
  /** Tracking mode */
  mode: 'normal' | 'active' | 'power_save' | 'lost_mode';
  /** Associated pet ID */
  petId?: string;
}

/**
 * Activity summary data
 */
export interface ActivitySummary {
  /** Pet identifier */
  petId: string;
  /** Start of time period */
  startTime: string;
  /** End of time period */
  endTime: string;
  /** Total distance traveled in meters */
  distanceMeters: number;
  /** Active time in seconds */
  activeTimeSeconds: number;
  /** Resting time in seconds */
  restingTimeSeconds: number;
  /** Number of walks/outings */
  walks: number;
  /** Calories burned estimate */
  caloriesBurned?: number;
}

/**
 * SDK configuration options
 */
export interface SDKConfig {
  /** API base URL */
  apiUrl: string;
  /** API key for authentication */
  apiKey: string;
  /** WebSocket URL for real-time updates */
  wsUrl?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Enable debug logging */
  debug?: boolean;
}

/**
 * Query parameters for location history
 */
export interface LocationHistoryQuery {
  /** Pet identifier */
  petId: string;
  /** Start timestamp (ISO 8601) */
  startTime?: string;
  /** End timestamp (ISO 8601) */
  endTime?: string;
  /** Maximum number of results */
  limit?: number;
  /** Pagination offset */
  offset?: number;
}

/**
 * Lost pet report
 */
export interface LostPetReport {
  /** Unique report identifier */
  id: string;
  /** Pet information */
  pet: Pet;
  /** Last known location */
  lastKnownLocation: Location;
  /** Last seen timestamp */
  lastSeenTime: string;
  /** Search radius in meters */
  searchRadius: number;
  /** Contact information */
  contactInfo: {
    /** Masked phone number */
    maskedPhone?: string;
    /** Verified email for contact */
    email?: string;
  };
  /** Reward offered */
  reward?: number;
  /** Special instructions */
  instructions?: string;
  /** Report status */
  status: 'active' | 'found' | 'cancelled';
}
