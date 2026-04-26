/**
 * WIA-LEG-008: Data Portability Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 */

/**
 * Data Portability Package - Main container for exported data
 */
export interface DataPortabilityPackage {
  '@context': string;
  '@type': 'DataPortabilityPackage';
  id: string; // urn:uuid:...
  version: string;
  generated_at: string; // ISO 8601
  expires_at?: string;
  
  deceased: Person;
  executor: LegalExecutor;
  data_inventory: DataInventory;
  encryption?: EncryptionMetadata;
  data: CategoryData;
  metadata: PackageMetadata;
}

/**
 * Person information
 */
export interface Person {
  '@type': 'Person';
  id: string; // DID: did:wia:...
  name: string;
  dateOfBirth?: string;
  dateOfDeath?: string;
  lastKnownLocation?: Place;
}

/**
 * Place information
 */
export interface Place {
  '@type': 'Place';
  address: string;
  city?: string;
  country?: string;
  coordinates?: {
    latitude: number;
    longitude: number;
  };
}

/**
 * Legal executor information
 */
export interface LegalExecutor {
  '@type': 'LegalExecutor';
  id: string; // DID: did:wia:...
  name: string;
  relationship: string;
  contactEmail: string;
  contactPhone?: string;
  authorization: Authorization;
}

/**
 * Authorization details
 */
export interface Authorization {
  type: 'court_appointed' | 'will_designated' | 'family_consent';
  document_ref: string;
  issued_by: string;
  valid_from: string;
  valid_until: string;
}

/**
 * Data inventory summary
 */
export interface DataInventory {
  total_categories: number;
  total_items: number;
  total_size_bytes: number;
  categories: {
    [category: string]: CategoryInventory;
  };
}

/**
 * Category inventory
 */
export interface CategoryInventory {
  item_count: number;
  size_bytes: number;
  platforms: string[];
}

/**
 * Encryption metadata
 */
export interface EncryptionMetadata {
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  key_derivation: 'PBKDF2' | 'Argon2id';
  iterations: number;
  salt: string; // Base64
  iv: string; // Base64
  auth_tag?: string; // Base64
}

/**
 * Category data container
 */
export interface CategoryData {
  social_media?: SocialMediaData[];
  financial?: FinancialData[];
  creative_works?: CreativeWork[];
  communications?: Communication[];
  cloud_storage?: CloudFile[];
  gaming?: GamingData[];
  health?: HealthData[];
  professional?: ProfessionalData[];
  [key: string]: any[];
}

/**
 * Package metadata
 */
export interface PackageMetadata {
  export_method: 'manual' | 'automated' | 'api';
  export_tool: string;
  checksum: string; // sha256:...
  signature?: string;
  audit_trail: AuditEvent[];
}

/**
 * Audit event
 */
export interface AuditEvent {
  timestamp: string; // ISO 8601
  action: string;
  actor: string; // DID
  details?: any;
}

/**
 * Social media data
 */
export interface SocialMediaData {
  '@type': 'SocialMediaData';
  platform: string;
  account_id: string;
  profile: SocialProfile;
  posts?: SocialPost[];
  photos?: MediaAsset[];
  videos?: MediaAsset[];
  friends?: Connection[];
  messages?: Message[];
}

/**
 * Social profile
 */
export interface SocialProfile {
  username: string;
  display_name: string;
  bio?: string;
  profile_picture_url?: string;
  cover_photo_url?: string;
  verified?: boolean;
  follower_count?: number;
  following_count?: number;
}

/**
 * Social post
 */
export interface SocialPost {
  id: string;
  created_at: string;
  content: string;
  media?: MediaAsset[];
  reactions?: {
    [type: string]: number;
  };
  comments?: Comment[];
  shares?: number;
  privacy: 'public' | 'friends' | 'private';
}

/**
 * Comment
 */
export interface Comment {
  id: string;
  author: string;
  content: string;
  created_at: string;
  reactions?: {
    [type: string]: number;
  };
}

/**
 * Connection (friend, follower, etc.)
 */
export interface Connection {
  id: string;
  name: string;
  username?: string;
  relationship_type: 'friend' | 'follower' | 'following' | 'connection';
  since?: string;
}

/**
 * Message
 */
export interface Message {
  id: string;
  thread_id?: string;
  sender: string;
  recipients: string[];
  subject?: string;
  content: string;
  timestamp: string;
  attachments?: Attachment[];
  read?: boolean;
}

/**
 * Attachment
 */
export interface Attachment {
  id: string;
  filename: string;
  mime_type: string;
  size_bytes: number;
  url?: string;
}

/**
 * Media asset
 */
export interface MediaAsset {
  '@type': 'MediaAsset';
  id: string;
  asset_type: 'photo' | 'video' | 'audio';
  file_name: string;
  size_bytes: number;
  mime_type: string;
  dimensions?: {
    width: number;
    height: number;
  };
  duration_seconds?: number;
  exif?: ExifData;
  people_tagged?: string[];
  albums?: string[];
  caption?: string;
  privacy: 'public' | 'friends' | 'private';
  download_url?: string;
  created_at: string;
}

/**
 * EXIF data
 */
export interface ExifData {
  camera_make?: string;
  camera_model?: string;
  date_taken?: string;
  location?: {
    latitude: number;
    longitude: number;
    altitude?: number;
  };
  settings?: {
    iso?: number;
    aperture?: string;
    shutter_speed?: string;
    focal_length?: string;
  };
}

/**
 * Financial data
 */
export interface FinancialData {
  '@type': 'FinancialData';
  institution: string;
  account_type: 'checking' | 'savings' | 'credit' | 'investment' | 'cryptocurrency';
  account_number: string; // Masked
  balance?: {
    amount: number;
    currency: string;
    as_of_date: string;
  };
  transactions?: Transaction[];
  statements?: Statement[];
  tax_documents?: TaxDocument[];
}

/**
 * Transaction
 */
export interface Transaction {
  id: string;
  date: string;
  description: string;
  amount: number;
  currency: string;
  category?: string;
  merchant?: string;
  type: 'debit' | 'credit';
}

/**
 * Statement
 */
export interface Statement {
  id: string;
  period_start: string;
  period_end: string;
  document_url?: string;
  summary?: {
    opening_balance: number;
    closing_balance: number;
    total_credits: number;
    total_debits: number;
  };
}

/**
 * Tax document
 */
export interface TaxDocument {
  id: string;
  tax_year: number;
  document_type: string; // e.g., 'W-2', '1099', '1040'
  document_url?: string;
}

/**
 * Creative work
 */
export interface CreativeWork {
  '@type': 'CreativeWork';
  id: string;
  title: string;
  description?: string;
  work_type: 'blog_post' | 'article' | 'photo' | 'video' | 'music' | 'code' | 'art';
  content?: string;
  url?: string;
  file_url?: string;
  created_at: string;
  published_at?: string;
  author: string;
  license?: string;
  tags?: string[];
  views?: number;
  downloads?: number;
}

/**
 * Communication
 */
export interface Communication {
  '@type': 'Communication';
  id: string;
  type: 'email' | 'sms' | 'chat' | 'voicemail';
  platform: string;
  sender: string;
  recipients: string[];
  subject?: string;
  content: string;
  timestamp: string;
  attachments?: Attachment[];
  thread_id?: string;
}

/**
 * Cloud file
 */
export interface CloudFile {
  '@type': 'CloudFile';
  id: string;
  name: string;
  path: string;
  mime_type: string;
  size_bytes: number;
  created_at: string;
  modified_at: string;
  owner: string;
  shared_with?: string[];
  download_url?: string;
  checksum?: string;
}

/**
 * Gaming data
 */
export interface GamingData {
  '@type': 'GamingData';
  platform: string;
  username: string;
  profile_id: string;
  achievements?: Achievement[];
  game_saves?: GameSave[];
  virtual_items?: VirtualItem[];
  friends?: Connection[];
  stats?: {
    [key: string]: number | string;
  };
}

/**
 * Achievement
 */
export interface Achievement {
  id: string;
  name: string;
  description: string;
  unlocked_at?: string;
  rarity?: number; // 0-100
  icon_url?: string;
}

/**
 * Game save
 */
export interface GameSave {
  id: string;
  game_id: string;
  game_title: string;
  save_name: string;
  progress_percent: number;
  playtime_hours: number;
  last_played: string;
  save_data_url?: string;
}

/**
 * Virtual item
 */
export interface VirtualItem {
  id: string;
  name: string;
  description?: string;
  rarity: 'common' | 'uncommon' | 'rare' | 'epic' | 'legendary';
  acquired_at: string;
  tradeable: boolean;
  market_value?: {
    amount: number;
    currency: string;
  };
}

/**
 * Health data
 */
export interface HealthData {
  '@type': 'HealthData';
  source: string;
  data_type: string;
  value: number | string;
  unit?: string;
  timestamp: string;
  metadata?: {
    [key: string]: any;
  };
}

/**
 * Professional data
 */
export interface ProfessionalData {
  '@type': 'ProfessionalData';
  platform: string;
  profile: ProfessionalProfile;
  work_history?: WorkExperience[];
  education?: Education[];
  skills?: Skill[];
  projects?: Project[];
  certifications?: Certification[];
}

/**
 * Professional profile
 */
export interface ProfessionalProfile {
  name: string;
  headline?: string;
  summary?: string;
  profile_url?: string;
  connections_count?: number;
}

/**
 * Work experience
 */
export interface WorkExperience {
  company: string;
  title: string;
  start_date: string;
  end_date?: string;
  description?: string;
  location?: string;
}

/**
 * Education
 */
export interface Education {
  institution: string;
  degree?: string;
  field_of_study?: string;
  start_date?: string;
  end_date?: string;
  gpa?: number;
}

/**
 * Skill
 */
export interface Skill {
  name: string;
  proficiency?: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  endorsements?: number;
}

/**
 * Project
 */
export interface Project {
  id: string;
  name: string;
  description?: string;
  url?: string;
  repository_url?: string;
  start_date?: string;
  end_date?: string;
  technologies?: string[];
}

/**
 * Certification
 */
export interface Certification {
  id: string;
  name: string;
  issuer: string;
  issue_date: string;
  expiry_date?: string;
  credential_id?: string;
  credential_url?: string;
}

/**
 * Consent record
 */
export interface ConsentRecord {
  '@type': 'ConsentRecord';
  id: string;
  version: string;
  user_id: string;
  granted_at: string;
  expires_at?: string;
  consent_type: string;
  granted_permissions: GrantedPermissions;
  authorized_executors: AuthorizedExecutor[];
  special_instructions?: {
    [category: string]: string;
  };
  blockchain_proof?: BlockchainProof;
  signature?: DigitalSignature;
}

/**
 * Granted permissions
 */
export interface GrantedPermissions {
  data_export?: {
    enabled: boolean;
    categories: string[];
    format: string[];
    encryption_required: boolean;
  };
  data_transfer?: {
    enabled: boolean;
    allowed_destinations: string[];
    prohibited_destinations?: string[];
  };
  data_deletion?: {
    enabled: boolean;
    delay_days?: number;
    preserve_categories?: string[];
  };
}

/**
 * Authorized executor
 */
export interface AuthorizedExecutor {
  executor_id: string;
  name: string;
  relationship: string;
  email: string;
  phone?: string;
  permissions: string[];
  priority: number;
  alternate_contact?: {
    email?: string;
    phone?: string;
  };
}

/**
 * Blockchain proof
 */
export interface BlockchainProof {
  chain: string;
  contract_address?: string;
  transaction_hash: string;
  block_number: number;
  timestamp: string;
}

/**
 * Digital signature
 */
export interface DigitalSignature {
  algorithm: string;
  public_key: string;
  signature_value: string;
}

/**
 * Export configuration
 */
export interface ExportConfig {
  deceased_id: string;
  executor_id: string;
  categories: string[];
  format: 'json-ld' | 'xml' | 'csv';
  encryption?: {
    enabled: boolean;
    algorithm?: 'AES-256-GCM' | 'ChaCha20-Poly1305';
    password?: string;
  };
  include_metadata?: boolean;
}

/**
 * Export result
 */
export interface ExportResult {
  export_id: string;
  status: 'initiated' | 'in_progress' | 'completed' | 'failed';
  progress_percent?: number;
  estimated_completion?: string;
  download_url?: string;
  dpp?: DataPortabilityPackage;
  error?: string;
}

/**
 * Import configuration
 */
export interface ImportConfig {
  dpp: DataPortabilityPackage;
  destination_platform: string;
  import_options?: {
    privacy_level?: 'public' | 'friends' | 'private';
    enable_comments?: boolean;
    notification_preferences?: 'all' | 'some' | 'none';
  };
}

/**
 * Import result
 */
export interface ImportResult {
  import_id: string;
  status: 'validating' | 'importing' | 'completed' | 'failed';
  validation_results?: ValidationResult;
  items_imported?: number;
  error?: string;
}

/**
 * Validation result
 */
export interface ValidationResult {
  valid: boolean;
  format_valid?: boolean;
  schema_valid?: boolean;
  encryption_valid?: boolean;
  errors?: string[];
  warnings?: string[];
}

/**
 * Transfer configuration
 */
export interface TransferConfig {
  source: string;
  destination: string;
  dataTypes: string[];
  executorConsent: boolean;
  schema_mapping?: any;
}

/**
 * Transfer result
 */
export interface TransferResult {
  transferId: string;
  status: 'initiated' | 'in_progress' | 'completed' | 'failed';
  items_transferred?: number;
  error?: string;
}

/**
 * SDK configuration
 */
export interface SDKConfig {
  deceasedId?: string;
  executorCredentials?: {
    id: string;
    privateKey: string;
  };
  apiEndpoint?: string;
  apiKey?: string;
  environment?: 'development' | 'staging' | 'production';
}
