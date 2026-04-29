// ===== WIA Language Modal v2.1 (254 languages) =====
const WIA_MODAL_CONFIG = window.WIA_MODAL_CONFIG || {};
const I18N_BASE_PATH = WIA_MODAL_CONFIG.i18nBasePath || '/i18n/';
const LANG_COUNT = String(WIA_MODAL_CONFIG.languageCount || 254);
const DEFAULT_LANG = WIA_MODAL_CONFIG.defaultLang || 'ko';
const STORAGE_KEY = WIA_MODAL_CONFIG.storageKey || 'wia-nav-lang';

const RTL_CODES = new Set([
    'ar', 'ar-BH', 'ar-QA', 'he', 'fa', 'ur', 'ps', 'ckb', 'sd', 'ug',
    'bal', 'ks', 'yi', 'dv', 'ku', 'bjq'
]);

/**
 * WIA GIL - Language Modal (Unified UI)
 * kaitrust 스타일 UI + WIA GIL i18n 시스템
 * 독립적으로 작동 (외부 의존 없음)
 * @version 2.0.0
 */

// ============================================
// 📋 254개 언어 목록 (국기 포함)
// ============================================
const LANGUAGE_LIST = [
    { code: 'en', name: 'English', native: 'English', flag: '🇬🇧' },
    { code: 'ko', name: 'Korean', native: '한국어', flag: '🇰🇷' },
    { code: 'ja', name: 'Japanese', native: '日本語', flag: '🇯🇵' },
    { code: 'zh-CN', name: 'Chinese (Simplified)', native: '简体中文', flag: '🇨🇳' },
    { code: 'ar', name: 'Arabic', native: 'العربية', flag: '🇸🇦' },
    { code: 'hi', name: 'Hindi', native: 'हिन्दी', flag: '🇮🇳' },
    { code: 'bn', name: 'Bengali', native: 'বাংলা', flag: '🇧🇩' },
    { code: 'fr', name: 'French', native: 'Français', flag: '🇫🇷' },
    { code: 'es', name: 'Spanish', native: 'Español', flag: '🇪🇸' },
    { code: 'de', name: 'German', native: 'Deutsch', flag: '🇩🇪' },
    { code: 'pt', name: 'Portuguese', native: 'Português', flag: '🇵🇹' },
    { code: 'vi', name: 'Vietnamese', native: 'Tiếng Việt', flag: '🇻🇳' },
    { code: 'id', name: 'Indonesian', native: 'Bahasa Indonesia', flag: '🇮🇩' },
    { code: 'zh-TW', name: 'Chinese (Traditional)', native: '繁體中文', flag: '🇹🇼' },
    { code: 'zh-HK', name: 'Chinese (HK)', native: '粵語', flag: '🇭🇰' },
    { code: 'pt-BR', name: 'Portuguese (BR)', native: 'Português (BR)', flag: '🇧🇷' },
    { code: 'ru', name: 'Russian', native: 'Русский', flag: '🇷🇺' },
    { code: 'th', name: 'Thai', native: 'ไทย', flag: '🇹🇭' },
    { code: 'ms', name: 'Malay', native: 'Bahasa Melayu', flag: '🇲🇾' },
    { code: 'tl', name: 'Filipino', native: 'Filipino', flag: '🇵🇭' },
    { code: 'tr', name: 'Turkish', native: 'Türkçe', flag: '🇹🇷' },
    { code: 'pl', name: 'Polish', native: 'Polski', flag: '🇵🇱' },
    { code: 'nl', name: 'Dutch', native: 'Nederlands', flag: '🇳🇱' },
    { code: 'it', name: 'Italian', native: 'Italiano', flag: '🇮🇹' },
    { code: 'sv', name: 'Swedish', native: 'Svenska', flag: '🇸🇪' },
    { code: 'da', name: 'Danish', native: 'Dansk', flag: '🇩🇰' },
    { code: 'no', name: 'Norwegian', native: 'Norsk', flag: '🇳🇴' },
    { code: 'fi', name: 'Finnish', native: 'Suomi', flag: '🇫🇮' },
    { code: 'el', name: 'Greek', native: 'Ελληνικά', flag: '🇬🇷' },
    { code: 'he', name: 'Hebrew', native: 'עברית', flag: '🇮🇱' },
    { code: 'uk', name: 'Ukrainian', native: 'Українська', flag: '🇺🇦' },
    { code: 'cs', name: 'Czech', native: 'Čeština', flag: '🇨🇿' },
    { code: 'ro', name: 'Romanian', native: 'Română', flag: '🇷🇴' },
    { code: 'hu', name: 'Hungarian', native: 'Magyar', flag: '🇭🇺' },
    { code: 'bg', name: 'Bulgarian', native: 'Български', flag: '🇧🇬' },
    { code: 'hr', name: 'Croatian', native: 'Hrvatski', flag: '🇭🇷' },
    { code: 'sk', name: 'Slovak', native: 'Slovenčina', flag: '🇸🇰' },
    { code: 'sl', name: 'Slovenian', native: 'Slovenščina', flag: '🇸🇮' },
    { code: 'lt', name: 'Lithuanian', native: 'Lietuvių', flag: '🇱🇹' },
    { code: 'lv', name: 'Latvian', native: 'Latviešu', flag: '🇱🇻' },
    { code: 'et', name: 'Estonian', native: 'Eesti', flag: '🇪🇪' },
    { code: 'ta', name: 'Tamil', native: 'தமிழ்', flag: '🇮🇳' },
    { code: 'te', name: 'Telugu', native: 'తెలుగు', flag: '🇮🇳' },
    { code: 'mr', name: 'Marathi', native: 'मराठी', flag: '🇮🇳' },
    { code: 'gu', name: 'Gujarati', native: 'ગુજરાતી', flag: '🇮🇳' },
    { code: 'kn', name: 'Kannada', native: 'ಕನ್ನಡ', flag: '🇮🇳' },
    { code: 'ml', name: 'Malayalam', native: 'മലയാളം', flag: '🇮🇳' },
    { code: 'pa', name: 'Punjabi', native: 'ਪੰਜਾਬੀ', flag: '🇮🇳' },
    { code: 'or', name: 'Odia', native: 'ଓଡ଼ିଆ', flag: '🇮🇳' },
    { code: 'si', name: 'Sinhala', native: 'සිංහල', flag: '🇱🇰' },
    { code: 'ne', name: 'Nepali', native: 'नेपाली', flag: '🇳🇵' },
    { code: 'ur', name: 'Urdu', native: 'اردو', flag: '🇵🇰' },
    { code: 'fa', name: 'Persian', native: 'فارسی', flag: '🇮🇷' },
    { code: 'ps', name: 'Pashto', native: 'پښتو', flag: '🇦🇫' },
    { code: 'ku', name: 'Kurdish', native: 'Kurdî', flag: '🇮🇶' },
    { code: 'ckb', name: 'Kurdish (Sorani)', native: 'سۆرانی', flag: '🇮🇶' },
    { code: 'my', name: 'Myanmar', native: 'မြန်မာ', flag: '🇲🇲' },
    { code: 'km', name: 'Khmer', native: 'ខ្មែរ', flag: '🇰🇭' },
    { code: 'lo', name: 'Lao', native: 'ລາວ', flag: '🇱🇦' },
    { code: 'ka', name: 'Georgian', native: 'ქართული', flag: '🇬🇪' },
    { code: 'hy', name: 'Armenian', native: 'Հայերեն', flag: '🇦🇲' },
    { code: 'az', name: 'Azerbaijani', native: 'Azərbaycan', flag: '🇦🇿' },
    { code: 'kk', name: 'Kazakh', native: 'Қазақша', flag: '🇰🇿' },
    { code: 'ky', name: 'Kyrgyz', native: 'Кыргызча', flag: '🇰🇬' },
    { code: 'uz', name: 'Uzbek', native: "O'zbek", flag: '🇺🇿' },
    { code: 'tg', name: 'Tajik', native: 'Тоҷикӣ', flag: '🇹🇯' },
    { code: 'mn', name: 'Mongolian', native: 'Монгол', flag: '🇲🇳' },
    { code: 'bo', name: 'Tibetan', native: 'བོད་སྐད', flag: '🏔️' },
    { code: 'dz', name: 'Dzongkha', native: 'རྫོང་ཁ', flag: '🇧🇹' },
    { code: 'sw', name: 'Swahili', native: 'Kiswahili', flag: '🇹🇿' },
    { code: 'am', name: 'Amharic', native: 'አማርኛ', flag: '🇪🇹' },
    { code: 'ha', name: 'Hausa', native: 'Hausa', flag: '🇳🇬' },
    { code: 'yo', name: 'Yoruba', native: 'Yorùbá', flag: '🇳🇬' },
    { code: 'ig', name: 'Igbo', native: 'Igbo', flag: '🇳🇬' },
    { code: 'zu', name: 'Zulu', native: 'isiZulu', flag: '🇿🇦' },
    { code: 'xh', name: 'Xhosa', native: 'isiXhosa', flag: '🇿🇦' },
    { code: 'af', name: 'Afrikaans', native: 'Afrikaans', flag: '🇿🇦' },
    { code: 'so', name: 'Somali', native: 'Soomaali', flag: '🇸🇴' },
    { code: 'rw', name: 'Kinyarwanda', native: 'Kinyarwanda', flag: '🇷🇼' },
    { code: 'sn', name: 'Shona', native: 'chiShona', flag: '🇿🇼' },
    { code: 'ny', name: 'Chichewa', native: 'Chichewa', flag: '🇲🇼' },
    { code: 'mg', name: 'Malagasy', native: 'Malagasy', flag: '🇲🇬' },
    { code: 'tn', name: 'Tswana', native: 'Setswana', flag: '🇧🇼' },
    { code: 'st', name: 'Sesotho', native: 'Sesotho', flag: '🇱🇸' },
    { code: 'ss', name: 'Swati', native: 'siSwati', flag: '🇸🇿' },
    { code: 'ti', name: 'Tigrinya', native: 'ትግርኛ', flag: '🇪🇷' },
    { code: 'wo', name: 'Wolof', native: 'Wolof', flag: '🇸🇳' },
    { code: 'ff', name: 'Fulah', native: 'Fulfulde', flag: '🇬🇳' },
    { code: 'lg', name: 'Luganda', native: 'Luganda', flag: '🇺🇬' },
    { code: 'om', name: 'Oromo', native: 'Afaan Oromoo', flag: '🇪🇹' },
    { code: 'ak', name: 'Akan', native: 'Akan', flag: '🇬🇭' },
    { code: 'tw', name: 'Twi', native: 'Twi', flag: '🇬🇭' },
    { code: 'rn', name: 'Kirundi', native: 'Ikirundi', flag: '🇧🇮' },
    { code: 'ln', name: 'Lingala', native: 'Lingála', flag: '🇨🇩' },
    { code: 'kg', name: 'Kongo', native: 'Kikongo', flag: '🇨🇩' },
    { code: 'bm', name: 'Bambara', native: 'Bamanankan', flag: '🇲🇱' },
    { code: 'ee', name: 'Ewe', native: 'Eʋegbe', flag: '🇬🇭' },
    { code: 'sg', name: 'Sango', native: 'Sängö', flag: '🇨🇫' },
    { code: 've', name: 'Venda', native: 'Tshivenḓa', flag: '🇿🇦' },
    { code: 'nr', name: 'Ndebele', native: 'isiNdebele', flag: '🇿🇦' },
    { code: 'nso', name: 'Northern Sotho', native: 'Sepedi', flag: '🇿🇦' },
    { code: 'ts', name: 'Tsonga', native: 'Xitsonga', flag: '🇿🇦' },
    { code: 'sr', name: 'Serbian', native: 'Српски', flag: '🇷🇸' },
    { code: 'bs', name: 'Bosnian', native: 'Bosanski', flag: '🇧🇦' },
    { code: 'mk', name: 'Macedonian', native: 'Македонски', flag: '🇲🇰' },
    { code: 'sq', name: 'Albanian', native: 'Shqip', flag: '🇦🇱' },
    { code: 'mt', name: 'Maltese', native: 'Malti', flag: '🇲🇹' },
    { code: 'ga', name: 'Irish', native: 'Gaeilge', flag: '🇮🇪' },
    { code: 'cy', name: 'Welsh', native: 'Cymraeg', flag: '🏴' },
    { code: 'gd', name: 'Scottish Gaelic', native: 'Gàidhlig', flag: '🏴' },
    { code: 'eu', name: 'Basque', native: 'Euskara', flag: '🇪🇸' },
    { code: 'ca', name: 'Catalan', native: 'Català', flag: '🇪🇸' },
    { code: 'gl', name: 'Galician', native: 'Galego', flag: '🇪🇸' },
    { code: 'is', name: 'Icelandic', native: 'Íslenska', flag: '🇮🇸' },
    { code: 'be', name: 'Belarusian', native: 'Беларуская', flag: '🇧🇾' },
    { code: 'tt', name: 'Tatar', native: 'Татарча', flag: '🇷🇺' },
    { code: 'ce', name: 'Chechen', native: 'Нохчийн', flag: '🇷🇺' },
    { code: 'cv', name: 'Chuvash', native: 'Чӑвашла', flag: '🇷🇺' },
    { code: 'la', name: 'Latin', native: 'Latina', flag: '🏛️' },
    { code: 'sa', name: 'Sanskrit', native: 'संस्कृतम्', flag: '🕉️' },
    { code: 'mi', name: 'Māori', native: 'Te Reo Māori', flag: '🇳🇿' },
    { code: 'haw', name: 'Hawaiian', native: 'ʻŌlelo Hawaiʻi', flag: '🌺' },
    { code: 'sm', name: 'Samoan', native: 'Gagana Samoa', flag: '🇼🇸' },
    { code: 'to', name: 'Tongan', native: 'Lea Fakatonga', flag: '🇹🇴' },
    { code: 'fj', name: 'Fijian', native: 'Na Vosa Vakaviti', flag: '🇫🇯' },
    { code: 'ty', name: 'Tahitian', native: 'Reo Tahiti', flag: '🇵🇫' },
    { code: 'qu', name: 'Quechua', native: 'Runasimi', flag: '🇵🇪' },
    { code: 'gn', name: 'Guaraní', native: "Avañe'ẽ", flag: '🇵🇾' },
    { code: 'ay', name: 'Aymara', native: 'Aymar aru', flag: '🇧🇴' },
    { code: 'nah', name: 'Nahuatl', native: 'Nāhuatl', flag: '🇲🇽' },
    { code: 'ht', name: 'Haitian Creole', native: 'Kreyòl ayisyen', flag: '🇭🇹' },
    { code: 'ks', name: 'Kashmiri', native: 'कॉशुर', flag: '🇮🇳' },
    { code: 'sd', name: 'Sindhi', native: 'سنڌي', flag: '🇵🇰' },
    { code: 'ug', name: 'Uyghur', native: 'ئۇيغۇرچە', flag: '🇨🇳' },
    { code: 'as', name: 'Assamese', native: 'অসমীয়া', flag: '🇮🇳' },
    { code: 'sc', name: 'Sardinian', native: 'Sardu', flag: '🇮🇹' },
    { code: 'oc', name: 'Occitan', native: 'Occitan', flag: '🇫🇷' },
    { code: 'br', name: 'Breton', native: 'Brezhoneg', flag: '🇫🇷' },
    { code: 'co', name: 'Corsican', native: 'Corsu', flag: '🇫🇷' },
    { code: 'lb', name: 'Luxembourgish', native: 'Lëtzebuergesch', flag: '🇱🇺' },
    { code: 'fo', name: 'Faroese', native: 'Føroyskt', flag: '🇫🇴' },
    { code: 'rm', name: 'Romansh', native: 'Rumantsch', flag: '🇨🇭' },
    { code: 'fil', name: 'Filipino (Tagalog)', native: 'Tagalog', flag: '🇵🇭' },
    { code: 'es-MX', name: 'Spanish (Mexico)', native: 'Español (MX)', flag: '🇲🇽' },
    { code: 'es-AR', name: 'Spanish (Argentina)', native: 'Español (AR)', flag: '🇦🇷' },
    { code: 'es-CO', name: 'Spanish (Colombia)', native: 'Español (CO)', flag: '🇨🇴' },
    { code: 'es-CL', name: 'Spanish (Chile)', native: 'Español (CL)', flag: '🇨🇱' },
    { code: 'es-PE', name: 'Spanish (Peru)', native: 'Español (PE)', flag: '🇵🇪' },
    { code: 'fr-CA', name: 'French (Canada)', native: 'Français (CA)', flag: '🇨🇦' },
    { code: 'se', name: 'Northern Sami', native: 'Davvisámegiella', flag: '🇳🇴' },
    { code: 'gv', name: 'Manx', native: 'Gaelg', flag: '🇮🇲' },
    { code: 'kw', name: 'Cornish', native: 'Kernewek', flag: '🏴' },
    { code: 'an', name: 'Aragonese', native: 'Aragonés', flag: '🇪🇸' },
    { code: 'ast', name: 'Asturian', native: 'Asturianu', flag: '🇪🇸' },
    { code: 'fur', name: 'Friulian', native: 'Furlan', flag: '🇮🇹' },
    { code: 'scn', name: 'Sicilian', native: 'Sicilianu', flag: '🇮🇹' },
    { code: 'nap', name: 'Neapolitan', native: 'Napulitano', flag: '🇮🇹' },
    { code: 'lij', name: 'Ligurian', native: 'Ligure', flag: '🇮🇹' },
    { code: 'lmo', name: 'Lombard', native: 'Lombard', flag: '🇮🇹' },
    { code: 'vec', name: 'Venetian', native: 'Vèneto', flag: '🇮🇹' },
    { code: 'pms', name: 'Piedmontese', native: 'Piemontèis', flag: '🇮🇹' },
    { code: 'eml', name: 'Emilian-Romagnol', native: 'Emiliàn', flag: '🇮🇹' },
    { code: 'rgn', name: 'Romagnol', native: 'Rumagnòl', flag: '🇮🇹' },
    { code: 'ext', name: 'Extremaduran', native: 'Estremeñu', flag: '🇪🇸' },
    { code: 'mwl', name: 'Mirandese', native: 'Mirandés', flag: '🇵🇹' },
    { code: 'wa', name: 'Walloon', native: 'Walon', flag: '🇧🇪' },
    { code: 'pgl', name: 'Primitive Irish', native: 'Gaeilge', flag: '🇮🇪' },
    { code: 'krl', name: 'Karelian', native: 'Karjala', flag: '🇫🇮' },
    { code: 'vro', name: 'Võro', native: 'Võro', flag: '🇪🇪' },
    { code: 'liv', name: 'Livonian', native: 'Līvõ', flag: '🇱🇻' },
    { code: 'izh', name: 'Ingrian', native: 'Ižoran', flag: '🇷🇺' },
    { code: 'vot', name: 'Votic', native: 'Vaďďa', flag: '🇷🇺' },
    { code: 'me', name: 'Montenegrin', native: 'Crnogorski', flag: '🇲🇪' },
    { code: 'lad', name: 'Ladino', native: 'Judeoespañol', flag: '🇮🇱' },
    { code: 'yue', name: 'Cantonese', native: '廣東話', flag: '🇭🇰' },
    { code: 'nan', name: 'Min Nan', native: '閩南語', flag: '🇹🇼' },
    { code: 'hak', name: 'Hakka', native: '客家語', flag: '🇹🇼' },
    { code: 'wuu', name: 'Wu Chinese', native: '吳語', flag: '🇨🇳' },
    { code: 'gan', name: 'Gan Chinese', native: '贛語', flag: '🇨🇳' },
    { code: 'hsn', name: 'Xiang Chinese', native: '湘語', flag: '🇨🇳' },
    { code: 'cdo', name: 'Min Dong', native: '閩東語', flag: '🇨🇳' },
    { code: 'za', name: 'Zhuang', native: 'Vahcuengh', flag: '🇨🇳' },
    { code: 'bi', name: 'Bislama', native: 'Bislama', flag: '🇻🇺' },
    { code: 'tpi', name: 'Tok Pisin', native: 'Tok Pisin', flag: '🇵🇬' },
    { code: 'tvl', name: 'Tuvaluan', native: 'Te Ggana Tuuvalu', flag: '🇹🇻' },
    { code: 'nau', name: 'Nauruan', native: 'Ekakairũ Naoero', flag: '🇳🇷' },
    { code: 'mh', name: 'Marshallese', native: 'Kajin M̧ajeļ', flag: '🇲🇭' },
    { code: 'tkl', name: 'Tokelauan', native: 'Te Gagana Tokelau', flag: '🇹🇰' },
    { code: 'niu', name: 'Niuean', native: 'Vagahau Niuē', flag: '🇳🇺' },
    { code: 'rar', name: 'Cook Islands Māori', native: 'Māori Kūki ʻĀirani', flag: '🇨🇰' },
    { code: 'pau', name: 'Palauan', native: 'a tekoi er a Belau', flag: '🇵🇼' },
    { code: 'chk', name: 'Chuukese', native: 'Chuuk', flag: '🇫🇲' },
    { code: 'pon', name: 'Pohnpeian', native: 'Pohnpei', flag: '🇫🇲' },
    { code: 'kos', name: 'Kosraean', native: 'Kosrae', flag: '🇫🇲' },
    { code: 'yap', name: 'Yapese', native: 'Yapese', flag: '🇫🇲' },
    { code: 'gil', name: 'Gilbertese', native: 'Taetae ni Kiribati', flag: '🇰🇮' },
    { code: 'aue', name: 'Australian English', native: 'Australian English', flag: '🇦🇺' },
    { code: 'pap', name: 'Papiamento', native: 'Papiamentu', flag: '🇨🇼' },
    { code: 'srn', name: 'Sranan Tongo', native: 'Sranantongo', flag: '🇸🇷' },
    { code: 'djk', name: 'Aukan', native: 'Ndyuka', flag: '🇸🇷' },
    { code: 'arn', name: 'Mapuche', native: 'Mapudungun', flag: '🇨🇱' },
    { code: 'aig', name: 'Antiguan Creole', native: 'Antiguan Creole', flag: '🇦🇬' },
    { code: 'ar-BH', name: 'Arabic (Bahrain)', native: 'العربية (البحرين)', flag: '🇧🇭' },
    { code: 'ar-QA', name: 'Arabic (Qatar)', native: 'العربية (قطر)', flag: '🇶🇦' },
    { code: 'au-kriol', name: 'Australian Kriol', native: 'Kriol', flag: '🇦🇺' },
    { code: 'bal', name: 'Balochi', native: 'بلوچی', flag: '🇵🇰' },
    { code: 'bdr', name: 'Bajau', native: 'Bajau', flag: '🇲🇾' },
    { code: 'bhs', name: 'Bahamian Creole', native: 'Bahamian Creole', flag: '🇧🇸' },
    { code: 'bjq', name: 'Southern Sama', native: 'Bajau Laut', flag: '🇲🇾' },
    { code: 'ca-AD', name: 'Catalan (Andorra)', native: 'Català (Andorra)', flag: '🇦🇩' },
    { code: 'ceb', name: 'Cebuano', native: 'Cebuano', flag: '🇵🇭' },
    { code: 'cim', name: 'Cimbrian', native: 'Zimbrisch', flag: '🇮🇹' },
    { code: 'crg', name: 'Michif', native: 'Michif', flag: '🇨🇦' },
    { code: 'crs', name: 'Seychellois Creole', native: 'Kreol Seselwa', flag: '🇸🇨' },
    { code: 'csb', name: 'Kashubian', native: 'Kaszëbsczi', flag: '🇵🇱' },
    { code: 'de-AT', name: 'German (Austria)', native: 'Deutsch (Österreich)', flag: '🇦🇹' },
    { code: 'de-LI', name: 'German (Liechtenstein)', native: 'Deutsch (Liechtenstein)', flag: '🇱🇮' },
    { code: 'dsb', name: 'Lower Sorbian', native: 'Dolnoserbšćina', flag: '🇩🇪' },
    { code: 'dtp', name: 'Kadazandusun', native: 'Kadazandusun', flag: '🇲🇾' },
    { code: 'esu', name: 'Central Alaskan Yupik', native: 'Yugtun', flag: '🇺🇸' },
    { code: 'fr-MC', name: 'French (Monaco)', native: 'Français (Monaco)', flag: '🇲🇨' },
    { code: 'frp', name: 'Franco-Provençal', native: 'Arpitan', flag: '🇫🇷' },
    { code: 'gcl', name: 'Grenadian Creole', native: 'Grenadian Creole', flag: '🇬🇩' },
    { code: 'grc', name: 'Ancient Greek', native: 'Ἑλληνική', flag: '🏛️' },
    { code: 'hsb', name: 'Upper Sorbian', native: 'Hornjoserbšćina', flag: '🇩🇪' },
    { code: 'it-SM', name: 'Italian (San Marino)', native: 'Italiano (San Marino)', flag: '🇸🇲' },
    { code: 'it-VA', name: 'Italian (Vatican)', native: 'Italiano (Vaticano)', flag: '🇻🇦' },
    { code: 'jv', name: 'Javanese', native: 'Basa Jawa', flag: '🇮🇩' },
    { code: 'kea', name: 'Cape Verdean Creole', native: 'Kabuverdianu', flag: '🇨🇻' },
    { code: 'kl', name: 'Greenlandic', native: 'Kalaallisut', flag: '🇬🇱' },
    { code: 'ky-CI', name: 'Cayman Islands English', native: 'Cayman English', flag: '🇰🇾' },
    { code: 'lld', name: 'Ladin', native: 'Ladin', flag: '🇮🇹' },
    { code: 'mfe', name: 'Mauritian Creole', native: 'Kreol Morisien', flag: '🇲🇺' },
    { code: 'mhn', name: 'Mòcheno', native: 'Mócheno', flag: '🇮🇹' },
    { code: 'mnc', name: 'Manchu', native: 'ᠮᠠᠨᠵᡠ ᡤᡳᠰᡠᠨ', flag: '🇨🇳' },
    { code: 'moe', name: 'Montagnais', native: 'Innu-aimun', flag: '🇨🇦' },
    { code: 'ms-BN', name: 'Malay (Brunei)', native: 'Melayu (Brunei)', flag: '🇧🇳' },
    { code: 'nde', name: 'Northern Ndebele', native: 'isiNdebele', flag: '🇿🇼' },
    { code: 'nl-BE', name: 'Dutch (Belgium)', native: 'Nederlands (België)', flag: '🇧🇪' },
    { code: 'nl-SR', name: 'Dutch (Suriname)', native: 'Nederlands (Suriname)', flag: '🇸🇷' },
    { code: 'nrf', name: 'Jèrriais', native: 'Jèrriais', flag: '🇯🇪' },
    { code: 'pih', name: 'Pitcairn-Norfolk', native: 'Pitkern', flag: '🇵🇳' },
    { code: 'sa-Deva', name: 'Sanskrit (Devanagari)', native: 'संस्कृतम् (देवनागरी)', flag: '🕉️' },
    { code: 'sco', name: 'Scots', native: 'Scots', flag: '🏴' },
    { code: 'su', name: 'Sundanese', native: 'Basa Sunda', flag: '🇮🇩' },
    { code: 'tcs', name: 'Torres Strait Creole', native: 'Torres Strait Creole', flag: '🇦🇺' },
    { code: 'tet', name: 'Tetum', native: 'Tetun', flag: '🇹🇱' },
    { code: 'tk', name: 'Turkmen', native: 'Türkmen', flag: '🇹🇲' },
    { code: 'vic', name: 'Virgin Islands Creole', native: 'Virgin Islands Creole', flag: '🇻🇮' },
    { code: 'wae', name: 'Walser German', native: 'Walliserdütsch', flag: '🇨🇭' },
    { code: 'wls', name: 'Wallisian', native: 'Fakaʻuvea', flag: '🇼🇫' },
    { code: 'yua', name: 'Yucatec Maya', native: "Maaya T'aan", flag: '🇲🇽' },
    { code: 'zdj', name: 'Comorian', native: 'Shindzuani', flag: '🇰🇲' },
    { code: 'zea', name: 'Zeelandic', native: 'Zeêuws', flag: '🇳🇱' },
];

// ============================================
// 🗺️ 대륙별 분류
// ============================================
const CONTINENT_MAP = {
    popular: ['en','ko','zh-CN','es','hi','ar','bn','pt','ru','ja','de','fr','tr','vi','it','th','id','pl','nl','ms'],
    asian: ['ko','ja','zh-CN','zh-TW','zh-HK','th','vi','id','ms','tl','fil','my','km','lo','hi','bn','ta','te','mr','gu','kn','ml','si','ne','ur','pa','ps','fa','ku','ckb','or','as','ks','sd','ug','ka','hy','az','kk','ky','uz','tg','mn','bo','dz','yue','nan','hak','wuu','gan','hsn','cdo','za','bal','bdr','bjq','ceb','dtp','jv','mnc','ms-BN','su','tk'],
    european: ['en','es','fr','de','it','pt','ru','pl','uk','nl','el','sv','no','da','fi','is','et','lv','lt','cs','sk','hu','ro','bg','hr','sr','sl','mk','sq','mt','ga','cy','gd','eu','ca','gl','tr','be','tt','ce','cv','bs','me','fo','lb','rm','an','ast','oc','br','co','sc','fur','scn','nap','lij','lmo','vec','pms','eml','rgn','ext','mwl','wa','pgl','krl','vro','liv','izh','vot','gv','kw','lad','se','ca-AD','cim','csb','de-AT','de-LI','dsb','frp','grc','hsb','it-SM','it-VA','lld','mhn','nl-BE','nrf','sa-Deva','sco','wae','zea','fr-MC','la'],
    african: ['ar','am','ha','yo','ig','sw','zu','xh','af','so','rw','sn','ny','mg','tn','st','ss','ti','wo','ff','lg','om','ak','tw','rn','ln','kg','bm','ee','sg','ve','nr','nso','ts','crs','kea','mfe','nde','zdj'],
    americas: ['es-MX','es-AR','es-CO','es-CL','es-PE','pt-BR','fr-CA','qu','gn','ay','nah','ht','pap','srn','djk','arn','aig','bhs','crg','esu','gcl','ky-CI','moe','nl-SR','pih','vic','yua'],
    rtl: ['ar','he','ur','fa','ps','ku','ckb','ks','sd','ug','ar-BH','ar-QA','bal','bjq'],
    oceanic: ['mi','fj','sm','to','ty','tvl','nau','mh','bi','tpi','tkl','niu','rar','pau','chk','pon','kos','yap','gil','aue','au-kriol','tcs','tet','wls','haw']
};

// ============================================
// 🌐 i18n 시스템
// ============================================
let currentLang = localStorage.getItem(STORAGE_KEY) || DEFAULT_LANG;
// 설정 모달 키도 동기화
localStorage.setItem('wiaLang', currentLang);
let translations = {};

async function loadTranslations(lang) {
    try {
        const basePath = I18N_BASE_PATH;
        const response = await fetch(`${basePath}${lang}.json`);
        if (response.ok) {
            translations = await response.json();
            currentLang = lang;
            localStorage.setItem(STORAGE_KEY, lang);
            applyTranslations();
            // 국가명 번역 리로드
            if (typeof loadCountryTranslations === 'function') {
                loadCountryTranslations(lang).then(function() {
                    if (typeof initCountryBar === 'function') initCountryBar();
                });
            }
            return true;
        }
    } catch (e) {
        console.warn(`Translation not found for ${lang}`);
    }
    if (lang !== 'en') return loadTranslations('en');
    return false;
}

function getNestedValue(obj, path) {
    return path.split('.').reduce((o, k) => (o || {})[k], obj);
}

function t(key) {
    return getNestedValue(translations, key) || key;
}

function applyTranslations() {
    document.querySelectorAll('[data-i18n]').forEach(el => {
        const key = el.getAttribute('data-i18n');
        const val = getNestedValue(translations, key);
        if (val) el.textContent = val;
    });
    document.querySelectorAll('[data-i18n-placeholder]').forEach(el => {
        const key = el.getAttribute('data-i18n-placeholder');
        const val = getNestedValue(translations, key);
        if (val) el.placeholder = val;
    });
    document.querySelectorAll('[data-i18n-title]').forEach(el => {
        const key = el.getAttribute('data-i18n-title');
        const val = getNestedValue(translations, key);
        if (val) el.title = val;
    });
}

// ============================================
// 🎨 언어 모달 (kaitrust 통일 UI)
// ============================================
let langModalFilter = 'all';
let currentLangNames = {}; // 현재 언어 기준 254개 언어명

function injectLanguageModalStyles() {
    if (document.getElementById('wia-nav-lang-modal-styles')) return;
    const style = document.createElement('style');
    style.id = 'wia-nav-lang-modal-styles';
    style.textContent = `
        #wiaNavLangModal {
            display: none; position: fixed; top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(0,0,0,0.95); z-index: 99999;
            align-items: flex-start; padding-top: 3vh; justify-content: center;
            animation: wiaFadeIn 0.3s;
        }
        #wiaNavLangModal.show { display: flex; }
        @keyframes wiaFadeIn { from{opacity:0} to{opacity:1} }
        @keyframes wiaSlideUp { from{transform:translateY(30px);opacity:0} to{transform:translateY(0);opacity:1} }

        .wia-nav-modal-content {
            background: white; border-radius: 24px; max-width: 1600px; width: 95%;
            max-height: 95vh; overflow: hidden;
            box-shadow: 0 30px 60px rgba(0,0,0,0.5); animation: wiaSlideUp 0.3s;
        }
        .wia-nav-modal-header {
            background: linear-gradient(135deg, #667eea, #764ba2); color: white;
            padding: 14px 35px; text-align: center; position: relative;
        }
        .wia-nav-modal-header h2 { margin: 0; font-size: 1.8rem; font-weight: 700; color: white; }
        .wia-nav-modal-header p { margin: 10px 0 0; font-size: 0.95rem; opacity: 0.95; color: white; }
        .wia-nav-modal-subtitle { margin: 5px 0 0 !important; font-size: 0.85rem !important; opacity: 0.85; font-style: italic; color: white; }
        .wia-nav-modal-close {
            position: absolute; top: 20px; right: 20px;
            background: rgba(255,255,255,0.2); border: none; color: white;
            font-size: 1.5rem; width: 48px; height: 48px; border-radius: 50%;
            cursor: pointer; transition: all 0.3s;
            display: flex; align-items: center; justify-content: center;
            line-height: 1;
        }
        .wia-nav-modal-close:hover { background: rgba(255,255,255,0.3); transform: rotate(90deg); }

        .wia-nav-modal-tabs {
            display: grid; grid-template-columns: repeat(8, 1fr);
            gap: 6px; padding: 12px 30px; background: #f8f9fa; border-bottom: 2px solid #e5e7eb;
        }
        .wia-nav-modal-tab {
            padding: 8px 4px; background: white; border: 2px solid #e5e7eb;
            border-radius: 12px; cursor: pointer; font-weight: 600; font-size: 0.8rem;
            transition: all 0.3s; text-align: center; color: #333; white-space: nowrap;
        }
        .wia-nav-modal-tab:hover { background: #f3f4f6; transform: translateY(-2px); }
        .wia-nav-modal-tab.active {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white; border-color: #667eea;
            box-shadow: 0 4px 15px rgba(102,126,234,0.3);
        }

        .wia-nav-modal-body { padding: 20px 30px; overflow-y: auto; max-height: 520px; }
        .wia-nav-lang-search {
            width: 100% !important; box-sizing: border-box !important; padding: 12px 14px !important;
            border: 2px solid #e5e7eb !important; border-radius: 14px !important; font-size: 0.95rem !important; margin-bottom: 15px !important;
        }
        .wia-nav-lang-search:focus { outline: none; border-color: #667eea; box-shadow: 0 0 0 3px rgba(102,126,234,0.1); }
        .wia-nav-lang-count {
            text-align: center; padding: 10px;
            background: linear-gradient(135deg, rgba(102,126,234,0.15), rgba(118,75,162,0.15));
            border-radius: 12px; margin-bottom: 12px; font-weight: 600; font-size: 0.9rem; color: #667eea;
        }
        .wia-nav-lang-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 10px; }
        .wia-nav-lang-item {
            color: #1f2937; padding: 8px 10px; background: white; border: 2px solid #d1d5db;
            border-radius: 12px; text-align: center; cursor: pointer; transition: all 0.2s;
            font-size: 0.9rem; font-weight: 500; min-height: 38px;
            display: flex; align-items: center; justify-content: center; gap: 6px;
        }
        .wia-nav-lang-item:hover {
            background: linear-gradient(135deg, rgba(102,126,234,0.25), rgba(118,75,162,0.25));
            border-color: #667eea; transform: translateY(-3px);
            box-shadow: 0 5px 15px rgba(102,126,234,0.2);
        }
        .wia-nav-lang-item.selected {
            background: linear-gradient(135deg, rgba(102,126,234,0.2), rgba(118,75,162,0.2));
            border-color: #667eea; font-weight: 600;
            box-shadow: 0 5px 20px rgba(102,126,234,0.4);
        }
        .wia-nav-modal-footer {
            text-align: center; padding: 20px 35px; background: #f8f9fa;
            border-top: 1px solid #e5e7eb; font-size: 0.85rem; color: #666;
        }
        .wia-nav-modal-footer a { color: #667eea; text-decoration: none; font-weight: 600; }

        @media (max-width: 768px) {
            .wia-nav-modal-tabs { grid-template-columns: repeat(4, 1fr); gap: 8px; padding: 15px 20px; }
            .wia-nav-modal-tab { padding: 8px 4px; font-size: 0.75rem; }
            .wia-nav-lang-grid { grid-template-columns: repeat(3, 1fr); }
            .wia-nav-modal-header { padding: 15px 20px; }
            .wia-nav-modal-header h2 { font-size: 1.8rem; }
            .wia-nav-modal-body { padding: 20px; }
        }
        @media (max-width: 480px) {
            .wia-nav-modal-content { width: 100%; border-radius: 16px; }
            .wia-nav-modal-tabs { grid-template-columns: repeat(3, 1fr); }
            .wia-nav-modal-tab { padding: 6px 2px; font-size: 0.65rem; }
            .wia-nav-lang-grid { grid-template-columns: repeat(2, 1fr); }
            .wia-nav-modal-body { max-height: 280px; padding: 10px; }
            .wia-nav-modal-header h2 { font-size: 1.4rem; }
            .wia-nav-modal-header p { font-size: 0.75rem; }
            .wia-nav-modal-subtitle { font-size: 0.65rem; }
            .wia-nav-lang-count { display: none; }
            .wia-nav-lang-search { padding: 12px; font-size: 0.95rem; }
            .wia-nav-modal-close { width: 40px; height: 40px; font-size: 2rem; top: 12px; right: 12px; }
            .wia-nav-modal-footer { padding: 12px 15px; font-size: 0.75rem; }
        }
    `;
    document.head.appendChild(style);
}

function injectLanguageModalHTML() {
    if (document.getElementById('wiaNavLangModal')) return;
    const div = document.createElement('div');
    div.innerHTML = `
    <div id="wiaNavLangModal">
        <div class="wia-nav-modal-content">
            <div class="wia-nav-modal-header">
                <button class="wia-nav-modal-close" onclick="closeLanguageModal()">×</button>
                <h2 id="lmTitle">🌐 Language / 언어</h2>
                <p id="lmSupports">SmileStory supports 254 languages</p>
                <p class="wia-nav-modal-subtitle" id="lmSubtitle">Universal Auth - Made with ❤️ for Humanity</p>
            </div>
            <div class="wia-nav-modal-tabs">
                <button class="wia-nav-modal-tab active" onclick="filterNavLang('all')" id="lmTabAll">All (254)</button>
                <button class="wia-nav-modal-tab" onclick="filterNavLang('popular')" id="lmTabPopular">⭐ Popular (20)</button>
                <button class="wia-nav-modal-tab" onclick="filterNavLang('asian')" id="lmTabAsian">🌏 Asian (63)</button>
                <button class="wia-nav-modal-tab" onclick="filterNavLang('european')" id="lmTabEuropean">🌍 European (95)</button>
                <button class="wia-nav-modal-tab" onclick="filterNavLang('african')" id="lmTabAfrican">🌍 African (39)</button>
                <button class="wia-nav-modal-tab" onclick="filterNavLang('americas')" id="lmTabAmericas">🌎 Americas (27)</button>
                <button class="wia-nav-modal-tab" onclick="filterNavLang('oceanic')" id="lmTabOceanic">🏝️ Oceanic (25)</button>
                <button class="wia-nav-modal-tab" onclick="filterNavLang('rtl')" id="lmTabRtl">← RTL (14)</button>
            </div>
            <div class="wia-nav-modal-body">
                <input type="text" class="wia-nav-lang-search" id="wiaNavLangSearch" placeholder="🔍 Search 254 languages..." oninput="searchNavLang(this.value)">
                <div class="wia-nav-lang-count" id="wiaNavLangCount">Loading...</div>
                <div class="wia-nav-lang-grid" id="wiaNavLangGrid"></div>
            </div>
            <div class="wia-nav-modal-footer">
                <div style="margin-bottom:8px; color:#888;" id="lmApology">We apologize if your language is not listed yet. <a href="mailto:global@thekoreantoday.com">📧 global@thekoreantoday.com</a></div>
                <span id="lmCopyright">© 2026 <a href="https://smilestory.ai/" target="_blank">SmileStory</a> Group. All rights reserved.</span> Made with ❤️ for Humanity
            </div>
        </div>
    </div>`;
    document.body.appendChild(div.firstElementChild);
}

function renderNavLangGrid(langs) {
    const grid = document.getElementById('wiaNavLangGrid');
    const count = document.getElementById('wiaNavLangCount');
    if (!grid) return;
    // ★ 모달 카운트도 현지화
    const showingText = window._i18n && window._i18n.language_modal && window._i18n.language_modal.showing 
        ? window._i18n.language_modal.showing.replace('{count}', langs.length)
        : 'Showing ' + langs.length + ' languages';
    count.textContent = showingText;
    grid.innerHTML = langs.map(l => `
        <div class="wia-nav-lang-item ${l.code === currentLang ? 'selected' : ''}" data-code="${l.code}" onclick="selectNavLang('${l.code}')">
            ${l.flag} ${currentLangNames[l.code] || l.native}
        </div>
    `).join('');
}

function filterNavLang(filter) {
    langModalFilter = filter;
    document.querySelectorAll('.wia-nav-modal-tab').forEach((tab, i) => {
        const filters = ['all','popular','asian','european','african','americas','oceanic','rtl'];
        tab.classList.toggle('active', filters[i] === filter);
    });
    if (filter === 'all') {
        renderNavLangGrid(LANGUAGE_LIST);
    } else {
        const codes = CONTINENT_MAP[filter] || [];
        renderNavLangGrid(LANGUAGE_LIST.filter(l => codes.includes(l.code)));
    }
}

function searchNavLang(query) {
    const q = query.toLowerCase();
    if (!q) { filterNavLang(langModalFilter); return; }
    renderNavLangGrid(LANGUAGE_LIST.filter(l =>
        l.native.toLowerCase().includes(q) || l.name.toLowerCase().includes(q) || l.code.toLowerCase().includes(q)
    ));
}

async function selectNavLang(code) {
    currentLang = code;
    localStorage.setItem(STORAGE_KEY, code);
    localStorage.setItem('wiaLang', code);
    const display = document.getElementById('currentLangCode');
    if (display) display.textContent = code.split('-')[0].toUpperCase().substring(0, 2);
    document.querySelectorAll('.wia-nav-lang-item').forEach(item => {
        item.classList.toggle('selected', item.dataset.code === code);
    });
    await loadTranslations(code);
    // ★ 현재 언어의 language_names 로드 → 그리드 현지화
    try {
        const langData = await fetch(I18N_BASE_PATH + code + '.json').then(r => r.json());
        if (langData.language_names) {
            currentLangNames = langData.language_names;
        } else {
            currentLangNames = {};
        }
    } catch(e) { currentLangNames = {}; }
    // ★ 모달 헤더/푸터 현지화
    updateModalLocale(code);
    setTimeout(closeLanguageModal, 300);
    window.dispatchEvent(new CustomEvent('wia-language-changed', { detail: { language: code } }));
}

function openLanguageModal() {
    const modal = document.getElementById('wiaNavLangModal');
    if (!modal) { injectLanguageModalHTML(); }
    document.getElementById('wiaNavLangModal').classList.add('show');
    filterNavLang('all');
}

function updateModalLocale(code) {
    try {
        fetch(I18N_BASE_PATH + code + '.json').then(r => r.json()).then(d => {
            const lm = d.language_modal;
            if (!lm) return;
            const el = (id) => document.getElementById(id);
            if (lm.title && el('lmTitle')) el('lmTitle').textContent = '🌐 ' + lm.title;
            if (lm.supports && el('lmSupports')) el('lmSupports').textContent = lm.supports.replace('{count}', LANG_COUNT);
            if (lm.subtitle && el('lmSubtitle')) el('lmSubtitle').textContent = lm.subtitle;
            if (lm.search && el('wiaNavLangSearch')) el('wiaNavLangSearch').placeholder = '🔍 ' + lm.search.replace('{count}', LANG_COUNT);
            if (lm.apology && el('lmApology')) el('lmApology').innerHTML = lm.apology + ' <a href="mailto:global@thekoreantoday.com">📧 global@thekoreantoday.com</a>';
            if (lm.copyright && el('lmCopyright')) el('lmCopyright').innerHTML = lm.copyright.replace('{year}', '2026').replace('{link}', '<a href="https://smilestory.ai/" target="_blank">SmileStory</a>');
            // 탭 i18n (숫자 포함)
            if (lm.tab_all && el('lmTabAll')) el('lmTabAll').textContent = lm.tab_all.replace('{count}', LANG_COUNT);
            if (lm.tab_popular && el('lmTabPopular')) el('lmTabPopular').textContent = '⭐ ' + lm.tab_popular + ' (20)';
            if (lm.tab_asian && el('lmTabAsian')) el('lmTabAsian').textContent = '🌏 ' + lm.tab_asian + ' (63)';
            if (lm.tab_european && el('lmTabEuropean')) el('lmTabEuropean').textContent = '🌍 ' + lm.tab_european + ' (95)';
            if (lm.tab_african && el('lmTabAfrican')) el('lmTabAfrican').textContent = '🌍 ' + lm.tab_african + ' (39)';
            if (lm.tab_americas && el('lmTabAmericas')) el('lmTabAmericas').textContent = '🌎 ' + lm.tab_americas + ' (27)';
            if (lm.tab_oceanic && el('lmTabOceanic')) el('lmTabOceanic').textContent = '🏝️ ' + lm.tab_oceanic + ' (25)';
            if (el('lmTabRtl')) el('lmTabRtl').textContent = '← RTL (14)';
        });
    } catch(e) {}
}

function closeLanguageModal() {
    const modal = document.getElementById('wiaNavLangModal');
    if (modal) modal.classList.remove('show');
}

// ============================================
// 🚀 초기화
// ============================================
(function initLanguageSystem() {
    injectLanguageModalStyles();
    injectLanguageModalHTML();
    renderNavLangGrid(LANGUAGE_LIST);

    // 저장된 언어 적용
    const saved = localStorage.getItem(STORAGE_KEY) || DEFAULT_LANG;
    const display = document.getElementById('currentLangCode');
    if (display) display.textContent = saved.split('-')[0].toUpperCase().substring(0, 2);
    
    // ★ 초기 language_names 로드
    fetch(I18N_BASE_PATH + saved + '.json')
        .then(r => r.json())
        .then(d => { if (d.language_names) { currentLangNames = d.language_names; renderNavLangGrid(LANGUAGE_LIST); } })
        .catch(() => {});
    
    // ★ 초기 모달 로케일
    updateModalLocale(saved);
    loadTranslations(saved);
})();

console.log('[WIA GIL] Language Modal v2.0 (Unified UI) - 254 languages ready');
