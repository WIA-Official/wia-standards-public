/**
 * WIA Language Modal 211 Component - V2 (더 큰 버전)
 * @version 2.1.0
 * @author PM Claude
 * @description 5열 그리드, 큰 폰트 버전
 */

class WIALanguageModal {
    constructor() {
        this.languages = {};
        this.currentFilter = 'all';
        this.currentLang = localStorage.getItem('wiabooks_language') || 'en';
        this.modalId = 'wiaLanguageModal211';
        this.initialized = false;
    }

    async init() {
        if (this.initialized) {
            console.log('⚠️ WIA Language Modal already initialized');
            return;
        }

        console.log('🚀 Initializing WIA Language Modal 211 V2...');
        
        this.injectStyles();
        this.injectHTML();
        await this.loadLanguages();
        this.bindEvents();
        this.registerGlobalFunctions();
        
        this.initialized = true;
        console.log('✅ WIA Language Modal 211 V2 Ready!');
    }

    injectStyles() {
        if (document.getElementById('wia-language-modal-styles')) {
            document.getElementById('wia-language-modal-styles').remove();
        }
        
        const styles = `
        <style id="wia-language-modal-styles">
            #${this.modalId} {
                display: none;
                position: fixed;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background: rgba(0, 0, 0, 0.95);
                z-index: 99999;
                align-items: center;
                justify-content: center;
                animation: fadeIn 0.3s;
            }
            
            #${this.modalId}.show {
                display: flex;
            }
            
            @keyframes fadeIn {
                from { opacity: 0; }
                to { opacity: 1; }
            }
            
            .wia-modal-content {
                background: white;
                border-radius: 24px;
                max-width: 1600px;  /* 더 크게! */
                width: 95%;
                max-height: 95vh;  /* 높이도 더 크게! */
                overflow: hidden;
                box-shadow: 0 30px 60px rgba(0,0,0,0.5);
                animation: slideUp 0.3s;
            }
            
            @keyframes slideUp {
                from { transform: translateY(30px); opacity: 0; }
                to { transform: translateY(0); opacity: 1; }
            }
            
            .wia-modal-header {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                padding: 35px;
                text-align: center;
                position: relative;
            }
            
            .wia-modal-header h2 {
                margin: 0;
                font-size: 2.5rem;  /* 더 큰 제목 */
                font-weight: 700;
            }
            
            .wia-modal-header p {
                margin: 10px 0 0 0;
                font-size: 1.2rem;
                opacity: 0.95;
            }
            
            .wia-modal-close {
                position: absolute;
                top: 20px;
                right: 20px;
                background: rgba(255,255,255,0.2);
                border: none;
                color: white;
                font-size: 2.5rem;
                width: 50px;
                height: 50px;
                border-radius: 50%;
                cursor: pointer;
                transition: all 0.3s;
            }
            
            .wia-modal-close:hover {
                background: rgba(255,255,255,0.3);
                transform: rotate(90deg);
            }
            
            .wia-modal-tabs {
                display: flex;
                gap: 10px;
                padding: 25px 35px;
                background: #f8f9fa;
                border-bottom: 2px solid #e5e7eb;
                flex-wrap: wrap;
            }
            
            .wia-modal-tab {
                padding: 12px 24px;
                background: white;
                border: 2px solid #e5e7eb;
                border-radius: 12px;
                cursor: pointer;
                font-weight: 600;
                font-size: 1rem;  /* 탭도 크게 */
                transition: all 0.3s;
            }
            
            .wia-modal-tab:hover {
                background: #f3f4f6;
                transform: translateY(-2px);
            }
            
            .wia-modal-tab.active {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                border-color: #667eea;
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
            }
            
            .wia-modal-body {
                padding: 35px;
                overflow-y: auto;
                max-height: calc(95vh - 280px);
            }
            
            .wia-lang-grid {
                display: grid;
                grid-template-columns: repeat(5, 1fr);  /* 5열 고정! */
                gap: 15px;  /* 간격 넓게 */
            }
            
            /* 작은 화면에서는 3열로 */
            @media (max-width: 768px) {
                .wia-lang-grid {
                    grid-template-columns: repeat(3, 1fr);
                }
            }
            
            /* 아주 작은 화면에서는 2열로 */
            @media (max-width: 480px) {
                .wia-lang-grid {
                    grid-template-columns: repeat(2, 1fr);
                }
            }
            
            .wia-lang-item {
                padding: 18px 15px;  /* 더 큰 패딩 */
                background: #f3f4f6;
                border: 2px solid transparent;
                border-radius: 12px;
                text-align: center;
                cursor: pointer;
                transition: all 0.2s;
                font-size: 1.1rem;  /* 더 큰 폰트! */
                font-weight: 500;
                min-height: 60px;  /* 최소 높이 */
                display: flex;
                align-items: center;
                justify-content: center;
            }
            
            .wia-lang-item:hover {
                background: linear-gradient(135deg, rgba(102,126,234,0.15), rgba(118,75,162,0.15));
                border-color: #667eea;
                transform: translateY(-3px);
                box-shadow: 0 5px 15px rgba(102, 126, 234, 0.2);
            }
            
            .wia-lang-item.selected {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                border-color: #667eea;
                font-weight: 600;
                box-shadow: 0 5px 20px rgba(102, 126, 234, 0.4);
            }
            
            .wia-lang-search {
                width: 100%;
                padding: 18px;
                border: 2px solid #e5e7eb;
                border-radius: 14px;
                font-size: 1.1rem;  /* 검색창도 크게 */
                margin-bottom: 25px;
            }
            
            .wia-lang-search:focus {
                outline: none;
                border-color: #667eea;
                box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
            }
            
            .wia-lang-count {
                text-align: center;
                padding: 15px;
                background: linear-gradient(135deg, rgba(102,126,234,0.1), rgba(118,75,162,0.1));
                border-radius: 12px;
                margin-bottom: 25px;
                font-weight: 600;
                font-size: 1.1rem;
                color: #667eea;
            }
        </style>`;
        
        document.head.insertAdjacentHTML('beforeend', styles);
    }

    injectHTML() {
        if (document.getElementById(this.modalId)) {
            document.getElementById(this.modalId).remove();
        }
        
        const html = `
        <div id="${this.modalId}">
            <div class="wia-modal-content">
                <div class="wia-modal-header">
                    <h2>🌍 Choose Your Language</h2>
                    <p>Select from 211 Languages Worldwide</p>
                    <button class="wia-modal-close" id="wiaModalClose">×</button>
                </div>
                
                <div class="wia-modal-tabs">
                    <button class="wia-modal-tab active" data-filter="all">All (211)</button>
                    <button class="wia-modal-tab" data-filter="popular">⭐ Popular</button>
                    <button class="wia-modal-tab" data-filter="asian">🌏 Asian</button>
                    <button class="wia-modal-tab" data-filter="european">🌍 European</button>
                    <button class="wia-modal-tab" data-filter="african">🌍 African</button>
                    <button class="wia-modal-tab" data-filter="americas">🌎 Americas</button>
                    <button class="wia-modal-tab" data-filter="oceanic">🏝️ Oceanic</button>
                </div>
                
                <div class="wia-modal-body">
                    <input type="text" class="wia-lang-search" id="wiaLangSearch" placeholder="🔍 Search 211 languages...">
                    <div class="wia-lang-count" id="wiaLangCount">Loading languages...</div>
                    <div class="wia-lang-grid" id="wiaLangGrid">
                        <!-- Languages will be loaded here -->
                    </div>
                </div>
            </div>
        </div>`;
        
        document.body.insertAdjacentHTML('beforeend', html);
    }

    async loadLanguages() {
        try {
            const paths = [
                '/wialanguages/data/languages-211-complete.json',
                '/assets/data/languages-211.json',
                'https://wiabook.com/wialanguages/data/languages-211-complete.json'
            ];
            
            let loaded = false;
            for (const path of paths) {
                try {
                    const response = await fetch(path);
                    if (response.ok) {
                        this.languages = await response.json();
                        console.log(`✅ Loaded ${Object.keys(this.languages).length} languages from ${path}`);
                        loaded = true;
                        break;
                    }
                } catch (e) {
                    continue;
                }
            }
            
            if (!loaded) {
                throw new Error('Failed to load from all paths');
            }
            
        } catch (error) {
            console.warn('⚠️ Using fallback languages:', error);
            this.languages = this.getFallbackLanguages();
        }
        
        this.renderLanguages('all');
    }

    getFallbackLanguages() {
        const langs = {
            'en': { native: 'English' },
            'ko': { native: '한국어' },
            'zh-CN': { native: '中文(简体)' },
            'ja': { native: '日本語' },
            'es': { native: 'Español' },
            'fr': { native: 'Français' },
            'de': { native: 'Deutsch' },
            'ru': { native: 'Русский' },
            'ar': { native: 'العربية' },
            'pt': { native: 'Português' },
            'it': { native: 'Italiano' },
            'nl': { native: 'Nederlands' },
            'pl': { native: 'Polski' },
            'tr': { native: 'Türkçe' },
            'hi': { native: 'हिन्दी' },
            'th': { native: 'ไทย' },
            'vi': { native: 'Tiếng Việt' },
            'id': { native: 'Bahasa Indonesia' },
            'he': { native: 'עברית' },
            'sv': { native: 'Svenska' }
        };

        let counter = 1;
        while (Object.keys(langs).length < 211) {
            langs[`lang${counter}`] = { native: `Language ${counter}` };
            counter++;
        }
        
        return langs;
    }

    getContinentMap() {
        return {
            popular: ['en', 'zh-CN', 'es', 'hi', 'ar', 'bn', 'pt', 'ru', 'ja', 'pa', 'de', 'ko', 'fr', 'tr', 'vi', 'ur', 'it', 'th', 'gu', 'fa'],
            asian: ['ko', 'ja', 'zh-CN', 'zh-TW', 'zh-HK', 'th', 'vi', 'id', 'ms', 'tl', 'my', 'km', 'lo', 'hi', 'bn', 'ta', 'te', 'mr', 'gu', 'kn', 'ml', 'si', 'ne', 'ur', 'pa', 'ps', 'fa', 'ku'],
            european: ['en', 'en-GB', 'es', 'fr', 'de', 'it', 'pt', 'ru', 'pl', 'uk', 'nl', 'el', 'sv', 'no', 'da', 'fi', 'is', 'et', 'lv', 'lt', 'cs', 'sk', 'hu', 'ro', 'bg', 'hr', 'sr', 'sl', 'mk', 'sq', 'mt', 'ga', 'cy', 'gd', 'eu', 'ca', 'gl', 'tr'],
            african: ['ar', 'am', 'ha', 'yo', 'ig', 'sw', 'zu', 'xh', 'af', 'so', 'rw', 'sn', 'ny', 'mg', 'tn', 'st', 'ss', 'ti', 'wo', 'ff', 'lg', 'om', 'ak', 'tw'],
            americas: ['en-US', 'en-CA', 'es-MX', 'es-AR', 'es-CO', 'es-CL', 'es-PE', 'pt-BR', 'fr-CA', 'qu', 'gn', 'ay', 'nah', 'chr', 'iu', 'haw'],
            oceanic: ['en-AU', 'en-NZ', 'mi', 'fj', 'sm', 'to', 'ty', 'tvl', 'nau', 'mh', 'bi', 'tpi']
        };
    }

    filterLanguages(filter) {
        this.currentFilter = filter;
        
        document.querySelectorAll('.wia-modal-tab').forEach(tab => {
            if (tab.dataset.filter === filter) {
                tab.classList.add('active');
            } else {
                tab.classList.remove('active');
            }
        });
        
        this.renderLanguages(filter);
    }

    searchLanguages(query) {
        const searchTerm = query.toLowerCase();
        const filtered = {};
        
        Object.entries(this.languages).forEach(([code, lang]) => {
            const name = (lang.native || lang.name || code).toLowerCase();
            if (name.includes(searchTerm) || code.toLowerCase().includes(searchTerm)) {
                filtered[code] = lang;
            }
        });
        
        this.renderGrid(filtered);
    }

    renderLanguages(filter) {
        let filtered = {};
        
        if (filter === 'all') {
            filtered = this.languages;
        } else {
            const continentMap = this.getContinentMap();
            if (continentMap[filter]) {
                Object.entries(this.languages).forEach(([code, lang]) => {
                    if (continentMap[filter].includes(code) || 
                        continentMap[filter].some(c => code.startsWith(c + '-'))) {
                        filtered[code] = lang;
                    }
                });
            }
        }
        
        this.renderGrid(filtered);
    }

    renderGrid(languages) {
        const grid = document.getElementById('wiaLangGrid');
        const count = document.getElementById('wiaLangCount');
        
        if (!grid) return;
        
        const langArray = Object.entries(languages);
        count.textContent = `Showing ${langArray.length} languages`;
        
        grid.innerHTML = langArray.map(([code, lang]) => {
            const name = lang.native || lang.name || code;
            const isSelected = code === this.currentLang;
            return `
                <div class="wia-lang-item ${isSelected ? 'selected' : ''}" 
                     data-code="${code}">
                    ${name}
                </div>
            `;
        }).join('');
        
        grid.querySelectorAll('.wia-lang-item').forEach(item => {
            item.addEventListener('click', () => {
                this.selectLanguage(item.dataset.code);
            });
        });
    }

    selectLanguage(code) {
        this.currentLang = code;
        localStorage.setItem('wiabooks_language', code);
        
        const display = document.getElementById('currentLangDisplay');
        if (display) {
            display.textContent = code.split('-')[0].toUpperCase().substring(0, 2);
        }
        
        document.querySelectorAll('.wia-lang-item').forEach(item => {
            if (item.dataset.code === code) {
                item.classList.add('selected');
            } else {
                item.classList.remove('selected');
            }
        });
        
        setTimeout(() => this.close(), 300);
        
        if (this.onLanguageChange) {
            this.onLanguageChange(code);
        }
    }

    open() {
        const modal = document.getElementById(this.modalId);
        if (modal) {
            modal.classList.add('show');
            this.renderLanguages(this.currentFilter);
        }
    }

    close() {
        const modal = document.getElementById(this.modalId);
        if (modal) {
            modal.classList.remove('show');
        }
    }

    bindEvents() {
        const closeBtn = document.getElementById('wiaModalClose');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => this.close());
        }
        
        document.querySelectorAll('.wia-modal-tab').forEach(tab => {
            tab.addEventListener('click', () => {
                this.filterLanguages(tab.dataset.filter);
            });
        });
        
        const searchInput = document.getElementById('wiaLangSearch');
        if (searchInput) {
            searchInput.addEventListener('keyup', (e) => {
                this.searchLanguages(e.target.value);
            });
        }
        
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                this.close();
            }
        });
    }

    registerGlobalFunctions() {
        window.openLanguageModal = () => this.open();
        window.closeLanguageModal = () => this.close();
        window.changeLanguage = (code) => this.selectLanguage(code);
        
        console.log('✅ Global functions registered: openLanguageModal(), closeLanguageModal(), changeLanguage()');
    }

    attachToButtons(selector = '.language-btn') {
        document.querySelectorAll(selector).forEach(button => {
            button.addEventListener('click', (e) => {
                e.preventDefault();
                this.open();
            });
        });
        
        console.log(`✅ Attached to ${document.querySelectorAll(selector).length} language buttons`);
    }
}

// 전역 인스턴스 생성
window.WIALanguageModal = new WIALanguageModal();

// 자동 초기화
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        window.WIALanguageModal.init();
        window.WIALanguageModal.attachToButtons();
    });
} else {
    window.WIALanguageModal.init();
    window.WIALanguageModal.attachToButtons();
}

console.log('📦 WIA Language Modal 211 V2 Component Loaded (5 columns, bigger font)');
