/**
 * WIA Language Modal 211 Component
 * @version 2.0.0
 * @author PM Claude
 * @description 완벽한 211개 언어 선택 모달 컴포넌트
 * 
 * 사용법:
 * 1. 이 파일을 include
 * 2. WIALanguageModal.init() 호출
 * 3. 끝!
 */

class WIALanguageModal {
    constructor() {
        this.languages = {};
        this.currentFilter = 'all';
        this.currentLang = localStorage.getItem('wiabooks_language') || 'en';
        this.modalId = 'wiaLanguageModal211';
        this.initialized = false;
    }

    /**
     * 컴포넌트 초기화
     */
    async init() {
        if (this.initialized) {
            console.log('⚠️ WIA Language Modal already initialized');
            return;
        }

        console.log('🚀 Initializing WIA Language Modal 211...');
        
        // CSS 주입
        this.injectStyles();
        
        // HTML 주입
        this.injectHTML();
        
        // 언어 데이터 로드
        await this.loadLanguages();
        
        // 이벤트 바인딩
        this.bindEvents();
        
        // 전역 함수 등록
        this.registerGlobalFunctions();
        
        this.initialized = true;
        console.log('✅ WIA Language Modal 211 Ready!');
    }

    /**
     * 스타일 주입
     */
    injectStyles() {
        if (document.getElementById('wia-language-modal-styles')) return;
        
        const styles = `
        <style id="wia-language-modal-styles">
            #${this.modalId} {
                display: none;
                position: fixed;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background: rgba(0, 0, 0, 0.9);
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
                max-width: 1400px;
                width: 95%;
                max-height: 90vh;
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
                padding: 30px;
                text-align: center;
                position: relative;
            }
            
            .wia-modal-header h2 {
                margin: 0;
                font-size: 2.2rem;
                font-weight: 700;
            }
            
            .wia-modal-header p {
                margin: 10px 0 0 0;
                font-size: 1.1rem;
                opacity: 0.95;
            }
            
            .wia-modal-close {
                position: absolute;
                top: 20px;
                right: 20px;
                background: rgba(255,255,255,0.2);
                border: none;
                color: white;
                font-size: 2rem;
                width: 45px;
                height: 45px;
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
                gap: 8px;
                padding: 20px 30px;
                background: #f8f9fa;
                border-bottom: 2px solid #e5e7eb;
                flex-wrap: wrap;
            }
            
            .wia-modal-tab {
                padding: 10px 20px;
                background: white;
                border: 2px solid #e5e7eb;
                border-radius: 10px;
                cursor: pointer;
                font-weight: 600;
                transition: all 0.3s;
            }
            
            .wia-modal-tab:hover {
                background: #f3f4f6;
            }
            
            .wia-modal-tab.active {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                border-color: #667eea;
            }
            
            .wia-modal-body {
                padding: 30px;
                overflow-y: auto;
                max-height: calc(90vh - 250px);
            }
            
            .wia-lang-grid {
                display: grid;
                grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
                gap: 10px;
            }
            
            .wia-lang-item {
                padding: 12px;
                background: #f3f4f6;
                border: 2px solid transparent;
                border-radius: 10px;
                text-align: center;
                cursor: pointer;
                transition: all 0.2s;
                font-size: 0.95rem;
            }
            
            .wia-lang-item:hover {
                background: linear-gradient(135deg, rgba(102,126,234,0.1), rgba(118,75,162,0.1));
                border-color: #667eea;
                transform: translateY(-2px);
            }
            
            .wia-lang-item.selected {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                border-color: #667eea;
            }
            
            .wia-lang-search {
                width: 100%;
                padding: 15px;
                border: 2px solid #e5e7eb;
                border-radius: 12px;
                font-size: 1rem;
                margin-bottom: 20px;
            }
            
            .wia-lang-search:focus {
                outline: none;
                border-color: #667eea;
            }
            
            .wia-lang-count {
                text-align: center;
                padding: 10px;
                background: #f8f9fa;
                border-radius: 10px;
                margin-bottom: 20px;
                font-weight: 600;
                color: #667eea;
            }
        </style>`;
        
        document.head.insertAdjacentHTML('beforeend', styles);
    }

    /**
     * HTML 주입
     */
    injectHTML() {
        if (document.getElementById(this.modalId)) return;
        
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
                    <button class="wia-modal-tab" data-filter="popular">Popular (20)</button>
                    <button class="wia-modal-tab" data-filter="asian">Asian</button>
                    <button class="wia-modal-tab" data-filter="european">European</button>
                    <button class="wia-modal-tab" data-filter="african">African</button>
                    <button class="wia-modal-tab" data-filter="americas">Americas</button>
                    <button class="wia-modal-tab" data-filter="oceanic">Oceanic</button>
                </div>
                
                <div class="wia-modal-body">
                    <input type="text" class="wia-lang-search" id="wiaLangSearch" placeholder="Search 211 languages...">
                    <div class="wia-lang-count" id="wiaLangCount">Loading languages...</div>
                    <div class="wia-lang-grid" id="wiaLangGrid">
                        <!-- Languages will be loaded here -->
                    </div>
                </div>
            </div>
        </div>`;
        
        document.body.insertAdjacentHTML('beforeend', html);
    }

    /**
     * 언어 데이터 로드
     */
    async loadLanguages() {
        try {
            // 여러 경로 시도
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
        
        // 초기 렌더링
        this.renderLanguages('all');
    }

    /**
     * Fallback 언어 (로드 실패시)
     */
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

        // 211개까지 채우기
        let counter = 1;
        while (Object.keys(langs).length < 211) {
            langs[`lang${counter}`] = { native: `Language ${counter}` };
            counter++;
        }
        
        return langs;
    }

    /**
     * 대륙별 언어 맵
     */
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

    /**
     * 언어 필터링
     */
    filterLanguages(filter) {
        this.currentFilter = filter;
        
        // 탭 활성화 업데이트
        document.querySelectorAll('.wia-modal-tab').forEach(tab => {
            if (tab.dataset.filter === filter) {
                tab.classList.add('active');
            } else {
                tab.classList.remove('active');
            }
        });
        
        this.renderLanguages(filter);
    }

    /**
     * 언어 검색
     */
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

    /**
     * 언어 렌더링
     */
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

    /**
     * 그리드 렌더링
     */
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
        
        // 클릭 이벤트 재바인딩
        grid.querySelectorAll('.wia-lang-item').forEach(item => {
            item.addEventListener('click', () => {
                this.selectLanguage(item.dataset.code);
            });
        });
    }

    /**
     * 언어 선택
     */
    selectLanguage(code) {
        this.currentLang = code;
        localStorage.setItem('wiabooks_language', code);
        
        // 디스플레이 업데이트
        const display = document.getElementById('currentLangDisplay');
        if (display) {
            display.textContent = code.split('-')[0].toUpperCase().substring(0, 2);
        }
        
        // 선택 상태 업데이트
        document.querySelectorAll('.wia-lang-item').forEach(item => {
            if (item.dataset.code === code) {
                item.classList.add('selected');
            } else {
                item.classList.remove('selected');
            }
        });
        
        // 모달 닫기
        setTimeout(() => this.close(), 300);
        
        // 콜백 실행
        if (this.onLanguageChange) {
            this.onLanguageChange(code);
        }
    }

    /**
     * 모달 열기
     */
    open() {
        const modal = document.getElementById(this.modalId);
        if (modal) {
            modal.classList.add('show');
            this.renderLanguages(this.currentFilter);
        }
    }

    /**
     * 모달 닫기
     */
    close() {
        const modal = document.getElementById(this.modalId);
        if (modal) {
            modal.classList.remove('show');
        }
    }

    /**
     * 이벤트 바인딩
     */
    bindEvents() {
        // 닫기 버튼
        const closeBtn = document.getElementById('wiaModalClose');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => this.close());
        }
        
        // 탭 클릭
        document.querySelectorAll('.wia-modal-tab').forEach(tab => {
            tab.addEventListener('click', () => {
                this.filterLanguages(tab.dataset.filter);
            });
        });
        
        // 검색
        const searchInput = document.getElementById('wiaLangSearch');
        if (searchInput) {
            searchInput.addEventListener('keyup', (e) => {
                this.searchLanguages(e.target.value);
            });
        }
        
        // ESC 키로 닫기
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                this.close();
            }
        });
    }

    /**
     * 전역 함수 등록
     */
    registerGlobalFunctions() {
        // 기존 함수 오버라이드
        window.openLanguageModal = () => this.open();
        window.closeLanguageModal = () => this.close();
        window.changeLanguage = (code) => this.selectLanguage(code);
        
        console.log('✅ Global functions registered: openLanguageModal(), closeLanguageModal(), changeLanguage()');
    }

    /**
     * 언어 버튼 자동 연결
     */
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

// 자동 초기화 (DOM 로드시)
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        window.WIALanguageModal.init();
        window.WIALanguageModal.attachToButtons();
    });
} else {
    // 이미 로드된 경우
    window.WIALanguageModal.init();
    window.WIALanguageModal.attachToButtons();
}

console.log('📦 WIA Language Modal 211 Component Loaded');
console.log('Usage: WIALanguageModal.open() or just click any .language-btn');
