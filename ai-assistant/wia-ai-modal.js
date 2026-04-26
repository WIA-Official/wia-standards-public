/**
 * WIA Standards AI Assistant
 * Full-featured: Search, 3 AI engines, Drag, Resize, Auto-collapse
 * Version: 2.0.0
 */

class WIAStandardsAI {
    constructor() {
        this.currentAI = 'gemini';
        this.apiKeys = {
            gemini: localStorage.getItem('wia_gemini_key') || '',
            claude: localStorage.getItem('wia_claude_key') || '',
            chatgpt: localStorage.getItem('wia_chatgpt_key') || ''
        };
        this.conversations = { gemini: [], claude: [], chatgpt: [] };
        this.standards = [];
        this.freeMessageCount = parseInt(localStorage.getItem('wia_free_count') || '0');
        this.maxFreeMessages = 3;
        this.currentStandard = this.detectCurrentStandard();
        this.init();
    }

    detectCurrentStandard() {
        const path = window.location.pathname;
        const match = path.match(/^\/([a-z0-9-]+)\/?/);
        if (match && match[1] !== 'ai-assistant') {
            return match[1];
        }
        return null;
    }

    async init() {
        await this.loadStandards();
        this.createModal();
        this.createFAB();
        this.bindEvents();
        this.initDrag();
        this.initResize();
        console.log('✅ WIA AI Assistant initialized');
        if (this.currentStandard) {
            console.log(`📍 Current standard: ${this.currentStandard}`);
        }
    }

    async loadStandards() {
        try {
            const response = await fetch('/ai-assistant/standards.json?v=' + Date.now());
            const data = await response.json();
            this.standards = data.standards || [];
            console.log(`✅ ${this.standards.length} standards loaded`);
        } catch (error) {
            console.error('Failed to load standards:', error);
        }
    }

    createFAB() {
        const fab = document.createElement('button');
        fab.className = 'wia-ai-fab';
        fab.innerHTML = '🤖';
        fab.onclick = () => this.open();
        document.body.appendChild(fab);
    }

    createModal() {
        const standardInfo = this.currentStandard ? 
            this.standards.find(s => s.id === this.currentStandard) : null;
        const title = standardInfo ? standardInfo.title.split(' - ')[0] + ' AI' : 'WIA Standards AI';

        const modal = document.createElement('div');
        modal.id = 'wiaAiModal';
        modal.className = 'wia-ai-overlay';
        modal.innerHTML = `
            <div class="wia-ai-modal">
                <div class="wia-resize-handle left"></div>
                <div class="wia-resize-handle right"></div>
                <div class="wia-ai-header">
                    <h3><span class="logo-icon">🌐</span>${title}</h3>
                    <button class="wia-ai-close" onclick="wiaAI.close()">×</button>
                </div>
                
                <div class="wia-ai-tabs">
                    <button class="wia-ai-tab active" data-ai="gemini" onclick="wiaAI.switchAI('gemini')">
                        💎 Gemini <span class="badge">Free</span>
                    </button>
                    <button class="wia-ai-tab" data-ai="claude" onclick="wiaAI.switchAI('claude')">
                        🧠 Claude
                    </button>
                    <button class="wia-ai-tab" data-ai="chatgpt" onclick="wiaAI.switchAI('chatgpt')">
                        🤖 ChatGPT
                    </button>
                </div>
                
                <div class="wia-ai-api-section">
                    <input type="password" id="wiaApiInput" placeholder="Enter API key...">
                    <button class="wia-save-btn" onclick="wiaAI.saveApiKey()">Save</button>
                    <a href="https://aistudio.google.com/apikey" target="_blank" class="wia-get-key">🔑 Get Free Key →</a>
                </div>
                
                <div class="wia-collapsible" id="wiaToolsSection">
                    <div class="wia-section-header" onclick="wiaAI.toggleSection('wiaToolsSection')">
                        <span>✨ AI Tools</span>
                        <span class="wia-toggle-icon">▼</span>
                    </div>
                    <div class="wia-section-content">
                        <div class="wia-tools-grid">
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('What is WIA?')">🏢 About WIA</button>
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('Search standards')">🔍 Search</button>
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('Why standardization?')">📋 Why Standards</button>
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('Certification info')">🏆 Certification</button>
                        </div>
                    </div>
                </div>
                
                <div class="wia-collapsible" id="wiaCategorySection">
                    <div class="wia-section-header" onclick="wiaAI.toggleSection('wiaCategorySection')">
                        <span>📂 Categories</span>
                        <span class="wia-toggle-icon">▼</span>
                    </div>
                    <div class="wia-section-content">
                        <div class="wia-tools-grid">
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('accessibility standards')">♿ Accessibility</button>
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('robot standards')">🤖 Robotics</button>
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('medical standards')">🏥 Medical</button>
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('space standards')">🚀 Space</button>
                            <button class="wia-tool-btn" onclick="wiaAI.quickAsk('quantum standards')">⚛️ Quantum</button>
                        </div>
                    </div>
                </div>
                
                <div class="wia-ai-body">
                    <div class="wia-chat-area" id="wiaChatArea">
                        <div class="wia-welcome">
                            <div class="wia-welcome-icon">🤖</div>
                            <h4>Ask me anything!</h4>
                            <p>Standard search, WIA introduction, AI conversation supported.</p>
                        </div>
                    </div>
                </div>
                
                <div class="wia-tools-row">
                    <button class="wia-tool-btn" onclick="wiaAI.useTool('summarize')" title="Summarize">📋 <span data-i18n="ai_summarize">요약</span></button>
                    <button class="wia-tool-btn" onclick="wiaAI.useTool('translate')" title="Translate">🌐 <span data-i18n="ai_translate">번역</span></button>
                    <button class="wia-tool-btn" onclick="wiaAI.useTool('question')" title="Question">❓ <span data-i18n="ai_question">질문</span></button>
                    <button class="wia-tool-btn" onclick="wiaAI.useTool('keywords')" title="Keywords">🏷️ <span data-i18n="ai_keywords">키워드</span></button>
                    <button class="wia-tool-btn" onclick="wiaAI.useTool('explain')" title="Explain">💡 <span data-i18n="ai_explain">설명</span></button>
                </div>
                <div class="wia-input-area">
                    <input type="text" id="wiaInput" placeholder="Search standards or ask a question..." 
                           onkeypress="if(event.key==='Enter')wiaAI.send()">
                    <button class="wia-send-btn" onclick="wiaAI.send()">➤</button>
                </div>
            </div>
        `;
        document.body.appendChild(modal);
    }

    bindEvents() {
        document.getElementById('wiaAiModal').addEventListener('click', (e) => {
            if (e.target.id === 'wiaAiModal') this.close();
        });
        this.updateApiKeyInput();
    }

    initDrag() {
        const modal = document.querySelector('.wia-ai-modal');
        const header = document.querySelector('.wia-ai-header');
        if (!modal || !header) return;

        let isDragging = false;
        let startX, startY, initialX, initialY;
        const isMobile = () => window.innerWidth <= 768;

        header.style.cursor = 'grab';

        header.addEventListener('mousedown', (e) => {
            if (isMobile() || e.target.closest('.wia-ai-close')) return;
            isDragging = true;
            header.style.cursor = 'grabbing';
            const rect = modal.getBoundingClientRect();
            startX = e.clientX;
            startY = e.clientY;
            initialX = rect.left;
            initialY = rect.top;
            modal.style.position = 'fixed';
            modal.style.left = initialX + 'px';
            modal.style.top = initialY + 'px';
            modal.style.transform = 'none';
            e.preventDefault();
        });

        document.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            const dx = e.clientX - startX;
            const dy = e.clientY - startY;
            let newX = Math.max(0, Math.min(initialX + dx, window.innerWidth - modal.offsetWidth));
            let newY = Math.max(0, Math.min(initialY + dy, window.innerHeight - modal.offsetHeight));
            modal.style.left = newX + 'px';
            modal.style.top = newY + 'px';
        });

        document.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                header.style.cursor = 'grab';
            }
        });
    }

    initResize() {
        const modal = document.querySelector('.wia-ai-modal');
        const leftHandle = modal?.querySelector('.wia-resize-handle.left');
        const rightHandle = modal?.querySelector('.wia-resize-handle.right');
        if (!modal || !leftHandle || !rightHandle) return;

        let isResizing = false;
        let startX, startWidth, startLeft, direction;
        const isMobile = () => window.innerWidth <= 768;

        const startResize = (e, dir) => {
            if (isMobile()) return;
            isResizing = true;
            direction = dir;
            startX = e.clientX;
            startWidth = modal.offsetWidth;
            const rect = modal.getBoundingClientRect();
            startLeft = rect.left;
            modal.style.position = 'fixed';
            modal.style.left = rect.left + 'px';
            modal.style.top = rect.top + 'px';
            modal.style.transform = 'none';
            document.body.style.cursor = 'ew-resize';
            e.preventDefault();
        };

        leftHandle.addEventListener('mousedown', (e) => startResize(e, 'left'));
        rightHandle.addEventListener('mousedown', (e) => startResize(e, 'right'));

        document.addEventListener('mousemove', (e) => {
            if (!isResizing) return;
            const dx = e.clientX - startX;
            let newWidth = direction === 'right' ? startWidth + dx : startWidth - dx;
            let newLeft = direction === 'left' ? startLeft + dx : startLeft;
            if (newWidth >= 320 && newWidth <= window.innerWidth * 0.95) {
                modal.style.width = newWidth + 'px';
                if (direction === 'left') modal.style.left = newLeft + 'px';
            }
        });

        document.addEventListener('mouseup', () => {
            if (isResizing) {
                isResizing = false;
                document.body.style.cursor = '';
            }
        });
    }

    toggleSection(sectionId) {
        const section = document.getElementById(sectionId);
        if (section) {
            section.classList.toggle('collapsed');
            const icon = section.querySelector('.wia-toggle-icon');
            if (icon) {
                icon.textContent = section.classList.contains('collapsed') ? '▶' : '▼';
            }
        }
    }

    collapseAllSections() {
        document.querySelectorAll('.wia-collapsible').forEach(section => {
            section.classList.add('collapsed');
            const icon = section.querySelector('.wia-toggle-icon');
            if (icon) icon.textContent = '▶';
        });
    }

    expandAllSections() {
        document.querySelectorAll('.wia-collapsible').forEach(section => {
            section.classList.remove('collapsed');
            const icon = section.querySelector('.wia-toggle-icon');
            if (icon) icon.textContent = '▼';
        });
    }

    open() {
        document.getElementById('wiaAiModal').classList.add('show');
        document.getElementById('wiaInput').focus();
        
        const modal = document.querySelector('.wia-ai-modal');
        if (modal && window.innerWidth > 768) {
            modal.style.position = '';
            modal.style.left = '';
            modal.style.top = '';
            modal.style.transform = '';
            modal.style.width = '';
        }
        
        // Expand sections, then auto-collapse after 2.5s
        this.expandAllSections();
        setTimeout(() => this.collapseAllSections(), 2500);
    }

    close() {
        document.getElementById('wiaAiModal').classList.remove('show');
    }

    switchAI(aiType) {
        document.querySelectorAll('.wia-ai-tab').forEach(tab => tab.classList.remove('active'));
        document.querySelector(`[data-ai="${aiType}"]`).classList.add('active');
        this.currentAI = aiType;
        this.updateApiKeyInput();
        this.loadConversation();
    }

    updateApiKeyInput() {
        const input = document.getElementById('wiaApiInput');
        const link = document.querySelector('.wia-get-key');
        const urls = {
            gemini: 'https://aistudio.google.com/apikey',
            claude: 'https://console.anthropic.com/',
            chatgpt: 'https://platform.openai.com/api-keys'
        };
        const labels = {
            gemini: 'Gemini API Key',
            claude: 'Claude API Key',
            chatgpt: 'ChatGPT API Key'
        };
        input.placeholder = `Enter ${labels[this.currentAI]}...`;
        input.value = this.apiKeys[this.currentAI];
        link.href = urls[this.currentAI];
        link.textContent = this.currentAI === 'gemini' ? '🔑 Get Free Key →' : '🔑 Get API Key →';
    }

    saveApiKey() {
        const input = document.getElementById('wiaApiInput');
        const key = input.value.trim();
        this.apiKeys[this.currentAI] = key;
        localStorage.setItem(`wia_${this.currentAI}_key`, key);
        alert(`${this.currentAI.charAt(0).toUpperCase() + this.currentAI.slice(1)} API key saved!`);
    }

    loadConversation() {
        const chatArea = document.getElementById('wiaChatArea');
        chatArea.innerHTML = '';
        
        if (this.conversations[this.currentAI].length === 0) {
            chatArea.innerHTML = `
                <div class="wia-welcome">
                    <div class="wia-welcome-icon">🤖</div>
                    <h4>Ask me anything!</h4>
                    <p>Standard search, WIA introduction, AI conversation supported.</p>
                </div>
            `;
        } else {
            this.conversations[this.currentAI].forEach(msg => {
                this.addMessageToUI(msg.text, msg.type);
            });
        }
    }

    quickAsk(question) {
        document.getElementById('wiaInput').value = question;
        this.send();
    }

    useTool(tool) {
        const prompts = {
            summarize: "Summarize the current page content in 3-5 bullet points:",
            translate: "Translate the following to the users preferred language:",
            question: "Generate 5 key questions about this page content:",
            keywords: "Extract the top 10 keywords and key concepts from this page:",
            explain: "Explain the main concept of this page in simple terms for a beginner:"
        };
        const prefix = prompts[tool] || "";
        const pageText = document.body.innerText.substring(0, 3000);
        const input = document.getElementById("wiaInput");
        if (input) {
            input.value = prefix + "\n\n" + pageText.substring(0, 500);
            this.send();
        }
    }

    async send() {
        const input = document.getElementById('wiaInput');
        const message = input.value.trim();
        if (!message) return;

        this.addMessage(message, 'user');
        input.value = '';

        // Check for API key
        if (this.apiKeys[this.currentAI]) {
            await this.callAI(message);
        } else {
            // Basic response without API
            const response = this.getBasicResponse(message);
            this.addMessage(response, 'assistant');
        }
    }

    addMessage(text, type) {
        this.conversations[this.currentAI].push({ text, type });
        this.addMessageToUI(text, type);
    }

    addMessageToUI(text, type) {
        const chatArea = document.getElementById('wiaChatArea');
        const welcome = chatArea.querySelector('.wia-welcome');
        if (welcome) welcome.remove();

        const msg = document.createElement('div');
        msg.className = `wia-message ${type}`;
        msg.innerHTML = `<div class="wia-message-content">${this.formatMessage(text)}</div>`;
        chatArea.appendChild(msg);
        chatArea.scrollTop = chatArea.scrollHeight;
    }

    formatMessage(text) {
        // HTML 이스케이프 먼저
        text = text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
        // 마크다운 처리
        text = text.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank">$1</a>');
        text = text.replace(/\n/g, '<br>');
        text = text.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
        return text;
    }

    async callAI(message) {
        const aiConfig = {
            gemini: {
                url: 'https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent',
                getBody: (msg) => ({ contents: [{ parts: [{ text: this.getSystemPrompt() + '\n\nUser: ' + msg }] }] }),
                getResponse: (data) => data.candidates?.[0]?.content?.parts?.[0]?.text
            },
            claude: {
                url: 'https://api.anthropic.com/v1/messages',
                getBody: (msg) => ({
                    model: 'claude-sonnet-4-20250514',
                    max_tokens: 1024,
                    system: this.getSystemPrompt(),
                    messages: [{ role: 'user', content: msg }]
                }),
                getResponse: (data) => data.content?.[0]?.text
            },
            chatgpt: {
                url: 'https://api.openai.com/v1/chat/completions',
                getBody: (msg) => ({
                    model: 'gpt-4o-mini',
                    messages: [
                        { role: 'system', content: this.getSystemPrompt() },
                        { role: 'user', content: msg }
                    ]
                }),
                getResponse: (data) => data.choices?.[0]?.message?.content
            }
        };

        const config = aiConfig[this.currentAI];
        const headers = { 'Content-Type': 'application/json' };
        
        if (this.currentAI === 'gemini') {
            config.url += `?key=${this.apiKeys.gemini}`;
        } else if (this.currentAI === 'claude') {
            headers['x-api-key'] = this.apiKeys.claude;
            headers['anthropic-version'] = '2023-06-01';
            headers['anthropic-dangerous-direct-browser-access'] = 'true';
        } else {
            headers['Authorization'] = `Bearer ${this.apiKeys.chatgpt}`;
        }

        try {
            this.addMessageToUI('Thinking...', 'assistant');
            
            const response = await fetch(config.url, {
                method: 'POST',
                headers,
                body: JSON.stringify(config.getBody(message))
            });

            const data = await response.json();
            const reply = config.getResponse(data) || 'Sorry, I could not process your request.';
            
            // Remove "Thinking..."
            const chatArea = document.getElementById('wiaChatArea');
            const lastMsg = chatArea.lastElementChild;
            if (lastMsg) lastMsg.remove();
            this.conversations[this.currentAI].pop();
            
            this.addMessage(reply, 'assistant');
        } catch (error) {
            console.error('AI API Error:', error);
            const chatArea = document.getElementById('wiaChatArea');
            const lastMsg = chatArea.lastElementChild;
            if (lastMsg) lastMsg.remove();
            this.conversations[this.currentAI].pop();
            this.addMessage('API connection error. Please check your API key.', 'assistant');
        }
    }

    getSystemPrompt() {
        const standardsList = this.standards.map(s => `- ${s.id}: ${s.title}`).join('\n');
        return `You are WIA Standards AI Assistant. WIA (World Industry Association) follows the philosophy of "弘益人間" (Hongik Ingan - Benefit all humanity).

WIA has developed 177+ standards across 25 categories with 109+ simulators over 18 years.

Available standards:
${standardsList}

When users ask about standards:
1. Provide clear explanation of what the standard covers
2. Explain why standardization is important for that field
3. Include link to the standard page: /[standard-id]/

Be friendly, professional, and helpful. Respond in the same language the user uses.`;
    }

    getBasicResponse(message) {
        const msg = message.toLowerCase();

        // Current standard context
        if (this.currentStandard) {
            const std = this.standards.find(s => s.id === this.currentStandard);
            if (std && (msg.includes('this') || msg.includes('current') || msg.includes('here') || msg.includes('이') || msg.includes('여기'))) {
                return this.getStandardExplanation(std);
            }
        }

        // WIA Introduction
        if (msg.includes('wia') && (msg.includes('what') || msg.includes('about') || msg.includes('intro'))) {
            return this.getWIAIntro();
        }

        // Why standardization
        if (msg.includes('why') && msg.includes('standard')) {
            return this.getWhyStandards();
        }

        // Certification
        if (msg.includes('certif')) {
            return this.getCertificationInfo();
        }

        // Standard search
        const searchResults = this.searchStandards(message);
        if (searchResults.length > 0) {
            let response = `🔍 **Search Results for "${message}"** (${searchResults.length} found)\n\n`;
            searchResults.forEach(std => {
                response += `📋 [${std.title}](${std.url})\n\n`;
            });
            response += `💡 Click to visit the standard page!`;
            return response;
        }

        // Category responses
        if (msg.includes('access')) {
            return this.getCategoryResponse('Accessibility', ['aac', 'bci', 'eye-gaze', 'haptic', 'voice', 'cognitive-aac', 'ci', 'exoskeleton', 'smart-wheelchair', 'bionic-eye']);
        }
        if (msg.includes('robot')) {
            return this.getCategoryResponse('Robotics', ['robot', 'carebot', 'exoskeleton', 'myoelectric']);
        }
        if (msg.includes('medical') || msg.includes('health')) {
            return this.getCategoryResponse('Medical/Health', ['medical', 'health', 'bio', 'bci', 'bionic-eye', 'ci']);
        }
        if (msg.includes('space')) {
            return this.getCategoryResponse('Space', ['space']);
        }
        if (msg.includes('quantum')) {
            return this.getCategoryResponse('Quantum', ['quantum', 'pq-crypto']);
        }

        // Default
        this.freeMessageCount++;
        localStorage.setItem('wia_free_count', this.freeMessageCount.toString());

        if (this.freeMessageCount >= this.maxFreeMessages) {
            return `Hello! I'm WIA Standards AI. 😊

For more detailed answers, please connect a free Gemini API key!

🔑 Click "Get Free Key" above to get your API key.

Currently **${this.standards.length} standards** are registered!`;
        }

        return `Hello! I'm WIA Standards AI. 😊

Try asking:
• "What is WIA?"
• "Search robot standards"
• "Why is standardization important?"
• "AAC standard"

Currently **${this.standards.length} standards** are available!`;
    }

    getWIAIntro() {
        return `**WIA (World Industry Association)**

🌐 **弘益人間 - Hongik Ingan**
"Benefit all humanity through technology"

📊 **Overview**
• 177+ Standards developed
• 109+ Simulators operating
• 25 Categories covered
• 18 years of dedication

🎯 **Mission**
We develop future technology standards to provide certification and consulting services, ensuring technology benefits reach all of humanity.

🏢 **Services**
• Standard Development
• Certification & Compliance
• Consulting & Training
• Simulator & Testing

👉 Learn more: [WIA Homepage](/)`;
    }

    getWhyStandards() {
        return `**Why Standardization Matters**

🌍 **Global Interoperability**
Standards ensure devices and systems work together seamlessly across borders.

🛡️ **Safety & Quality**
Standardized testing ensures products meet safety requirements.

♿ **Accessibility**
Standards guarantee technology is usable by everyone, including people with disabilities.

💰 **Cost Efficiency**
Standardization reduces development costs and time-to-market.

🔮 **Future-Proofing**
Early standardization in emerging tech (AI, Quantum, Space) shapes industry direction.

📈 **Market Trust**
Certified products gain consumer and business confidence.

WIA develops standards **before** industries mature, positioning for future certification and consulting services.`;
    }

    getCertificationInfo() {
        return `**WIA Certification Services**

🏆 **What We Offer**
• Compliance Assessment
• Certification Testing
• Consulting Services
• Training Programs

📋 **Process**
1. Application Submission
2. Document Review
3. Testing & Evaluation
4. Certification Issuance
5. Ongoing Compliance Support

💼 **Benefits**
• Global Recognition
• Market Differentiation
• Regulatory Compliance
• Consumer Trust

📧 Contact: certification@wia.family

👉 View all standards: [Standards List](/)`;
    }

    getStandardExplanation(std) {
        return `**${std.title}**

📋 **Overview**
This standard defines requirements, testing procedures, and compliance criteria for ${std.id.replace(/-/g, ' ')} technology.

🎯 **Why This Standard Matters**
• Ensures interoperability across vendors
• Establishes safety and quality benchmarks
• Enables accessibility compliance
• Facilitates global market entry

📊 **Key Components**
• Technical Specifications
• Testing Procedures
• Compliance Requirements
• Certification Process

🔗 **Learn More**
Visit the full standard documentation: [${std.title}](${std.url})

💼 **Certification Available**
Contact WIA for certification services.`;
    }

    searchStandards(query) {
        const keywords = query.toLowerCase().replace(/standard|search|find|show|about/g, '').trim().split(/\s+/);
        
        return this.standards.filter(std => {
            const title = std.title.toLowerCase();
            const id = std.id.toLowerCase();
            return keywords.some(kw => kw.length >= 2 && (title.includes(kw) || id.includes(kw)));
        }).slice(0, 10);
    }

    getCategoryResponse(category, ids) {
        const results = this.standards.filter(std => ids.includes(std.id));
        if (results.length === 0) return `No ${category} standards found.`;

        let response = `🏷️ **${category} Standards** (${results.length} found)\n\n`;
        results.forEach(std => {
            response += `📋 [${std.title}](${std.url})\n\n`;
        });
        return response;
    }
}

// Initialize
const wiaAI = new WIAStandardsAI();
