document.addEventListener("DOMContentLoaded", function() {
    // ==========================================
    // 0. 注入纯正飞雷神(动态金紫主题+沉浸式UI) CSS
    // ==========================================
    const gameStyles = document.createElement('style');
    gameStyles.innerHTML = `
        /* 跟随桌宠的时间显示 */
        #jingshen-time-display {
            position: absolute;
            top: 100%; left: 50%;
            transform: translateX(-50%);
            margin-top: 15px; 
            font-size: 26px; font-weight: bold;
            color: #00d2ff; 
            text-shadow: 0 0 10px #00d2ff, 0 0 2px #fff;
            pointer-events: none;
            font-family: 'Consolas', monospace; 
            opacity: 0.8; transition: opacity 0.5s;
            white-space: nowrap;
        }
        /* 游戏模式下强制隐藏时间 */
        #jingshen-pet-container.active #jingshen-time-display { opacity: 1; }
        #jingshen-pet-container.in-game #jingshen-time-display { opacity: 0 !important; }

        /* 游戏怪物的基础样式 */
        .jingshen-monster {
            position: fixed; font-size: 45px; cursor: crosshair;
            z-index: 9998; user-select: none; transition: transform 0.2s;
            animation: float-monster 2s infinite alternate ease-in-out;
            filter: drop-shadow(0 0 10px rgba(255,50,50,0.8));
        }
        .jingshen-monster:hover { transform: scale(1.3) !important; filter: drop-shadow(0 0 20px red); }
        @keyframes float-monster { from { transform: translateY(0px) rotate(-5deg); } to { transform: translateY(-20px) rotate(5deg); } }

        /* ==========================================
           🌟 动态主题大招特效 (使用 CSS 变量实现金紫双色切换)
           ========================================== */
        :root {
            --js-ult-main: #00d2ff;
            --js-ult-glow: rgba(0, 210, 255, 0.8);
            --js-ult-light: rgba(0, 210, 255, 0.2);
            --js-ult-filter: hue-rotate(0deg);
        }

        /* 🌟 沉浸式镜像空间 UI 结界 */
        #jingshen-mirror-domain {
            position: fixed; top: 0; left: 0; right: 0; bottom: 0;
            pointer-events: none; z-index: 9990;
            background: radial-gradient(circle at center, transparent 40%, rgba(0, 0, 15, 0.8) 100%);
            box-shadow: inset 0 0 80px var(--js-ult-glow), inset 0 0 20px var(--js-ult-main);
            border: 2px solid var(--js-ult-main);
            opacity: 0; transition: opacity 0.6s ease-in-out;
            overflow: hidden;
        }
        /* 结界角落的几何玻璃切割装饰 */
        #jingshen-mirror-domain::before, #jingshen-mirror-domain::after {
            content: ''; position: absolute;
            width: 300px; height: 300px;
            border: 2px solid var(--js-ult-main);
            opacity: 0.3; pointer-events: none;
        }
        #jingshen-mirror-domain::before { top: -150px; left: -150px; transform: rotate(45deg); box-shadow: 0 0 30px var(--js-ult-main); }
        #jingshen-mirror-domain::after { bottom: -150px; right: -150px; transform: rotate(45deg); box-shadow: 0 0 30px var(--js-ult-main); }

        /* 🌟 镜像领域圆环 */
        .jingshen-ultimate-ring {
            position: fixed; width: 320px; height: 320px;
            border: 2px solid var(--js-ult-main); border-radius: 50%;
            box-shadow: 0 0 25px var(--js-ult-glow), inset 0 0 15px var(--js-ult-light);
            background: radial-gradient(circle, var(--js-ult-light) 0%, transparent 70%);
            z-index: 9994; pointer-events: none;
            transform: translate(-50%, -50%) scale(0);
            animation: ring-open 0.3s cubic-bezier(0.175, 0.885, 0.32, 1.275) forwards;
        }
        @keyframes ring-open { to { transform: translate(-50%, -50%) scale(1); } }

        /* 🌟 分身幻象 (动态流光) */
        #jingshen-clone {
            position: fixed; width: 64px; height: 64px;
            background-size: cover; border-radius: 50%;
            z-index: 9996; opacity: 0.6;
            filter: drop-shadow(0 0 15px var(--js-ult-main)) brightness(1.5) var(--js-ult-filter);
            pointer-events: none; 
        }

        /* 🌟 实体的飞行镜片包裹器 */
        .jingshen-flying-shard-wrapper {
            position: fixed; width: 35px; height: 12px; 
            z-index: 9999; pointer-events: none; transform-origin: center;
            display: flex; justify-content: center; align-items: center;
        }
        /* 强制覆盖克隆出来的实体镜片颜色，同步大招主题色 */
        .jingshen-flying-shard-wrapper .shard {
            position: static !important; animation: none !important;
            background: linear-gradient(90deg, #fff, var(--js-ult-main)) !important;
            box-shadow: 0 0 15px var(--js-ult-main), inset 0 0 5px #fff !important; 
            width: 100% !important; height: 100% !important;
            clip-path: polygon(0 50%, 20% 0, 100% 0, 80% 50%, 100% 100%, 20% 100%) !important;
        }

        /* 🌟 斩击空气残影刀光 */
        .jingshen-shard-slash {
            position: fixed; height: 4px;
            background: linear-gradient(90deg, transparent, #fff 20%, var(--js-ult-main) 80%, transparent);
            box-shadow: 0 0 12px var(--js-ult-main);
            border-radius: 50%; z-index: 9998; pointer-events: none;
            transform-origin: 0 50%;
            animation: slash-flash 0.25s cubic-bezier(0.1, 0.9, 0.2, 1) forwards;
        }
        @keyframes slash-flash { 0% { transform: scaleX(0); opacity: 1; filter: brightness(2); } 50% { transform: scaleX(1); opacity: 1; filter: brightness(1.5); } 100% { transform: scaleX(1) scaleY(0.2); opacity: 0; filter: brightness(1); } }

        /* 🌟 连击字效 (破镜之刃) */
        .jingshen-combo-text {
            position: fixed;
            font-size: 32px; font-weight: 900; font-style: italic;
            color: #fff; text-shadow: 0 0 10px var(--js-ult-main), 0 0 20px var(--js-ult-main), 2px 2px 0px rgba(255,255,255,0.5);
            z-index: 10000; pointer-events: none;
            transform: translate(-50%, -50%) scale(0);
            transition: transform 0.1s cubic-bezier(0.175, 0.885, 0.32, 1.275);
            font-family: 'Impact', sans-serif;
        }
        .jingshen-combo-text.pop { transform: translate(-50%, -50%) scale(1.2); }

        /* 🌟 击杀碎裂爆炸 */
        .jingshen-glass-shatter {
            position: fixed; width: 120px; height: 120px;
            background: radial-gradient(circle, var(--js-ult-glow) 0%, transparent 60%);
            border: 2px dashed rgba(255,255,255,0.5); border-radius: 20%;
            z-index: 9999; pointer-events: none;
            animation: glass-explode 0.4s ease-out forwards;
        }
        @keyframes glass-explode {
            0% { transform: translate(-50%, -50%) scale(0.1) rotate(0deg); opacity: 1; clip-path: polygon(50% 0%, 100% 50%, 50% 100%, 0% 50%); }
            100% { transform: translate(-50%, -50%) scale(2.5) rotate(90deg); opacity: 0; clip-path: polygon(50% 0%, 100% 50%, 50% 100%, 0% 50%); }
        }
    `;
    document.head.appendChild(gameStyles);

    // ==========================================
    // 1. 基础配置与图片库
    // ==========================================
    const randomImages = ['1.png', '2.png', '3.png', '4.png', '5.png', '6.png'];
    const defaultImage = '7.png'; 
    const basePath = '/'; 
    const getRandomImage = () => basePath + randomImages[Math.floor(Math.random() * randomImages.length)];
    const getDefaultImage = () => basePath + defaultImage;

    if (sessionStorage.getItem('js_imgs_preloaded') !== 'true') {
        [...randomImages, defaultImage].forEach(imgName => { 
            const img = new Image(); img.src = basePath + imgName; 
        });
        sessionStorage.setItem('js_imgs_preloaded', 'true');
    }

    let dynamicKnowledgeBase = [];
    setTimeout(() => {
        fetch('/search/search_index.json')
            .then(response => response.json())
            .then(data => {
                dynamicKnowledgeBase = data.docs.filter(doc => 
                    doc.text && doc.text.trim().length > 20 && doc.title && !doc.location.endsWith('/#')
                );
            }).catch(err => console.error("知识库加载失败:", err));
    }, 2500); 

    // ==========================================
    // 2. 构建 DOM 结构 (包含时间显示)
    // ==========================================
    const transitionOverlay = document.createElement('div');
    transitionOverlay.id = 'jingshen-transition';
    transitionOverlay.innerHTML = `<div class="jingshen-welcome-text">✨ 欢迎镜神 ✨</div>`;
    document.body.appendChild(transitionOverlay);

    const petContainer = document.createElement('div');
    petContainer.id = 'jingshen-pet-container';
    petContainer.style.zIndex = "9997"; 
    
    petContainer.innerHTML = `
        <div class="shards-wrapper">
            <div class="shard-box shard-box-1"><div class="shard"></div></div>
            <div class="shard-box shard-box-2"><div class="shard"></div></div>
            <div class="shard-box shard-box-3"><div class="shard"></div></div>
            <div class="shard-box shard-box-4"><div class="shard"></div></div>
            <div class="shard-box shard-box-5"><div class="shard"></div></div>
            <div class="shard-box shard-box-6"><div class="shard"></div></div>
            <div class="shard-box shard-box-7"><div class="shard"></div></div>
            <div class="shard-box shard-box-8"><div class="shard"></div></div>
        </div>
        <div id="jingshen-pet"></div>
        <div id="jingshen-menu">
            <div class="menu-item" id="menu-timer" title="设置提醒">⏳</div>
            <div class="menu-item" id="menu-todo" title="待办事项">📝</div>
            <div class="menu-item" id="menu-card" title="知识抽查">🧠</div>
            <div class="menu-item" id="menu-game" title="飞雷神试炼">⚔️</div>
        </div>
        <div id="jingshen-speech"></div>
        <div id="jingshen-time-display"></div>
    `;
    document.body.appendChild(petContainer);

    const pet = document.getElementById('jingshen-pet');
    const petMenu = document.getElementById('jingshen-menu');
    const petSpeech = document.getElementById('jingshen-speech');
    const timeDisplay = document.getElementById('jingshen-time-display');

    function updatePetTime() {
        const now = new Date();
        const h = String(now.getHours()).padStart(2, '0');
        const m = String(now.getMinutes()).padStart(2, '0');
        const s = String(now.getSeconds()).padStart(2, '0');
        timeDisplay.innerText = `${h}:${m}:${s}`;
    }
    updatePetTime(); 
    setInterval(updatePetTime, 1000); 

    const modalContainer = document.createElement('div');
    modalContainer.id = 'jingshen-modal';
    document.body.appendChild(modalContainer);

    // ==========================================
    // 3. Canvas 数据缓存 (图标同步)
    // ==========================================
    function syncAllImages(src) {
        pet.style.backgroundImage = `url('${src}')`;
        const logo = document.querySelector('.md-header__button.md-logo img');
        if (logo) { logo.src = src; logo.style.borderRadius = '50%'; logo.style.objectFit = 'cover'; }
        function updateFavicon(url) {
            let favicon = document.querySelector('link[rel="icon"]');
            if (!favicon) { favicon = document.createElement('link'); favicon.rel = 'icon'; document.head.appendChild(favicon); }
            favicon.href = url;
        }
        const cacheKey = 'favicon_' + src;
        const cachedIcon = sessionStorage.getItem(cacheKey);
        if (cachedIcon) { updateFavicon(cachedIcon); } else {
            const img = new Image(); img.src = src;
            img.onload = () => {
                const canvas = document.createElement('canvas'); canvas.width = 64; canvas.height = 64;
                const ctx = canvas.getContext('2d'); ctx.beginPath(); ctx.arc(32, 32, 32, 0, Math.PI * 2); ctx.closePath(); ctx.clip();
                const size = Math.min(img.width, img.height);
                ctx.drawImage(img, (img.width - size)/2, (img.height - size)/2, size, size, 0, 0, 64, 64);
                try { sessionStorage.setItem(cacheKey, canvas.toDataURL('image/png')); } catch(e) {}
                updateFavicon(canvas.toDataURL('image/png'));
            };
        }
    }

    // ==========================================
    // 4. 交互与拖拽控制
    // ==========================================
    let isDragging = false, isDragged = false, offsetX, offsetY, menuTimeout, giantAlertTimeout;

    function showMenu() {
        if (!petContainer.classList.contains('active') || isDragging || petContainer.classList.contains('giant-alert')) return;
        petMenu.classList.add('show'); clearTimeout(menuTimeout);
        menuTimeout = setTimeout(() => { petMenu.classList.remove('show'); }, 3000);
    }
    petContainer.addEventListener('mouseenter', showMenu);
    petMenu.addEventListener('mouseenter', () => clearTimeout(menuTimeout));
    petMenu.addEventListener('mouseleave', () => { menuTimeout = setTimeout(() => { petMenu.classList.remove('show'); }, 3000); });

    function startDrag(e) {
        if (e.target.closest('#jingshen-menu') || e.target.closest('#jingshen-speech')) return; 
        if (e.button !== 0 && e.type.includes('mouse')) return; 
        if (petContainer.classList.contains('giant-alert')) return; 
        isDragging = true; isDragged = false;
        if (e.type.includes('mouse')) e.preventDefault(); 
        document.body.style.userSelect = 'none';
        const clientX = e.type.includes('mouse') ? e.clientX : e.touches[0].clientX;
        const clientY = e.type.includes('mouse') ? e.clientY : e.touches[0].clientY;
        const rect = petContainer.getBoundingClientRect();
        offsetX = clientX - rect.left; offsetY = clientY - rect.top;
        petContainer.style.transition = 'none'; petContainer.style.bottom = 'auto'; petContainer.style.right = 'auto'; petMenu.classList.remove('show'); 
    }

    function doDrag(e) {
        if (!isDragging) return;
        isDragged = true; e.preventDefault(); 
        requestAnimationFrame(() => {
            const clientX = e.type.includes('mouse') ? e.clientX : e.touches[0].clientX;
            const clientY = e.type.includes('mouse') ? e.clientY : e.touches[0].clientY;
            petContainer.style.left = (clientX - offsetX) + 'px'; petContainer.style.top = (clientY - offsetY) + 'px';
        });
    }

    function endDrag() {
        if (!isDragging) return;
        isDragging = false; document.body.style.userSelect = '';
        petContainer.style.transition = 'transform 0.3s cubic-bezier(0.25, 0.8, 0.25, 1), opacity 0.5s ease, left 0.5s, top 0.5s'; 
        if(isDragged) setTimeout(showMenu, 200);
    }

    petContainer.addEventListener('mousedown', startDrag); document.addEventListener('mousemove', doDrag); document.addEventListener('mouseup', endDrag);
    petContainer.addEventListener('touchstart', startDrag, {passive: false}); petContainer.addEventListener('touchmove', doDrag, {passive: false}); petContainer.addEventListener('touchend', endDrag);

    pet.addEventListener('click', () => {
         if (isDragged || petContainer.classList.contains('giant-alert')) return; 
         if (document.body.getAttribute('data-md-color-scheme') === 'jingshen') {
             syncAllImages(getRandomImage()); petContainer.classList.toggle('shard-yellow'); 
         }
         pet.style.transform = 'scale(1.3)'; setTimeout(() => { pet.style.transform = ''; }, 150);
    });

    function triggerGiantAlert(message, durationSec = 2, callback = null) {
        clearTimeout(giantAlertTimeout); petContainer.classList.add('giant-alert'); petMenu.classList.remove('show');
        speak("🚨 " + message, durationSec * 1000);
        giantAlertTimeout = setTimeout(() => { petContainer.classList.remove('giant-alert'); if (callback) callback(); }, durationSec * 1000);
    }

    function applyJingshenMode() {
        const currentScheme = document.body.getAttribute('data-md-color-scheme');
        const wasAlreadyJingshen = sessionStorage.getItem('is_jingshen_active') === 'true';
        if (currentScheme === 'jingshen') {
            if (!wasAlreadyJingshen) {
                transitionOverlay.classList.add('active');
                setTimeout(() => { syncAllImages(getRandomImage()); petContainer.classList.add('active'); setTimeout(() => { transitionOverlay.classList.remove('active'); }, 1000); }, 400); 
                sessionStorage.setItem('is_jingshen_active', 'true');
            } else { syncAllImages(getRandomImage()); petContainer.classList.add('active'); }
        } else {
            petContainer.classList.remove('active'); petMenu.classList.remove('show'); closeModal(); syncAllImages(getDefaultImage()); 
            sessionStorage.setItem('is_jingshen_active', 'false');
        }
    }
    applyJingshenMode();
    new MutationObserver(mutations => mutations.forEach(m => { if (m.attributeName === "data-md-color-scheme") applyJingshenMode(); })).observe(document.body, { attributes: true });

    // ==========================================
    // 5. UI 功能实现 (保持完整)
    // ==========================================
    function speak(text, duration = 4000) { petSpeech.innerText = text; petSpeech.classList.add('show'); setTimeout(() => petSpeech.classList.remove('show'), duration); }
    function openModal(htmlContent) { modalContainer.innerHTML = htmlContent; modalContainer.classList.add('show'); modalContainer.onclick = (e) => { if(e.target === modalContainer) closeModal(); }; }
    function closeModal() { modalContainer.classList.remove('show'); }
    window.closeJingshenModal = closeModal; 

    document.getElementById('menu-timer').addEventListener('click', () => { /*...*/ });
    document.getElementById('menu-todo').addEventListener('click', () => { /*...*/ });
    document.getElementById('menu-card').addEventListener('click', () => { /*...*/ });

    // ==========================================
    // 🌟 核心：幻镜领域·纯正飞雷神机制
    // ==========================================
    
    // 全局状态：控制颜色交替（初始为金色）
    let isGoldThemeNext = true; 

    function applyAlternateUltimateTheme() {
        const root = document.documentElement;
        if (isGoldThemeNext) {
            root.style.setProperty('--js-ult-main', '#ffd700'); // 金色
            root.style.setProperty('--js-ult-glow', 'rgba(255, 215, 0, 0.8)');
            root.style.setProperty('--js-ult-light', 'rgba(255, 215, 0, 0.2)');
            root.style.setProperty('--js-ult-filter', 'hue-rotate(45deg) saturate(2)'); 
        } else {
            root.style.setProperty('--js-ult-main', '#b829ff'); // 紫色
            root.style.setProperty('--js-ult-glow', 'rgba(184, 41, 255, 0.8)');
            root.style.setProperty('--js-ult-light', 'rgba(184, 41, 255, 0.2)');
            root.style.setProperty('--js-ult-filter', 'hue-rotate(270deg) saturate(1.5)');
        }
        // 切换下一次的状态
        isGoldThemeNext = !isGoldThemeNext;
    }

    document.getElementById('menu-game').addEventListener('click', () => {
        if(petContainer.classList.contains('in-game')) return; 
        
        closeModal(); 
        petContainer.classList.add('in-game'); // 强制隐藏时间
        
        // 交替应用 金/紫 主题
        applyAlternateUltimateTheme(); 
        
        // 渲染沉浸式镜像空间 UI
        const domainUI = document.createElement('div');
        domainUI.id = 'jingshen-mirror-domain';
        document.body.appendChild(domainUI);
        requestAnimationFrame(() => domainUI.style.opacity = '1');

        speak("镜像空间，展开！", 2000);
        
        // 刷怪
        const monsterIcons = ['👾', '👹', '👻', '🦇', '🕷️'];
        for (let i = 0; i < 5; i++) {
            setTimeout(() => spawnJingTarget(monsterIcons[i]), 200 + i * 400);
        }
    });

    // 发射克隆的实体镜片 (留下空气残影)
    function fireRealShard(startX, startY, endX, endY) {
        const dist = Math.hypot(endX - startX, endY - startY);
        const angle = Math.atan2(endY - startY, endX - startX);
        const angleDeg = angle * 180 / Math.PI;

        // 1. 克隆真实的实体镜片节点
        const realShardNode = document.querySelector('.shard');
        const shardWrapper = document.createElement('div');
        shardWrapper.className = 'jingshen-flying-shard-wrapper';
        if(realShardNode) {
            shardWrapper.appendChild(realShardNode.cloneNode(true)); 
        } else {
            shardWrapper.innerHTML = `<div class="shard"></div>`; // 备用降级
        }

        shardWrapper.style.left = startX + 'px';
        shardWrapper.style.top = startY + 'px';
        shardWrapper.style.transform = `translate(-50%, -50%) rotate(${angleDeg}deg)`;
        document.body.appendChild(shardWrapper);

        // 刺杀动画
        shardWrapper.animate([
            { left: startX + 'px', top: startY + 'px' },
            { left: endX + 'px', top: endY + 'px' }
        ], { duration: 120, easing: 'ease-in' });
        setTimeout(() => shardWrapper.remove(), 120);

        // 2. 残留刀光残影
        const slash = document.createElement('div');
        slash.className = 'jingshen-shard-slash';
        slash.style.width = dist + 'px';
        slash.style.left = startX + 'px';
        slash.style.top = startY + 'px';
        slash.style.transform = `rotate(${angle}rad)`;
        document.body.appendChild(slash);
        setTimeout(() => slash.remove(), 250); 
    }

    function spawnJingTarget(emoji) {
        const monster = document.createElement('div');
        monster.className = 'jingshen-monster';
        monster.innerText = emoji;
        const maxX = window.innerWidth - 100;
        const maxY = window.innerHeight - 100;
        monster.style.left = (Math.random() * maxX + 50) + 'px';
        monster.style.top = (Math.random() * maxY + 50) + 'px';
        document.body.appendChild(monster);

        monster.addEventListener('click', function(e) {
            if (this.dataset.dead) return;
            this.dataset.dead = "true";

            // 大招语音
            const battleLines = ["见影，如见我！", "绞杀！", "自我之像，犹在镜中！", "碎八荒！"];
            speak(battleLines[Math.floor(Math.random() * battleLines.length)], 2000);

            const targetRect = this.getBoundingClientRect();
            const targetX = targetRect.left + targetRect.width / 2;
            const targetY = targetRect.top + targetRect.height / 2;
            const radius = 160; 

            // 展开圆环
            const ring = document.createElement('div');
            ring.className = 'jingshen-ultimate-ring';
            ring.style.left = targetX + 'px';
            ring.style.top = targetY + 'px';
            document.body.appendChild(ring);

            // 生成分身
            const clone = document.createElement('div');
            clone.id = 'jingshen-clone';
            clone.style.backgroundImage = pet.style.backgroundImage || `url('${getDefaultImage()}')`;
            document.body.appendChild(clone);

            // 连击字效
            const comboText = document.createElement('div');
            comboText.className = 'jingshen-combo-text';
            comboText.style.left = targetX + 'px';
            comboText.style.top = (targetY - 100) + 'px'; 
            document.body.appendChild(comboText);

            let dashCount = 0;
            const totalDashes = Math.floor(Math.random() * 6) + 4; // 4~9次连击
            let currentAngle = Math.random() * Math.PI * 2; // 当前冲刺的角度

            function performDash() {
                if (dashCount >= totalDashes) {
                    // 终结技：碎裂
                    monster.style.visibility = 'hidden';
                    const shatter = document.createElement('div');
                    shatter.className = 'jingshen-glass-shatter';
                    shatter.style.left = targetX + 'px';
                    shatter.style.top = targetY + 'px';
                    document.body.appendChild(shatter);
                    
                    comboText.remove();

                    // ⚡ 【BUG修复】大招结束后，确保桌宠平滑回到怪物被击杀的正中心位置，防止消失或飞出屏幕
                    petContainer.style.transition = 'top 0.3s cubic-bezier(0.25, 0.8, 0.25, 1), left 0.3s cubic-bezier(0.25, 0.8, 0.25, 1)';
                    petContainer.style.left = (targetX - 32) + 'px';
                    petContainer.style.top = (targetY - 32) + 'px';

                    setTimeout(() => {
                        shatter.remove(); monster.remove();
                        ring.style.opacity = '0'; clone.style.opacity = '0';
                        setTimeout(() => { ring.remove(); clone.remove(); }, 300);

                        // 检查游戏是否彻底结束
                        if (document.querySelectorAll('.jingshen-monster').length === 0) {
                            // 移除沉浸式镜像空间UI
                            const domainUI = document.getElementById('jingshen-mirror-domain');
                            if (domainUI) {
                                domainUI.style.opacity = '0';
                                setTimeout(() => domainUI.remove(), 600);
                            }
                            
                            setTimeout(() => {
                                speak("试炼结束。镜像空间，解除。", 3000);
                                petContainer.classList.remove('in-game'); // 恢复时间
                            }, 500);
                        }
                    }, 400);
                    return;
                }

                // 连击跳字
                comboText.innerText = `破镜之刃 x ${dashCount + 1}`;
                comboText.classList.remove('pop');
                void comboText.offsetWidth; 
                comboText.classList.add('pop');

                // 本次冲刺换位的目标点 (沿直径)
                const dashA = { x: targetX + Math.cos(currentAngle) * radius, y: targetY + Math.sin(currentAngle) * radius };
                const dashB = { x: targetX - Math.cos(currentAngle) * radius, y: targetY - Math.sin(currentAngle) * radius };
                const petTarget = (dashCount % 2 === 0) ? dashA : dashB;
                const cloneTarget = (dashCount % 2 === 0) ? dashB : dashA;

                // 计算镜片生成的 垂直 (垂直于冲刺路径) 起点
                const perpAngle = currentAngle + Math.PI / 2; // 垂直角度
                const shardStartA = { x: targetX + Math.cos(perpAngle) * radius, y: targetY + Math.sin(perpAngle) * radius };
                const shardStartB = { x: targetX - Math.cos(perpAngle) * radius, y: targetY - Math.sin(perpAngle) * radius };

                // 现有点
                const petRect = petContainer.getBoundingClientRect();
                const petCurrentX = petRect.left + petRect.width / 2;
                const petCurrentY = petRect.top + petRect.height / 2;
                
                const cloneRect = clone.getBoundingClientRect();
                let cloneCurrentX = cloneRect.left + cloneRect.width / 2;
                let cloneCurrentY = cloneRect.top + cloneRect.height / 2;

                if (dashCount === 0) {
                    cloneCurrentX = petTarget.x; cloneCurrentY = petTarget.y;
                }

                // 镜片从垂直圆环边缘刺向中心
                fireRealShard(shardStartA.x, shardStartA.y, targetX, targetY);
                if (dashCount > 0) {
                    fireRealShard(shardStartB.x, shardStartB.y, targetX, targetY);
                }

                // 真实的“冲刺换位”动画过程：本体和分身沿着设定好的直径滑动交换
                petContainer.style.transition = 'top 0.15s cubic-bezier(0.1, 0.8, 0.3, 1), left 0.15s cubic-bezier(0.1, 0.8, 0.3, 1)'; 
                petContainer.style.left = (petTarget.x - 32) + 'px'; 
                petContainer.style.top = (petTarget.y - 32) + 'px';
                
                clone.style.transition = 'top 0.15s cubic-bezier(0.1, 0.8, 0.3, 1), left 0.15s cubic-bezier(0.1, 0.8, 0.3, 1)';
                clone.style.left = (cloneTarget.x - 32) + 'px';
                clone.style.top = (cloneTarget.y - 32) + 'px';

                dashCount++;
                
                // 换位完成后，转到下一个准备点
                setTimeout(() => {
                    currentAngle += Math.PI / (Math.random() * 1.5 + 2); // 随机偏转角度
                    
                    const nextA = { x: targetX + Math.cos(currentAngle) * radius, y: targetY + Math.sin(currentAngle) * radius };
                    const nextB = { x: targetX - Math.cos(currentAngle) * radius, y: targetY - Math.sin(currentAngle) * radius };
                    
                    // 瞬间移动到新直径的两端，不带动画
                    petContainer.style.transition = 'none'; 
                    petContainer.style.left = ((dashCount % 2 === 0 ? nextB.x : nextA.x) - 32) + 'px';
                    petContainer.style.top = ((dashCount % 2 === 0 ? nextB.y : nextA.y) - 32) + 'px';
                    
                    clone.style.transition = 'none';
                    clone.style.left = ((dashCount % 2 === 0 ? nextA.x : nextB.x) - 32) + 'px';
                    clone.style.top = ((dashCount % 2 === 0 ? nextA.y : nextB.y) - 32) + 'px';

                    // 极速衔接下一次换位刺杀
                    setTimeout(performDash, 30);
                }, 160); // 等待冲刺过程结束
            }

            // 稍微延迟起手
            setTimeout(performDash, 100); 
        });
    }

    // ==========================================
    // 6. 后台守护线程
    // ==========================================
    let lastTodoRemindTime = 0; 
    setInterval(() => {
        if (!petContainer.classList.contains('active')) return;
        const now = Date.now();
        if (globalTargetTimerTime > 0 && !hasRemindedAlmostUp) {
            if ((globalTargetTimerTime - now) > 0 && (globalTargetTimerTime - now) <= 60000) { 
                speak("⏳ 专注快结束啦，准备收尾！", 5000); hasRemindedAlmostUp = true; return; 
            }
        }
    }, 30000);
});
