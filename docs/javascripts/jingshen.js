document.addEventListener("DOMContentLoaded", function() {
    // ==========================================
    // 1. 基础配置与图片库
    // ==========================================
    const randomImages = ['1.png', '2.png', '3.png', '4.png', '5.png', '6.png'];
    const defaultImage = '7.png'; 
    const basePath = '/'; 
    const getRandomImage = () => basePath + randomImages[Math.floor(Math.random() * randomImages.length)];
    const getDefaultImage = () => basePath + defaultImage;

    // 🌟【核心优化：静默预加载机制】
    // 网页一打开，就在后台偷偷把 1~7.png 全部下载进浏览器缓存
    // 这样用户在点击换装或者进入镜神模式时，图片是直接从本地内存秒出的，绝对不卡！
    const preloadCache = [];
    [...randomImages, defaultImage].forEach(imgName => {
        const img = new Image();
        img.src = basePath + imgName;
        preloadCache.push(img); 
    });

    let dynamicKnowledgeBase = [];
    fetch('/search/search_index.json')
        .then(response => response.json())
        .then(data => {
            dynamicKnowledgeBase = data.docs.filter(doc => 
                doc.text && doc.text.trim().length > 20 && doc.title && !doc.location.endsWith('/#')
            );
        })
        .catch(err => console.error("知识库加载失败:", err));

    // ==========================================
    // 2. 构建 DOM 结构
    // ==========================================
    const transitionOverlay = document.createElement('div');
    transitionOverlay.id = 'jingshen-transition';
    transitionOverlay.innerHTML = `<div class="jingshen-welcome-text">✨ 欢迎镜神 ✨</div>`;
    document.body.appendChild(transitionOverlay);

    const petContainer = document.createElement('div');
    petContainer.id = 'jingshen-pet-container';
    
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
        </div>
        <div id="jingshen-speech"></div>
    `;
    document.body.appendChild(petContainer);

    const pet = document.getElementById('jingshen-pet');
    const petMenu = document.getElementById('jingshen-menu');
    const petSpeech = document.getElementById('jingshen-speech');

    const modalContainer = document.createElement('div');
    modalContainer.id = 'jingshen-modal';
    document.body.appendChild(modalContainer);

    // ==========================================
    // 3. 全局图标同步
    // ==========================================
    function syncAllImages(src) {
        pet.style.backgroundImage = `url('${src}')`;
        const logo = document.querySelector('.md-header__button.md-logo img');
        if (logo) { logo.src = src; logo.style.borderRadius = '50%'; logo.style.objectFit = 'cover'; }
        const img = new Image();
        img.src = src;
        img.onload = () => {
            const canvas = document.createElement('canvas');
            canvas.width = 64; canvas.height = 64;
            const ctx = canvas.getContext('2d');
            ctx.beginPath(); ctx.arc(32, 32, 32, 0, Math.PI * 2); ctx.closePath(); ctx.clip();
            const size = Math.min(img.width, img.height);
            ctx.drawImage(img, (img.width - size)/2, (img.height - size)/2, size, size, 0, 0, 64, 64);
            let favicon = document.querySelector('link[rel="icon"]');
            if (!favicon) { favicon = document.createElement('link'); favicon.rel = 'icon'; document.head.appendChild(favicon); }
            favicon.href = canvas.toDataURL('image/png');
        };
    }

    // ==========================================
    // 4. 交互与拖拽控制
    // ==========================================
    let isDragging = false, isDragged = false, offsetX, offsetY, menuTimeout, giantAlertTimeout;

    function showMenu() {
        if (!petContainer.classList.contains('active') || isDragging || petContainer.classList.contains('giant-alert')) return;
        petMenu.classList.add('show');
        clearTimeout(menuTimeout);
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
        
        petContainer.style.transition = 'none'; 
        petContainer.style.bottom = 'auto'; petContainer.style.right = 'auto';
        petMenu.classList.remove('show'); 
    }

    function doDrag(e) {
        if (!isDragging) return;
        isDragged = true; e.preventDefault(); 
        requestAnimationFrame(() => {
            const clientX = e.type.includes('mouse') ? e.clientX : e.touches[0].clientX;
            const clientY = e.type.includes('mouse') ? e.clientY : e.touches[0].clientY;
            petContainer.style.left = (clientX - offsetX) + 'px';
            petContainer.style.top = (clientY - offsetY) + 'px';
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
             syncAllImages(getRandomImage()); 
             petContainer.classList.toggle('shard-yellow'); 
         }
         pet.style.transform = 'scale(1.3)'; setTimeout(() => { pet.style.transform = ''; }, 150);
    });

    function triggerGiantAlert(message, durationSec = 2, callback = null) {
        clearTimeout(giantAlertTimeout);
        petContainer.classList.add('giant-alert'); petMenu.classList.remove('show');
        speak("🚨 " + message, durationSec * 1000);
        giantAlertTimeout = setTimeout(() => {
            petContainer.classList.remove('giant-alert');
            if (callback) callback();
        }, durationSec * 1000);
    }

    // ==========================================
    // 5. 模式切换调度与缓存拦截
    // ==========================================
    function applyJingshenMode() {
        const currentScheme = document.body.getAttribute('data-md-color-scheme');
        const wasAlreadyJingshen = sessionStorage.getItem('is_jingshen_active') === 'true';
        
        if (currentScheme === 'jingshen') {
            if (!wasAlreadyJingshen) {
                transitionOverlay.classList.add('active');
                setTimeout(() => {
                    syncAllImages(getRandomImage()); petContainer.classList.add('active'); 
                    setTimeout(() => { transitionOverlay.classList.remove('active'); checkTodoStatus(true); }, 1000); 
                }, 400); 
                sessionStorage.setItem('is_jingshen_active', 'true');
            } else {
                syncAllImages(getRandomImage());
                petContainer.classList.add('active'); 
            }
        } else {
            petContainer.classList.remove('active'); petMenu.classList.remove('show'); closeModal(); syncAllImages(getDefaultImage()); 
            sessionStorage.setItem('is_jingshen_active', 'false');
        }
    }
    applyJingshenMode();
    new MutationObserver(mutations => mutations.forEach(m => { 
        if (m.attributeName === "data-md-color-scheme") applyJingshenMode(); 
    })).observe(document.body, { attributes: true });

    // ==========================================
    // 6. UI 功能实现 (弹窗与工具)
    // ==========================================
    function speak(text, duration = 4000) { petSpeech.innerText = text; petSpeech.classList.add('show'); setTimeout(() => petSpeech.classList.remove('show'), duration); }
    function openModal(htmlContent) { modalContainer.innerHTML = htmlContent; modalContainer.classList.add('show'); modalContainer.onclick = (e) => { if(e.target === modalContainer) closeModal(); }; }
    function closeModal() { modalContainer.classList.remove('show'); }
    window.closeJingshenModal = closeModal; 

    document.getElementById('menu-timer').addEventListener('click', () => {
        openModal(`
            <div class="jingshen-card">
                <h3 style="margin-bottom: 15px;">⏳ 设定提醒</h3>
                <div style="display:flex; gap:10px; margin-bottom:20px;">
                    <button id="tab-countdown" onclick="switchTimerTab('countdown')" style="flex:1; padding:10px; background:var(--md-accent-fg-color); color:#fff; border-radius:5px; border:none; cursor:pointer; font-size:16px;">倒计时</button>
                    <button id="tab-fixedtime" onclick="switchTimerTab('fixedtime')" style="flex:1; padding:10px; background:#eee; color:#333; border-radius:5px; border:none; cursor:pointer; font-size:16px;">指定时间</button>
                </div>
                
                <div id="panel-countdown">
                    <div style="display:flex; justify-content:space-between; align-items:center; width:100%; box-sizing:border-box;">
                        <div style="flex:1; padding-right:10px;">
                            <input type="number" id="timer-min" placeholder="分钟" min="0" style="width:100%; padding:12px; border-radius:8px; border:1px solid #ccc; text-align:center; font-size:16px; box-sizing:border-box;">
                        </div>
                        <span style="font-weight:bold; font-size:24px; flex-shrink:0;">:</span>
                        <div style="flex:1; padding-left:10px;">
                            <input type="number" id="timer-sec" placeholder="秒钟" min="0" max="59" style="width:100%; padding:12px; border-radius:8px; border:1px solid #ccc; text-align:center; font-size:16px; box-sizing:border-box;">
                        </div>
                    </div>
                </div>

                <div id="panel-fixedtime" style="display:none; width:100%;">
                    <input type="time" id="timer-time" style="width:100%; padding:12px; border-radius:8px; border:1px solid #ccc; font-size:18px; box-sizing:border-box;">
                </div>
                
                <button onclick="startJingshenTimer()" style="width:100%; padding: 14px; background: var(--md-accent-fg-color); color: white; border: none; border-radius: 8px; cursor: pointer; margin-top: 25px; font-weight:bold; font-size:18px; box-sizing:border-box;">开启提醒</button>
            </div>
        `);
    });

    window.switchTimerTab = (type) => {
        const tabCd = document.getElementById('tab-countdown'); const pnlCd = document.getElementById('panel-countdown');
        const tabFt = document.getElementById('tab-fixedtime'); const pnlFt = document.getElementById('panel-fixedtime');
        if(type === 'countdown') { tabCd.style.background = 'var(--md-accent-fg-color)'; tabCd.style.color = '#fff'; tabFt.style.background = '#eee'; tabFt.style.color = '#333'; pnlCd.style.display = 'block'; pnlFt.style.display = 'none'; } 
        else { tabFt.style.background = 'var(--md-accent-fg-color)'; tabFt.style.color = '#fff'; tabCd.style.background = '#eee'; tabCd.style.color = '#333'; pnlFt.style.display = 'block'; pnlCd.style.display = 'none'; }
    };

    let reminderTimer = null, reminderAlertTimer = null;
    let globalTargetTimerTime = 0;      
    let hasRemindedAlmostUp = false;    

    window.startJingshenTimer = () => {
        let delayMs = 0, timeMsg = "";
        if (document.getElementById('panel-countdown').style.display !== 'none') {
            const mins = parseInt(document.getElementById('timer-min').value) || 0; const secs = parseInt(document.getElementById('timer-sec').value) || 0;
            if (mins === 0 && secs === 0) return alert("请输入有效的时间哦！");
            delayMs = (mins * 60 + secs) * 1000; timeMsg = `${mins > 0 ? mins+'分' : ''}${secs}秒`;
        } else {
            const timeStr = document.getElementById('timer-time').value; if (!timeStr) return alert("请选择时间！");
            const [h, m] = timeStr.split(':'); const now = new Date(); const target = new Date(); target.setHours(h, m, 0, 0);
            delayMs = target.getTime() - now.getTime();
            if (delayMs <= 0) return alert("这个时间已经过去啦，设个未来的时间吧！");
            timeMsg = `今日 ${timeStr}`;
        }
        closeModal(); speak(`好嘞！已设好 ${timeMsg} 的提醒~`);
        
        globalTargetTimerTime = Date.now() + delayMs;
        hasRemindedAlmostUp = false;

        clearTimeout(reminderTimer);
        reminderTimer = setTimeout(() => {
            globalTargetTimerTime = 0; 
            triggerGiantAlert("时间到啦！！！", 3, () => {
                pet.classList.add('ringing');
                openModal(`
                    <div class="jingshen-card" style="text-align: center;">
                        <h2>⏰ 提醒！</h2>
                        <p style="font-size: 18px; margin: 20px 0;">你设定的 ${timeMsg} 已经到啦！</p>
                        <button onclick="stopJingshenTimer()" style="padding: 12px 40px; background: #ff4757; color: white; border: none; border-radius: 8px; cursor: pointer; font-size:16px;">关闭提醒</button>
                    </div>
                `);
                reminderAlertTimer = setTimeout(stopJingshenTimer, 30000); 
            });
        }, delayMs);
    };
    window.stopJingshenTimer = () => { 
        pet.classList.remove('ringing'); clearTimeout(reminderAlertTimer); 
        globalTargetTimerTime = 0; closeModal(); speak("提醒已关闭，继续加油！"); 
    };

    // 待办事项
    function getTodos() {
        const today = new Date().toDateString(); const savedDate = localStorage.getItem('js_todo_date');
        if (savedDate !== today) { localStorage.setItem('js_todo_date', today); localStorage.setItem('js_todo_list', JSON.stringify([])); return []; }
        return JSON.parse(localStorage.getItem('js_todo_list') || '[]');
    }
    function saveTodos(todos) { localStorage.setItem('js_todo_list', JSON.stringify(todos)); }

    document.getElementById('menu-todo').addEventListener('click', renderTodoModal);

    function renderTodoModal() {
        const todos = getTodos();
        let html = `<div class="jingshen-card"><h3>📝 今日待办</h3><div style="max-height: 250px; overflow-y: auto; overflow-x: hidden; margin-bottom: 20px; padding-right:5px;">`;
        if (todos.length === 0) html += `<p style="color: #888; text-align:center; font-size:16px;">今天还没有任务，快来添加吧！</p>`;
        todos.forEach((t, i) => {
            html += `<label style="display: flex; align-items: center; padding: 12px 0; border-bottom: 1px solid #eee; cursor: pointer; text-decoration: ${t.done ? 'line-through' : 'none'}; color: ${t.done ? '#999' : '#333'}; font-size:16px;">
                    <input type="checkbox" onchange="toggleTodo(${i})" ${t.done ? 'checked' : ''} style="margin-right: 15px; width:20px; height:20px; flex-shrink:0;">
                    <span style="flex:1; word-break:break-all;">${t.text}</span>
                    <span onclick="deleteTodo(${i}); event.preventDefault();" style="color:#ff4757; padding: 0 5px; flex-shrink:0; font-size:22px;">×</span></label>`;
        });
        html += `</div><div style="display:flex; gap: 10px;">
                <input type="text" id="todo-input" placeholder="新任务..." style="flex:1; padding:12px; border:1px solid #ccc; border-radius:6px; min-width:0; font-size:16px;">
                <button onclick="addTodo()" style="padding: 12px 20px; background: var(--md-accent-fg-color); color: white; border: none; border-radius: 6px; cursor:pointer; flex-shrink:0; font-size:16px;">添加</button>
            </div></div>`;
        openModal(html);
    }

    window.addTodo = () => { const val = document.getElementById('todo-input').value.trim(); if(!val) return; const todos = getTodos(); todos.push({text: val, done: false}); saveTodos(todos); renderTodoModal(); checkTodoStatus(); };
    window.toggleTodo = (index) => { const todos = getTodos(); todos[index].done = !todos[index].done; saveTodos(todos); renderTodoModal(); checkTodoStatus(); };
    window.deleteTodo = (index) => { const todos = getTodos(); todos.splice(index, 1); saveTodos(todos); renderTodoModal(); };

    function checkTodoStatus(isInitial = false) {
        if(!petContainer.classList.contains('active')) return;
        const todos = getTodos();
        if (isInitial && todos.length === 0) triggerGiantAlert("今天还没有写待办哦~ 📝", 2);
        else if (!isInitial && todos.length > 0 && todos.every(t => t.done)) triggerGiantAlert("✨ 太棒了！今日任务全部达成！", 2);
    }
    
    document.getElementById('menu-card').addEventListener('click', () => {
        if (dynamicKnowledgeBase.length === 0) return speak("还在读取你的海量文档，稍微等我一下下哦！");
        const item = dynamicKnowledgeBase[Math.floor(Math.random() * dynamicKnowledgeBase.length)];
        let cleanText = item.text.replace(/¶/g, '').trim();
        let shortAnswer = cleanText.length > 250 ? cleanText.substring(0, 250) + "..." : cleanText;
        
        openModal(`
            <div class="jingshen-flashcard-scene" onclick="this.classList.toggle('is-flipped')">
                <div class="jingshen-flashcard">
                    <div class="card-face card-front">
                        <span class="card-tag">💡 智能知识回顾</span>
                        <h3 style="font-size:22px; line-height:1.5; word-break:break-word;">关于『 ${item.title} 』的知识点是什么？</h3>
                        <p style="color:#888; font-size:14px; position:absolute; bottom: 25px; left:0; width:100%;">[ 点击卡片查看答案 ]</p>
                    </div>
                    <div class="card-face card-back">
                        <span class="card-tag" style="background:var(--md-accent-fg-color); color:white;">✨ 答案</span>
                        <div class="card-scroll-area">
                            <p style="text-align: left; line-height: 1.7; font-size:16px; margin:0;">${shortAnswer}</p>
                        </div>
                        <a href="${basePath}${item.location}" onclick="event.stopPropagation(); closeJingshenModal();" 
                           class="card-btn-link">前往原文复习 ➔</a>
                    </div>
                </div>
            </div>
        `);
    });

    // ==========================================
    // 7. 全局智能隐式后台巡逻（守护机制）
    // ==========================================
    let lastTodoRemindTime = 0; 
    
    setInterval(() => {
        if (!petContainer.classList.contains('active')) return;

        const now = Date.now();

        // 1. 倒计时即将结束的预提醒 (剩1分钟内触发，仅触发一次)
        if (globalTargetTimerTime > 0 && !hasRemindedAlmostUp) {
            const timeLeft = globalTargetTimerTime - now;
            if (timeLeft > 0 && timeLeft <= 60000) { 
                speak("⏳ 专注时间快结束啦，准备收尾哦！", 5000);
                hasRemindedAlmostUp = true;
                return; 
            }
        }

        // 2. 未完成待办佛系提醒 (内置冷却5分钟，概率触发)
        if (now - lastTodoRemindTime > 5 * 60 * 1000) { 
            if (Math.random() < 0.25) { 
                const todos = getTodos();
                if (todos.length > 0) {
                    const unfinished = todos.filter(t => !t.done);
                    if (unfinished.length > 0) {
                        speak(`📝 你还有 ${unfinished.length} 个待办没写完呢，抓紧时间哦！`, 5000);
                        lastTodoRemindTime = now; 
                    }
                }
            }
        }
    }, 30000);
});
