// 选项卡切换逻辑
const tabs = document.querySelectorAll('.tab');
tabs.forEach(tab => {
    tab.addEventListener('click', () => {
        // 移除所有active类
        tabs.forEach(t => t.classList.remove('active'));
        // 为当前点击的tab添加active类
        tab.classList.add('active');
    });
});

// 模拟数据更新（可扩展为WebSocket或API调用）
setInterval(() => {
    // 示例：更新某个任务的状态
    const taskCards = document.querySelectorAll('.task-card');
    if (taskCards.length > 0) {
        const randomIndex = Math.floor(Math.random() * taskCards.length);
        const statusSpan = taskCards[randomIndex].querySelector('.status');
        const statuses = ['running', 'pending', 'completed', 'failed'];
        const randomStatus = statuses[Math.floor(Math.random() * statuses.length)];
        statusSpan.className = `status ${randomStatus}`;
        statusSpan.textContent = randomStatus.charAt(0).toUpperCase() + randomStatus.slice(1);
    }
}, 5000); // 每5秒更新一次

// 侧边栏导航高亮
const sidebarLinks = document.querySelectorAll('.sidebar-nav a');
sidebarLinks.forEach(link => {
    link.addEventListener('click', (e) => {
        e.preventDefault();
        sidebarLinks.forEach(l => l.classList.remove('active'));
        link.classList.add('active');
    });
});