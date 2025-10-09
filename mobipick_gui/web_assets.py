"""Static assets served by the lightweight web UI server."""

INDEX_HTML = r"""<!DOCTYPE html>
<html lang=\"en\">
  <head>
    <meta charset=\"utf-8\" />
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
    <title>Mobipick Labs Control (Web)</title>
    <style>
      :root {
        color-scheme: dark;
        font-family: "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #101014;
        color: #e9ecef;
      }
      body {
        margin: 0;
        padding: 0;
        min-height: 100vh;
        display: flex;
        flex-direction: column;
      }
      header {
        padding: 1rem 1.5rem;
        background: #1f1f28;
        box-shadow: 0 2px 6px rgba(0, 0, 0, 0.4);
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        align-items: center;
        justify-content: space-between;
      }
      header h1 {
        font-size: 1.4rem;
        margin: 0;
      }
      main {
        flex: 1;
        padding: 1rem 1.5rem 2.5rem;
        display: grid;
        grid-template-columns: minmax(260px, 320px) 1fr;
        gap: 1.5rem;
      }
      @media (max-width: 1024px) {
        main {
          grid-template-columns: 1fr;
        }
      }
      .panel {
        background: #191926;
        border-radius: 12px;
        padding: 1rem 1.25rem;
        box-shadow: 0 10px 25px rgba(0, 0, 0, 0.35);
      }
      #control-panel h2,
      #layout-panel h2 {
        font-size: 1.1rem;
        margin: 0 0 0.75rem 0;
      }
      #toggle-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
        gap: 0.75rem;
      }
      .toggle-button {
        border: none;
        border-radius: 10px;
        padding: 0.8rem 0.6rem;
        font-size: 0.95rem;
        font-weight: 600;
        cursor: pointer;
        transition: transform 0.12s ease, filter 0.12s ease;
      }
      .toggle-button:disabled {
        cursor: not-allowed;
        opacity: 0.6;
      }
      .toggle-button:not(:disabled):hover {
        transform: translateY(-1px);
        filter: brightness(1.1);
      }
      .combo-group {
        margin-top: 1rem;
        display: grid;
        gap: 0.75rem;
      }
      .combo-group label {
        display: flex;
        flex-direction: column;
        gap: 0.35rem;
        font-size: 0.9rem;
      }
      .combo-group select {
        border-radius: 8px;
        padding: 0.5rem 0.6rem;
        border: 1px solid rgba(255,255,255,0.08);
        background: #1f1f2c;
        color: inherit;
      }
      #actions-row {
        margin-top: 1rem;
        display: flex;
        flex-wrap: wrap;
        gap: 0.6rem;
      }
      #actions-row button {
        border-radius: 8px;
        padding: 0.45rem 0.9rem;
        border: 1px solid rgba(255,255,255,0.12);
        background: transparent;
        color: inherit;
        cursor: pointer;
        transition: background 0.12s ease;
      }
      #actions-row button:hover {
        background: rgba(255,255,255,0.08);
      }
      #layout-panel {
        display: flex;
        flex-direction: column;
        gap: 1rem;
      }
      #layout-config {
        display: flex;
        flex-wrap: wrap;
        gap: 0.75rem;
        align-items: center;
      }
      #layout-config select {
        border-radius: 8px;
        padding: 0.4rem 0.7rem;
        border: 1px solid rgba(255,255,255,0.12);
        background: #1f1f2c;
        color: inherit;
      }
      #tab-selector {
        display: grid;
        gap: 0.4rem;
        grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      }
      #tab-selector label {
        background: rgba(255,255,255,0.05);
        padding: 0.45rem 0.6rem;
        border-radius: 8px;
        display: flex;
        align-items: center;
        gap: 0.5rem;
      }
      #log-container {
        display: grid;
        gap: 1rem;
      }
      #log-container.columns-1 {
        grid-template-columns: 1fr;
      }
      #log-container.columns-2 {
        grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      }
      #log-container.columns-3 {
        grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      }
      .log-card {
        background: rgba(17, 18, 30, 0.92);
        border-radius: 12px;
        overflow: hidden;
        display: flex;
        flex-direction: column;
        border: 1px solid rgba(255,255,255,0.05);
      }
      .log-card header {
        background: rgba(255,255,255,0.05);
        padding: 0.6rem 0.85rem;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }
      .log-card header h3 {
        margin: 0;
        font-size: 1rem;
        font-weight: 600;
      }
      .log-card header button {
        border: none;
        border-radius: 6px;
        padding: 0.25rem 0.55rem;
        background: rgba(255,255,255,0.08);
        color: inherit;
        cursor: pointer;
      }
      .log-scroll {
        overflow-y: auto;
        max-height: 380px;
        padding: 0.8rem 0.85rem;
        font-family: Menlo, Consolas, "Fira Code", monospace;
        font-size: 0.85rem;
        line-height: 1.35rem;
        background: rgba(0, 0, 0, 0.35);
      }
      .log-line {
        padding-bottom: 0.2rem;
      }
      .log-line.origin-gui {
        color: #50fa7b;
      }
      .log-line.origin-container {
        color: #f8f9fa;
      }
      .status-pill {
        background: rgba(80, 250, 123, 0.15);
        border-radius: 999px;
        padding: 0.15rem 0.6rem;
        font-size: 0.8rem;
      }
    </style>
  </head>
  <body>
    <header>
      <h1>Mobipick Labs Control (Web)</h1>
      <div id=\"status-line\" class=\"status-pill\">Connecting...</div>
    </header>
    <main>
      <section id=\"control-panel\" class=\"panel\">
        <h2>Primary Controls</h2>
        <div id=\"toggle-grid\"></div>
        <div class=\"combo-group\" id=\"combo-group\"></div>
        <div id=\"actions-row\">
          <button data-action=\"refresh_status\">Refresh Status</button>
          <button data-action=\"refresh_images\">Refresh Images</button>
          <button data-action=\"refresh_scripts\">Refresh Scripts</button>
        </div>
      </section>
      <section id=\"layout-panel\" class=\"panel\">
        <div>
          <h2>Embedded Process Layout</h2>
          <div id=\"layout-config\">
            <label>
              View Mode:
              <select id=\"layout-select\">
                <option value=\"1\">Single Column</option>
                <option value=\"2\" selected>Two Columns</option>
                <option value=\"3\">Three Columns</option>
              </select>
            </label>
          </div>
        </div>
        <div>
          <h2>Visible Tabs</h2>
          <div id=\"tab-selector\"></div>
        </div>
      </section>
      <section id=\"log-panel\" class=\"panel\" style=\"grid-column: 1 / -1;\">
        <h2>Processes &amp; Logs</h2>
        <div id=\"log-container\" class=\"columns-2\"></div>
      </section>
    </main>
    <script>
      const state = {
        toggles: {},
        tabs: {},
        combobox: {},
        status: {},
        selectedTabs: new Set(),
        layoutColumns: 2,
        lastEvent: 0,
      };

      async function getJSON(url) {
        const res = await fetch(url, { cache: 'no-store' });
        if (!res.ok) {
          throw new Error(`Request failed: ${res.status}`);
        }
        return res.json();
      }

      async function postJSON(url, body) {
        const res = await fetch(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        });
        if (!res.ok) {
          const text = await res.text();
          throw new Error(`Request failed: ${res.status} ${text}`);
        }
        return res.json();
      }

      function applySnapshot(snapshot) {
        state.toggles = snapshot.toggles || {};
        state.combobox = snapshot.combobox || {};
        state.status = snapshot.status || {};
        state.tabs = {};
        const tabs = snapshot.tabs || {};
        Object.keys(tabs).forEach((key) => {
          const tab = tabs[key];
          state.tabs[key] = {
            key,
            label: tab.label || key,
            closable: Boolean(tab.closable),
            logs: Array.isArray(tab.logs) ? tab.logs.slice() : [],
          };
        });
        const events = snapshot.events || [];
        events.forEach((evt) => {
          state.lastEvent = Math.max(state.lastEvent, evt.id || 0);
          applyEvent(evt);
        });
        if (!state.selectedTabs.size) {
          Object.keys(state.tabs).slice(0, 3).forEach((key) => state.selectedTabs.add(key));
        }
        renderAll();
      }

      function applyEvent(evt) {
        if (!evt || !evt.type) return;
        const payload = evt.payload || {};
        switch (evt.type) {
          case 'toggle':
            state.toggles[payload.key] = payload;
            break;
          case 'tab': {
            const key = payload.key;
            if (!key) break;
            const existing = state.tabs[key];
            if (existing) {
              existing.label = payload.label || existing.label;
              existing.closable = Boolean(payload.closable);
            } else {
              state.tabs[key] = {
                key,
                label: payload.label || key,
                closable: Boolean(payload.closable),
                logs: [],
              };
            }
            break;
          }
          case 'tab_removed':
            if (payload.key && state.tabs[payload.key]) {
              delete state.tabs[payload.key];
              state.selectedTabs.delete(payload.key);
            }
            break;
          case 'log': {
            const key = payload.key;
            if (!state.tabs[key]) {
              state.tabs[key] = { key, label: key, closable: true, logs: [] };
            }
            const tab = state.tabs[key];
            tab.logs.push({ html: payload.html || '', origin: payload.origin || 'container' });
            if (tab.logs.length > 400) {
              tab.logs.splice(0, tab.logs.length - 400);
            }
            break;
          }
          case 'combobox':
            state.combobox[payload.name] = {
              options: payload.options || [],
              current: payload.current || null,
            };
            break;
          case 'status':
            state.status[payload.key] = payload.value;
            break;
          case 'shutdown':
            document.getElementById('status-line').textContent = 'Application shutting down';
            break;
          default:
            break;
        }
      }

      function renderAll() {
        renderToggles();
        renderCombos();
        renderTabSelector();
        renderLogs();
        updateStatusLine();
      }

      function renderToggles() {
        const grid = document.getElementById('toggle-grid');
        grid.innerHTML = '';
        Object.keys(state.toggles).forEach((key) => {
          const data = state.toggles[key];
          const btn = document.createElement('button');
          btn.className = 'toggle-button';
          btn.dataset.toggle = key;
          btn.textContent = data.text || key;
          btn.disabled = !data.enabled;
          const colors = data.colors || {};
          btn.style.backgroundColor = colors.bg || '#343a40';
          btn.style.color = colors.fg || '#fff';
          btn.style.padding = `${colors.padding || 6}px`;
          btn.addEventListener('click', () => handleAction(`toggle_${key}`));
          grid.appendChild(btn);
        });
      }

      function renderCombos() {
        const wrapper = document.getElementById('combo-group');
        wrapper.innerHTML = '';
        Object.entries(state.combobox).forEach(([name, data]) => {
          const label = document.createElement('label');
          label.textContent = `${name} configuration`;
          const select = document.createElement('select');
          (data.options || []).forEach((opt) => {
            const option = document.createElement('option');
            option.value = opt;
            option.textContent = opt;
            if (opt === data.current) {
              option.selected = true;
            }
            select.appendChild(option);
          });
          select.addEventListener('change', () => handleAction('set_combo', { name, value: select.value }));
          label.appendChild(select);
          wrapper.appendChild(label);
        });
      }

      function renderTabSelector() {
        const container = document.getElementById('tab-selector');
        container.innerHTML = '';
        Object.values(state.tabs)
          .sort((a, b) => a.label.localeCompare(b.label))
          .forEach((tab) => {
            const label = document.createElement('label');
            const cb = document.createElement('input');
            cb.type = 'checkbox';
            cb.checked = state.selectedTabs.has(tab.key);
            cb.addEventListener('change', () => {
              if (cb.checked) {
                state.selectedTabs.add(tab.key);
              } else {
                state.selectedTabs.delete(tab.key);
              }
              renderLogs();
            });
            const span = document.createElement('span');
            span.textContent = tab.label;
            label.appendChild(cb);
            label.appendChild(span);
            container.appendChild(label);
          });
      }

      function renderLogs() {
        const container = document.getElementById('log-container');
        container.className = `columns-${state.layoutColumns}`;
        container.innerHTML = '';
        Array.from(state.selectedTabs)
          .filter((key) => state.tabs[key])
          .forEach((key) => {
            const tab = state.tabs[key];
            const card = document.createElement('div');
            card.className = 'log-card';
            const header = document.createElement('header');
            const title = document.createElement('h3');
            title.textContent = tab.label;
            header.appendChild(title);
            const pop = document.createElement('button');
            pop.textContent = 'Pop-out';
            pop.addEventListener('click', () => openTabWindow(tab));
            header.appendChild(pop);
            card.appendChild(header);
            const scroller = document.createElement('div');
            scroller.className = 'log-scroll';
            scroller.innerHTML = tab.logs
              .map((line) => `<div class=\"log-line origin-${line.origin || 'container'}\">${line.html || ''}</div>`)
              .join('');
            scroller.scrollTop = scroller.scrollHeight;
            card.appendChild(scroller);
            container.appendChild(card);
          });
      }

      function openTabWindow(tab) {
        const popup = window.open('', `_tab_${tab.key}`, 'width=720,height=560');
        if (!popup) return;
        popup.document.write(`<!DOCTYPE html><html><head><title>${tab.label}</title>` +
          '<meta charset="utf-8" /><style>body{background:#0b0b10;color:#f8f9fa;font-family:Menlo,monospace;padding:1rem;}h1{font-size:1.2rem;margin-bottom:0.75rem;}div{line-height:1.35rem;font-size:0.9rem;} .log-line{margin-bottom:0.25rem;} .origin-gui{color:#50fa7b;}</style></head><body>' +
          `<h1>${tab.label}</h1><div id="log-view"></div></body></html>`);
        popup.document.close();
        const target = popup.document.getElementById('log-view');
        target.innerHTML = tab.logs
          .map((line) => `<div class="log-line ${line.origin ? `origin-${line.origin}` : ''}">${line.html || ''}</div>`)
          .join('');
      }

      function updateStatusLine() {
        const element = document.getElementById('status-line');
        if (!element) return;
        const pieces = [];
        if (state.toggles.sim) {
          pieces.push(`Sim: ${state.toggles.sim.state || 'unknown'}`);
        }
        if (state.toggles.roscore) {
          pieces.push(`Roscore: ${state.toggles.roscore.state || 'unknown'}`);
        }
        element.textContent = pieces.length ? pieces.join(' • ') : 'Ready';
      }

      async function handleAction(action, payload = {}) {
        try {
          await postJSON('/api/action', { action, payload });
        } catch (err) {
          console.error(err);
          alert(err.message || 'Action failed');
        }
      }

      async function pollEvents() {
        try {
          const events = await getJSON(`/api/events?since=${state.lastEvent}`);
          events.forEach((evt) => {
            state.lastEvent = Math.max(state.lastEvent, evt.id || 0);
            applyEvent(evt);
          });
          renderLogs();
          renderToggles();
          updateStatusLine();
        } catch (err) {
          console.warn('Polling failed', err);
          document.getElementById('status-line').textContent = 'Connection lost – retrying…';
        }
      }

      function initLayoutControls() {
        const layoutSelect = document.getElementById('layout-select');
        layoutSelect.value = String(state.layoutColumns);
        layoutSelect.addEventListener('change', () => {
          state.layoutColumns = Number(layoutSelect.value) || 1;
          renderLogs();
        });
        document.querySelectorAll('#actions-row button').forEach((btn) => {
          btn.addEventListener('click', () => handleAction(btn.dataset.action));
        });
      }

      async function bootstrap() {
        try {
          const snapshot = await getJSON('/api/snapshot');
          applySnapshot(snapshot);
          initLayoutControls();
          document.getElementById('status-line').textContent = 'Connected';
          setInterval(pollEvents, 1200);
        } catch (err) {
          console.error(err);
          document.getElementById('status-line').textContent = 'Failed to connect';
        }
      }

      bootstrap();
    </script>
  </body>
</html>
"""

__all__ = ["INDEX_HTML"]

