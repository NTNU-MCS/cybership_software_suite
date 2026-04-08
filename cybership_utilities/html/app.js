// ---------------- Simple JSON WebSocket Protocol ----------------
let ws;
let services = [];
const el = (id) => document.getElementById(id);
const logEl = el("log");
const log = (...a) => {
    logEl.textContent += a.map(x => typeof x === 'string' ? x : JSON.stringify(x)).join(' ') + "\n";
    logEl.scrollTop = logEl.scrollHeight;
};
const setStatus = (s) => el("status").textContent = s;
const getNamespace = () => (el("ns").value || "").trim().replace(/\/+$/, "");
const getAllocatorNode = () => (el("allocNode").value || "").trim();

// Periodic covariance update
let covarianceInterval = null;
const MAX_ODOM_AGE_SECONDS = 10;

// Default WebSocket URL from the current address bar.
// Examples:
// - http://192.168.1.20/...  -> ws://192.168.1.20:8765
// - https://robot.local/...  -> wss://robot.local:8765
// - file:///...              -> ws://localhost:8765
(() => {
    const urlInput = el("url");
    const host = window.location.hostname || "localhost";
    const wsProto = window.location.protocol === "https:" ? "wss" : "ws";
    urlInput.value = `${wsProto}://${host}:8765`;
})();

// Send a message and wait for response
async function sendMessage(message, timeoutMs = 5000) {
    return new Promise((resolve, reject) => {
        const handler = (event) => {
            try {
                const response = JSON.parse(event.data);
                if (response.type === "error") {
                    reject(new Error(response.message));
                } else {
                    resolve(response);
                }
                ws.removeEventListener("message", handler);
            } catch (e) {
                reject(e);
            }
        };

        const timeout = setTimeout(() => {
            ws.removeEventListener("message", handler);
            reject(new Error("Request timeout"));
        }, timeoutMs);

        ws.addEventListener("message", handler);
        ws.send(JSON.stringify(message));

        // Clear timeout when done
        ws.addEventListener("message", () => clearTimeout(timeout), { once: true });
    });
}

el("connect").onclick = () => {
    const url = el("url").value.trim();
    ws = new WebSocket(url);
    ws.onopen = () => {
        setStatus("connected to " + url);
        log("Connected to server");
    };
    ws.onclose = () => {
        setStatus("closed");
        log("Connection closed");
    };
    ws.onerror = (e) => {
        setStatus("error");
        log("WebSocket error:", e?.message ?? "");
    };

    ws.onmessage = (ev) => {
        try {
            const msg = JSON.parse(ev.data);
            if (msg.type === "serverInfo") {
                log("Server:", msg.name, msg.version);
            } else if (msg.type === "services") {
                services = msg.services || [];
                log(`Found ${services.length} services`);

                // Extract and populate namespace buttons
                populateNamespaceButtons(services);
            }
        } catch (e) {
            log("Error parsing message:", e.message);
        }
    };
};


// ---------------- UI Helpers ----------------
function fillSelect(selectEl, topics, countEl) {
    selectEl.innerHTML = "";
    topics.forEach(t => {
        const opt = document.createElement("option");
        opt.value = t;
        opt.textContent = t;
        selectEl.appendChild(opt);
    });
    countEl.textContent = `${topics.length} topic(s)`;
}

function extractNamespaces(services) {
    const namespaces = new Set();
    services.forEach(svc => {
        // Extract the first component after the root /
        // e.g., /voyager/control/force/command/mux -> voyager
        const parts = svc.name.split('/').filter(p => p.length > 0);
        if (parts.length > 0) {
            namespaces.add(parts[0]);
        }
    });
    return Array.from(namespaces).sort();
}

function populateNamespaceButtons(services) {
    const list = el('nsList');
    list.innerHTML = '';
    const namespaces = extractNamespaces(services);

    namespaces.forEach(ns => {
        const opt = document.createElement('option');
        opt.value = `/${ns}`;
        list.appendChild(opt);
    });
}

function lifecycleClassForState(state) {
    const label = (state?.label || '').toLowerCase();
    const id = Number(state?.id);
    if (label.includes('active') && !label.includes('inactive')) {
        return 'lifecycle-state-active';
    }
    if (label.includes('inactive') || label.includes('deactive') || label.includes('unconfigured') || id === 2 || id === 1) {
        return 'lifecycle-state-deactive';
    }
    if (id === 3) {
        return 'lifecycle-state-active';
    }
    return 'lifecycle-state-unknown';
}

function updateLifecycleBadge(state) {
    const badge = el('allocStateBadge');
    const cls = lifecycleClassForState(state);
    badge.className = `lifecycle-state ${cls}`;
    if (!state) {
        badge.textContent = 'unknown';
        return;
    }
    const label = (state.label || '').trim().toLowerCase();
    if (!label) {
        badge.textContent = 'unknown';
        return;
    }
    if (label.includes('active') && !label.includes('inactive')) {
        badge.textContent = 'active';
        return;
    }
    if (label.includes('inactive') || label.includes('deactive') || label.includes('unconfigured')) {
        badge.textContent = 'deactive';
        return;
    }
    badge.textContent = 'unknown';
}

function updateAllocationPanelState(state) {
    const panel = el('allocPanel');
    if (!panel) return;
    panel.classList.remove('mini-panel-state-unknown', 'mini-panel-state-active', 'mini-panel-state-deactive');
    const lifecycleClass = lifecycleClassForState(state);
    if (lifecycleClass === 'lifecycle-state-active') {
        panel.classList.add('mini-panel-state-active');
        return;
    }
    if (lifecycleClass === 'lifecycle-state-deactive') {
        panel.classList.add('mini-panel-state-deactive');
        return;
    }
    panel.classList.add('mini-panel-state-unknown');
}

// ---------------- UI Actions ----------------
el("forceList").onclick = async () => {
    try {
        const namespace = getNamespace();
        const response = await sendMessage({
            type: "mux_list",
            namespace: namespace,
            mux_name: "force_mux"
        });

        if (response.type === "mux_list_response") {
            fillSelect(el('forceSelect'), response.topics, el('forceCount'));
            log(`Force mux: ${response.topics.length} topics`);
        }
    } catch (e) {
        el('forceCount').textContent = `Error: ${e.message}`;
        log("Error listing force topics:", e.message);
    }
};

el("velList").onclick = async () => {
    try {
        const namespace = getNamespace();
        const response = await sendMessage({
            type: "mux_list",
            namespace: namespace,
            mux_name: "velocity_mux"
        });

        if (response.type === "mux_list_response") {
            fillSelect(el('velSelect'), response.topics, el('velCount'));
            log(`Velocity mux: ${response.topics.length} topics`);
        }
    } catch (e) {
        el('velCount').textContent = `Error: ${e.message}`;
        log("Error listing velocity topics:", e.message);
    }
};

el("forceChoose").onclick = async () => {
    try {
        const namespace = getNamespace();
        const topic = el('forceSelect').value.trim();
        if (!topic) {
            el('forceResult').textContent = 'Pick a topic';
            return;
        }

        const response = await sendMessage({
            type: "mux_select",
            namespace: namespace,
            mux_name: "force_mux",
            topic: topic
        });

        if (response.type === "mux_select_response") {
            el('forceResult').textContent = response.success
                ? `Selected (prev: ${response.prev_topic || '-'})`
                : 'Select rejected';
            log(`Force mux selected: ${topic}`);
        }
    } catch (e) {
        el('forceResult').textContent = `Error: ${e.message}`;
        log("Error selecting force topic:", e.message);
    }
};

el("velChoose").onclick = async () => {
    try {
        const namespace = getNamespace();
        const topic = el('velSelect').value.trim();
        if (!topic) {
            el('velResult').textContent = 'Pick a topic';
            return;
        }

        const response = await sendMessage({
            type: "mux_select",
            namespace: namespace,
            mux_name: "velocity_mux",
            topic: topic
        });

        if (response.type === "mux_select_response") {
            el('velResult').textContent = response.success
                ? `Selected (prev: ${response.prev_topic || '-'})`
                : 'Select rejected';
            log(`Velocity mux selected: ${topic}`);
        }
    } catch (e) {
        el('velResult').textContent = `Error: ${e.message}`;
        log("Error selecting velocity topic:", e.message);
    }
};

// -------------- Thrusters (Empty) --------------
el("thrActivate").onclick = async () => {
    try {
        el('thrResult').textContent = 'Activating…';
        const namespace = getNamespace();

        const response = await sendMessage({
            type: "thruster_activate",
            namespace: namespace,
            candidates: ["/thruster/enable", "/thrusters/activate", "/thruster/activate"]
        });

        if (response.type === "thruster_response" && response.success) {
            el('thrResult').textContent = `Activated via ${response.service}`;
            log(`Thrusters activated: ${response.service}`);
        }
    } catch (e) {
        el('thrResult').textContent = `Error: ${e.message}`;
        log("Error activating thrusters:", e.message);
    }
};

el("thrDeactivate").onclick = async () => {
    try {
        el('thrResult').textContent = 'Deactivating…';
        const namespace = getNamespace();

        const response = await sendMessage({
            type: "thruster_deactivate",
            namespace: namespace,
            candidates: ["/thruster/disable", "/thrusters/deactivate"]
        });

        if (response.type === "thruster_response" && response.success) {
            el('thrResult').textContent = `Deactivated via ${response.service}`;
            log(`Thrusters deactivated: ${response.service}`);
        }
    } catch (e) {
        el('thrResult').textContent = `Error: ${e.message}`;
        log("Error deactivating thrusters:", e.message);
    }
};

// -------------- Thrust Allocation (Lifecycle) --------------
function formatLifecycleState(state) {
    if (!state) return "";
    const label = state.label || "";
    const id = (state.id ?? "");
    return label ? `${label} (${id})` : String(id);
}

el("allocState").onclick = async () => {
    try {
        el('allocResult').textContent = '';
        el('allocStateText').textContent = 'Loading…';
        updateLifecycleBadge(null);
        updateAllocationPanelState(null);
        const namespace = getNamespace();
        const nodeName = getAllocatorNode();

        const response = await sendMessage({
            type: "allocator_state",
            namespace: namespace,
            node_name: nodeName || undefined,
        });

        if (response.type === "allocator_state_response") {
            el('allocStateText').textContent = `${response.node_name}: ${formatLifecycleState(response.state)}`;
            updateLifecycleBadge(response.state);
            updateAllocationPanelState(response.state);
            log(`Allocator state: ${response.node_name} -> ${formatLifecycleState(response.state)}`);
        }
    } catch (e) {
        el('allocStateText').textContent = `Error: ${e.message}`;
        updateLifecycleBadge(null);
        updateAllocationPanelState(null);
        log("Error getting allocator state:", e.message);
    }
};

el("allocActivate").onclick = async () => {
    try {
        el('allocResult').textContent = 'Activating…';
        const namespace = getNamespace();
        const nodeName = getAllocatorNode();

        const response = await sendMessage({
            type: "allocator_activate",
            namespace: namespace,
            node_name: nodeName || undefined,
        });

        if (response.type === "allocator_response" && response.success) {
            el('allocResult').textContent = `Activated (${formatLifecycleState(response.state)})`;
            el('allocStateText').textContent = `${response.node_name}: ${formatLifecycleState(response.state)}`;
            updateLifecycleBadge(response.state);
            updateAllocationPanelState(response.state);
            log(`Allocator activated: ${response.node_name}`);
        }
    } catch (e) {
        el('allocResult').textContent = `Error: ${e.message}`;
        updateLifecycleBadge(null);
        updateAllocationPanelState(null);
        log("Error activating allocator:", e.message);
    }
};

el("allocDeactivate").onclick = async () => {
    try {
        el('allocResult').textContent = 'Deactivating…';
        const namespace = getNamespace();
        const nodeName = getAllocatorNode();

        const response = await sendMessage({
            type: "allocator_deactivate",
            namespace: namespace,
            node_name: nodeName || undefined,
        });

        if (response.type === "allocator_response" && response.success) {
            el('allocResult').textContent = `Deactivated (${formatLifecycleState(response.state)})`;
            el('allocStateText').textContent = `${response.node_name}: ${formatLifecycleState(response.state)}`;
            updateLifecycleBadge(response.state);
            updateAllocationPanelState(response.state);
            log(`Allocator deactivated: ${response.node_name}`);
        }
    } catch (e) {
        el('allocResult').textContent = `Error: ${e.message}`;
        updateLifecycleBadge(null);
        updateAllocationPanelState(null);
        log("Error deactivating allocator:", e.message);
    }
};

// -------------- Localization (Covariance) --------------
function updateCovarianceIndicator(covariance, ageSec = null) {
    const indicator = el('covarianceIndicator');
    const valueEl = el('covarianceValue');
    const panel = el('localizationPanel');

    const setPanelState = (stateClass) => {
        panel.classList.remove('localization-pane-state-unknown', 'localization-pane-state-good', 'localization-pane-state-bad');
        panel.classList.add(stateClass);
    };

    if (covariance === null || covariance === undefined) {
        valueEl.textContent = '-';
        indicator.className = 'covariance-indicator';
        setPanelState('localization-pane-state-unknown');
        return;
    }

    const covNum = parseFloat(covariance);
    if (isNaN(covNum)) {
        valueEl.textContent = '-';
        indicator.className = 'covariance-indicator';
        setPanelState('localization-pane-state-unknown');
        return;
    }

    const ageNum = Number(ageSec);
    const isStale = Number.isFinite(ageNum) && ageNum > MAX_ODOM_AGE_SECONDS;
    if (isStale) {
        valueEl.textContent = `${covNum.toFixed(4)} (stale ${ageNum.toFixed(1)}s)`;
        indicator.className = 'covariance-indicator covariance-stale';
        setPanelState('localization-pane-state-unknown');
        return;
    }

    valueEl.textContent = covNum.toFixed(4);
    if (covNum < 3) {
        indicator.className = 'covariance-indicator covariance-good';
        setPanelState('localization-pane-state-good');
    } else {
        indicator.className = 'covariance-indicator covariance-bad';
        setPanelState('localization-pane-state-bad');
    }
}

async function updateCovariance(manual = false) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        if (manual) {
            el('covarianceValue').textContent = 'disconnected';
            updateCovarianceIndicator(null, null);
            log('Covariance refresh skipped: websocket not connected');
        }
        return;
    }

    try {
        const namespace = getNamespace();
        const response = await sendMessage({
            type: "get_covariance",
            namespace: namespace
        });

        if (response.type === "covariance_response") {
            updateCovarianceIndicator(response.covariance, response.age_sec);
            if (manual) {
                const ageNum = Number(response.age_sec);
                const ageTxt = Number.isFinite(ageNum) ? ageNum.toFixed(1) : '-';
                log(`Covariance refreshed: value=${response.covariance ?? '-'} age=${ageTxt}s`);
            }
        }
    } catch (e) {
        if (manual) {
            el('covarianceValue').textContent = `error: ${e.message}`;
            updateCovarianceIndicator(null, null);
            log("Error refreshing covariance:", e.message);
        }
        // Silently ignore errors during periodic updates
    }
}

el("covarianceRefresh").onclick = () => updateCovariance(true);

// Start periodic covariance updates when connected
const origOnclose = (() => {
    const closeHandler = el("connect").onclick;
    return function() {
        if (ws) {
            ws.onclose = () => {
                setStatus("closed");
                log("Connection closed");
                if (covarianceInterval) {
                    clearInterval(covarianceInterval);
                    covarianceInterval = null;
                }
            };
        }
    };
})();

const origConnectOnclick = el("connect").onclick;
el("connect").onclick = function() {
    origConnectOnclick.call(this);
    // Start periodic updates after connection
    setTimeout(() => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            if (covarianceInterval) clearInterval(covarianceInterval);
            covarianceInterval = setInterval(updateCovariance, 1000);
            updateCovariance();
        }
    }, 500);
};

// -------------- Set Pose --------------
el("setPose").onclick = async () => {
    try {
        el('poseResult').textContent = 'Setting pose…';
        const namespace = "/" + (getNamespace().replace(/^\/+/, ""));
        const x = parseFloat(el('poseX').value) || 0.0;
        const y = parseFloat(el('poseY').value) || 0.0;
        const theta = parseFloat(el('poseTheta').value) || 0.0;

        const response = await sendMessage({
            type: "set_pose",
            namespace: namespace,
            x: x,
            y: y,
            theta: theta
        });

        if (response.type === "set_pose_response" && response.success) {
            el('poseResult').textContent = `Pose set: (${x}, ${y}, ${theta})`;
            log(`Pose set to (${x}, ${y}, ${theta})`);
        } else {
            el('poseResult').textContent = response.message || 'Unknown response';
        }
    } catch (e) {
        el('poseResult').textContent = `Error: ${e.message}`;
        log("Error setting pose:", e.message);
    }
};
