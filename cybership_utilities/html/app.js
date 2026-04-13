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
const LIVE_MAP_UPDATE_MS = 250;

// Map state
let mapState = {
    x: 0,
    y: 0,
    yaw: 0,
    x_cov: 0.1,
    y_cov: 0.1,
    cov_age_sec: null,
};

function drawPositionMap() {
    const canvas = el('positionMap');
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const scale = parseFloat(el('mapScale').value) || 2;

    const width = canvas.width;
    const height = canvas.height;
    const centerX = width / 2;
    const centerY = height / 2;
    const pixelsPerMeter = 50 * scale;

    // Clear canvas
    ctx.fillStyle = '#f9fcff';
    ctx.fillRect(0, 0, width, height);

    // Draw grid (1m spacing)
    ctx.strokeStyle = '#e8f0f8';
    ctx.lineWidth = 1;
    const gridSpacing = pixelsPerMeter;

    // Vertical lines (East/West - Y axis)
    for (let x = centerX % gridSpacing - gridSpacing; x < width; x += gridSpacing) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }

    // Horizontal lines (North/South - X axis)
    for (let y = centerY % gridSpacing - gridSpacing; y < height; y += gridSpacing) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    // Draw axes (thicker)
    ctx.strokeStyle = '#b0c4dc';
    ctx.lineWidth = 2;

    // North-South axis (X)
    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, height);
    ctx.stroke();

    // East-West axis (Y)
    ctx.beginPath();
    ctx.moveTo(0, centerY);
    ctx.lineTo(width, centerY);
    ctx.stroke();

    // Draw axis labels
    ctx.fillStyle = '#486081';
    ctx.font = 'bold 14px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('N', centerX, 25);  // North (up)
    ctx.textAlign = 'left';
    ctx.fillText('E', width - 25, centerY + 5);  // East (right)

    // NED frame conversion:
    // NED: X=North, Y=East
    // Screen: +X=right, +Y=down
    // So: screen_x = centerX + ned_y * ppm, screen_y = centerY - ned_x * ppm
    const screenX = centerX + mapState.y * pixelsPerMeter;
    const screenY = centerY - mapState.x * pixelsPerMeter;

    // Draw covariance ellipse (2 sigma = 95% confidence)
    const sigma = 2;
    const ellipseA = Math.sqrt(mapState.y_cov) * sigma * pixelsPerMeter;  // Y variance
    const ellipseB = Math.sqrt(mapState.x_cov) * sigma * pixelsPerMeter;  // X variance
    const covAgeNum = Number(mapState.cov_age_sec);
    const covarianceIsStale = Number.isFinite(covAgeNum) && covAgeNum > MAX_ODOM_AGE_SECONDS;

    ctx.fillStyle = covarianceIsStale
        ? 'rgba(158, 167, 179, 0.35)'
        : 'rgba(227, 111, 101, 0.35)';
    ctx.beginPath();
    ctx.ellipse(screenX, screenY, ellipseA, ellipseB, 0, 0, 2 * Math.PI);
    ctx.fill();

    // Draw position point
    ctx.fillStyle = '#0f9d8b';
    ctx.beginPath();
    ctx.arc(screenX, screenY, 7, 0, 2 * Math.PI);
    ctx.fill();

    // Draw yaw arrow
    // In NED: yaw=0 points North (+X), yaw=π/2 points East (+Y)
    // On screen: N=up(-Y), E=right(+X)
    const screenYaw = Math.PI / 2 + mapState.yaw;
    const arrowLength = 40;
    const arrowX = screenX - arrowLength * Math.cos(screenYaw);
    const arrowY = screenY - arrowLength * Math.sin(screenYaw);

    ctx.strokeStyle = '#0f9d8b';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(screenX, screenY);
    ctx.lineTo(arrowX, arrowY);
    ctx.stroke();

    // Draw arrowhead
    const headLen = 12;
    const angle1 = screenYaw + Math.PI * 0.75;
    const angle2 = screenYaw - Math.PI * 0.75;

    ctx.beginPath();
    ctx.moveTo(arrowX, arrowY);
    ctx.lineTo(arrowX - headLen * Math.cos(angle1), arrowY - headLen * Math.sin(angle1));
    ctx.moveTo(arrowX, arrowY);
    ctx.lineTo(arrowX - headLen * Math.cos(angle2), arrowY - headLen * Math.sin(angle2));
    ctx.stroke();

    // Draw position values overlay
    const padding = 12;
    const lineHeight = 18;
    const textX = padding + 10;
    let textY = padding + 20;

    ctx.font = 'bold 12px monospace';
    ctx.fillStyle = 'rgba(19, 35, 61, 0.85)';
    ctx.textAlign = 'left';

    ctx.fillText(`  X: ${mapState.x.toFixed(2)} m`, textX, textY);
    textY += lineHeight;
    ctx.fillText(`  Y: ${mapState.y.toFixed(2)} m`, textX, textY);
    textY += lineHeight;
    ctx.fillText(`Yaw: ${mapState.yaw.toFixed(3)} rad`, textX, textY);
}

el('mapScale').oninput = function() {
    el('mapScaleValue').textContent = this.value + 'x';
    drawPositionMap();
};

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
        drawPositionMap();
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
    panel.classList.remove('panel-unknown', 'panel-good', 'panel-bad');
    const lifecycleClass = lifecycleClassForState(state);
    if (lifecycleClass === 'lifecycle-state-active') {
        panel.classList.add('panel-good');
        return;
    }
    if (lifecycleClass === 'lifecycle-state-deactive') {
        panel.classList.add('panel-bad');
        return;
    }
    panel.classList.add('panel-unknown');
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
function formatCompactScientific(value) {
    const num = Number(value);
    if (!Number.isFinite(num)) return '-';
    if (num === 0) return '0.000e00';

    const [mantissa, exponentRaw] = num.toExponential(3).split('e');
    const exponent = Number(exponentRaw);
    const expSign = exponent < 0 ? '-' : '';
    const expAbs = Math.abs(exponent).toString().padStart(2, '0');

    return `${mantissa}e${expSign}${expAbs}`;
}

function updateCovarianceIndicator(covariance, ageSec = null, isY = false) {
    const indicatorId = isY ? 'covarianceIndicatorY' : 'covarianceIndicatorX';
    const valueId = isY ? 'covarianceValueY' : 'covarianceValueX';
    const indicator = el(indicatorId);
    const valueEl = el(valueId);

    if (covariance === null || covariance === undefined) {
        valueEl.textContent = '-';
        indicator.className = 'covariance-indicator';
        return;
    }

    const covNum = parseFloat(covariance);
    if (isNaN(covNum)) {
        valueEl.textContent = '-';
        indicator.className = 'covariance-indicator';
        return;
    }

    const ageNum = Number(ageSec);
    const isStale = Number.isFinite(ageNum) && ageNum > MAX_ODOM_AGE_SECONDS;
    const covText = formatCompactScientific(covNum);
    if (isStale) {
        const ageText = formatCompactScientific(ageNum);
        valueEl.textContent = `${covText} (stale ${ageText}s)`;
        indicator.className = 'covariance-indicator covariance-stale';
        return;
    }

    valueEl.textContent = covText;
    if (covNum < 3) {
        indicator.className = 'covariance-indicator covariance-good';
    } else {
        indicator.className = 'covariance-indicator covariance-bad';
    }
}

function updateLocalizationPanelState(x_cov, y_cov, ageSec = null) {
    const panel = el('localizationPanel');
    if (!panel) return;

    panel.classList.remove('panel-unknown', 'panel-good', 'panel-bad');

    const ageNum = Number(ageSec);
    const isStale = Number.isFinite(ageNum) && ageNum > MAX_ODOM_AGE_SECONDS;
    if (isStale) {
        panel.classList.add('panel-unknown');
        return;
    }

    if (x_cov === null || x_cov === undefined || y_cov === null || y_cov === undefined) {
        panel.classList.add('panel-unknown');
        return;
    }

    const xNum = parseFloat(x_cov);
    const yNum = parseFloat(y_cov);

    if (isNaN(xNum) || isNaN(yNum)) {
        panel.classList.add('panel-unknown');
        return;
    }

    // Green if both are good, red if either is bad
    if (xNum < 3 && yNum < 3) {
        panel.classList.add('panel-good');
    } else {
        panel.classList.add('panel-bad');
    }
}

function updatePoseDisplay(x, y, yaw) {
    if (x === null || x === undefined) {
        mapState.x = 0;
    } else {
        mapState.x = parseFloat(x);
    }

    if (y === null || y === undefined) {
        mapState.y = 0;
    } else {
        mapState.y = parseFloat(y);
    }

    if (yaw === null || yaw === undefined) {
        mapState.yaw = 0;
    } else {
        mapState.yaw = parseFloat(yaw);
    }

    drawPositionMap();
}

async function updateCovariance(manual = false) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        if (manual) {
            el('covarianceValueX').textContent = 'disconnected';
            el('covarianceValueY').textContent = 'disconnected';
            updateCovarianceIndicator(null, null, false);
            updateCovarianceIndicator(null, null, true);
            updateLocalizationPanelState(null, null);
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
            updateCovarianceIndicator(response.x_covariance, response.age_sec, false);
            updateCovarianceIndicator(response.y_covariance, response.age_sec, true);
            updateLocalizationPanelState(response.x_covariance, response.y_covariance, response.age_sec);
            mapState.cov_age_sec = Number.isFinite(Number(response.age_sec))
                ? Number(response.age_sec)
                : null;

            // Update map state with covariance
            if (response.x_covariance !== null && response.x_covariance !== undefined) {
                mapState.x_cov = parseFloat(response.x_covariance);
            }
            if (response.y_covariance !== null && response.y_covariance !== undefined) {
                mapState.y_cov = parseFloat(response.y_covariance);
            }

            updatePoseDisplay(response.x, response.y, response.yaw);
            if (manual) {
                const ageNum = Number(response.age_sec);
                const ageTxt = Number.isFinite(ageNum) ? formatCompactScientific(ageNum) : '-';
                log(`Covariance refreshed: X=${response.x_covariance ?? '-'} Y=${response.y_covariance ?? '-'} age=${ageTxt}s`);
            }
        }
    } catch (e) {
        if (manual) {
            el('covarianceValueX').textContent = `error: ${e.message}`;
            el('covarianceValueY').textContent = `error: ${e.message}`;
            updateCovarianceIndicator(null, null, false);
            updateCovarianceIndicator(null, null, true);
            updateLocalizationPanelState(null, null);
            log("Error refreshing covariance:", e.message);
        }
        // Silently ignore errors during periodic updates
    }
}

// Initialize map on page load
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', drawPositionMap);
} else {
    drawPositionMap();
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
            covarianceInterval = setInterval(updateCovariance, LIVE_MAP_UPDATE_MS);
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

// -------------- Go To Point (NavigateToPose Action) --------------
el("goToGetPose").onclick = async () => {
    try {
        if (!ws || ws.readyState !== WebSocket.OPEN) {
            el('goToResult').textContent = 'Error: websocket not connected';
            return;
        }

        const namespace = "/" + (getNamespace().replace(/^\/+/, ""));
        const response = await sendMessage({
            type: "get_covariance",
            namespace: namespace,
        });

        if (response.type !== "covariance_response") {
            el('goToResult').textContent = 'Error: invalid pose response';
            return;
        }

        const x = Number(response.x);
        const y = Number(response.y);
        const yaw = Number(response.yaw);

        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(yaw)) {
            el('goToResult').textContent = 'Error: pose not available';
            return;
        }

        el('gotoX').value = x.toFixed(3);
        el('gotoY').value = y.toFixed(3);
        el('gotoYaw').value = yaw.toFixed(3);
        el('goToResult').textContent = 'Pose loaded into Go To fields';
        log(`Loaded pose for Go To: x=${x.toFixed(3)} y=${y.toFixed(3)} yaw=${yaw.toFixed(3)}`);
    } catch (e) {
        el('goToResult').textContent = `Error: ${e.message}`;
        log("Error getting pose for Go To:", e.message);
    }
};

el("goToPoint").onclick = async () => {
    try {
        el('goToResult').textContent = 'Sending goal…';
        const namespace = "/" + (getNamespace().replace(/^\/+/, ""));
        const x = parseFloat(el('gotoX').value) || 0.0;
        const y = parseFloat(el('gotoY').value) || 0.0;
        const yaw = parseFloat(el('gotoYaw').value) || 0.0;
        const frameId = (el('gotoFrame').value || 'world').trim() || 'world';

        const response = await sendMessage({
            type: "go_to_point",
            namespace: namespace,
            x: x,
            y: y,
            yaw: yaw,
            frame_id: frameId,
            wait_result: false,
        });

        if (response.type === "go_to_point_response" && response.success && response.accepted) {
            el('goToResult').textContent = `Goal accepted: (${x}, ${y}, ${yaw})`;
            log(`NavigateToPose goal accepted in ${frameId}: x=${x} y=${y} yaw=${yaw}`);
        } else {
            el('goToResult').textContent = 'Goal rejected';
            log('NavigateToPose goal rejected:', response);
        }
    } catch (e) {
        el('goToResult').textContent = `Error: ${e.message}`;
        log("Error sending NavigateToPose goal:", e.message);
    }
};
