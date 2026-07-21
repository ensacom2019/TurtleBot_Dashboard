const state = {
  data: null,
  mapImage: new Image(),
  mapLoaded: false,
  mapWallCache: null,
  selectedGoal: null,
  goalDrag: null,
  waypoints: [],
  waypointMode: false,
  plannedPath: { points: [], cells: [], status: "not planned" },
  manualTimer: null,
  manualCommand: { linear: 0, angular: 0 },
  cameraEnabled: true,
  pressedKeys: new Set(),
  setupTool: "pose",
  setupDrag: null,
  setupDirty: false,
  setupSaving: false,
  setupObjectRect: null,
  mapEditor: {
    raster: null,
    context: null,
    drawing: false,
    lastPixel: null,
    tool: "pen",
    dirty: false,
    transform: null,
    widthCm: 0,
    heightCm: 0,
    cmPerPixel: 1,
    loadedMapId: null,
    zoom: 1,
    panX: 0,
    panY: 0,
    panning: false,
    lastPanClient: null,
  },
};

const ASTAR_SOFT_CELL_MULTIPLIER = 12;

const DEFAULT_ROBOT_PROFILES = {
  tb3_2: {
    label: "TurtleBot 2",
    namespace: "/",
    body: { length: 0.18, width: 0.14 },
    mapPose: { enabled: false, x: null, y: null, yaw: 0 },
    connection: {
      robotIp: "192.168.20.7",
      rosDomainId: "1",
      rosLocalhostOnly: "0",
      robotSshHost: "192.168.20.7",
      robotSshUser: "kim",
      robotSshPassword: "",
    },
    topics: topicsForNamespace("/", "plain"),
  },
};

const els = {};

document.addEventListener("DOMContentLoaded", () => {
  bindElements();
  bindTabs();
  bindActions();
  renderWaypoints();
  resizeCanvases();
  window.addEventListener("resize", resizeCanvases);
  loadState();
  connectEvents();
  startCameraLoop();
  startDrawLoop();
});

function bindElements() {
  for (const id of [
    "connectionText",
    "saveSetupButton",
    "setupSaveStatus",
    "setupToolHint",
    "mapSelect",
    "mapResolution",
    "mapOriginX",
    "mapOriginY",
    "mapOriginYaw",
    "gridCellSize",
    "plannerHardClearanceRange",
    "plannerHardClearance",
    "showGrid",
    "showInflation",
    "showLidarPoints",
    "detectLidarObstacles",
    "detectBlackWalls",
    "clearGridButton",
    "robotIp",
    "activeRobotProfile",
    "otherRobotList",
    "serverIp",
    "rosDomainId",
    "rosLocalhostOnly",
    "robotSshHost",
    "robotSshUser",
    "robotSshPassword",
    "topicScan",
    "initialX",
    "initialY",
    "initialYaw",
    "robotLength",
    "robotWidth",
    "accessoryFront",
    "accessoryBack",
    "accessoryLeft",
    "accessoryRight",
    "accessoryHeight",
    "safetyMargin",
    "effectiveFront",
    "effectiveBack",
    "effectiveLeft",
    "effectiveRight",
    "effectiveLength",
    "effectiveWidth",
    "nav2Footprint",
    "copyFootprintButton",
    "robotBringupButton",
    "robotHardwareCheckButton",
    "robotSshDetailButton",
    "robotBringupStopButton",
    "robotCheckButton",
    "copyDiagnosticsButton",
    "refreshConnectionButton",
    "addRobotProfileButton",
    "deleteRobotProfileButton",
    "resetTopicsButton",
    "robotCheckResults",
    "diagnosticReport",
    "discoveryResults",
    "connDomain",
    "connLocalhost",
    "connServerIps",
    "connScan",
    "connPose",
    "connNav2",
    "connOpencr",
    "objectWidth",
    "objectHeight",
    "objectInflation",
    "fallbackEnabled",
    "fallbackMaxLinear",
    "fallbackMinLinear",
    "fallbackMaxAngular",
    "fallbackLookahead",
    "fallbackSoftDistanceRange",
    "fallbackSoftDistance",
    "fallbackHardMargin",
    "fallbackScanTimeoutRange",
    "fallbackScanTimeout",
    "fallbackOdomTimeoutRange",
    "fallbackOdomTimeout",
    "fallbackCollisionHorizon",
    "clearObstaclesButton",
    "obstacleList",
    "topicPose",
    "topicOdom",
    "topicCamera",
    "topicCompressedCamera",
    "topicInitialPose",
    "topicGoalAction",
    "topicRouteAction",
    "topicGoalTopic",
    "topicCmdVel",
    "setupMapCanvas",
    "driveMapCanvas",
    "mapEditorCanvas",
    "mapEditorName",
    "mapEditorLoadSelect",
    "mapEditorLoadButton",
    "mapEditorWidth",
    "mapEditorHeight",
    "mapEditorCmPerPixel",
    "mapEditorCreateButton",
    "mapEditorPenButton",
    "mapEditorEraserButton",
    "mapEditorClearButton",
    "mapEditorSaveButton",
    "mapEditorDeleteButton",
    "mapEditorSize",
    "mapEditorStatus",
    "cameraFrame",
    "cameraFpsBadge",
    "cameraStamp",
    "cameraRawButton",
    "cameraCompressedButton",
    "cameraToggleButton",
    "goalX",
    "goalY",
    "goalYaw",
    "goalYawRange",
    "goalModeButton",
    "waypointModeButton",
    "clearWaypointsButton",
    "waypointList",
    "repeatRouteEnabled",
    "repeatRouteStart",
    "repeatRouteEnd",
    "repeatRoutePause",
    "repeatRouteStatus",
    "forceLidarFallback",
    "manualLinearSpeed",
    "manualAngularSpeed",
    "manualStopButton",
    "sendGoalButton",
    "resumeRouteButton",
    "routeResumeStatus",
    "cancelGoalButton",
    "stopButton",
    "statusMode",
    "statusNav",
    "statusMessage",
    "statusPose",
    "statusGoal",
    "statusPath",
    "statusSafety",
    "copyRunLogsButton",
    "downloadRunLogsButton",
    "clearRunLogsButton",
    "runLogSummary",
    "robotDiscoveryNotice",
    "robotDiscoveryNoticeSummary",
    "robotDiscoveryNoticeClose",
    "robotDiscoveryNoticeCloseButton",
    "toast",
  ]) {
    els[id] = document.getElementById(id);
  }
}

function bindTabs() {
  document.querySelectorAll(".tab-button").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".tab-button").forEach((tab) => tab.classList.remove("active"));
      document.querySelectorAll(".tab-panel").forEach((panel) => panel.classList.remove("active"));
      button.classList.add("active");
      document.getElementById(button.dataset.tab).classList.add("active");
      if (button.dataset.tab !== "drive") stopManualDrive();
      if (button.dataset.tab === "drive") refreshRunLogSummary();
      resizeCanvases();
    });
  });
}

function bindActions() {
  els.mapSelect.addEventListener("change", selectSavedMap);
  els.saveSetupButton.addEventListener("click", saveSetup);
  els.sendGoalButton.addEventListener("click", sendGoal);
  els.resumeRouteButton.addEventListener("click", resumeRoute);
  els.goalModeButton.addEventListener("click", () => setWaypointMode(false));
  els.waypointModeButton.addEventListener("click", () => setWaypointMode(true));
  els.clearWaypointsButton.addEventListener("click", clearWaypoints);
  els.copyFootprintButton.addEventListener("click", copyFootprint);
  els.clearGridButton.addEventListener("click", clearBlockedCells);
  els.clearObstaclesButton.addEventListener("click", clearObstacles);
  els.robotBringupButton.addEventListener("click", robotBringup);
  els.robotHardwareCheckButton.addEventListener("click", checkRobotHardware);
  els.robotSshDetailButton.addEventListener("click", () => robotSshCheck(true));
  els.robotBringupStopButton.addEventListener("click", stopRobotBringup);
  els.robotCheckButton.addEventListener("click", checkRobot);
  els.copyDiagnosticsButton.addEventListener("click", copyDiagnostics);
  els.copyRunLogsButton.addEventListener("click", copyRunLogs);
  els.downloadRunLogsButton.addEventListener("click", downloadRunLogs);
  els.clearRunLogsButton.addEventListener("click", clearRunLogs);
  els.refreshConnectionButton.addEventListener("click", refreshConnectionStatus);
  els.addRobotProfileButton.addEventListener("click", addRobotProfile);
  els.deleteRobotProfileButton.addEventListener("click", deleteActiveRobotProfile);
  els.resetTopicsButton.addEventListener("click", resetTopicsFromRobot);
  els.robotDiscoveryNoticeClose.addEventListener("click", closeRobotDiscoveryNotice);
  els.robotDiscoveryNoticeCloseButton.addEventListener("click", closeRobotDiscoveryNotice);
  els.robotDiscoveryNotice.addEventListener("click", (event) => {
    if (event.target === els.robotDiscoveryNotice) closeRobotDiscoveryNotice();
  });
  els.activeRobotProfile.addEventListener("change", () => applyRobotProfile(els.activeRobotProfile.value));
  els.manualStopButton.addEventListener("click", () => stopManualDrive({ force: true }));
  els.cancelGoalButton.addEventListener("click", () => postJson("/api/cancel", {}).then(showResult));
  els.stopButton.addEventListener("click", stopAllMotion);
  els.cameraToggleButton.addEventListener("click", toggleCamera);
  els.cameraRawButton.addEventListener("click", () => setCameraStreamMode("raw"));
  els.cameraCompressedButton.addEventListener("click", () => setCameraStreamMode("compressed"));
  document.querySelectorAll("[data-setup-tool]").forEach((button) => {
    button.addEventListener("click", () => setSetupTool(button.dataset.setupTool));
  });
  document.querySelectorAll("[data-object-preset]").forEach((button) => {
    button.addEventListener("click", () => applyObjectPreset(button));
  });
  setupInputs().forEach((input) => input.addEventListener("input", markSetupDirty));
  setupInputs().forEach((input) => input.addEventListener("change", markSetupDirty));
  bindSynchronizedRange(els.plannerHardClearanceRange, els.plannerHardClearance);
  bindSynchronizedRange(els.fallbackSoftDistanceRange, els.fallbackSoftDistance);
  bindSynchronizedRange(els.fallbackScanTimeoutRange, els.fallbackScanTimeout);
  bindSynchronizedRange(els.fallbackOdomTimeoutRange, els.fallbackOdomTimeout);
  bindGoalControls();
  for (const input of [
    els.repeatRouteEnabled,
    els.repeatRouteStart,
    els.repeatRouteEnd,
    els.repeatRoutePause,
  ]) {
    input.addEventListener("input", onRepeatRouteChanged);
    input.addEventListener("change", onRepeatRouteChanged);
  }
  els.setupMapCanvas.addEventListener("pointerdown", onSetupPointerDown);
  els.setupMapCanvas.addEventListener("pointermove", onSetupPointerMove);
  els.setupMapCanvas.addEventListener("pointerup", onSetupPointerUp);
  els.setupMapCanvas.addEventListener("pointercancel", onSetupPointerUp);
  document.querySelectorAll("[data-drive-linear]").forEach((button) => {
    button.addEventListener("click", (event) => {
      event.preventDefault();
      startManualDrive(Number(button.dataset.driveLinear), Number(button.dataset.driveAngular));
    });
  });
  window.addEventListener("keydown", onManualKeyDown);
  window.addEventListener("keyup", onManualKeyUp);
  window.addEventListener("blur", stopManualDriveFromFocusLoss);
  window.addEventListener("pagehide", sendManualStopBeacon);
  window.addEventListener("beforeunload", sendManualStopBeacon);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopManualDriveFromFocusLoss();
  });
  els.driveMapCanvas.addEventListener("pointerdown", onDrivePointerDown);
  els.driveMapCanvas.addEventListener("pointermove", onDrivePointerMove);
  els.driveMapCanvas.addEventListener("pointerup", onDrivePointerUp);
  els.driveMapCanvas.addEventListener("pointercancel", onDrivePointerUp);
  els.mapEditorCreateButton.addEventListener("click", createMapEditorGrid);
  els.mapEditorPenButton.addEventListener("click", () => setMapEditorTool("pen"));
  els.mapEditorEraserButton.addEventListener("click", () => setMapEditorTool("eraser"));
  els.mapEditorClearButton.addEventListener("click", clearMapEditorGrid);
  els.mapEditorSaveButton.addEventListener("click", saveEditorMap);
  els.mapEditorLoadSelect.addEventListener("change", updateMapEditorDeleteButton);
  els.mapEditorLoadButton.addEventListener("click", loadEditorMap);
  els.mapEditorDeleteButton.addEventListener("click", deleteEditorMap);
  els.mapEditorCanvas.addEventListener("pointerdown", onMapEditorPointerDown);
  els.mapEditorCanvas.addEventListener("pointermove", onMapEditorPointerMove);
  els.mapEditorCanvas.addEventListener("pointerup", onMapEditorPointerUp);
  els.mapEditorCanvas.addEventListener("pointercancel", onMapEditorPointerUp);
  els.mapEditorCanvas.addEventListener("wheel", onMapEditorWheel, { passive: false });
}

function bindSynchronizedRange(rangeInput, numberInput, minimumProvider = null) {
  const sync = (source, target) => {
    const value = optionalNumberValue(source);
    if (value === null) return;
    const configuredMinimum = Number(source.min || target.min || 0);
    const dynamicMinimum = minimumProvider ? Number(minimumProvider()) : configuredMinimum;
    const minimum = Math.max(configuredMinimum, Number.isFinite(dynamicMinimum) ? dynamicMinimum : configuredMinimum);
    const maximum = Number(source.max || target.max || value);
    const next = Math.min(maximum, Math.max(minimum, value));
    const normalized = String(round(next));
    source.value = normalized;
    target.value = normalized;
    planPathToGoal();
  };
  rangeInput.addEventListener("input", () => sync(rangeInput, numberInput));
  numberInput.addEventListener("input", () => sync(numberInput, rangeInput));
  numberInput.addEventListener("change", () => sync(numberInput, rangeInput));
}

function bindGoalControls() {
  const syncPosition = () => {
    const x = optionalNumberValue(els.goalX);
    const y = optionalNumberValue(els.goalY);
    if (x === null || y === null) return;
    const yaw = normalizedYaw(optionalNumberValue(els.goalYaw) ?? 0);
    state.selectedGoal = { x, y, yaw };
    planPathToGoal();
    renderWaypoints();
  };
  els.goalX.addEventListener("input", syncPosition);
  els.goalY.addEventListener("input", syncPosition);

  const syncYaw = (source, target) => {
    const raw = optionalNumberValue(source);
    if (raw === null) return;
    const yaw = normalizedYaw(raw);
    source.value = String(round(yaw));
    target.value = String(round(yaw));
    syncPosition();
  };
  els.goalYawRange.addEventListener("input", () => syncYaw(els.goalYawRange, els.goalYaw));
  els.goalYaw.addEventListener("input", () => syncYaw(els.goalYaw, els.goalYawRange));
  els.goalYaw.addEventListener("change", () => syncYaw(els.goalYaw, els.goalYawRange));
}

function onDrivePointerDown(event) {
  if (event.button !== 0) return;
  const world = canvasEventToWorld(event, els.driveMapCanvas);
  if (!world) return;
  event.preventDefault();
  if (state.waypointMode) {
    addWaypoint(world);
    return;
  }
  const pose = state.data?.runtime?.pose || currentSetup().initialPose;
  const yaw = pose
    ? Math.atan2(world.y - Number(pose.y), world.x - Number(pose.x))
    : optionalNumberValue(els.goalYaw) ?? 0;
  state.goalDrag = {
    pointerId: event.pointerId,
    origin: { x: world.x, y: world.y },
  };
  try {
    els.driveMapCanvas.setPointerCapture(event.pointerId);
  } catch (_error) {
    // Pointer capture is optional in older embedded browsers.
  }
  setSelectedGoal(world.x, world.y, yaw);
}

function onDrivePointerMove(event) {
  const drag = state.goalDrag;
  if (!drag || drag.pointerId !== event.pointerId) return;
  const world = canvasEventToWorld(event, els.driveMapCanvas);
  if (!world) return;
  const dx = world.x - drag.origin.x;
  const dy = world.y - drag.origin.y;
  if (Math.hypot(dx, dy) < 0.01) return;
  event.preventDefault();
  setSelectedGoal(drag.origin.x, drag.origin.y, Math.atan2(dy, dx));
}

function onDrivePointerUp(event) {
  const drag = state.goalDrag;
  if (!drag || drag.pointerId !== event.pointerId) return;
  onDrivePointerMove(event);
  state.goalDrag = null;
  try {
    els.driveMapCanvas.releasePointerCapture(event.pointerId);
  } catch (_error) {
    // The pointer may already have been released by the browser.
  }
}

function setSelectedGoal(x, y, yaw) {
  const normalized = normalizedYaw(yaw);
  setNumber(els.goalX, x);
  setNumber(els.goalY, y);
  setNumber(els.goalYaw, normalized);
  setNumber(els.goalYawRange, normalized);
  state.selectedGoal = { x: Number(x), y: Number(y), yaw: normalized };
  planPathToGoal();
  renderWaypoints();
}

function normalizedYaw(value) {
  let yaw = Number(value) || 0;
  while (yaw > Math.PI) yaw -= Math.PI * 2;
  while (yaw < -Math.PI) yaw += Math.PI * 2;
  return yaw;
}

async function loadState() {
  try {
    const data = await fetchJson("/api/state");
    applyState(data);
  } catch (error) {
    toast(error.message);
  }
}

function connectEvents() {
  const source = new EventSource("/api/events");
  source.onmessage = (event) => {
    applyState(JSON.parse(event.data));
  };
  source.onerror = () => {
    els.connectionText.textContent = "이벤트 연결 재시도 중";
  };
}

function applyState(data) {
  const previousMapUrl = state.data?.setup?.map?.imageUrl;
  const localSetup = state.data?.setup;
  const preserveLocalSetup = Boolean((state.setupDirty || state.setupDrag) && localSetup);
  state.data = preserveLocalSetup
    ? { ...data, setup: { ...localSetup, mapLibrary: data.setup?.mapLibrary || localSetup.mapLibrary } }
    : data;
  renderMapEditorLibrary(state.data.setup);
  fillSetupForm(state.data.setup);
  fillRuntime(state.data.runtime);
  fillConnectionStatus({
    runtime: state.data.runtime,
    connection: state.data.runtime.connection,
    network: state.data.setup.network,
  });
  if (state.data.setup.map.imageUrl && state.data.setup.map.imageUrl !== previousMapUrl) {
    loadMapImage(state.data.setup.map.imageUrl);
  }
}

function fillSetupForm(setup) {
  if (state.setupDirty || state.setupDrag) return;
  renderMapSelector(setup);
  setInputIfIdle(els.mapResolution, setup.map.resolution);
  setInputIfIdle(els.mapOriginX, setup.map.originX);
  setInputIfIdle(els.mapOriginY, setup.map.originY);
  setInputIfIdle(els.mapOriginYaw, setup.map.originYaw);
  setInputIfIdle(els.gridCellSize, setup.planner?.cellSize ?? 0.02);
  setInputIfIdle(els.plannerHardClearanceRange, setup.planner?.hardClearance ?? 0.05);
  setInputIfIdle(els.plannerHardClearance, setup.planner?.hardClearance ?? 0.05);
  setInputIfIdle(els.showGrid, setup.planner?.showGrid ?? true);
  setInputIfIdle(els.showInflation, setup.planner?.showInflation ?? true);
  setInputIfIdle(els.showLidarPoints, setup.planner?.showLidarPoints ?? true);
  setInputIfIdle(els.detectLidarObstacles, setup.planner?.detectLidarObstacles ?? true);
  setInputIfIdle(els.detectBlackWalls, setup.planner?.detectBlackWalls ?? true);
  renderRobotProfileControls(setup);
  renderOtherRobotList(setup);
  setInputIfIdle(els.activeRobotProfile, setup.activeRobot || "tb3_2");
  const activeProfile = normalizedRobotProfiles(setup)[setup.activeRobot || "tb3_2"] || {};
  const connection = activeProfile.connection || setup.network || {};
  setInputIfIdle(els.robotIp, connection.robotIp ?? "");
  setInputIfIdle(els.serverIp, setup.network?.serverIp ?? "");
  setInputIfIdle(els.rosDomainId, connection.rosDomainId ?? "");
  setInputIfIdle(els.rosLocalhostOnly, connection.rosLocalhostOnly ?? "");
  setInputIfIdle(els.robotSshHost, connection.robotSshHost ?? connection.robotIp ?? "");
  setInputIfIdle(els.robotSshUser, connection.robotSshUser ?? "");
  setInputIfIdle(els.robotSshPassword, connection.robotSshPassword ?? "");
  els.robotSshPassword.placeholder = setup.network?.robotSshPasswordConfigured ? "저장된 비밀번호 사용" : "";
  setInputIfIdle(els.initialX, setup.initialPose.x);
  setInputIfIdle(els.initialY, setup.initialPose.y);
  setInputIfIdle(els.initialYaw, setup.initialPose.yaw);
  setInputIfIdle(els.robotLength, setup.robot.length);
  setInputIfIdle(els.robotWidth, setup.robot.width);
  setInputIfIdle(els.accessoryFront, setup.accessory?.front ?? 0);
  setInputIfIdle(els.accessoryBack, setup.accessory?.back ?? 0);
  setInputIfIdle(els.accessoryLeft, setup.accessory?.left ?? 0);
  setInputIfIdle(els.accessoryRight, setup.accessory?.right ?? 0);
  setInputIfIdle(els.accessoryHeight, setup.accessory?.height ?? 0);
  setInputIfIdle(els.safetyMargin, setup.safety?.margin ?? 0.08);
  updateFootprintReadout(currentSetup());
  setInputIfIdle(els.objectWidth, setup.object.width);
  setInputIfIdle(els.objectHeight, setup.object.height);
  setInputIfIdle(els.objectInflation, setup.object.inflation);
  const fallback = setup.fallbackNavigation || {};
  setInputIfIdle(els.fallbackEnabled, fallback.enabled ?? true);
  setInputIfIdle(els.fallbackMaxLinear, fallback.maxLinear ?? 0.06);
  setInputIfIdle(els.fallbackMinLinear, fallback.minLinear ?? 0.02);
  setInputIfIdle(els.fallbackMaxAngular, fallback.maxAngular ?? 0.6);
  setInputIfIdle(els.fallbackLookahead, fallback.lookahead ?? 0.12);
  setInputIfIdle(els.fallbackSoftDistanceRange, fallback.softDistance ?? 0.10);
  setInputIfIdle(els.fallbackSoftDistance, fallback.softDistance ?? 0.10);
  setInputIfIdle(els.fallbackHardMargin, fallback.hardMargin ?? 0.03);
  setInputIfIdle(els.fallbackScanTimeoutRange, fallback.scanTimeout ?? 0.6);
  setInputIfIdle(els.fallbackScanTimeout, fallback.scanTimeout ?? 0.6);
  setInputIfIdle(els.fallbackOdomTimeoutRange, fallback.odomTimeout ?? 0.6);
  setInputIfIdle(els.fallbackOdomTimeout, fallback.odomTimeout ?? 0.6);
  setInputIfIdle(els.fallbackCollisionHorizon, fallback.collisionHorizon ?? 1.5);
  renderObstacleList(normalizeObstacles(setup.obstacles || []));
  const topicDefaults =
    normalizedRobotProfiles(setup)[setup.activeRobot || "tb3_2"]?.topics || DEFAULT_ROBOT_PROFILES.tb3_2.topics;
  setInputIfIdle(els.topicScan, setup.topics.scan || topicDefaults.scan);
  setInputIfIdle(els.topicPose, setup.topics.pose || topicDefaults.pose);
  setInputIfIdle(els.topicOdom, setup.topics.odom || topicDefaults.odom);
  setInputIfIdle(els.topicCamera, setup.topics.camera || topicDefaults.camera);
  setInputIfIdle(
    els.topicCompressedCamera,
    setup.topics.compressedCamera || topicDefaults.compressedCamera,
  );
  setInputIfIdle(els.topicInitialPose, setup.topics.initialPose || topicDefaults.initialPose);
  setInputIfIdle(els.topicGoalAction, setup.topics.goalAction || topicDefaults.goalAction);
  setInputIfIdle(els.topicRouteAction, setup.topics.routeAction || topicDefaults.routeAction);
  setInputIfIdle(els.topicGoalTopic, setup.topics.goalTopic || topicDefaults.goalTopic);
  setInputIfIdle(els.topicCmdVel, setup.topics.cmdVel || topicDefaults.cmdVel);
}

function mapLibraryEntries(setup) {
  const library = Array.isArray(setup?.mapLibrary) ? setup.mapLibrary : [];
  const entries = library.filter((entry) => entry && entry.id && entry.imageUrl);
  if (entries.length) return entries;
  return setup?.map?.imageUrl ? [setup.map] : [];
}

function renderMapSelector(setup) {
  const entries = mapLibraryEntries(setup);
  const selectedId = String(setup?.map?.id || entries.find((entry) => entry.imageUrl === setup?.map?.imageUrl)?.id || "");
  const signature = entries.map((entry) => `${entry.id}:${entry.name}:${entry.widthPixels}:${entry.heightPixels}`).join("|");
  if (els.mapSelect.dataset.signature !== signature) {
    els.mapSelect.replaceChildren();
    for (const entry of entries) {
      const option = document.createElement("option");
      option.value = entry.id;
      const widthCm = Math.round((Number(entry.widthPixels) || 0) * (Number(entry.resolution) || 0.01) * 100);
      const heightCm = Math.round((Number(entry.heightPixels) || 0) * (Number(entry.resolution) || 0.01) * 100);
      option.textContent = `${entry.name || "맵"} (${widthCm} x ${heightCm}cm)`;
      els.mapSelect.append(option);
    }
    els.mapSelect.dataset.signature = signature;
  }
  if (selectedId) els.mapSelect.value = selectedId;
}

function renderMapEditorLibrary(setup) {
  const entries = mapLibraryEntries(setup);
  const signature = entries.map((entry) => `${entry.id}:${entry.name}:${entry.imageUrl}`).join("|");
  const currentValue = els.mapEditorLoadSelect.value;
  if (els.mapEditorLoadSelect.dataset.signature !== signature) {
    els.mapEditorLoadSelect.replaceChildren();
    const placeholder = document.createElement("option");
    placeholder.value = "";
    placeholder.textContent = "맵 선택";
    els.mapEditorLoadSelect.append(placeholder);
    for (const entry of entries) {
      const option = document.createElement("option");
      option.value = entry.id;
      option.textContent = entry.name || "맵";
      els.mapEditorLoadSelect.append(option);
    }
    els.mapEditorLoadSelect.dataset.signature = signature;
  }
  const selectedId = entries.some((entry) => entry.id === currentValue)
    ? currentValue
    : state.mapEditor.loadedMapId;
  els.mapEditorLoadSelect.value = entries.some((entry) => entry.id === selectedId) ? selectedId : "";
  updateMapEditorDeleteButton();
}

function updateMapEditorDeleteButton() {
  const selected = mapLibraryEntries(state.data?.setup || {}).find(
    (entry) => entry.id === els.mapEditorLoadSelect.value,
  );
  els.mapEditorLoadButton.disabled = !selected;
  els.mapEditorDeleteButton.disabled = !selected || selected.id === "default-map";
}

async function selectSavedMap() {
  const id = els.mapSelect.value;
  if (!id || id === state.data?.setup?.map?.id) return;
  try {
    const result = await postJson("/api/maps/select", { id });
    state.setupDirty = false;
    if (result.state) applyState(result.state);
    planPathToGoal();
    toast("선택한 맵을 적용했습니다.");
  } catch (error) {
    renderMapSelector(state.data?.setup || {});
    toast(error.message);
  }
}

function fillRuntime(runtime) {
  const connected = runtime.rosConnected ? "ROS2 연결" : "프리뷰 모드";
  const version = runtime.appVersion ? ` · v${runtime.appVersion}` : "";
  els.connectionText.textContent = `${connected} · ${runtime.navStatus || "idle"}${version}`;
  state.cameraEnabled = runtime.cameraEnabled !== false;
  updateCameraUi(runtime);
  els.statusMode.textContent = runtime.mode || "-";
  const repeatStatus = runtime.routeRepeatEnabled
    ? ` · 반복 ${runtime.routeRepeatCycle || 1}회`
    : "";
  els.statusNav.textContent = `${runtime.navStatus || "-"}${repeatStatus}`;
  els.statusMessage.textContent = runtime.navMessage || "-";
  els.statusPose.textContent = formatPose(runtime.pose);
  els.statusGoal.textContent = formatRuntimeGoal(runtime);
  els.statusPath.textContent = state.plannedPath.status || "-";
  const lidarPointCount = runtime.lidarPointCount ?? runtime.lidarPoints?.length ?? 0;
  const dynamicObstacleCount = runtime.dynamicLidarObstacleCount ?? runtime.dynamicLidarObstacles?.length ?? 0;
  const lidarConnection = runtime.lidarConnection || "unknown";
  const lidarState = lidarConnection === "connected"
    ? "연결"
    : lidarConnection === "degraded"
      ? "저속 유예"
      : lidarConnection === "lost"
        ? "끊김"
        : "대기";
  els.statusSafety.textContent = runtime.fallbackActive
    ? `LiDAR ${lidarState} · scale ${runtime.fallbackSpeedScale ?? "-"} · clearance ${runtime.lidarMinClearance ?? "-"} m · recovery ${runtime.fallbackRecoveryPhase ?? "none"} · ${lidarPointCount} pts · 임시 ${dynamicObstacleCount} · scan ${runtime.scanAgeMs ?? "-"} ms`
    : `LiDAR ${lidarState} · ${lidarPointCount} pts · 임시 ${dynamicObstacleCount}`;
  updateRouteResumeUi(runtime);
}

function updateRouteResumeUi(runtime = state.data?.runtime || {}) {
  const checkpoint = runtime.routeCheckpoint || {};
  const route = Array.isArray(checkpoint.route) ? checkpoint.route : [];
  const nextIndex = Math.max(0, Math.trunc(Number(checkpoint.nextIndex) || 0));
  const nextWaypointNumber = Number(checkpoint.nextWaypointNumber);
  const completed = Array.isArray(checkpoint.completedWaypointNumbers)
    ? checkpoint.completedWaypointNumbers.map(Number).filter(Number.isFinite)
    : [];
  const navStatus = String(runtime.navStatus || "");
  const activeRobot = state.data?.setup?.activeRobot || "";
  const checkpointRobot = String(checkpoint.robotId || "");
  const activelyDriving = [
    "moving",
    "accepted",
    "sending_goal",
    "sending_route",
    "fallback_starting",
    "fallback_moving",
    "fallback_slow",
    "fallback_replanned",
    "fallback_lidar_coast",
  ].includes(navStatus) || navStatus.startsWith("fallback_recovery_");
  const available = Boolean(checkpoint.available)
    && (!checkpointRobot || checkpointRobot === activeRobot)
    && nextIndex < route.length
    && Number.isInteger(nextWaypointNumber)
    && nextWaypointNumber > 0;
  els.resumeRouteButton.disabled = !available || activelyDriving;
  els.resumeRouteButton.textContent = available
    ? `이어서 주행 (#${nextWaypointNumber}부터)`
    : "이어서 주행";
  if (!available) {
    els.routeResumeStatus.textContent = checkpoint.available && checkpointRobot !== activeRobot
      ? "다른 로봇의 저장 경로"
      : checkpoint.status === "completed"
      ? "저장 경로 완료"
      : "저장된 이어 주행 없음";
    return;
  }
  const lastCompleted = completed.length ? completed[completed.length - 1] : null;
  const prefix = lastCompleted ? `마지막 통과 #${lastCompleted}` : "통과 지점 없음";
  const reason = checkpoint.status === "interrupted" && checkpoint.interruptionReason
    ? ` · ${checkpoint.interruptionReason}`
    : "";
  els.routeResumeStatus.textContent = `${prefix} · 다음 #${nextWaypointNumber}${reason}`;
}

function updateCameraUi(runtime = state.data?.runtime || {}) {
  const enabled = runtime.cameraEnabled !== false;
  const streamMode = runtime.cameraStream === "raw" ? "raw" : "compressed";
  state.cameraEnabled = enabled;
  els.cameraToggleButton.textContent = enabled ? "카메라 끄기" : "카메라 켜기";
  els.cameraToggleButton.classList.toggle("active", enabled);
  els.cameraRawButton.classList.toggle("active", streamMode === "raw");
  els.cameraCompressedButton.classList.toggle("active", streamMode === "compressed");
  els.cameraRawButton.setAttribute("aria-pressed", String(streamMode === "raw"));
  els.cameraCompressedButton.setAttribute("aria-pressed", String(streamMode === "compressed"));
  const fps = runtime.cameraFps;
  els.cameraFpsBadge.textContent = enabled && Number.isFinite(fps) ? `FPS ${fps.toFixed(1)}` : "FPS -";
  els.cameraFpsBadge.classList.toggle("off", !enabled);
  els.cameraFrame.classList.toggle("camera-off", !enabled);
  els.cameraStamp.textContent = enabled ? runtime.lastCameraAt || "-" : "OFF";
  updateCameraTransport();
}

function updateCameraTransport(force = false) {
  const runtime = state.data?.runtime || {};
  const enabled = state.cameraEnabled !== false;
  const streamMode = runtime.cameraStream === "raw" ? "raw" : "compressed";
  const transport = enabled ? (streamMode === "compressed" ? "mjpeg" : "raw") : "off";
  if (!force && els.cameraFrame.dataset.cameraTransport === transport) return;

  els.cameraFrame.dataset.cameraTransport = transport;
  els.cameraFrame.dataset.cameraState = enabled ? "on" : "off";
  els.cameraFrame.src = transport === "mjpeg"
    ? `/api/camera/stream?t=${Date.now()}`
    : `/api/camera/frame?t=${Date.now()}`;
}

function formatRuntimeGoal(runtime) {
  const route = Array.isArray(runtime.route) ? runtime.route : [];
  if (!runtime.goal && runtime.routeRepeatEnabled && route.length) {
    return `휴식 · ${route.length}/${route.length} · 재시작 ${runtime.routeRepeatResumeAt || "대기 중"}`;
  }
  if (!runtime.goal) return "-";
  if (route.length > 1) {
    const index = Math.min(route.length, Math.max(1, Number(runtime.routeIndex || 0) + 1));
    return `${index}/${route.length} · ${formatPose(runtime.goal)}`;
  }
  return formatPose(runtime.goal);
}

function fillConnectionStatus(payload) {
  const runtime = payload?.runtime || state.data?.runtime || {};
  const connection = payload?.connection || runtime.connection || {};
  const network = payload?.network || state.data?.setup?.network || {};
  const serverIps = connection.serverIps?.length ? connection.serverIps.join(", ") : network.serverIp || "-";
  els.connDomain.textContent = connection.rosDomainId || network.rosDomainId || "-";
  els.connLocalhost.textContent = connection.rosLocalhostOnly || network.rosLocalhostOnly || "-";
  els.connServerIps.textContent = serverIps;
  els.connScan.textContent = runtime.lastScanAt ? `수신 ${runtime.lastScanAt}` : "대기";
  els.connPose.textContent = runtime.lastPoseAt || runtime.pose?.stamp ? `수신 ${runtime.lastPoseAt || runtime.pose.stamp}` : "대기";
  if (connection.routeActionAvailable) {
    els.connNav2.textContent = "Goal + Route OK";
  } else if (connection.actionAvailable) {
    els.connNav2.textContent = "Goal OK · 경유지 순차 실행";
  } else {
    els.connNav2.textContent = "Action 대기";
  }
  const opencr = runtime.opencr || {};
  if (opencr.ready) {
    els.connOpencr.textContent = `정상 · ${opencr.port || "-"}`;
  } else if (opencr.connected) {
    els.connOpencr.textContent = `연결됨 · base 대기 (${opencr.port || "-"})`;
  } else if (opencr.state && opencr.state !== "unknown") {
    els.connOpencr.textContent = `점검 필요 · ${opencr.state}`;
  } else {
    els.connOpencr.textContent = "미확인";
  }
}

async function refreshConnectionStatus() {
  try {
    const payload = await fetchJson("/api/connection");
    fillConnectionStatus(payload);
    toast("연결 상태를 갱신했습니다.");
  } catch (error) {
    toast(error.message);
  }
}

async function toggleCamera() {
  const enabled = !state.cameraEnabled;
  els.cameraToggleButton.disabled = true;
  try {
    const payload = await postJson("/api/camera_control", { enabled });
    const runtime = payload.runtime || { ...(state.data?.runtime || {}), cameraEnabled: enabled };
    if (state.data?.runtime) {
      state.data.runtime = { ...state.data.runtime, ...runtime };
    }
    updateCameraUi(runtime);
    toast(enabled ? "카메라를 켰습니다." : "카메라를 껐습니다.");
  } catch (error) {
    toast(error.message);
  } finally {
    els.cameraToggleButton.disabled = false;
  }
}

async function setCameraStreamMode(mode) {
  if (mode !== "raw" && mode !== "compressed") return;
  els.cameraRawButton.disabled = true;
  els.cameraCompressedButton.disabled = true;
  try {
    const payload = await postJson("/api/camera_stream", { mode });
    const runtime = payload.runtime || { ...(state.data?.runtime || {}), cameraStream: mode };
    if (state.data?.runtime) {
      state.data.runtime = { ...state.data.runtime, ...runtime };
    }
    updateCameraUi(runtime);
    toast(mode === "raw" ? "Raw 카메라 스트림으로 전환했습니다." : "Compressed 카메라 스트림으로 전환했습니다.");
  } catch (error) {
    toast(error.message);
  } finally {
    els.cameraRawButton.disabled = false;
    els.cameraCompressedButton.disabled = false;
  }
}

async function discoverRobots() {
  els.discoverRobotButton.disabled = true;
  els.discoveryResults.innerHTML = `<div class="discovery-card"><p>ROS 그래프와 같은 네트워크의 SSH 장비를 검색 중...</p></div>`;
  try {
    const payload = await fetchJson("/api/discover");
    fillConnectionStatus(payload);
    const addedProfiles = addDiscoveredRobotProfiles(payload.candidates || []);
    if (addedProfiles.length) {
      await saveSetup({
        successMessage: `${addedProfiles.join(", ")} 로봇 탭을 추가했습니다.`,
      });
    }
    renderDiscoveryResults(payload);
    if (!addedProfiles.length) showRobotDiscoveryNotice(payload);
  } catch (error) {
    els.discoveryResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
  } finally {
    els.discoverRobotButton.disabled = false;
  }
}

async function resetTopicsFromRobot() {
  const activeRobot = els.activeRobotProfile.value || state.data?.setup?.activeRobot || "tb3_2";
  els.resetTopicsButton.disabled = true;
  els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>활성 로봇의 ROS2 토픽을 검색 중...</p></div>`;
  try {
    const payload = await postJson("/api/topics/reset", { activeRobot });
    if (payload.rosReload?.ok === false) {
      throw new Error(payload.rosReload.message || "ROS 브릿지 재로딩에 실패했습니다.");
    }
    state.setupDirty = false;
    if (payload.state) applyState(payload.state);
    const found = Object.entries(payload.topics || {})
      .map(([key, value]) => `${key}: ${value}`)
      .join("\n");
    els.robotCheckResults.innerHTML = `
      <div class="discovery-card">
        <strong>토픽 초기화 완료</strong>
        <p>${escapeHtml(found || "발견된 토픽 없음")}</p>
      </div>`;
    toast("검색된 토픽을 적용하고 ROS 브릿지를 갱신했습니다.");
  } catch (error) {
    els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
    toast(error.message);
  } finally {
    els.resetTopicsButton.disabled = false;
  }
}

function showRobotDiscoveryNotice(payload) {
  const candidates = payload.candidates || [];
  const profiles = normalizedRobotProfiles(state.data?.setup || {});
  const existing = candidates.filter((candidate) => profileIdForNamespace(candidate?.namespace, profiles));
  if (existing.length) {
    els.robotDiscoveryNoticeSummary.textContent = `ROS 후보 ${candidates.length}대 중 ${existing.length}대는 이미 로봇 탭에 등록되어 있습니다.`;
  } else if (candidates.length) {
    els.robotDiscoveryNoticeSummary.textContent = "ROS 후보는 보였지만 /scan, /odom, /cmd_vel 신호가 부족해 자동 등록하지 않았습니다.";
  } else {
    els.robotDiscoveryNoticeSummary.textContent = "현재 ROS 그래프에서 새 TurtleBot 후보를 찾지 못했습니다.";
  }
  if (!els.robotDiscoveryNotice.open) els.robotDiscoveryNotice.showModal();
}

function closeRobotDiscoveryNotice() {
  if (els.robotDiscoveryNotice.open) els.robotDiscoveryNotice.close();
}

async function persistSelectedRobotBeforeControl() {
  if (!state.setupDirty) return;
  await saveSetup({ successMessage: "선택 로봇 설정을 저장했습니다." });
}

async function checkRobotHardware() {
  els.robotHardwareCheckButton.disabled = true;
  els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>OpenCR와 base 드라이버를 SSH로 확인하는 중...</p></div>`;
  try {
    await persistSelectedRobotBeforeControl();
    const payload = await postJson("/api/robot_hardware_check", {});
    const hardware = payload.hardware || {};
    if (state.data?.runtime) {
      state.data.runtime.opencr = hardware;
      fillConnectionStatus({ runtime: state.data.runtime });
    }
    const rows = [
      ["OpenCR", hardware.state || "unknown"],
      ["포트", hardware.port || "-"],
      ["base service", hardware.baseService || "-"],
      ["turtlebot3_node", hardware.turtlebotNode ? "present" : "missing"],
      ["odom publishers", hardware.odomPublishers ?? 0],
      ["cmd_vel subscribers", hardware.cmdVelSubscribers ?? 0],
    ].map(([label, value]) => `<p><strong>${escapeHtml(label)}</strong>: ${escapeHtml(value)}</p>`).join("");
    const stateText = hardware.ready
      ? "OpenCR와 TurtleBot base 연결이 정상입니다."
      : hardware.connected
        ? "OpenCR는 연결됐지만 TurtleBot base가 준비되지 않았습니다. 브링업 상태를 확인하세요."
        : "검증 가능한 OpenCR 연결이 아닙니다. Arduino 또는 알 수 없는 ACM 장치를 사용하지 않습니다.";
    els.robotCheckResults.innerHTML = `
      <div class="discovery-card">
        <strong>${escapeHtml(stateText)}</strong>
        <p>${escapeHtml(hardware.detail || "-")}</p>
        ${rows}
      </div>`;
    els.diagnosticReport.classList.add("visible");
    els.diagnosticReport.value = [
      `$ ${payload.command || "robot hardware check"}`,
      `returncode: ${payload.returncode ?? "-"}`,
      "",
      payload.stdout || "",
      payload.stderr ? `\n[stderr]\n${payload.stderr}` : "",
    ].filter(Boolean).join("\n");
    toast(stateText);
  } catch (error) {
    els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
    toast(error.message);
  } finally {
    els.robotHardwareCheckButton.disabled = false;
  }
}

async function checkRobot() {
  els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>체크 중...</p></div>`;
  try {
    await persistSelectedRobotBeforeControl();
    const payload = await fetchJson("/api/robot_check");
    fillConnectionStatus(payload);
    renderRobotCheckResults(payload);
  } catch (error) {
    els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
  }
}

async function robotBringup() {
  els.robotBringupButton.disabled = true;
  els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>SSH로 로봇 브링업 실행 중...</p></div>`;
  els.diagnosticReport.classList.add("visible");
  els.diagnosticReport.value = "로봇 SSH 브링업 실행 중...";
  try {
    await persistSelectedRobotBeforeControl();
    const payload = await postJson("/api/robot_bringup", {});
    const mapLine = payload.map?.ok
      ? `map: ${payload.map.source || "-"} (${payload.map.width}x${payload.map.height}, ${payload.map.resolution} m/px)`
      : `map: export failed (${payload.map?.error || "-"})`;
    const output = [
      `$ ${payload.command || "robot bringup"}`,
      `returncode: ${payload.returncode ?? "-"}`,
      mapLine,
      "",
      payload.stdout || "",
      payload.stderr ? `\n[stderr]\n${payload.stderr}` : "",
    ]
      .filter(Boolean)
      .join("\n");
    els.diagnosticReport.value = output;
    if (payload.check) {
      fillConnectionStatus(payload.check);
      renderRobotCheckResults(payload.check);
    } else {
      els.robotCheckResults.innerHTML = `
        <div class="discovery-card">
          <strong>${payload.ok ? "브링업 시작 요청 완료" : "브링업 실패"}</strong>
          <p>${escapeHtml(payload.stderr || payload.stdout || "-")}</p>
        </div>
      `;
    }
    toast(payload.ok ? "로봇 브링업 시작 요청을 보냈습니다." : "로봇 브링업 실패. 출력 확인 필요.");
  } catch (error) {
    els.diagnosticReport.value = error.message;
    els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
    toast(error.message);
  } finally {
    els.robotBringupButton.disabled = false;
  }
}

async function stopRobotBringup() {
  if (!window.confirm("로봇의 대시보드 브링업과 Nav2, 카메라를 종료할까요?")) return;
  els.robotBringupStopButton.disabled = true;
  els.diagnosticReport.classList.add("visible");
  els.diagnosticReport.value = "SSH로 로봇 브링업 종료 중...";
  try {
    await persistSelectedRobotBeforeControl();
    const payload = await postJson("/api/robot_bringup_stop", {});
    const output = [
      `$ ${payload.command || "robot bringup stop"}`,
      `returncode: ${payload.returncode ?? "-"}`,
      "",
      payload.stdout || "",
      payload.stderr ? `\n[stderr]\n${payload.stderr}` : "",
    ]
      .filter(Boolean)
      .join("\n");
    els.diagnosticReport.value = output;
    els.robotCheckResults.innerHTML = `
      <div class="discovery-card">
        <strong>${payload.ok ? "브링업 종료 요청 완료" : "브링업 종료 실패"}</strong>
        <p>${escapeHtml(payload.stderr || payload.stdout || "-")}</p>
      </div>
    `;
    toast(payload.ok ? "로봇 브링업 종료 요청을 보냈습니다." : "브링업 종료 실패. 출력 확인 필요.");
  } catch (error) {
    els.diagnosticReport.value = error.message;
    els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
    toast(error.message);
  } finally {
    els.robotBringupStopButton.disabled = false;
  }
}

async function robotSshCheck(detailed = false) {
  els.robotSshCheckButton.disabled = true;
  els.robotSshDetailButton.disabled = true;
  els.diagnosticReport.classList.add("visible");
  els.diagnosticReport.value = detailed ? "로봇 SSH 상세점검 중..." : "로봇 SSH 빠른점검 중...";
  els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>SSH로 로봇 내부 상태 확인 중...</p></div>`;
  try {
    await persistSelectedRobotBeforeControl();
    const payload = await postJson("/api/robot_ssh_check", { detailed });
    const output = [
      `$ ${payload.command || "robot ssh diagnostics"}`,
      `mode: ${payload.detailed ? "detailed" : "quick"}`,
      `returncode: ${payload.returncode ?? "-"}`,
      "",
      payload.stdout || "",
      payload.stderr ? `\n[stderr]\n${payload.stderr}` : "",
    ]
      .filter(Boolean)
      .join("\n");
    els.diagnosticReport.value = output;
    els.robotCheckResults.innerHTML = `
      <div class="discovery-card">
        <strong>${payload.ok ? (payload.detailed ? "SSH 상세점검 완료" : "SSH 빠른점검 완료") : "SSH 점검 실패"}</strong>
        <p>${escapeHtml(payload.stderr || "결과는 진단 박스에서 확인하세요.")}</p>
      </div>
    `;
    toast(payload.ok ? "로봇 SSH 점검을 완료했습니다." : "SSH 점검 실패. 출력 확인 필요.");
  } catch (error) {
    els.diagnosticReport.value = error.message;
    els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
    toast(error.message);
  } finally {
    els.robotSshCheckButton.disabled = false;
    els.robotSshDetailButton.disabled = false;
  }
}

async function manualDriveCheck() {
  els.manualDriveCheckButton.disabled = true;
  els.diagnosticReport.classList.add("visible");
  els.diagnosticReport.value = "로봇에서 cmd_vel 0속도 수신을 확인하는 중...";
  els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>안전한 0속도 전달점검 중...</p></div>`;
  try {
    const response = await fetch("/api/manual_drive_check", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{}",
    });
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.error || `/api/manual_drive_check ${response.status}`);
    const output = [
      "# cmd_vel delivery check",
      `ok: ${Boolean(payload.ok)}`,
      `checkedAt: ${payload.checkedAt || "-"}`,
      `topic: ${payload.topic || "-"}`,
      `messageType: ${payload.messageType || "-"}`,
      `stampSource: ${payload.clock?.cmdVelStampSource || "-"}`,
      `clockSkewMs: ${payload.clock?.cmdVelClockSkewMs ?? "-"}`,
      `returncode: ${payload.returncode ?? "-"}`,
      "",
      payload.stdout || "",
      payload.error ? `\n[error]\n${payload.error}` : "",
      payload.stderr ? `\n[stderr]\n${payload.stderr}` : "",
    ]
      .filter(Boolean)
      .join("\n");
    els.diagnosticReport.value = output;
    els.robotCheckResults.innerHTML = `
      <div class="discovery-card">
        <strong>${payload.ok ? "cmd_vel 전달 확인" : "cmd_vel 전달 실패"}</strong>
        <p>${escapeHtml(payload.error || `${payload.topic} · ${payload.messageType} · ${payload.clock?.cmdVelStampSource || "-"}`)}</p>
      </div>
    `;
    toast(
      payload.ok
        ? "로봇이 대시보드의 0속도 cmd_vel을 수신했습니다."
        : "cmd_vel 전달을 확인하지 못했습니다. 진단 출력을 확인하세요.",
    );
  } catch (error) {
    els.diagnosticReport.value = error.message;
    els.robotCheckResults.innerHTML = `<div class="discovery-card"><p>${escapeHtml(error.message)}</p></div>`;
    toast(error.message);
  } finally {
    els.manualDriveCheckButton.disabled = false;
  }
}

async function copyDiagnostics() {
  els.diagnosticReport.classList.add("visible");
  els.diagnosticReport.value = "진단 리포트 생성 중...";
  try {
    const payload = await fetchJson("/api/diagnostics");
    const report = payload.report || JSON.stringify(payload, null, 2);
    els.diagnosticReport.value = report;
    els.diagnosticReport.focus();
    els.diagnosticReport.select();
    try {
      await navigator.clipboard.writeText(report);
      toast("진단 리포트를 복사했습니다.");
    } catch {
      toast("진단 리포트를 선택했습니다. Ctrl+C로 복사하세요.");
    }
  } catch (error) {
    els.diagnosticReport.value = error.message;
    toast(error.message);
  }
}

async function refreshRunLogSummary() {
  if (!els.runLogSummary) return null;
  try {
    const payload = await fetchJson("/api/run_logs");
    els.runLogSummary.textContent = `${payload.sessionId} · ${payload.eventCount}개 이벤트 · ${formatBytes(payload.fileBytes)}`;
    return payload;
  } catch (error) {
    els.runLogSummary.textContent = `로그 조회 실패: ${error.message}`;
    return null;
  }
}

async function copyRunLogs() {
  els.copyRunLogsButton.disabled = true;
  try {
    const payload = await fetchJson("/api/run_logs");
    await writeClipboardText(payload.report || "");
    els.runLogSummary.textContent = `${payload.sessionId} · ${payload.eventCount}개 이벤트 · ${formatBytes(payload.fileBytes)}`;
    toast("주행 로그를 복사했습니다.");
  } catch (error) {
    toast(`로그 복사 실패: ${error.message}`);
  } finally {
    els.copyRunLogsButton.disabled = false;
  }
}

async function writeClipboardText(text) {
  if (navigator.clipboard?.writeText) {
    try {
      await Promise.race([
        navigator.clipboard.writeText(text),
        new Promise((_, reject) => setTimeout(() => reject(new Error("clipboard timeout")), 1000)),
      ]);
      return;
    } catch {
      // Continue with the selection-based fallback used by older browser shells.
    }
  }
  const textarea = document.createElement("textarea");
  textarea.value = text;
  textarea.setAttribute("readonly", "");
  textarea.style.position = "fixed";
  textarea.style.left = "-9999px";
  document.body.appendChild(textarea);
  textarea.select();
  const copied = document.execCommand("copy");
  textarea.remove();
  if (!copied) throw new Error("브라우저가 클립보드 복사를 허용하지 않았습니다.");
}

async function downloadRunLogs() {
  els.downloadRunLogsButton.disabled = true;
  try {
    const response = await fetch("/api/run_logs/download");
    if (!response.ok) throw new Error(`/api/run_logs/download ${response.status}`);
    const blob = await response.blob();
    const disposition = response.headers.get("Content-Disposition") || "";
    const filename = disposition.match(/filename="?([^";]+)"?/i)?.[1] || "turtlebot_run_log.md";
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
    await refreshRunLogSummary();
    toast("주행 로그 파일을 저장했습니다.");
  } catch (error) {
    toast(`로그 저장 실패: ${error.message}`);
  } finally {
    els.downloadRunLogsButton.disabled = false;
  }
}

async function clearRunLogs() {
  els.clearRunLogsButton.disabled = true;
  try {
    const payload = await postJson("/api/run_logs/clear", {});
    els.runLogSummary.textContent = `${payload.sessionId} · ${payload.eventCount}개 이벤트 · ${formatBytes(payload.fileBytes)}`;
    toast("새 주행 로그 세션을 시작했습니다.");
  } catch (error) {
    toast(`로그 초기화 실패: ${error.message}`);
  } finally {
    els.clearRunLogsButton.disabled = false;
  }
}

function formatBytes(bytes) {
  const value = Number(bytes) || 0;
  if (value < 1024) return `${value} B`;
  if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KiB`;
  return `${(value / (1024 * 1024)).toFixed(2)} MiB`;
}

function renderRobotCheckResults(payload) {
  const checks = payload.checks || [];
  const advice = payload.advice || [];
  const rows = checks
    .map((check) => {
      const level = check.level || (check.ok ? "ok" : "fail");
      const icon = level === "ok" ? "✓" : level === "warn" ? "!" : "×";
      return `
        <div class="check-row ${escapeHtml(level)}">
          <strong>${icon}</strong>
          <div>
            <strong>${escapeHtml(check.label || check.id || "-")}</strong>
            <p>${escapeHtml(check.detail || "-")}</p>
          </div>
        </div>
      `;
    })
    .join("");
  const adviceHtml = advice.length
    ? `<p>${advice.map((item) => escapeHtml(item)).join("<br>")}</p>`
    : "";
  els.robotCheckResults.innerHTML = `
    <div class="discovery-card">
      <strong>${escapeHtml(payload.summary || "로봇 체크 완료")}</strong>
      <div class="check-list">${rows}</div>
      ${adviceHtml}
    </div>
  `;
}

function renderDiscoveryResults(payload) {
  const candidates = payload.candidates || [];
  const topicCount = payload.topics?.length || 0;
  const nodeCount = payload.nodes?.length || 0;
  const networkDiscovery = payload.networkDiscovery || {};
  const networkHosts = networkDiscovery.hosts || [];
  const networkSummary = networkDiscovery.subnet
    ? `${networkDiscovery.subnet} · SSH 응답 ${networkHosts.length}대`
    : networkDiscovery.message || "네트워크 대역을 확인하지 못했습니다.";
  const networkDetail = networkHosts.length
    ? networkHosts
      .map((host) => `${host.ip}${host.configured ? " (현재 설정)" : ""}`)
      .join(", ")
    : "SSH 응답 장비 없음";
  const networkCard = `
    <div class="discovery-card">
      <strong>네트워크 장비</strong>
      <p>${escapeHtml(networkSummary)}</p>
      <p>${escapeHtml(networkDetail)}</p>
      <p>SSH 응답만으로는 로봇을 추가하지 않습니다. ROS 토픽이 확인된 장비만 로봇 탭으로 등록됩니다.</p>
    </div>
  `;
  if (candidates.length === 0) {
    els.discoveryResults.innerHTML = `
      <div class="discovery-card">
        <strong>후보 없음</strong>
        <p>${escapeHtml(payload.message || "TurtleBot 후보를 찾지 못했습니다.")}</p>
        <p>nodes ${nodeCount} · topics ${topicCount}</p>
      </div>${networkCard}
    `;
    return;
  }
  els.discoveryResults.innerHTML = candidates
    .map((candidate, index) => {
      const checks = Object.entries(candidate.checks || {})
        .map(([key, ok]) => `${ok ? "✓" : "·"} ${key}`)
        .join(" ");
      const topics = (candidate.matchedTopics || []).join(", ") || "-";
      const missing = (candidate.missing || []).join(", ") || "-";
      return `
        <div class="discovery-card">
          <strong>${escapeHtml(candidate.name || "TurtleBot 후보")} · ${candidate.score || 0}%</strong>
          <p>${escapeHtml(checks)}</p>
          <p>topics: ${escapeHtml(topics)}</p>
          <p>missing: ${escapeHtml(missing)}</p>
          <button type="button" data-apply-candidate="${index}">이 로봇 선택</button>
        </div>
      `;
    })
    .join("") + networkCard;
  els.discoveryResults.querySelectorAll("[data-apply-candidate]").forEach((button) => {
    button.addEventListener("click", async () => {
      const candidate = candidates[Number(button.dataset.applyCandidate)];
      await applyDiscoveredCandidate(candidate);
    });
  });
}

function addDiscoveredRobotProfiles(candidates) {
  if (!state.data?.setup) return [];
  const profiles = normalizedRobotProfiles(state.data.setup);
  const added = [];
  for (const candidate of candidates) {
    const namespace = String(candidate?.namespace || "/").replace(/\/$/, "") || "/";
    const checks = candidate?.checks || {};
    const hasRobotSignals = Boolean(checks.scan || checks.odom || checks.cmdVel);
    if (!hasRobotSignals || profileIdForNamespace(namespace, profiles)) continue;
    const profileId = profileIdForDiscoveredNamespace(namespace, profiles);
    const robotNumber = profileId.match(/^tb3_(\d+)$/)?.[1];
    profiles[profileId] = {
      label: robotNumber ? `TurtleBot ${robotNumber}` : `TurtleBot ${namespace.replace(/^\//, "")}`,
      namespace,
      source: "discovered",
      body: { length: 0.18, width: 0.14 },
      mapPose: { enabled: false, x: null, y: null, yaw: 0 },
      topics: {
        ...topicsForNamespace(namespace, "color"),
        ...(candidate.recommendedTopics || {}),
      },
    };
    added.push(profiles[profileId].label);
  }
  if (added.length) {
    state.data.setup.robotProfiles = profiles;
    renderRobotProfileControls({ ...state.data.setup, robotProfiles: profiles });
    renderOtherRobotList({ ...state.data.setup, robotProfiles: profiles });
    markSetupDirty();
  }
  return added;
}

async function applyDiscoveredCandidate(candidate) {
  const topics = candidate?.recommendedTopics || {};
  const profileId = profileIdForNamespace(candidate?.namespace);
  if (profileId) {
    els.activeRobotProfile.value = profileId;
  }
  if (topics.scan) els.topicScan.value = topics.scan;
  if (topics.odom) els.topicOdom.value = topics.odom;
  if (topics.pose) els.topicPose.value = topics.pose;
  if (topics.camera) els.topicCamera.value = topics.camera;
  if (topics.compressedCamera) els.topicCompressedCamera.value = topics.compressedCamera;
  if (topics.initialPose) els.topicInitialPose.value = topics.initialPose;
  if (topics.goalTopic) els.topicGoalTopic.value = topics.goalTopic;
  if (topics.cmdVel) els.topicCmdVel.value = topics.cmdVel;
  if (topics.goalAction) els.topicGoalAction.value = topics.goalAction;
  if (topics.routeAction) els.topicRouteAction.value = topics.routeAction;
  renderRobotProfileControls({ ...currentSetup(), activeRobot: els.activeRobotProfile.value });
  markSetupDirty();
  toast("검색된 토픽을 저장하고 ROS 브릿지를 갱신하는 중...");
  try {
    await saveSetup({ successMessage: "검색된 토픽을 저장했고 ROS 브릿지를 갱신했습니다." });
    fillConnectionStatus(await fetchJson("/api/connection"));
  } catch (error) {
    toast(error.message);
  }
}

function profileIdForNamespace(namespace, profiles = normalizedRobotProfiles(state.data?.setup || {})) {
  const normalizedNamespace = String(namespace || "/").replace(/\/$/, "") || "/";
  for (const [id, profile] of Object.entries(profiles)) {
    const profileNamespace = String(profile.namespace || `/${id}`).replace(/\/$/, "") || "/";
    if (profileNamespace === normalizedNamespace) return id;
  }
  return "";
}

function profileIdForDiscoveredNamespace(namespace, profiles) {
  const stem = String(namespace || "")
    .replace(/^\/+|\/+$/g, "")
    .replace(/[^a-zA-Z0-9_-]+/g, "_") || "robot";
  if (!profiles[stem]) return stem;
  let index = 2;
  while (profiles[`${stem}_${index}`]) index += 1;
  return `${stem}_${index}`;
}

function renderRobotProfileControls(setup) {
  const profiles = normalizedRobotProfiles(setup);
  const activeRobot = setup.activeRobot || "tb3_2";
  const options = Object.entries(profiles)
    .map(([id, profile]) => {
      const selected = id === activeRobot ? " selected" : "";
      return `<option value="${escapeHtml(id)}"${selected}>${escapeHtml(profile.label)} (${escapeHtml(profile.namespace)})</option>`;
    })
    .join("");
  if (els.activeRobotProfile.innerHTML !== options) {
    els.activeRobotProfile.innerHTML = options;
  }
}

function renderOtherRobotList(setup) {
  if (!els.otherRobotList) return;
  const activeRobot = setup.activeRobot || "tb3_2";
  const profiles = Object.entries(normalizedRobotProfiles(setup))
    .filter(([id]) => id !== activeRobot);
  if (profiles.length === 0) {
    els.otherRobotList.innerHTML = `<div class="robot-pose-row empty"><p>탐색된 다른 로봇이 없습니다.</p></div>`;
    return;
  }
  els.otherRobotList.innerHTML = profiles.map(([id, profile]) => {
    const pose = profile.mapPose;
    const body = profile.body;
    return `
      <div class="robot-pose-row" data-other-robot="${escapeHtml(id)}">
        <strong>${escapeHtml(profile.label)} <span>${escapeHtml(profile.namespace)}</span></strong>
        <label class="check-row"><input type="checkbox" data-robot-field="enabled" ${pose.enabled ? "checked" : ""} />회피</label>
        <label>X<input type="number" step="0.01" data-robot-field="x" value="${pose.x ?? ""}" /></label>
        <label>Y<input type="number" step="0.01" data-robot-field="y" value="${pose.y ?? ""}" /></label>
        <label>방향<input type="number" step="0.01" data-robot-field="yaw" value="${round(pose.yaw)}" /></label>
        <label>가로 m<input type="number" step="0.01" min="0.05" data-robot-field="length" value="${round(body.length)}" /></label>
        <label>세로 m<input type="number" step="0.01" min="0.05" data-robot-field="width" value="${round(body.width)}" /></label>
      </div>`;
  }).join("");
  els.otherRobotList.querySelectorAll("[data-robot-field]").forEach((input) => {
    input.addEventListener("change", () => updateOtherRobotProfile(input.closest("[data-other-robot]")));
  });
}

function updateOtherRobotProfile(row) {
  if (!row || !state.data?.setup) return;
  const profileId = row.dataset.otherRobot;
  const profiles = normalizedRobotProfiles(state.data.setup);
  const profile = profiles[profileId];
  if (!profile) return;
  const value = (field) => row.querySelector(`[data-robot-field="${field}"]`);
  const x = optionalNumberValue(value("x"));
  const y = optionalNumberValue(value("y"));
  const mapPose = normalizedRobotMapPose({
    enabled: value("enabled").checked,
    x,
    y,
    yaw: numberValue(value("yaw")),
  });
  profiles[profileId] = {
    ...profile,
    body: normalizedRobotBody({
      length: numberValue(value("length")),
      width: numberValue(value("width")),
    }),
    mapPose,
  };
  state.data.setup.robotProfiles = profiles;
  markSetupDirty();
}

function applyRobotProfile(profileId) {
  const profiles = normalizedRobotProfiles(state.data?.setup || {});
  const profile = profiles[profileId];
  if (!profile) return;
  const topics = profile.topics || {};
  const connection = profile.connection || {};
  els.activeRobotProfile.value = profileId;
  els.robotIp.value = connection.robotIp || "";
  els.rosDomainId.value = connection.rosDomainId || "";
  els.rosLocalhostOnly.value = connection.rosLocalhostOnly || "0";
  els.robotSshHost.value = connection.robotSshHost || connection.robotIp || "";
  els.robotSshUser.value = connection.robotSshUser || "";
  els.robotSshPassword.value = "";
  els.robotSshPassword.placeholder = connection.robotSshPasswordConfigured ? "Saved password will be used" : "";
  if (topics.scan) els.topicScan.value = topics.scan;
  if (topics.odom) els.topicOdom.value = topics.odom;
  if (topics.pose) els.topicPose.value = topics.pose;
  if (topics.camera) els.topicCamera.value = topics.camera;
  if (topics.compressedCamera) els.topicCompressedCamera.value = topics.compressedCamera;
  if (topics.initialPose) els.topicInitialPose.value = topics.initialPose;
  if (topics.goalTopic) els.topicGoalTopic.value = topics.goalTopic;
  if (topics.cmdVel) els.topicCmdVel.value = topics.cmdVel;
  if (topics.goalAction) els.topicGoalAction.value = topics.goalAction;
  if (topics.routeAction) els.topicRouteAction.value = topics.routeAction;
  renderRobotProfileControls({ ...currentSetup(), activeRobot: profileId });
  renderOtherRobotList({ ...currentSetup(), activeRobot: profileId });
  markSetupDirty();
  toast(`${profile.label} 토픽을 적용했습니다. 저장하면 연결 대상이 바뀝니다.`);
}

function addRobotProfile() {
  if (!state.data?.setup) return;
  const profiles = normalizedRobotProfiles(state.data.setup);
  let index = 1;
  while (profiles[`tb3_${index}`]) index += 1;
  const usedDomains = new Set(
    Object.values(profiles).map((profile) => Number(profile.connection?.rosDomainId)).filter(Number.isFinite),
  );
  let domain = 1;
  while (usedDomains.has(domain)) domain += 1;
  const id = `tb3_${index}`;
  profiles[id] = {
    label: `TurtleBot ${index}`,
    namespace: "/",
    source: "manual",
    body: { length: 0.18, width: 0.14 },
    mapPose: { enabled: false, x: null, y: null, yaw: 0 },
    connection: {
      robotIp: "",
      rosDomainId: String(domain),
      rosLocalhostOnly: "0",
      robotSshHost: "",
      robotSshUser: "",
      robotSshPassword: "",
    },
    topics: topicsForNamespace("/", "plain"),
  };
  state.data.setup.robotProfiles = profiles;
  state.data.setup.activeRobot = id;
  renderRobotProfileControls(state.data.setup);
  applyRobotProfile(id);
  markSetupDirty();
  toast("새 로봇을 추가했습니다. IP, SSH, ROS Domain을 입력한 뒤 저장하세요.");
}

function deleteActiveRobotProfile() {
  if (!state.data?.setup) return;
  const setup = currentSetup();
  const profiles = { ...setup.robotProfiles };
  const activeRobot = setup.activeRobot;
  const profileIds = Object.keys(profiles);
  if (profileIds.length <= 1) {
    toast("마지막 로봇 프로필은 삭제할 수 없습니다.");
    return;
  }
  const label = profiles[activeRobot]?.label || activeRobot;
  if (!window.confirm(`${label} 프로필을 삭제할까요?`)) return;
  delete profiles[activeRobot];
  const nextRobot = Object.keys(profiles)[0];
  state.data.setup = {
    ...state.data.setup,
    ...setup,
    activeRobot: nextRobot,
    robotProfiles: profiles,
  };
  renderRobotProfileControls(state.data.setup);
  applyRobotProfile(nextRobot);
  markSetupDirty();
  toast(`${label} 프로필을 삭제했습니다. 저장하면 적용됩니다.`);
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function setInputIfIdle(input, value) {
  if (document.activeElement === input) return;
  if (value === undefined || value === null) return;
  if (input.type === "checkbox") {
    input.checked = Boolean(value);
    return;
  }
  if (input.type === "number") {
    const next = Number(value);
    if (Number.isFinite(next) && (input.value === "" || Number(input.value) !== next)) {
      input.value = String(round(next));
    }
  } else if (input.value !== String(value)) {
    input.value = String(value);
  }
}

function setNumber(input, value) {
  input.value = String(round(value));
}

function numberValue(input) {
  const value = Number(input.value);
  return Number.isFinite(value) ? value : 0;
}

function optionalNumberValue(input) {
  if (!String(input.value).trim()) return null;
  const value = Number(input.value);
  return Number.isFinite(value) ? value : null;
}

function checkboxValue(input) {
  return Boolean(input.checked);
}

function round(value) {
  return Math.round(value * 1000) / 1000;
}

function formatPose(pose) {
  if (!pose) return "-";
  const source = pose.source ? ` · ${pose.source}` : "";
  return `x ${round(pose.x)}, y ${round(pose.y)}, yaw ${round(pose.yaw)}${source}`;
}

function loadMapImage(url) {
  state.mapLoaded = false;
  state.mapWallCache = null;
  state.mapImage = new Image();
  state.mapImage.onload = () => {
    state.mapLoaded = true;
    state.mapWallCache = null;
    if (state.data) {
      state.data.setup.map.widthPixels = state.mapImage.naturalWidth;
      state.data.setup.map.heightPixels = state.mapImage.naturalHeight;
    }
    planPathToGoal();
  };
  state.mapImage.src = url;
}

async function saveSetup(options = {}) {
  if (state.setupSaving) return;
  const payload = readSetupForm();
  state.setupSaving = true;
  els.saveSetupButton.disabled = true;
  setSetupSaveStatus("saving", "저장·적용 중...");
  try {
    const result = await postJson("/api/setup", payload);
    if (result.rosReload?.ok === false) {
      throw new Error(result.rosReload.message || "ROS 브릿지를 다시 연결하지 못했습니다. 설정은 화면에 유지합니다.");
    }
    const mismatch = savedConnectionMismatch(result.state?.setup, payload);
    if (mismatch) {
      throw new Error(`저장 결과가 입력값과 다릅니다 (${mismatch}). 다른 대시보드 서버가 실행 중인지 확인하세요.`);
    }
    state.setupDirty = false;
    state.setupDrag = null;
    state.setupObjectRect = null;
    if (result.state) applyState(result.state);
    const initialPoseApplied = Boolean(result.initialPoseApply?.ok);
    setSetupSaveStatus(
      "saved",
      initialPoseApplied ? "저장됨 · 초기 위치 전송 완료" : "저장·적용됨",
    );
    if (options.successMessage) {
      toast(options.successMessage);
    } else {
      toast(initialPoseApplied ? "설정을 저장하고 초기 위치를 전송했습니다." : "설정을 저장·적용했습니다.");
      showResult(result);
    }
  } catch (error) {
    state.setupDirty = true;
    setSetupSaveStatus("error", "저장 실패 · 변경사항 유지됨");
    throw error;
  } finally {
    state.setupSaving = false;
    els.saveSetupButton.disabled = false;
  }
}

function savedConnectionMismatch(savedSetup, expectedSetup) {
  const activeRobot = expectedSetup?.activeRobot;
  const expected = expectedSetup?.robotProfiles?.[activeRobot]?.connection || {};
  const actual = savedSetup?.robotProfiles?.[activeRobot]?.connection || {};
  if (!activeRobot || !savedSetup?.robotProfiles?.[activeRobot]) return "선택 로봇 프로필";
  const fields = [
    ["robotIp", "로봇 IP"],
    ["robotSshHost", "SSH Host"],
    ["robotSshUser", "SSH 사용자"],
    ["rosLocalhostOnly", "ROS localhost 설정"],
  ];
  for (const [key, label] of fields) {
    if (String(actual[key] ?? "") !== String(expected[key] ?? "")) return label;
  }
  if (Number(actual.rosDomainId) !== Number(expected.rosDomainId)) return "ROS Domain";
  return "";
}

function setSetupSaveStatus(status, message) {
  if (!els.setupSaveStatus) return;
  els.setupSaveStatus.textContent = message;
  els.setupSaveStatus.dataset.status = status;
}

async function sendGoal() {
  const x = optionalNumberValue(els.goalX);
  const y = optionalNumberValue(els.goalY);
  if (x === null || y === null) {
    setWaypointMode(false);
    toast("최종 목표를 지도에서 지정하거나 X/Y 값을 입력하세요.");
    return;
  }
  const previous = state.waypoints[state.waypoints.length - 1]
    || state.data?.runtime?.pose
    || currentSetup().initialPose;
  const enteredYaw = optionalNumberValue(els.goalYaw);
  const yaw = enteredYaw ?? Math.atan2(y - previous.y, x - previous.x);
  setNumber(els.goalYaw, yaw);
  setNumber(els.goalYawRange, yaw);
  const payload = {
    x,
    y,
    yaw,
    forceFallback: checkboxValue(els.forceLidarFallback),
  };
  state.selectedGoal = payload;
  const repeat = repeatRouteConfig(state.waypoints.length + 1);
  if (repeat.enabled && !repeat.ok) {
    toast(repeat.error);
    return;
  }
  const plan = planPathToGoal();
  if (!plan.ok) {
    toast(`경로 불가: ${plan.status}`);
    return;
  }
  await stopManualDrive({ force: true });
  const route = Array.isArray(plan.targets) && plan.targets.length
    ? plan.targets
    : routeTargets();
  if (!route.length) {
    toast("실행할 경로 지점이 없습니다.");
    return;
  }
  if (repeat.enabled && route.length < 2) {
    toast("반복 운행에는 도달 가능한 지점이 두 개 이상 필요합니다.");
    return;
  }
  const path = navigationPathPayload(route[route.length - 1]);
  const repeatPathPlan = repeat.enabled
    ? planRepeatRoutePath(route)
    : { ok: true, points: [] };
  if (!repeatPathPlan.ok) {
    toast(`반복 복귀 경로 불가: ${repeatPathPlan.status}`);
    return;
  }
  const repeatPayload = {
    enabled: repeat.enabled,
    pauseSeconds: repeat.pauseSeconds,
    start: 1,
    end: route.length,
    sourceStart: repeat.start,
    sourceEnd: repeat.end,
  };
  const result =
    state.waypoints.length > 0 || repeat.enabled
      ? await postJson("/api/route", {
          poses: route,
          path,
          repeatPath: repeatPathPlan.ok
            ? navigationPathPayloadFromPoints(repeatPathPlan.points, route[route.length - 1])
            : [],
          forceFallback: payload.forceFallback,
          repeat: repeatPayload,
        })
      : await postJson("/api/goal", { ...payload, path });
  showResult(result);
}

async function resumeRoute() {
  const runtime = state.data?.runtime || {};
  const checkpoint = runtime.routeCheckpoint || {};
  const activeRobot = state.data?.setup?.activeRobot || "";
  if (checkpoint.robotId && checkpoint.robotId !== activeRobot) {
    toast("현재 로봇과 저장 경로의 로봇이 다릅니다.");
    return;
  }
  const savedRoute = Array.isArray(checkpoint.route) ? checkpoint.route : [];
  const nextIndex = Math.max(0, Math.trunc(Number(checkpoint.nextIndex) || 0));
  if (!checkpoint.available || nextIndex >= savedRoute.length) {
    toast("이어갈 저장 경로가 없습니다.");
    return;
  }
  const pose = runtime.pose;
  if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) {
    toast("현재 위치를 확인할 수 없어 이어서 주행할 수 없습니다.");
    return;
  }
  const remainingRoute = savedRoute.slice(nextIndex).map((target, index) => ({
    x: Number(target.x),
    y: Number(target.y),
    yaw: Number(target.yaw) || 0,
    sourceIndex: Number.isInteger(Number(target.sourceIndex))
      ? Number(target.sourceIndex)
      : nextIndex + index,
    final: Boolean(target.final),
  }));
  const plan = planRouteToGoal(remainingRoute);
  if (!plan.ok) {
    toast(`이어 주행 경로 불가: ${plan.status}`);
    return;
  }
  const route = Array.isArray(plan.targets) ? plan.targets : [];
  if (!route.length) {
    toast("이어갈 수 있는 웨이포인트가 없습니다.");
    return;
  }
  const path = navigationPathPayloadFromPoints(plan.points, route[route.length - 1]);
  await stopManualDrive({ force: true });
  await postJson("/api/cancel", {});
  const result = await postJson("/api/route", {
    poses: route,
    path,
    forceFallback: checkpoint.forceFallback !== false,
    resume: true,
    repeat: {
      enabled: false,
      start: 1,
      end: route.length,
      sourceStart: 1,
      sourceEnd: route.length,
      pauseSeconds: 5,
    },
  });
  showResult(result);
}

function navigationPathPayload(finalGoal) {
  return navigationPathPayloadFromPoints(state.plannedPath?.points || [], finalGoal);
}

function navigationPathPayloadFromPoints(points, finalGoal) {
  const path = (points || []).map((point) => {
    const routeIndex = Number(point.routeIndex);
    return {
      x: round(Number(point.x)),
      y: round(Number(point.y)),
      slow: Boolean(point.slow),
      ...(Number.isInteger(routeIndex) ? { routeIndex } : {}),
    };
  });
  if (!path.length) return [];
  const last = path[path.length - 1];
  if (Math.hypot(last.x - finalGoal.x, last.y - finalGoal.y) <= 0.03) {
    last.x = round(finalGoal.x);
    last.y = round(finalGoal.y);
  } else {
    path.push({
      x: round(finalGoal.x),
      y: round(finalGoal.y),
      slow: last.slow,
      ...(Number.isInteger(last.routeIndex) ? { routeIndex: last.routeIndex } : {}),
    });
  }
  return path;
}

async function stopAllMotion() {
  await stopManualDrive({ force: true });
  try {
    showResult(await postJson("/api/stop", {}));
  } catch (error) {
    toast(error.message);
  }
}

function manualSpeeds() {
  return {
    linear: Math.min(0.22, Math.max(0, numberValue(els.manualLinearSpeed) || 0.08)),
    angular: Math.min(2.84, Math.max(0, numberValue(els.manualAngularSpeed) || 0.7)),
  };
}

function startManualDrive(linearFactor, angularFactor) {
  const speeds = manualSpeeds();
  state.manualCommand = {
    linear: linearFactor * speeds.linear,
    angular: angularFactor * speeds.angular,
  };
  updateManualButtonState(linearFactor, angularFactor);
  sendManualDriveNow();
  clearInterval(state.manualTimer);
  state.manualTimer = setInterval(sendManualDriveNow, 400);
}

function updateManualButtonState(linearFactor = 0, angularFactor = 0) {
  document.querySelectorAll("[data-drive-linear]").forEach((button) => {
    const selected =
      Number(button.dataset.driveLinear) === Number(linearFactor) &&
      Number(button.dataset.driveAngular) === Number(angularFactor);
    button.classList.toggle("is-driving", selected);
  });
}

async function sendManualDriveNow() {
  try {
    await postJson("/api/manual_drive", state.manualCommand);
  } catch (error) {
    clearInterval(state.manualTimer);
    state.manualTimer = null;
    toast(error.message);
  }
}

function manualCommandActive() {
  return (
    Boolean(state.manualTimer) ||
    Math.abs(Number(state.manualCommand.linear) || 0) > 0 ||
    Math.abs(Number(state.manualCommand.angular) || 0) > 0
  );
}

async function stopManualDrive(options = {}) {
  const force = options?.force === true;
  const shouldSend = force || manualCommandActive();
  clearInterval(state.manualTimer);
  state.manualTimer = null;
  state.manualCommand = { linear: 0, angular: 0 };
  updateManualButtonState();
  if (!shouldSend) return;
  try {
    await postJson("/api/manual_drive", state.manualCommand);
  } catch (error) {
    toast(error.message);
  }
}

function stopManualDriveFromFocusLoss() {
  state.pressedKeys.clear();
  stopManualDrive();
}

function sendManualStopBeacon() {
  const shouldSend = manualCommandActive();
  clearInterval(state.manualTimer);
  state.manualTimer = null;
  state.manualCommand = { linear: 0, angular: 0 };
  state.pressedKeys.clear();
  updateManualButtonState();
  if (!shouldSend) return;
  const payload = JSON.stringify(state.manualCommand);
  if (navigator.sendBeacon) {
    navigator.sendBeacon("/api/manual_drive", new Blob([payload], { type: "application/json" }));
  } else {
    postJson("/api/manual_drive", state.manualCommand).catch(() => {});
  }
}

function setWaypointMode(enabled) {
  state.waypointMode = Boolean(enabled);
  els.waypointModeButton.classList.toggle("active", state.waypointMode);
  els.goalModeButton.classList.toggle("active", !state.waypointMode);
  els.waypointModeButton.setAttribute("aria-pressed", String(state.waypointMode));
  els.goalModeButton.setAttribute("aria-pressed", String(!state.waypointMode));
}

function addWaypoint(world) {
  state.waypoints.push({
    id: `wp-${Date.now()}-${Math.round(Math.random() * 1000)}`,
    x: round(world.x),
    y: round(world.y),
  });
  renderWaypoints();
  planPathToGoal();
}

function deleteWaypoint(id) {
  state.waypoints = state.waypoints.filter((waypoint) => waypoint.id !== id);
  renderWaypoints();
  planPathToGoal();
}

function clearWaypoints() {
  state.waypoints = [];
  renderWaypoints();
  planPathToGoal();
}

function onRepeatRouteChanged() {
  syncRepeatRouteControls(false);
  renderWaypoints();
  planPathToGoal();
}

function syncRepeatRouteControls(autoExtendEnd = true) {
  const pointCount = Math.max(1, state.waypoints.length + 1);
  const previousMax = Math.max(1, Number(els.repeatRouteEnd.max) || 1);
  const previousEnd = Math.max(1, Number(els.repeatRouteEnd.value) || 1);
  const shouldExtend = autoExtendEnd && previousEnd === previousMax;
  els.repeatRouteStart.max = String(pointCount);
  els.repeatRouteEnd.max = String(pointCount);
  const start = Math.min(pointCount, Math.max(1, Math.trunc(numberValue(els.repeatRouteStart) || 1)));
  const end = shouldExtend
    ? pointCount
    : Math.min(pointCount, Math.max(1, Math.trunc(numberValue(els.repeatRouteEnd) || pointCount)));
  els.repeatRouteStart.value = String(start);
  els.repeatRouteEnd.value = String(end);
  els.repeatRouteEnabled.disabled = pointCount < 2;
  if (pointCount < 2) els.repeatRouteEnabled.checked = false;
  updateRepeatRouteStatus();
}

function repeatRouteConfig(pointCount = state.waypoints.length + 1) {
  const enabled = checkboxValue(els.repeatRouteEnabled);
  const startValue = Number(els.repeatRouteStart.value);
  const endValue = Number(els.repeatRouteEnd.value);
  const pauseSeconds = Number(els.repeatRoutePause.value);
  const config = {
    enabled,
    start: startValue,
    end: endValue,
    pauseSeconds,
    ok: true,
    error: "",
  };
  if (!enabled) return config;
  if (!Number.isInteger(startValue) || !Number.isInteger(endValue)) {
    return { ...config, ok: false, error: "반복 시작/종료 번호는 정수여야 합니다." };
  }
  if (startValue < 1 || endValue > pointCount || endValue <= startValue) {
    return {
      ...config,
      ok: false,
      error: `반복 구간은 1~${pointCount} 사이에서 시작 번호보다 종료 번호가 커야 합니다.`,
    };
  }
  if (!Number.isFinite(pauseSeconds) || pauseSeconds < 0.5 || pauseSeconds > 3600) {
    return { ...config, ok: false, error: "휴식 시간은 0.5~3600초로 입력하세요." };
  }
  return config;
}

function updateRepeatRouteStatus() {
  const pointCount = Math.max(1, state.waypoints.length + 1);
  const repeat = repeatRouteConfig(pointCount);
  if (!repeat.enabled) {
    els.repeatRouteStatus.textContent = `반복 꺼짐 · 최종 목표 #${pointCount}`;
    return;
  }
  els.repeatRouteStatus.textContent = repeat.ok
    ? `#${repeat.start} → #${repeat.end} · ${repeat.pauseSeconds}초 휴식 · 자동 반복`
    : repeat.error;
}

function renderWaypoints() {
  if (!els.waypointList) return;
  syncRepeatRouteControls(true);
  const pointCount = state.waypoints.length + 1;
  const repeat = repeatRouteConfig(pointCount);
  els.sendGoalButton.textContent = repeat.enabled
    ? `구간 반복 실행 (#${repeat.start}~#${repeat.end})`
    : state.waypoints.length > 0
      ? `경로 실행 (${pointCount}지점)`
      : "목표로 이동";
  const waypointRows = state.waypoints
    .map(
      (waypoint, index) => `
        <div class="waypoint-row">
          <p>#${index + 1} x ${round(waypoint.x)}, y ${round(waypoint.y)}</p>
          <button type="button" data-delete-waypoint="${escapeHtml(waypoint.id)}">삭제</button>
        </div>
      `,
    )
    .join("");
  const finalX = optionalNumberValue(els.goalX);
  const finalY = optionalNumberValue(els.goalY);
  const finalText = finalX === null || finalY === null
    ? `#${pointCount} 최종 목표 · 미지정`
    : `#${pointCount} 최종 목표 · x ${round(finalX)}, y ${round(finalY)}, yaw ${round(optionalNumberValue(els.goalYaw) ?? 0)}`;
  els.waypointList.innerHTML = `${
    waypointRows || `<div class="waypoint-row"><p>경유지 없음</p></div>`
  }<div class="waypoint-row final-point"><p>${finalText}</p></div>`;
  els.waypointList.querySelectorAll("[data-delete-waypoint]").forEach((button) => {
    button.addEventListener("click", () => deleteWaypoint(button.dataset.deleteWaypoint));
  });
}

function allRouteTargets() {
  if (!state.selectedGoal) return [];
  return [
    ...state.waypoints.map((waypoint, sourceIndex) => ({
      x: waypoint.x,
      y: waypoint.y,
      yaw: 0,
      sourceIndex,
      final: false,
    })),
    {
      ...state.selectedGoal,
      sourceIndex: state.waypoints.length,
      final: true,
    },
  ];
}

function routeTargets() {
  const allTargets = allRouteTargets();
  if (!allTargets.length) return [];
  const repeat = repeatRouteConfig(allTargets.length);
  if (repeat.enabled && !repeat.ok) return [];
  const targets = repeat.enabled
    ? allTargets.slice(repeat.start - 1, repeat.end)
    : allTargets;
  const pose = state.data?.runtime?.pose || currentSetup().initialPose;
  let previous = pose;
  return targets.map((target, index) => {
    const next = targets[index + 1];
    const requestedYaw = Number(target.yaw);
    const yaw = next
      ? Math.atan2(next.y - target.y, next.x - target.x)
      : repeat.enabled && !target.final && targets.length > 1
        ? Math.atan2(targets[0].y - target.y, targets[0].x - target.x)
      : Number.isFinite(requestedYaw)
        ? requestedYaw
        : Math.atan2(target.y - previous.y, target.x - previous.x);
    previous = target;
    return {
      x: Number(target.x),
      y: Number(target.y),
      yaw: normalizedYaw(yaw),
      sourceIndex: Number(target.sourceIndex),
      final: Boolean(target.final),
    };
  });
}

function onManualKeyDown(event) {
  if (isTypingTarget(event.target)) return;
  const key = event.key.toLowerCase();
  if (!["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright", " "].includes(key)) return;
  event.preventDefault();
  if (key === " ") {
    state.pressedKeys.clear();
    stopManualDrive({ force: true });
    return;
  }
  state.pressedKeys.clear();
  state.pressedKeys.add(key);
  const direction = {
    w: [1, 0],
    arrowup: [1, 0],
    s: [-1, 0],
    arrowdown: [-1, 0],
    a: [0, 1],
    arrowleft: [0, 1],
    d: [0, -1],
    arrowright: [0, -1],
  }[key];
  startManualDrive(direction[0], direction[1]);
}

function onManualKeyUp(event) {
  if (isTypingTarget(event.target)) return;
  const key = event.key.toLowerCase();
  if (!state.pressedKeys.has(key)) return;
  event.preventDefault();
  state.pressedKeys.delete(key);
}

function isTypingTarget(target) {
  const tag = target?.tagName?.toLowerCase();
  return tag === "input" || tag === "textarea" || target?.isContentEditable;
}

async function copyFootprint() {
  const text = els.nav2Footprint.value;
  if (!text) return;
  try {
    await navigator.clipboard.writeText(text);
    toast("Footprint를 복사했습니다.");
  } catch {
    els.nav2Footprint.focus();
    els.nav2Footprint.select();
    toast("Footprint 문자열을 선택했습니다.");
  }
}

function readSetupForm() {
  const setup = currentSetup();
  return {
    map: {
      resolution: setup.map.resolution,
      originX: setup.map.originX,
      originY: setup.map.originY,
      originYaw: setup.map.originYaw,
      id: setup.map.id,
      name: setup.map.name,
      widthPixels: state.mapLoaded ? state.mapImage.naturalWidth : state.data?.setup?.map?.widthPixels || 0,
      heightPixels: state.mapLoaded ? state.mapImage.naturalHeight : state.data?.setup?.map?.heightPixels || 0,
    },
    initialPose: setup.initialPose,
    robot: setup.robot,
    accessory: setup.accessory,
    safety: setup.safety,
    object: setup.object,
    fallbackNavigation: setup.fallbackNavigation,
    obstacles: setup.obstacles,
    planner: setup.planner,
    network: setup.network,
    activeRobot: setup.activeRobot,
    robotProfiles: setup.robotProfiles,
    topics: setup.topics,
  };
}

function currentSetup() {
  const fallback = state.data?.setup || {};
  const activeRobot = els.activeRobotProfile?.value || fallback.activeRobot || "tb3_2";
  const topics = currentTopicInputs();
  const robotProfiles = currentRobotProfiles(fallback, activeRobot, topics);
  const connection = {
    robotIp: els.robotIp.value.trim(),
    rosDomainId: els.rosDomainId.value.trim(),
    rosLocalhostOnly: els.rosLocalhostOnly.value.trim(),
    robotSshHost: els.robotSshHost.value.trim(),
    robotSshUser: els.robotSshUser.value.trim(),
    robotSshPassword: els.robotSshPassword.value,
  };
  if (robotProfiles[activeRobot]) {
    robotProfiles[activeRobot] = {
      ...robotProfiles[activeRobot],
      connection: { ...(robotProfiles[activeRobot].connection || {}), ...connection },
    };
  }
  return {
    map: {
      ...(fallback.map || {}),
      resolution: numberValue(els.mapResolution) || 0.05,
      originX: numberValue(els.mapOriginX),
      originY: numberValue(els.mapOriginY),
      originYaw: numberValue(els.mapOriginYaw),
    },
    initialPose: {
      x: numberValue(els.initialX),
      y: numberValue(els.initialY),
      yaw: numberValue(els.initialYaw),
    },
    robot: {
      length: Math.max(0.01, numberValue(els.robotLength)),
      width: Math.max(0.01, numberValue(els.robotWidth)),
    },
    accessory: {
      front: Math.max(0, numberValue(els.accessoryFront)),
      back: Math.max(0, numberValue(els.accessoryBack)),
      left: Math.max(0, numberValue(els.accessoryLeft)),
      right: Math.max(0, numberValue(els.accessoryRight)),
      height: Math.max(0, numberValue(els.accessoryHeight)),
    },
    safety: {
      margin: Math.max(0, numberValue(els.safetyMargin)),
    },
    object: {
      width: Math.max(0.01, numberValue(els.objectWidth)),
      height: Math.max(0.01, numberValue(els.objectHeight)),
      inflation: Math.max(0, numberValue(els.objectInflation)),
    },
    fallbackNavigation: {
      enabled: checkboxValue(els.fallbackEnabled),
      maxLinear: Math.min(0.12, Math.max(0.01, numberValue(els.fallbackMaxLinear) || 0.08)),
      minLinear: Math.min(0.08, Math.max(0.005, numberValue(els.fallbackMinLinear) || 0.04)),
      maxAngular: Math.min(1.5, Math.max(0.1, numberValue(els.fallbackMaxAngular) || 0.6)),
      lookahead: Math.min(0.35, Math.max(0.05, numberValue(els.fallbackLookahead) || 0.12)),
      goalTolerance: fallback.fallbackNavigation?.goalTolerance ?? 0.04,
      yawTolerance: fallback.fallbackNavigation?.yawTolerance ?? 0.12,
      softDistance: Math.min(
        0.4,
        Math.max(0, optionalNumberValue(els.fallbackSoftDistance) ?? 0.10),
      ),
      hardMargin: Math.min(0.15, Math.max(0.01, numberValue(els.fallbackHardMargin) || 0.03)),
      scanTimeout: Math.min(1, Math.max(0.5, numberValue(els.fallbackScanTimeout) || 0.6)),
      odomTimeout: Math.min(1, Math.max(0.5, numberValue(els.fallbackOdomTimeout) || 0.6)),
      collisionHorizon: Math.min(3, Math.max(0.4, numberValue(els.fallbackCollisionHorizon) || 1.5)),
    },
    obstacles: normalizeObstacles(fallback.obstacles || []),
    planner: {
      ...(fallback.planner || {}),
      cellSize: Math.max(0.005, numberValue(els.gridCellSize) || 0.02),
      hardClearance: Math.min(
        0.4,
        Math.max(0.05, optionalNumberValue(els.plannerHardClearance) ?? 0.05),
      ),
      showGrid: checkboxValue(els.showGrid),
      showInflation: checkboxValue(els.showInflation),
      showLidarPoints: checkboxValue(els.showLidarPoints),
      detectLidarObstacles: checkboxValue(els.detectLidarObstacles),
      detectBlackWalls: checkboxValue(els.detectBlackWalls),
      blockedCells: normalizeBlockedCells(fallback.planner?.blockedCells || []),
      freeCells: normalizeBlockedCells(fallback.planner?.freeCells || []),
    },
    network: {
      serverIp: els.serverIp.value.trim(),
      ...connection,
    },
    activeRobot,
    robotProfiles,
    topics,
  };
}

function topicsForNamespace(namespace, cameraStyle = "color") {
  const ns = String(namespace || "").replace(/\/$/, "");
  const topic = (suffix) => `${ns}${suffix}`;
  let cameraBase = topic("/camera/color/image_raw");
  if (cameraStyle === "plain") {
    cameraBase = topic("/camera/image_raw");
  } else if (cameraStyle === "camera_ros" || cameraStyle === "camera-ros") {
    cameraBase = topic("/camera/camera/image_raw");
  } else if (cameraStyle === "realsense") {
    cameraBase = topic("/camera/camera/color/image_raw");
  }
  return {
    scan: topic("/scan"),
    pose: topic("/amcl_pose"),
    odom: topic("/odom"),
    camera: cameraBase,
    compressedCamera: `${cameraBase}/compressed`,
    initialPose: topic("/initialpose"),
    goalAction: topic("/navigate_to_pose"),
    routeAction: topic("/navigate_through_poses"),
    goalTopic: topic("/goal_pose"),
    cmdVel: topic("/cmd_vel"),
    mapFrame: "map",
    baseFrame: "base_link",
  };
}

function normalizedRobotProfiles(setup = state.data?.setup || {}) {
  const incoming = setup.robotProfiles || {};
  const profiles = {};
  for (const [id, rawProfile] of Object.entries({ ...DEFAULT_ROBOT_PROFILES, ...incoming })) {
    const profile = rawProfile || {};
    const fallback = DEFAULT_ROBOT_PROFILES[id] || {};
    const namespace = profile.namespace || fallback.namespace || `/${id}`;
    const cameraStyle = id === "tb3_2" ? "plain" : "color";
    profiles[id] = {
      ...profile,
      label: profile.label || fallback.label || id,
      namespace,
      source: profile.source || fallback.source || "",
      body: normalizedRobotBody(profile.body || fallback.body),
      mapPose: normalizedRobotMapPose(profile.mapPose || fallback.mapPose),
      connection: {
        ...(fallback.connection || {}),
        ...(profile.connection || {}),
      },
      topics: {
        ...topicsForNamespace(namespace, cameraStyle),
        ...(fallback.topics || {}),
        ...(profile.topics || {}),
      },
    };
  }
  return profiles;
}

function normalizedRobotBody(body = {}) {
  return {
    length: Math.min(0.8, Math.max(0.05, Number(body.length) || 0.18)),
    width: Math.min(0.8, Math.max(0.05, Number(body.width) || 0.14)),
  };
}

function normalizedRobotMapPose(mapPose = {}) {
  const x = optionalFiniteNumber(mapPose.x);
  const y = optionalFiniteNumber(mapPose.y);
  return {
    enabled: Boolean(mapPose.enabled) && x !== null && y !== null,
    x,
    y,
    yaw: normalizedYaw(Number(mapPose.yaw) || 0),
  };
}

function optionalFiniteNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function avoidanceRobotObstacles(setup) {
  const activeRobot = setup.activeRobot || "tb3_2";
  const livePoses = state.data?.runtime?.robotPoses || {};
  return Object.entries(normalizedRobotProfiles(setup))
    .filter(([id, profile]) => id !== activeRobot && (livePoses[id]?.available || profile.mapPose.enabled))
    .map(([id, profile]) => {
      const livePose = livePoses[id]?.available ? livePoses[id] : null;
      const pose = livePose || profile.mapPose;
      const cosYaw = Math.abs(Math.cos(pose.yaw));
      const sinYaw = Math.abs(Math.sin(pose.yaw));
      return {
        id: `robot-${id}`,
        label: profile.label,
        x: pose.x,
        y: pose.y,
        yaw: pose.yaw,
        bodyLength: profile.body.length,
        bodyWidth: profile.body.width,
        width: profile.body.length * cosYaw + profile.body.width * sinYaw,
        height: profile.body.length * sinYaw + profile.body.width * cosYaw,
      };
    });
}

function currentTopicInputs() {
  return {
    scan: els.topicScan.value.trim(),
    pose: els.topicPose.value.trim(),
    odom: els.topicOdom.value.trim(),
    camera: els.topicCamera.value.trim(),
    compressedCamera: els.topicCompressedCamera.value.trim(),
    initialPose: els.topicInitialPose.value.trim(),
    goalAction: els.topicGoalAction.value.trim(),
    routeAction: els.topicRouteAction.value.trim(),
    goalTopic: els.topicGoalTopic.value.trim(),
    cmdVel: els.topicCmdVel.value.trim(),
    mapFrame: state.data?.setup?.topics?.mapFrame || "map",
    baseFrame: state.data?.setup?.topics?.baseFrame || "base_link",
  };
}

function currentRobotProfiles(fallback, activeRobot, topics) {
  const profiles = normalizedRobotProfiles(fallback);
  if (activeRobot) {
    profiles[activeRobot] = {
      ...(profiles[activeRobot] || { label: activeRobot, namespace: `/${activeRobot}` }),
      topics: {
        ...(profiles[activeRobot]?.topics || {}),
        ...topics,
      },
    };
  }
  return profiles;
}

function effectiveFootprint(setup) {
  const halfLength = setup.robot.length / 2;
  const halfWidth = setup.robot.width / 2;
  const margin = setup.safety?.margin ?? 0;
  const accessory = setup.accessory || {};
  const front = halfLength + (accessory.front || 0) + margin;
  const back = halfLength + (accessory.back || 0) + margin;
  const left = halfWidth + (accessory.left || 0) + margin;
  const right = halfWidth + (accessory.right || 0) + margin;
  return {
    front,
    back,
    left,
    right,
    length: front + back,
    width: left + right,
    points: [
      [front, left],
      [front, -right],
      [-back, -right],
      [-back, left],
    ],
  };
}

function configuredClearanceDistance(setup) {
  const configured = Number(setup.planner?.hardClearance);
  const clearance = Number.isFinite(configured) ? Math.max(0.05, configured) : 0.05;
  return Math.max(0, clearance + (Number(setup.object?.inflation) || 0));
}

function plannerClearanceExtents(setup, extraDistance = 0) {
  const footprint = effectiveFootprint(setup);
  const extra = Math.max(0, configuredClearanceDistance(setup) + Number(extraDistance || 0));
  return {
    x: Math.max(footprint.front, footprint.back) + extra,
    y: Math.max(footprint.left, footprint.right) + extra,
  };
}

function nav2FootprintString(setup) {
  const footprint = effectiveFootprint(setup);
  const points = footprint.points.map(([x, y]) => `[${round(x)}, ${round(y)}]`).join(", ");
  return `footprint: "[${points}]"`;
}

function updateFootprintReadout(setup) {
  const footprint = effectiveFootprint(setup);
  els.effectiveFront.textContent = `${round(footprint.front)} m`;
  els.effectiveBack.textContent = `${round(footprint.back)} m`;
  els.effectiveLeft.textContent = `${round(footprint.left)} m`;
  els.effectiveRight.textContent = `${round(footprint.right)} m`;
  els.effectiveLength.textContent = `${round(footprint.length)} m`;
  els.effectiveWidth.textContent = `${round(footprint.width)} m`;
  els.nav2Footprint.value = nav2FootprintString(setup);
}

function normalizeObstacles(obstacles) {
  if (!Array.isArray(obstacles)) return [];
  return obstacles
    .map((obstacle, index) => ({
      id: String(obstacle.id || `obs-${index + 1}`),
      x: Number(obstacle.x) || 0,
      y: Number(obstacle.y) || 0,
      width: Math.max(0.01, Number(obstacle.width) || 0.3),
      height: Math.max(0.01, Number(obstacle.height) || 0.3),
    }))
    .filter((obstacle) => Number.isFinite(obstacle.x) && Number.isFinite(obstacle.y));
}

function renderObstacleList(obstacles = normalizeObstacles(state.data?.setup?.obstacles || [])) {
  if (!els.obstacleList) return;
  if (obstacles.length === 0) {
    els.obstacleList.innerHTML = `<div class="obstacle-row"><p>등록된 장애물 없음</p></div>`;
    return;
  }
  els.obstacleList.innerHTML = obstacles
    .map(
      (obstacle, index) => `
        <div class="obstacle-row">
          <p>#${index + 1} x ${round(obstacle.x)}, y ${round(obstacle.y)} · ${round(obstacle.width)} x ${round(obstacle.height)} m</p>
          <button type="button" data-delete-obstacle="${escapeHtml(obstacle.id)}">삭제</button>
        </div>
      `,
    )
    .join("");
  els.obstacleList.querySelectorAll("[data-delete-obstacle]").forEach((button) => {
    button.addEventListener("click", () => deleteObstacle(button.dataset.deleteObstacle));
  });
}

function setObstacles(obstacles) {
  if (!state.data?.setup) return;
  state.data.setup.obstacles = normalizeObstacles(obstacles);
  renderObstacleList(state.data.setup.obstacles);
  state.setupDirty = true;
  syncSetupShadow();
}

function obstacleAtPoint(world, setup) {
  const obstacles = normalizeObstacles(setup.obstacles || []);
  for (let index = obstacles.length - 1; index >= 0; index -= 1) {
    const obstacle = obstacles[index];
    const halfWidth = obstacle.width / 2;
    const halfHeight = obstacle.height / 2;
    if (
      world.x >= obstacle.x - halfWidth &&
      world.x <= obstacle.x + halfWidth &&
      world.y >= obstacle.y - halfHeight &&
      world.y <= obstacle.y + halfHeight
    ) {
      return obstacle;
    }
  }
  return null;
}

function moveObstacle(id, center) {
  const obstacles = normalizeObstacles(state.data?.setup?.obstacles || []);
  const index = obstacles.findIndex((obstacle) => obstacle.id === id);
  if (index < 0) return;
  obstacles[index] = {
    ...obstacles[index],
    x: round(center.x),
    y: round(center.y),
  };
  setObstacles(obstacles);
}

function addObstacle(rect) {
  const obstacles = normalizeObstacles(state.data?.setup?.obstacles || []);
  obstacles.push({
    id: `obs-${Date.now()}-${Math.round(Math.random() * 1000)}`,
    x: round(rect.x),
    y: round(rect.y),
    width: round(Math.max(0.01, rect.width)),
    height: round(Math.max(0.01, rect.height)),
  });
  setObstacles(obstacles);
  toast(`장애물 ${obstacles.length}개`);
}

function deleteObstacle(id) {
  const obstacles = normalizeObstacles(state.data?.setup?.obstacles || []).filter(
    (obstacle) => obstacle.id !== id,
  );
  setObstacles(obstacles);
}

function clearObstacles() {
  setObstacles([]);
  toast("장애물을 모두 삭제했습니다.");
}

function applyObjectPreset(button) {
  const width = Number(button.dataset.width);
  const height = Number(button.dataset.height);
  if (!Number.isFinite(width) || !Number.isFinite(height)) return;
  setNumber(els.objectWidth, width);
  setNumber(els.objectHeight, height);
  setSetupTool("object");
  markSetupDirty();
  toast(`${button.textContent.trim()} 적용`);
}

function normalizeBlockedCells(cells) {
  if (!Array.isArray(cells)) return [];
  const normalized = [];
  for (const cell of cells) {
    if (typeof cell === "string" && /^-?\d+,-?\d+$/.test(cell)) {
      normalized.push(cell);
    } else if (cell && Number.isInteger(cell.x) && Number.isInteger(cell.y)) {
      normalized.push(cellKey(cell.x, cell.y));
    }
  }
  return Array.from(new Set(normalized)).sort(sortCellKeys);
}

function cellKey(x, y) {
  return `${x},${y}`;
}

function parseCellKey(key) {
  const [x, y] = key.split(",").map(Number);
  return { x, y };
}

function sortCellKeys(a, b) {
  const ca = parseCellKey(a);
  const cb = parseCellKey(b);
  return ca.y === cb.y ? ca.x - cb.x : ca.y - cb.y;
}

function mapMetrics(setup) {
  const imageWidth = state.mapLoaded ? state.mapImage.naturalWidth : setup.map.widthPixels || 800;
  const imageHeight = state.mapLoaded ? state.mapImage.naturalHeight : setup.map.heightPixels || 600;
  const resolution = Number(setup.map.resolution) || 0.05;
  const cellSize = Math.max(0.005, Number(setup.planner?.cellSize) || 0.02);
  const widthMeters = imageWidth * resolution;
  const heightMeters = imageHeight * resolution;
  return {
    imageWidth,
    imageHeight,
    resolution,
    cellSize,
    widthMeters,
    heightMeters,
    cols: Math.max(1, Math.ceil(widthMeters / cellSize)),
    rows: Math.max(1, Math.ceil(heightMeters / cellSize)),
    originX: Number(setup.map.originX) || 0,
    originY: Number(setup.map.originY) || 0,
  };
}

function mapWallCellSet(setup) {
  if (!setup.planner?.detectBlackWalls || !state.mapLoaded || !state.mapImage.naturalWidth) {
    return new Set();
  }

  const metrics = mapMetrics(setup);
  const cacheKey = [
    state.mapImage.currentSrc || state.mapImage.src,
    metrics.imageWidth,
    metrics.imageHeight,
    metrics.resolution,
    metrics.cellSize,
  ].join("|");

  if (state.mapWallCache?.key === cacheKey) {
    return state.mapWallCache.cells;
  }

  const cells = new Set();
  try {
    const canvas = document.createElement("canvas");
    canvas.width = metrics.imageWidth;
    canvas.height = metrics.imageHeight;
    const ctx = canvas.getContext("2d", { willReadFrequently: true });
    ctx.drawImage(state.mapImage, 0, 0, metrics.imageWidth, metrics.imageHeight);
    const pixels = ctx.getImageData(0, 0, metrics.imageWidth, metrics.imageHeight).data;

    for (let py = 0; py < metrics.imageHeight; py += 1) {
      for (let px = 0; px < metrics.imageWidth; px += 1) {
        const offset = (py * metrics.imageWidth + px) * 4;
        const alpha = pixels[offset + 3];
        if (alpha <= 16) continue;
        const red = pixels[offset];
        const green = pixels[offset + 1];
        const blue = pixels[offset + 2];
        const luminance = red * 0.2126 + green * 0.7152 + blue * 0.0722;
        if (luminance >= 80) continue;

        const cellX = Math.floor((px * metrics.resolution) / metrics.cellSize);
        const cellY = Math.floor(((metrics.imageHeight - 1 - py) * metrics.resolution) / metrics.cellSize);
        if (cellX >= 0 && cellY >= 0 && cellX < metrics.cols && cellY < metrics.rows) {
          cells.add(cellKey(cellX, cellY));
        }
      }
    }
  } catch (error) {
    console.warn("Map wall detection failed", error);
  }

  state.mapWallCache = { key: cacheKey, cells };
  return cells;
}

function worldToGrid(world, setup) {
  const metrics = mapMetrics(setup);
  const x = Math.floor((world.x - metrics.originX) / metrics.cellSize);
  const y = Math.floor((world.y - metrics.originY) / metrics.cellSize);
  if (x < 0 || y < 0 || x >= metrics.cols || y >= metrics.rows) return null;
  return { x, y };
}

function gridToWorld(cell, setup) {
  const metrics = mapMetrics(setup);
  return {
    x: metrics.originX + (cell.x + 0.5) * metrics.cellSize,
    y: metrics.originY + (cell.y + 0.5) * metrics.cellSize,
  };
}

function blockedCellSet(setup) {
  const cells = new Set(mapWallCellSet(setup));
  for (const key of normalizeBlockedCells(setup.planner?.blockedCells || [])) {
    cells.add(key);
  }
  for (const key of normalizeBlockedCells(setup.planner?.freeCells || [])) {
    cells.delete(key);
  }
  for (const key of obstacleCellKeys(setup)) {
    cells.add(key);
  }
  for (const key of dynamicLidarObstacleCellKeys(setup)) {
    cells.add(key);
  }
  return cells;
}

function obstacleCellKeys(setup) {
  return obstacleCellsFor(
    [...normalizeObstacles(setup.obstacles || []), ...avoidanceRobotObstacles(setup)],
    setup,
  );
}

function dynamicLidarObstacleCellKeys(setup) {
  if (!setup.planner?.detectLidarObstacles) return new Set();
  const obstacles = normalizeObstacles(state.data?.runtime?.dynamicLidarObstacles || []);
  return obstacleCellsFor(obstacles, setup);
}

function obstacleCellsFor(obstacles, setup) {
  const metrics = mapMetrics(setup);
  const keys = new Set();
  for (const obstacle of obstacles) {
    const minX = Math.floor((obstacle.x - obstacle.width / 2 - metrics.originX) / metrics.cellSize);
    const maxX = Math.floor((obstacle.x + obstacle.width / 2 - metrics.originX) / metrics.cellSize);
    const minY = Math.floor((obstacle.y - obstacle.height / 2 - metrics.originY) / metrics.cellSize);
    const maxY = Math.floor((obstacle.y + obstacle.height / 2 - metrics.originY) / metrics.cellSize);
    for (let x = minX; x <= maxX; x += 1) {
      for (let y = minY; y <= maxY; y += 1) {
        if (x >= 0 && y >= 0 && x < metrics.cols && y < metrics.rows) {
          keys.add(cellKey(x, y));
        }
      }
    }
  }
  return keys;
}

function inflatedCellSetForExtents(setup, extents) {
  const metrics = mapMetrics(setup);
  const blocked = blockedCellSet(setup);
  const inflated = new Set(blocked);
  const clearanceX = Math.max(0, Number(extents?.x) || 0);
  const clearanceY = Math.max(0, Number(extents?.y) || 0);
  const xCells = Math.ceil(clearanceX / metrics.cellSize);
  const yCells = Math.ceil(clearanceY / metrics.cellSize);
  for (const key of blocked) {
    const cell = parseCellKey(key);
    for (let dx = -xCells; dx <= xCells; dx += 1) {
      for (let dy = -yCells; dy <= yCells; dy += 1) {
        const x = cell.x + dx;
        const y = cell.y + dy;
        if (x < 0 || y < 0 || x >= metrics.cols || y >= metrics.rows) continue;
        if (Math.abs(dx) * metrics.cellSize <= clearanceX && Math.abs(dy) * metrics.cellSize <= clearanceY) {
          inflated.add(cellKey(x, y));
        }
      }
    }
  }
  return inflated;
}

function inflatedCellSet(setup, extraRadius = 0) {
  return inflatedCellSetForExtents(setup, plannerClearanceExtents(setup, extraRadius));
}

function softInflatedCellSet(setup) {
  return inflatedCellSet(setup, setup.fallbackNavigation?.softDistance ?? 0.10);
}

// Keep the overlay tied to the configured margins while collision planning also
// accounts for the robot footprint internally.
function displayedHardInflatedCellSet(setup) {
  return inflatedCellSetForExtents(setup, plannerClearanceExtents(setup));
}

function displayedSoftInflatedCellSet(setup) {
  return inflatedCellSetForExtents(
    setup,
    plannerClearanceExtents(setup, setup.fallbackNavigation?.softDistance ?? 0.10),
  );
}

function planPathToGoal() {
  return planRouteToGoal();
  if (!state.data || !state.selectedGoal) {
    return setPlannedPath({ points: [], cells: [], status: "목표 없음", ok: false });
  }
  const setup = currentSetup();
  const pose = state.data.runtime?.pose || setup.initialPose;
  const start = worldToGrid(pose, setup);
  const goal = worldToGrid(state.selectedGoal, setup);
  if (!start || !goal) {
    return setPlannedPath({ points: [], cells: [], status: "맵 밖 목표", ok: false });
  }

  const metrics = mapMetrics(setup);
  if (metrics.cols * metrics.rows > 250000) {
    return setPlannedPath({ points: [], cells: [], status: "그리드가 너무 촘촘함", ok: false });
  }

  const blocked = inflatedCellSet(setup);
  const soft = softInflatedCellSet(setup);
  const startKey = cellKey(start.x, start.y);
  const goalKey = cellKey(goal.x, goal.y);
  if (blocked.has(startKey)) {
    return setPlannedPath({ points: [], cells: [], status: "시작 위치가 장애물 영역", ok: false });
  }
  if (blocked.has(goalKey)) {
    return setPlannedPath({ points: [], cells: [], status: "목표가 장애물 영역", ok: false });
  }

  const result = runAStar(start, goal, blocked, metrics, soft);
  if (!result.ok) {
    return setPlannedPath({ points: [], cells: [], status: "경로 없음", ok: false });
  }
  const points = result.cells.map((cell) => ({
    ...gridToWorld(cell, setup),
    slow: soft.has(cellKey(cell.x, cell.y)),
  }));
  return setPlannedPath({
    points,
    cells: result.cells,
    status: `OK · ${result.cells.length} cells · ${round(result.distance)} m`,
    ok: true,
  });
}

function planRouteToGoal(targetsOverride = null) {
  const resuming = Array.isArray(targetsOverride);
  const targets = resuming ? targetsOverride : routeTargets();
  if (!state.data || targets.length === 0) {
    return setPlannedPath({ points: [], cells: [], status: "목표 없음", ok: false });
  }
  const setup = currentSetup();
  const pose = state.data.runtime?.pose || setup.initialPose;
  let start = worldToGrid(pose, setup);
  if (!start) {
    return setPlannedPath({ points: [], cells: [], status: "시작 위치가 맵 밖", ok: false });
  }

  const metrics = mapMetrics(setup);
  if (metrics.cols * metrics.rows > 250000) {
    return setPlannedPath({ points: [], cells: [], status: "그리드가 너무 큼", ok: false });
  }

  const blocked = inflatedCellSet(setup);
  const soft = softInflatedCellSet(setup);
  const startKey = cellKey(start.x, start.y);
  if (blocked.has(startKey)) {
    return setPlannedPath({ points: [], cells: [], status: "시작 위치가 장애물 영역", ok: false });
  }

  let totalDistance = 0;
  const routeCells = [];
  const reachableTargets = [];
  const skippedWaypointNumbers = [];
  for (let index = 0; index < targets.length; index += 1) {
    const target = targets[index];
    const goal = worldToGrid(target, setup);
    const sourceNumber = Number(target.sourceIndex) + 1;
    const displayNumber = Number.isInteger(sourceNumber) && sourceNumber > 0
      ? sourceNumber
      : index + 1;
    const label = target.final || index === targets.length - 1
      ? "목표"
      : `경유지 ${displayNumber}`;
    let failure = "";
    if (!goal) {
      failure = `${label}가 맵 밖`;
    }
    const goalKey = goal ? cellKey(goal.x, goal.y) : "";
    if (!failure && blocked.has(goalKey)) {
      failure = `${label}가 장애물 영역`;
    }
    const result = failure ? null : runAStar(start, goal, blocked, metrics, soft);
    if (!failure && !result.ok) {
      failure = `${label}까지 경로 없음`;
    }
    if (failure) {
      if (index < targets.length - 1) {
        skippedWaypointNumbers.push(
          displayNumber,
        );
        continue;
      }
      return setPlannedPath({ points: [], cells: [], status: failure, ok: false });
    }
    const executableRouteIndex = reachableTargets.length;
    const segmentCells = routeCells.length ? result.cells.slice(1) : result.cells;
    routeCells.push(
      ...segmentCells.map((cell) => ({ ...cell, routeIndex: executableRouteIndex })),
    );
    totalDistance += result.distance;
    reachableTargets.push({ ...target });
    start = goal;
  }

  const executableTargets = reachableTargets.map((target, index) => {
    const next = reachableTargets[index + 1];
    return {
      ...target,
      yaw: next
        ? normalizedYaw(Math.atan2(next.y - target.y, next.x - target.x))
        : target.yaw,
    };
  });

  const points = routeCells.map((cell) => ({
    ...gridToWorld(cell, setup),
    slow: soft.has(cellKey(cell.x, cell.y)),
    routeIndex: cell.routeIndex,
  }));
  const slowCellCount = points.filter((point) => point.slow).length;
  const repeat = resuming
    ? { enabled: false }
    : repeatRouteConfig(state.waypoints.length + 1);
  const firstResumeNumber = Number(executableTargets[0]?.sourceIndex) + 1;
  const routeLabel = resuming
    ? `이어 주행 #${Number.isInteger(firstResumeNumber) ? firstResumeNumber : 1}부터`
    : repeat.enabled
    ? `반복 #${repeat.start}~#${repeat.end}`
    : `경유지 ${Math.max(0, executableTargets.length - 1)}개`;
  const skippedLabel = skippedWaypointNumbers.length
    ? ` · 건너뜀 #${skippedWaypointNumbers.join(", #")}`
    : "";
  return setPlannedPath({
    points,
    cells: routeCells,
    targets: executableTargets,
    skippedWaypointNumbers,
    status: `OK · ${routeLabel}${skippedLabel} · ${routeCells.length} cells · 감속 ${slowCellCount} · ${round(totalDistance)} m`,
    ok: true,
  });
}

// A repeat cycle starts at the prior cycle's final target, then returns to target 1.
// It is sent separately so the server can continue direct A* tracking without Nav2.
function planRepeatRoutePath(route) {
  if (!Array.isArray(route) || route.length < 2) {
    return { ok: false, points: [], cells: [], status: "반복에는 두 지점 이상 필요" };
  }
  const setup = currentSetup();
  let start = worldToGrid(route[route.length - 1], setup);
  if (!start) return { ok: false, points: [], cells: [], status: "복귀 시작점이 맵 밖" };

  const metrics = mapMetrics(setup);
  if (metrics.cols * metrics.rows > 250000) {
    return { ok: false, points: [], cells: [], status: "그리드가 너무 큼" };
  }
  const blocked = inflatedCellSet(setup);
  const soft = softInflatedCellSet(setup);
  if (blocked.has(cellKey(start.x, start.y))) {
    return { ok: false, points: [], cells: [], status: "복귀 시작점이 금지 영역" };
  }

  const cells = [];
  let totalDistance = 0;
  for (let index = 0; index < route.length; index += 1) {
    const goal = worldToGrid(route[index], setup);
    if (!goal || blocked.has(cellKey(goal.x, goal.y))) {
      return { ok: false, points: [], cells: [], status: `반복 지점 ${index + 1}이 금지 영역 또는 맵 밖` };
    }
    const result = runAStar(start, goal, blocked, metrics, soft);
    if (!result.ok) {
      return { ok: false, points: [], cells: [], status: `반복 지점 ${index + 1}까지 경로 없음` };
    }
    const segment = cells.length ? result.cells.slice(1) : result.cells;
    cells.push(...segment.map((cell) => ({ ...cell, routeIndex: index })));
    totalDistance += result.distance;
    start = goal;
  }
  return {
    ok: true,
    cells,
    points: cells.map((cell) => ({
      ...gridToWorld(cell, setup),
      slow: soft.has(cellKey(cell.x, cell.y)),
      routeIndex: cell.routeIndex,
    })),
    status: `반복 복귀 경로 ${cells.length} cells, ${round(totalDistance)} m`,
  };
}

function setPlannedPath(path) {
  state.plannedPath = path;
  if (els.statusPath) {
    els.statusPath.textContent = path.status || "-";
  }
  return path;
}

function runAStar(start, goal, blocked, metrics, soft = new Set()) {
  const open = new BinaryHeap((a, b) => a.f - b.f);
  const startKey = cellKey(start.x, start.y);
  const goalKey = cellKey(goal.x, goal.y);
  const cameFrom = new Map();
  const gScore = new Map([[startKey, 0]]);
  const closed = new Set();
  open.push({ ...start, key: startKey, f: heuristic(start, goal), g: 0 });

  while (open.size() > 0) {
    const current = open.pop();
    if (closed.has(current.key)) continue;
    if (current.key === goalKey) {
      return reconstructPath(current.key, cameFrom, gScore, metrics);
    }
    closed.add(current.key);

    for (const next of neighbors(current, metrics)) {
      const nextKey = cellKey(next.x, next.y);
      if (blocked.has(nextKey) || closed.has(nextKey)) continue;
      if (
        next.dx !== 0
        && next.dy !== 0
        && (
          blocked.has(cellKey(current.x + next.dx, current.y))
          || blocked.has(cellKey(current.x, current.y + next.dy))
        )
      ) {
        continue;
      }
      const softMultiplier = soft.has(nextKey) ? ASTAR_SOFT_CELL_MULTIPLIER : 1;
      const tentativeG = current.g + next.cost * softMultiplier;
      if (tentativeG >= (gScore.get(nextKey) ?? Infinity)) continue;
      cameFrom.set(nextKey, current.key);
      gScore.set(nextKey, tentativeG);
      open.push({
        x: next.x,
        y: next.y,
        key: nextKey,
        g: tentativeG,
        f: tentativeG + heuristic(next, goal),
      });
    }
  }
  return { ok: false, cells: [], distance: 0 };
}

function neighbors(cell, metrics) {
  const moves = [
    [1, 0, 1],
    [-1, 0, 1],
    [0, 1, 1],
    [0, -1, 1],
    [1, 1, Math.SQRT2],
    [1, -1, Math.SQRT2],
    [-1, 1, Math.SQRT2],
    [-1, -1, Math.SQRT2],
  ];
  const result = [];
  for (const [dx, dy, cost] of moves) {
    const x = cell.x + dx;
    const y = cell.y + dy;
    if (x >= 0 && y >= 0 && x < metrics.cols && y < metrics.rows) {
      result.push({ x, y, dx, dy, cost });
    }
  }
  return result;
}

function heuristic(a, b) {
  const dx = Math.abs(a.x - b.x);
  const dy = Math.abs(a.y - b.y);
  return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy);
}

function reconstructPath(goalKey, cameFrom, gScore, metrics) {
  const keys = [goalKey];
  let current = goalKey;
  while (cameFrom.has(current)) {
    current = cameFrom.get(current);
    keys.push(current);
  }
  keys.reverse();
  const cells = keys.map(parseCellKey);
  let distance = 0;
  for (let index = 1; index < cells.length; index += 1) {
    distance += Math.hypot(
      cells[index].x - cells[index - 1].x,
      cells[index].y - cells[index - 1].y,
    ) * metrics.cellSize;
  }
  return {
    ok: true,
    cells,
    distance,
  };
}

class BinaryHeap {
  constructor(compare) {
    this.items = [];
    this.compare = compare;
  }

  size() {
    return this.items.length;
  }

  push(item) {
    this.items.push(item);
    this.bubbleUp(this.items.length - 1);
  }

  pop() {
    const root = this.items[0];
    const last = this.items.pop();
    if (this.items.length > 0) {
      this.items[0] = last;
      this.sinkDown(0);
    }
    return root;
  }

  bubbleUp(index) {
    while (index > 0) {
      const parent = Math.floor((index - 1) / 2);
      if (this.compare(this.items[index], this.items[parent]) >= 0) break;
      [this.items[index], this.items[parent]] = [this.items[parent], this.items[index]];
      index = parent;
    }
  }

  sinkDown(index) {
    while (true) {
      const left = index * 2 + 1;
      const right = left + 1;
      let smallest = index;
      if (left < this.items.length && this.compare(this.items[left], this.items[smallest]) < 0) {
        smallest = left;
      }
      if (right < this.items.length && this.compare(this.items[right], this.items[smallest]) < 0) {
        smallest = right;
      }
      if (smallest === index) break;
      [this.items[index], this.items[smallest]] = [this.items[smallest], this.items[index]];
      index = smallest;
    }
  }
}

function setupInputs() {
  return [
    els.mapResolution,
    els.mapOriginX,
    els.mapOriginY,
    els.mapOriginYaw,
    els.gridCellSize,
    els.plannerHardClearanceRange,
    els.plannerHardClearance,
    els.showGrid,
    els.showInflation,
    els.showLidarPoints,
    els.detectLidarObstacles,
    els.detectBlackWalls,
    els.robotIp,
    els.serverIp,
    els.rosDomainId,
    els.rosLocalhostOnly,
    els.robotSshHost,
    els.robotSshUser,
    els.robotSshPassword,
    els.initialX,
    els.initialY,
    els.initialYaw,
    els.robotLength,
    els.robotWidth,
    els.accessoryFront,
    els.accessoryBack,
    els.accessoryLeft,
    els.accessoryRight,
    els.accessoryHeight,
    els.safetyMargin,
    els.objectWidth,
    els.objectHeight,
    els.objectInflation,
    els.fallbackEnabled,
    els.fallbackMaxLinear,
    els.fallbackMinLinear,
    els.fallbackMaxAngular,
    els.fallbackLookahead,
    els.fallbackSoftDistanceRange,
    els.fallbackSoftDistance,
    els.fallbackHardMargin,
    els.fallbackScanTimeoutRange,
    els.fallbackScanTimeout,
    els.fallbackOdomTimeoutRange,
    els.fallbackOdomTimeout,
    els.fallbackCollisionHorizon,
    els.topicScan,
    els.topicPose,
    els.topicOdom,
    els.topicCamera,
    els.topicCompressedCamera,
    els.topicInitialPose,
    els.topicGoalAction,
    els.topicRouteAction,
    els.topicGoalTopic,
    els.topicCmdVel,
  ];
}

function markSetupDirty() {
  state.setupDirty = true;
  if (!state.setupSaving) setSetupSaveStatus("dirty", "변경사항 있음");
  syncSetupShadow();
}

function syncSetupShadow() {
  if (!state.data?.setup) return;
  const setup = currentSetup();
  state.data.setup = {
    ...state.data.setup,
    map: { ...state.data.setup.map, ...setup.map },
    initialPose: setup.initialPose,
    robot: setup.robot,
    accessory: setup.accessory,
    safety: setup.safety,
    object: setup.object,
    fallbackNavigation: setup.fallbackNavigation,
    obstacles: setup.obstacles,
    planner: setup.planner,
    network: setup.network,
    activeRobot: setup.activeRobot,
    robotProfiles: setup.robotProfiles,
    topics: setup.topics,
  };
  updateFootprintReadout(setup);
  planPathToGoal();
}

function setSetupTool(tool) {
  state.setupTool = tool;
  document.querySelectorAll("[data-setup-tool]").forEach((button) => {
    button.classList.toggle("active", button.dataset.setupTool === tool);
  });
  els.setupMapCanvas.dataset.tool = tool;
  const hints = {
    pose: "초기: 지도를 클릭하거나 드래그해 시작 위치와 방향을 지정합니다.",
    robot: "몸체: 로봇 외곽을 드래그해 길이와 너비를 조절합니다.",
    accessory: "부속품: 로봇 가장자리를 드래그해 돌출부를 조절합니다.",
    object: "오브젝트: 지도에서 클릭·드래그를 끝내면 장애물로 등록됩니다. 주황 점선은 드래그 중 미리보기이며 저장 대상이 아닙니다.",
    wall: "벽: 지도 위를 드래그해 통행 불가 셀을 칠합니다.",
    erase: "지우기: 지도 위를 드래그해 직접 칠한 벽을 지웁니다.",
  };
  if (els.setupToolHint) els.setupToolHint.textContent = hints[tool] || "";
}

function onSetupPointerDown(event) {
  const world = canvasEventToWorld(event, els.setupMapCanvas);
  const point = canvasEventToCanvasPoint(event, els.setupMapCanvas);
  if (!world) return;
  event.preventDefault();
  const setup = currentSetup();
  const movingObstacle = state.setupTool === "object" ? obstacleAtPoint(world, setup) : null;
  if (state.setupTool === "object") {
    state.setupObjectRect = movingObstacle ? null : objectRectFromCenter(world, setup.object);
  }
  els.setupMapCanvas.setPointerCapture(event.pointerId);
  state.setupDrag = {
    tool: state.setupTool,
    start: world,
    startPoint: point,
    setup,
    movingObstacleId: movingObstacle?.id || null,
    movingObstacleStart: movingObstacle || null,
  };
  updateSetupFromPointer(world, point);
}

function onSetupPointerMove(event) {
  if (!state.setupDrag) return;
  const world = canvasEventToWorld(event, els.setupMapCanvas);
  const point = canvasEventToCanvasPoint(event, els.setupMapCanvas);
  if (!world) return;
  event.preventDefault();
  updateSetupFromPointer(world, point);
}

function onSetupPointerUp(event) {
  if (!state.setupDrag) return;
  const drag = state.setupDrag;
  const world = canvasEventToWorld(event, els.setupMapCanvas);
  const point = canvasEventToCanvasPoint(event, els.setupMapCanvas);
  if (world) updateSetupFromPointer(world, point);
  if (drag.tool === "object" && !drag.movingObstacleId && state.setupObjectRect) {
    addObstacle(state.setupObjectRect);
    state.setupObjectRect = null;
  }
  if (els.setupMapCanvas.hasPointerCapture(event.pointerId)) {
    els.setupMapCanvas.releasePointerCapture(event.pointerId);
  }
  state.setupDrag = null;
}

function updateSetupFromPointer(world, point) {
  const tool = state.setupDrag?.tool || state.setupTool;
  if (tool === "wall" || tool === "erase") {
    paintBlockedCell(world, tool === "wall");
    return;
  }

  if (tool === "pose") {
    const start = state.setupDrag?.start || world;
    const dx = world.x - start.x;
    const dy = world.y - start.y;
    const yaw = Math.hypot(dx, dy) > 0.02 ? Math.atan2(dy, dx) : numberValue(els.initialYaw);
    setNumber(els.initialX, start.x);
    setNumber(els.initialY, start.y);
    setNumber(els.initialYaw, yaw);
  }

  if (tool === "robot") {
    const setup = state.setupDrag?.setup || currentSetup();
    const startPoint = state.setupDrag?.startPoint || point;
    const pixelToMeter = canvasPixelToMeter(els.setupMapCanvas, setup);
    const startLocalPixels = canvasToPoseLocal(startPoint, setup.initialPose, els.setupMapCanvas);
    const currentLocalPixels = canvasToPoseLocal(point, setup.initialPose, els.setupMapCanvas);
    const startLocal = {
      x: startLocalPixels.x * pixelToMeter,
      y: startLocalPixels.y * pixelToMeter,
    };
    const currentLocal = {
      x: currentLocalPixels.x * pixelToMeter,
      y: currentLocalPixels.y * pixelToMeter,
    };
    const lengthSign = startLocal.x >= 0 ? 1 : -1;
    const widthSign = startLocal.y >= 0 ? 1 : -1;
    const length = setup.robot.length + (currentLocal.x - startLocal.x) * lengthSign * 2;
    const width = setup.robot.width + (currentLocal.y - startLocal.y) * widthSign * 2;
    setNumber(els.robotLength, Math.max(0.05, length));
    setNumber(els.robotWidth, Math.max(0.05, width));
  }

  if (tool === "accessory") {
    updateAccessoryFromPointer(point);
  }

  if (tool === "object") {
    const drag = state.setupDrag;
    const setup = currentSetup();
    if (drag?.movingObstacleId && drag.movingObstacleStart) {
      state.setupObjectRect = null;
      moveObstacle(drag.movingObstacleId, {
        x: drag.movingObstacleStart.x + (world.x - drag.start.x),
        y: drag.movingObstacleStart.y + (world.y - drag.start.y),
      });
      return;
    }
    state.setupObjectRect = objectRectFromCenter(world, setup.object);
    markSetupDirty();
    return;
  }

  markSetupDirty();
}

function clearBlockedCells() {
  if (!state.data?.setup) return;
  state.data.setup.planner = {
    ...(state.data.setup.planner || {}),
    blockedCells: [],
    freeCells: [],
  };
  state.setupDirty = true;
  planPathToGoal();
  toast("장애물 셀을 비웠습니다.");
}

function paintBlockedCell(world, blocked) {
  if (!state.data?.setup) return;
  const setup = currentSetup();
  const cell = worldToGrid(world, setup);
  if (!cell) return;
  const blockedSet = new Set(normalizeBlockedCells(setup.planner?.blockedCells || []));
  const freeSet = new Set(normalizeBlockedCells(setup.planner?.freeCells || []));
  const key = cellKey(cell.x, cell.y);
  const mapBlocked = mapWallCellSet(setup).has(key);
  if (blocked) {
    blockedSet.add(key);
    freeSet.delete(key);
  } else {
    blockedSet.delete(key);
    if (mapBlocked) {
      freeSet.add(key);
    } else {
      freeSet.delete(key);
    }
  }
  state.data.setup.planner = {
    ...(state.data.setup.planner || {}),
    blockedCells: Array.from(blockedSet).sort(sortCellKeys),
    freeCells: Array.from(freeSet).sort(sortCellKeys),
  };
  state.setupDirty = true;
  syncSetupShadow();
}

function worldToPoseLocal(world, pose) {
  const dx = world.x - pose.x;
  const dy = world.y - pose.y;
  const cos = Math.cos(pose.yaw);
  const sin = Math.sin(pose.yaw);
  return {
    x: dx * cos + dy * sin,
    y: -dx * sin + dy * cos,
  };
}

function canvasToPoseLocal(point, pose, canvas) {
  const center = worldToCanvas(pose.x, pose.y, canvas);
  if (!center) return { x: 0, y: 0 };
  const dx = point.x - center.x;
  const dy = point.y - center.y;
  const cos = Math.cos(pose.yaw);
  const sin = Math.sin(pose.yaw);
  return {
    x: dx * cos - dy * sin,
    y: dx * sin + dy * cos,
  };
}

function canvasDeltaToPoseLocal(startPoint, point, pose) {
  const dx = point.x - startPoint.x;
  const dy = point.y - startPoint.y;
  const cos = Math.cos(pose.yaw);
  const sin = Math.sin(pose.yaw);
  return {
    x: dx * cos - dy * sin,
    y: dx * sin + dy * cos,
  };
}

function canvasPixelToMeter(canvas, setup) {
  const transform = getMapTransform(canvas);
  const resolution = Number(setup.map?.resolution) || mapResolution();
  return resolution / transform.scale;
}

function objectRectFromCenter(center, object) {
  return {
    x: center.x,
    y: center.y,
    width: Math.max(0.01, Number(object?.width) || 0.22),
    height: Math.max(0.01, Number(object?.height) || 0.2),
  };
}

function updateAccessoryFromPointer(point) {
  const setup = currentSetup();
  const localPixels = canvasToPoseLocal(point, setup.initialPose, els.setupMapCanvas);
  const pixelToMeter = canvasPixelToMeter(els.setupMapCanvas, setup);
  const local = {
    x: localPixels.x * pixelToMeter,
    y: localPixels.y * pixelToMeter,
  };
  const halfLength = setup.robot.length / 2;
  const halfWidth = setup.robot.width / 2;
  const front = Math.max(0, local.x - halfLength);
  const back = Math.max(0, -local.x - halfLength);
  const left = Math.max(0, -local.y - halfWidth);
  const right = Math.max(0, local.y - halfWidth);

  if (Math.abs(local.x) >= Math.abs(local.y)) {
    if (local.x >= 0) {
      setNumber(els.accessoryFront, front);
    } else {
      setNumber(els.accessoryBack, back);
    }
  } else if (local.y < 0) {
    setNumber(els.accessoryLeft, left);
  } else {
    setNumber(els.accessoryRight, right);
  }
}

function resizeCanvases() {
  for (const canvas of [els.setupMapCanvas, els.driveMapCanvas, els.mapEditorCanvas]) {
    if (!canvas) continue;
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(320, Math.floor(rect.width * ratio));
    const height = Math.max(280, Math.floor(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
  }
}

function editorDimension(input) {
  const value = Number(input.value);
  const rounded = Math.round(value);
  if (!Number.isFinite(value) || Math.abs(value - rounded) > 1e-6 || rounded < 10 || rounded > 2000) {
    throw new Error("맵 크기는 10~2000cm 정수로 입력하세요.");
  }
  return rounded;
}

function editorCmPerPixel() {
  const value = Number(els.mapEditorCmPerPixel.value);
  if (!Number.isFinite(value) || value < 0.1 || value > 100) {
    throw new Error("1픽셀당 크기는 0.1~100cm로 입력하세요.");
  }
  return value;
}

function updateMapEditorInfo(status = null) {
  const raster = state.mapEditor.raster;
  els.mapEditorSize.textContent = raster
    ? `${state.mapEditor.widthCm} x ${state.mapEditor.heightCm}cm · ${raster.width} x ${raster.height}px · 1px = ${state.mapEditor.cmPerPixel}cm`
    : "-";
  if (status !== null) els.mapEditorStatus.textContent = status;
}

function createMapEditorGrid() {
  try {
    const width = editorDimension(els.mapEditorWidth);
    const height = editorDimension(els.mapEditorHeight);
    const cmPerPixel = editorCmPerPixel();
    const widthPixels = width / cmPerPixel;
    const heightPixels = height / cmPerPixel;
    if (Math.abs(widthPixels - Math.round(widthPixels)) > 1e-6 || Math.abs(heightPixels - Math.round(heightPixels)) > 1e-6) {
      throw new Error("가로와 세로는 1픽셀당 cm 값으로 나누어떨어져야 합니다.");
    }
    const raster = document.createElement("canvas");
    raster.width = Math.round(widthPixels);
    raster.height = Math.round(heightPixels);
    const context = raster.getContext("2d", { willReadFrequently: true });
    context.fillStyle = "#ffffff";
    context.fillRect(0, 0, width, height);
    state.mapEditor.raster = raster;
    state.mapEditor.context = context;
    state.mapEditor.lastPixel = null;
    state.mapEditor.dirty = false;
    state.mapEditor.widthCm = width;
    state.mapEditor.heightCm = height;
    state.mapEditor.cmPerPixel = cmPerPixel;
    state.mapEditor.zoom = 1;
    state.mapEditor.panX = 0;
    state.mapEditor.panY = 0;
    state.mapEditor.loadedMapId = null;
    els.mapEditorLoadSelect.value = "";
    renderMapEditorLibrary(state.data?.setup || {});
    updateMapEditorInfo("그리드 준비됨");
  } catch (error) {
    toast(error.message);
  }
}

function setMapEditorTool(tool) {
  state.mapEditor.tool = tool;
  els.mapEditorPenButton.classList.toggle("active", tool === "pen");
  els.mapEditorEraserButton.classList.toggle("active", tool === "eraser");
  updateMapEditorInfo(tool === "pen" ? "펜 선택" : "지우개 선택");
}

function clearMapEditorGrid() {
  const { raster, context } = state.mapEditor;
  if (!raster || !context) return;
  context.fillStyle = "#ffffff";
  context.fillRect(0, 0, raster.width, raster.height);
  state.mapEditor.dirty = true;
  updateMapEditorInfo("벽을 모두 지웠습니다.");
}

function mapEditorTransform(canvas) {
  const raster = state.mapEditor.raster;
  if (!raster) return null;
  const margin = 20 * (window.devicePixelRatio || 1);
  const fitScale = Math.max(
    0.05,
    Math.min((canvas.width - margin * 2) / raster.width, (canvas.height - margin * 2) / raster.height),
  );
  const scale = fitScale * state.mapEditor.zoom;
  return {
    x: (canvas.width - raster.width * scale) / 2 + state.mapEditor.panX,
    y: (canvas.height - raster.height * scale) / 2 + state.mapEditor.panY,
    scale,
    fitScale,
  };
}

function drawMapEditor() {
  const canvas = els.mapEditorCanvas;
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#10171b";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const { raster } = state.mapEditor;
  if (!raster) return;
  const transform = mapEditorTransform(canvas);
  state.mapEditor.transform = transform;
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(
    raster,
    transform.x,
    transform.y,
    raster.width * transform.scale,
    raster.height * transform.scale,
  );
  if (transform.scale >= 2) {
    ctx.beginPath();
    const startX = Math.max(0, Math.floor(-transform.x / transform.scale));
    const endX = Math.min(raster.width, Math.ceil((canvas.width - transform.x) / transform.scale));
    const startY = Math.max(0, Math.floor(-transform.y / transform.scale));
    const endY = Math.min(raster.height, Math.ceil((canvas.height - transform.y) / transform.scale));
    for (let x = startX; x <= endX; x += 1) {
      const px = transform.x + x * transform.scale;
      ctx.moveTo(px, transform.y);
      ctx.lineTo(px, transform.y + raster.height * transform.scale);
    }
    for (let y = startY; y <= endY; y += 1) {
      const py = transform.y + y * transform.scale;
      ctx.moveTo(transform.x, py);
      ctx.lineTo(transform.x + raster.width * transform.scale, py);
    }
    ctx.strokeStyle = "rgba(91, 111, 121, 0.32)";
    ctx.lineWidth = 1;
    ctx.stroke();
  }
  ctx.strokeStyle = "#6b7b83";
  ctx.lineWidth = 1.5;
  ctx.strokeRect(transform.x, transform.y, raster.width * transform.scale, raster.height * transform.scale);
  ctx.restore();
}

function mapEditorPointerPixel(event) {
  const { raster } = state.mapEditor;
  const transform = state.mapEditor.transform || mapEditorTransform(els.mapEditorCanvas);
  if (!raster || !transform) return null;
  const rect = els.mapEditorCanvas.getBoundingClientRect();
  const canvasX = (event.clientX - rect.left) * (els.mapEditorCanvas.width / rect.width);
  const canvasY = (event.clientY - rect.top) * (els.mapEditorCanvas.height / rect.height);
  const x = Math.floor((canvasX - transform.x) / transform.scale);
  const y = Math.floor((canvasY - transform.y) / transform.scale);
  if (x < 0 || y < 0 || x >= raster.width || y >= raster.height) return null;
  return { x, y };
}

function paintMapEditorCell(point) {
  const { raster, context, tool } = state.mapEditor;
  if (!raster || !context || !point) return;
  context.fillStyle = tool === "eraser" ? "#ffffff" : "#000000";
  context.fillRect(point.x, point.y, 1, 1);
}

function paintMapEditorLine(from, to) {
  let x = from.x;
  let y = from.y;
  const dx = Math.abs(to.x - x);
  const dy = Math.abs(to.y - y);
  const sx = x < to.x ? 1 : -1;
  const sy = y < to.y ? 1 : -1;
  let error = dx - dy;
  while (true) {
    paintMapEditorCell({ x, y });
    if (x === to.x && y === to.y) break;
    const doubled = error * 2;
    if (doubled > -dy) {
      error -= dy;
      x += sx;
    }
    if (doubled < dx) {
      error += dx;
      y += sy;
    }
  }
}

function onMapEditorPointerDown(event) {
  if (!state.mapEditor.raster) {
    toast("먼저 맵 크기를 입력하고 그리드를 생성하세요.");
    return;
  }
  const point = mapEditorPointerPixel(event);
  if (event.shiftKey) {
    state.mapEditor.panning = true;
    state.mapEditor.lastPanClient = { x: event.clientX, y: event.clientY };
    els.mapEditorCanvas.classList.add("panning");
    els.mapEditorCanvas.setPointerCapture(event.pointerId);
    return;
  }
  if (!point) return;
  state.mapEditor.drawing = true;
  state.mapEditor.lastPixel = point;
  state.mapEditor.dirty = true;
  paintMapEditorCell(point);
  els.mapEditorCanvas.setPointerCapture(event.pointerId);
  updateMapEditorInfo(state.mapEditor.tool === "pen" ? "벽 편집 중" : "벽 지우는 중");
}

function onMapEditorPointerMove(event) {
  if (state.mapEditor.panning && state.mapEditor.lastPanClient) {
    const rect = els.mapEditorCanvas.getBoundingClientRect();
    const ratioX = els.mapEditorCanvas.width / rect.width;
    const ratioY = els.mapEditorCanvas.height / rect.height;
    state.mapEditor.panX += (event.clientX - state.mapEditor.lastPanClient.x) * ratioX;
    state.mapEditor.panY += (event.clientY - state.mapEditor.lastPanClient.y) * ratioY;
    state.mapEditor.lastPanClient = { x: event.clientX, y: event.clientY };
    return;
  }
  if (!state.mapEditor.drawing) return;
  const point = mapEditorPointerPixel(event);
  if (!point || !state.mapEditor.lastPixel) return;
  paintMapEditorLine(state.mapEditor.lastPixel, point);
  state.mapEditor.lastPixel = point;
}

function onMapEditorPointerUp(event) {
  if (state.mapEditor.panning) {
    state.mapEditor.panning = false;
    state.mapEditor.lastPanClient = null;
    els.mapEditorCanvas.classList.remove("panning");
    if (els.mapEditorCanvas.hasPointerCapture(event.pointerId)) {
      els.mapEditorCanvas.releasePointerCapture(event.pointerId);
    }
    return;
  }
  if (!state.mapEditor.drawing) return;
  const point = mapEditorPointerPixel(event);
  if (point && state.mapEditor.lastPixel) paintMapEditorLine(state.mapEditor.lastPixel, point);
  state.mapEditor.drawing = false;
  state.mapEditor.lastPixel = null;
  if (els.mapEditorCanvas.hasPointerCapture(event.pointerId)) {
    els.mapEditorCanvas.releasePointerCapture(event.pointerId);
  }
  updateMapEditorInfo("편집됨");
}

function onMapEditorWheel(event) {
  const { raster } = state.mapEditor;
  if (!raster) return;
  event.preventDefault();
  const canvas = els.mapEditorCanvas;
  const rect = canvas.getBoundingClientRect();
  const canvasX = (event.clientX - rect.left) * (canvas.width / rect.width);
  const canvasY = (event.clientY - rect.top) * (canvas.height / rect.height);
  const before = state.mapEditor.transform || mapEditorTransform(canvas);
  const rasterX = (canvasX - before.x) / before.scale;
  const rasterY = (canvasY - before.y) / before.scale;
  const factor = event.deltaY < 0 ? 1.15 : 1 / 1.15;
  state.mapEditor.zoom = Math.max(0.25, Math.min(30, state.mapEditor.zoom * factor));
  const after = mapEditorTransform(canvas);
  state.mapEditor.panX += canvasX - (after.x + rasterX * after.scale);
  state.mapEditor.panY += canvasY - (after.y + rasterY * after.scale);
}

async function loadEditorMap() {
  const id = els.mapEditorLoadSelect.value;
  const entry = mapLibraryEntries(state.data?.setup || {}).find((item) => item.id === id);
  if (!entry) {
    state.mapEditor.loadedMapId = null;
    renderMapEditorLibrary(state.data?.setup || {});
    return;
  }
  const image = new Image();
  image.onload = () => {
    const raster = document.createElement("canvas");
    raster.width = image.naturalWidth;
    raster.height = image.naturalHeight;
    const context = raster.getContext("2d", { willReadFrequently: true });
    context.imageSmoothingEnabled = false;
    context.fillStyle = "#ffffff";
    context.fillRect(0, 0, raster.width, raster.height);
    context.drawImage(image, 0, 0);
    const cmPerPixel = (Number(entry.resolution) || 0.01) * 100;
    state.mapEditor.raster = raster;
    state.mapEditor.context = context;
    state.mapEditor.widthCm = Math.round(raster.width * cmPerPixel);
    state.mapEditor.heightCm = Math.round(raster.height * cmPerPixel);
    state.mapEditor.cmPerPixel = cmPerPixel;
    state.mapEditor.zoom = 1;
    state.mapEditor.panX = 0;
    state.mapEditor.panY = 0;
    state.mapEditor.dirty = false;
    state.mapEditor.loadedMapId = entry.id;
    els.mapEditorName.value = entry.name || "새 맵";
    els.mapEditorWidth.value = String(state.mapEditor.widthCm);
    els.mapEditorHeight.value = String(state.mapEditor.heightCm);
    els.mapEditorCmPerPixel.value = String(round(cmPerPixel));
    renderMapEditorLibrary(state.data?.setup || {});
    updateMapEditorInfo("저장된 맵을 불러왔습니다.");
  };
  image.onerror = () => {
    toast("저장된 맵 이미지를 불러오지 못했습니다.");
  };
  image.src = entry.imageUrl;
}

async function deleteEditorMap() {
  const id = els.mapEditorLoadSelect.value || state.mapEditor.loadedMapId;
  const entry = mapLibraryEntries(state.data?.setup || {}).find((item) => item.id === id);
  if (!entry || entry.id === "default-map") return;
  if (!window.confirm(`'${entry.name}' 맵을 삭제할까요?`)) return;
  els.mapEditorDeleteButton.disabled = true;
  try {
    const result = await postJson("/api/maps/delete", { id: entry.id });
    state.mapEditor.loadedMapId = null;
    state.setupDirty = false;
    if (result.state) applyState(result.state);
    updateMapEditorInfo("저장된 맵을 삭제했습니다.");
    toast("맵을 삭제했습니다.");
  } catch (error) {
    toast(error.message);
  } finally {
    renderMapEditorLibrary(state.data?.setup || {});
  }
}

async function saveEditorMap() {
  const raster = state.mapEditor.raster;
  if (!raster) {
    toast("저장할 그리드를 먼저 생성하세요.");
    return;
  }
  const name = els.mapEditorName.value.trim() || "새 맵";
  const widthCm = state.mapEditor.widthCm;
  const heightCm = state.mapEditor.heightCm;
  els.mapEditorSaveButton.disabled = true;
  try {
    const result = await postJson("/api/maps", {
      name,
      widthCm,
      heightCm,
      cmPerPixel: state.mapEditor.cmPerPixel,
      imageDataUrl: raster.toDataURL("image/png"),
    });
    state.mapEditor.dirty = false;
    state.mapEditor.loadedMapId = result.map?.id || null;
    if (result.state) applyState(result.state);
    updateMapEditorInfo("저장됨");
    toast(`맵 '${name}'을 저장했습니다.`);
  } catch (error) {
    toast(error.message);
  } finally {
    els.mapEditorSaveButton.disabled = false;
  }
}

function startCameraLoop() {
  const refresh = () => {
    if (!state.cameraEnabled) return;
    const streamMode = state.data?.runtime?.cameraStream === "raw" ? "raw" : "compressed";
    if (streamMode !== "raw") {
      updateCameraTransport();
      return;
    }
    els.cameraFrame.dataset.cameraState = "on";
    els.cameraFrame.dataset.cameraTransport = "raw";
    els.cameraFrame.src = `/api/camera/frame?t=${Date.now()}`;
  };
  els.cameraFrame.addEventListener("error", () => {
    if (
      state.cameraEnabled &&
      state.data?.runtime?.cameraStream !== "raw" &&
      els.cameraFrame.dataset.cameraTransport === "mjpeg"
    ) {
      els.cameraFrame.dataset.cameraTransport = "";
      window.setTimeout(() => updateCameraTransport(true), 500);
    }
  });
  refresh();
  setInterval(refresh, 180);
}

function startDrawLoop() {
  const draw = () => {
    drawMap(els.setupMapCanvas, "setup");
    drawMap(els.driveMapCanvas, "drive");
    drawMapEditor();
    requestAnimationFrame(draw);
  };
  requestAnimationFrame(draw);
}

function drawMap(canvas, mode) {
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#10171b";
  ctx.fillRect(0, 0, width, height);

  const transform = getMapTransform(canvas);
  drawGrid(ctx, width, height, transform);

  if (state.mapLoaded) {
    ctx.save();
    ctx.translate(transform.x, transform.y);
    ctx.scale(transform.scale, transform.scale);
    ctx.drawImage(state.mapImage, 0, 0);
    ctx.restore();
  }

  if (!state.data) return;
  const runtime = state.data.runtime;
  const setup = mode === "setup" ? currentSetup() : state.data.setup;
  const initialPose = {
    ...setup.initialPose,
    source: "initial",
  };
  const goal = mode === "drive" ? state.selectedGoal || runtime.goal : null;

  drawPlanningOverlay(ctx, canvas, setup, mode);
  drawLidarPoints(ctx, canvas, runtime, setup);
  drawAvoidanceRobots(ctx, canvas, setup);
  if (mode === "drive") drawLidarConnectionAlarm(ctx, canvas, runtime);
  if (mode === "drive") {
    drawWaypointMarkers(ctx, canvas);
  }

  if (mode === "setup") {
    drawSetupObject(ctx, canvas, setup, initialPose);
    drawEffectiveFootprint(ctx, canvas, initialPose, setup, "#5aa6d6", "유효");
    drawPose(ctx, canvas, initialPose, setup.robot, "#e8b85f", "초기");
    drawSetupAids(ctx, canvas, setup, initialPose);
  }
  if (runtime.pose) {
    drawEffectiveFootprint(ctx, canvas, runtime.pose, setup, "rgba(79, 180, 119, 0.65)", "");
    drawPose(ctx, canvas, runtime.pose, setup.robot, "#4fb477", "TB3");
  }
  if (goal) {
    drawGoal(ctx, canvas, goal);
  }
}

function drawLidarConnectionAlarm(ctx, canvas, runtime) {
  const connection = runtime.lidarConnection || "unknown";
  const messages = {
    connected: "LiDAR 연결 정상 · 속도 복구",
    degraded: "LiDAR 연결 끊김 · 저속 주행",
    lost: "LiDAR 연결 끊김 · 안전 정지",
    unknown: "LiDAR scan 대기 중",
  };
  const colors = {
    connected: { fill: "rgba(30, 102, 72, 0.93)", stroke: "#4fb477" },
    degraded: { fill: "rgba(105, 76, 20, 0.94)", stroke: "#e8b85f" },
    lost: { fill: "rgba(112, 42, 42, 0.94)", stroke: "#df6262" },
    unknown: { fill: "rgba(49, 62, 69, 0.94)", stroke: "#9eaeb6" },
  };
  const message = runtime.lidarConnectionMessage || messages[connection] || messages.unknown;
  const color = colors[connection] || colors.unknown;
  ctx.save();
  ctx.font = "600 13px Inter, system-ui, sans-serif";
  const width = Math.min(canvas.width - 24, Math.ceil(ctx.measureText(message).width) + 28);
  const height = 32;
  const x = canvas.width - width - 12;
  const y = 12;
  ctx.fillStyle = color.fill;
  ctx.strokeStyle = color.stroke;
  ctx.lineWidth = 1;
  ctx.fillRect(x, y, width, height);
  ctx.strokeRect(x + 0.5, y + 0.5, width - 1, height - 1);
  ctx.fillStyle = "#f5fbfc";
  ctx.textBaseline = "middle";
  ctx.fillText(message, x + 14, y + height / 2);
  ctx.restore();
}

function drawLidarPoints(ctx, canvas, runtime, setup) {
  const pose = runtime.lidarPose || runtime.pose;
  if (!setup.planner?.showLidarPoints || !pose) return;
  const points = Array.isArray(runtime.lidarPoints) ? runtime.lidarPoints : [];
  if (points.length === 0) return;
  const cosYaw = Math.cos(Number(pose.yaw) || 0);
  const sinYaw = Math.sin(Number(pose.yaw) || 0);
  const pointRadius = Math.max(1.5, Math.min(3, canvas.width / 500));
  ctx.save();
  ctx.fillStyle = "rgba(40, 210, 205, 0.88)";
  ctx.beginPath();
  for (const point of points) {
    if (!Array.isArray(point) || point.length < 2) continue;
    const localX = Number(point[0]);
    const localY = Number(point[1]);
    if (!Number.isFinite(localX) || !Number.isFinite(localY)) continue;
    const worldX = Number(pose.x) + cosYaw * localX - sinYaw * localY;
    const worldY = Number(pose.y) + sinYaw * localX + cosYaw * localY;
    const canvasPoint = worldToCanvas(worldX, worldY, canvas);
    if (!canvasPoint) continue;
    if (
      canvasPoint.x < 0 ||
      canvasPoint.y < 0 ||
      canvasPoint.x > canvas.width ||
      canvasPoint.y > canvas.height
    ) {
      continue;
    }
    ctx.moveTo(canvasPoint.x + pointRadius, canvasPoint.y);
    ctx.arc(canvasPoint.x, canvasPoint.y, pointRadius, 0, Math.PI * 2);
  }
  ctx.fill();
  ctx.restore();
}

function drawPlanningOverlay(ctx, canvas, setup, mode) {
  const occupied = blockedCellSet(setup);
  const dynamicOccupied = dynamicLidarObstacleCellKeys(setup);
  const staticOccupied = new Set(Array.from(occupied).filter((key) => !dynamicOccupied.has(key)));
  const hardInflated = displayedHardInflatedCellSet(setup);
  const softInflated = displayedSoftInflatedCellSet(setup);
  const free = new Set(normalizeBlockedCells(setup.planner?.freeCells || []));
  if (setup.planner?.showInflation) {
    const hardOnly = new Set(Array.from(hardInflated).filter((key) => !occupied.has(key)));
    const softOnly = new Set(Array.from(softInflated).filter((key) => !hardInflated.has(key)));
    drawCellSet(ctx, canvas, setup, softOnly, "rgba(232, 184, 95, 0.18)", 12000);
    drawCellSet(ctx, canvas, setup, hardOnly, "rgba(223, 98, 98, 0.22)", 12000);
  }
  drawCellSet(ctx, canvas, setup, free, "rgba(79, 180, 119, 0.34)", 12000);
  drawCellSet(ctx, canvas, setup, staticOccupied, "rgba(223, 98, 98, 0.48)", 12000);
  drawCellSet(ctx, canvas, setup, dynamicOccupied, "rgba(239, 165, 71, 0.62)", 12000);
  if (setup.planner?.showGrid) {
    drawPlannerGrid(ctx, canvas, setup);
  }
  if (mode === "drive" || mode === "setup") {
    drawPlannedPath(ctx, canvas);
  }
}

function drawCellSet(ctx, canvas, setup, cells, color, maxCells) {
  if (!cells || cells.size === 0) return;
  ctx.save();
  ctx.fillStyle = color;
  let count = 0;
  for (const key of cells) {
    if (count >= maxCells) break;
    const rect = gridCellCanvasRect(parseCellKey(key), canvas, setup);
    if (rect) ctx.fillRect(rect.x, rect.y, rect.width, rect.height);
    count += 1;
  }
  ctx.restore();
}

function drawPlannerGrid(ctx, canvas, setup) {
  const metrics = mapMetrics(setup);
  const start = worldToCanvas(metrics.originX, metrics.originY, canvas);
  const next = worldToCanvas(metrics.originX + metrics.cellSize, metrics.originY + metrics.cellSize, canvas);
  if (!start || !next) return;
  const cellPixels = Math.abs(next.x - start.x);
  if (cellPixels < 7 || metrics.cols + metrics.rows > 900) return;

  ctx.save();
  ctx.strokeStyle = "rgba(154, 174, 182, 0.24)";
  ctx.lineWidth = 1;
  for (let x = 0; x <= metrics.cols; x += 1) {
    const worldX = metrics.originX + x * metrics.cellSize;
    const p1 = worldToCanvas(worldX, metrics.originY, canvas);
    const p2 = worldToCanvas(worldX, metrics.originY + metrics.heightMeters, canvas);
    if (!p1 || !p2) continue;
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }
  for (let y = 0; y <= metrics.rows; y += 1) {
    const worldY = metrics.originY + y * metrics.cellSize;
    const p1 = worldToCanvas(metrics.originX, worldY, canvas);
    const p2 = worldToCanvas(metrics.originX + metrics.widthMeters, worldY, canvas);
    if (!p1 || !p2) continue;
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawPlannedPath(ctx, canvas) {
  const runtimePath = state.data?.runtime?.fallbackActive
    ? state.data.runtime.fallbackDisplayPath
    : null;
  const points = Array.isArray(runtimePath) && runtimePath.length > 0
    ? runtimePath
    : state.plannedPath?.points || [];
  if (points.length < 2) return;
  ctx.save();
  ctx.lineWidth = 4;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  for (let index = 1; index < points.length; index += 1) {
    const start = worldToCanvas(points[index - 1].x, points[index - 1].y, canvas);
    const end = worldToCanvas(points[index].x, points[index].y, canvas);
    if (!start || !end) continue;
    ctx.strokeStyle = points[index].slow ? "#e8b85f" : "#5aa6d6";
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
  }
  ctx.restore();
}

function gridCellCanvasRect(cell, canvas, setup) {
  const metrics = mapMetrics(setup);
  const left = metrics.originX + cell.x * metrics.cellSize;
  const right = left + metrics.cellSize;
  const bottom = metrics.originY + cell.y * metrics.cellSize;
  const top = bottom + metrics.cellSize;
  const p1 = worldToCanvas(left, top, canvas);
  const p2 = worldToCanvas(right, bottom, canvas);
  if (!p1 || !p2) return null;
  return {
    x: Math.min(p1.x, p2.x),
    y: Math.min(p1.y, p2.y),
    width: Math.abs(p2.x - p1.x),
    height: Math.abs(p2.y - p1.y),
  };
}

function drawGrid(ctx, width, height, transform) {
  ctx.save();
  ctx.strokeStyle = "#1d2930";
  ctx.lineWidth = 1;
  const gap = Math.max(18, transform.scale / 0.05);
  const startX = transform.x % gap;
  const startY = transform.y % gap;
  for (let x = startX; x < width; x += gap) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = startY; y < height; y += gap) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawPose(ctx, canvas, pose, robot, color, label) {
  const point = worldToCanvas(pose.x, pose.y, canvas);
  if (!point) return;
  const transform = getMapTransform(canvas);
  const resolution = mapResolution();
  const length = Math.max(14, (robot.length / resolution) * transform.scale);
  const width = Math.max(12, (robot.width / resolution) * transform.scale);

  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = color;
  ctx.strokeStyle = "#07130c";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.rect(-length / 2, -width / 2, length, width);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = "#07130c";
  ctx.beginPath();
  ctx.moveTo(length / 2 + 10, 0);
  ctx.lineTo(length / 2 - 4, -7);
  ctx.lineTo(length / 2 - 4, 7);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  ctx.save();
  ctx.fillStyle = "#e9f0f2";
  ctx.font = `${Math.max(11, Math.round(12 * (window.devicePixelRatio || 1)))}px sans-serif`;
  ctx.fillText(label, point.x + 10, point.y - 10);
  ctx.restore();
}

function drawEffectiveFootprint(ctx, canvas, pose, setup, color, label) {
  const center = worldToCanvas(pose.x, pose.y, canvas);
  if (!center) return;
  const transform = getMapTransform(canvas);
  const resolution = mapResolution();
  const footprint = effectiveFootprint(setup);
  const front = (footprint.front / resolution) * transform.scale;
  const back = (footprint.back / resolution) * transform.scale;
  const left = (footprint.left / resolution) * transform.scale;
  const right = (footprint.right / resolution) * transform.scale;

  ctx.save();
  ctx.translate(center.x, center.y);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = "rgba(90, 166, 214, 0.14)";
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.setLineDash([8, 5]);
  ctx.beginPath();
  ctx.moveTo(front, -left);
  ctx.lineTo(front, right);
  ctx.lineTo(-back, right);
  ctx.lineTo(-back, -left);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.setLineDash([]);
  if (setup.accessory?.height) {
    ctx.fillStyle = color;
    ctx.font = `${Math.max(10, Math.round(11 * (window.devicePixelRatio || 1)))}px sans-serif`;
    ctx.fillText(`h ${round(setup.accessory.height)}m`, -back, right + 16);
  }
  ctx.restore();

  if (label) {
    ctx.save();
    ctx.fillStyle = color;
    ctx.font = `${Math.max(11, Math.round(12 * (window.devicePixelRatio || 1)))}px sans-serif`;
    ctx.fillText(label, center.x + 14, center.y + 20);
    ctx.restore();
  }
}

function drawSetupAids(ctx, canvas, setup, pose) {
  const center = worldToCanvas(pose.x, pose.y, canvas);
  if (!center) return;

  if (state.setupTool === "pose") {
    const transform = getMapTransform(canvas);
    const resolution = mapResolution();
    const ray = Math.max(36, (setup.robot.length / resolution) * transform.scale * 1.2);
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(-pose.yaw);
    ctx.strokeStyle = "#e8b85f";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(ray, 0);
    ctx.stroke();
    drawHandle(ctx, ray, 0, "#e8b85f");
    ctx.restore();
  }

  if (state.setupTool === "robot") {
    const transform = getMapTransform(canvas);
    const resolution = mapResolution();
    const length = Math.max(14, (setup.robot.length / resolution) * transform.scale);
    const width = Math.max(12, (setup.robot.width / resolution) * transform.scale);
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(-pose.yaw);
    ctx.strokeStyle = "#f2cf83";
    ctx.setLineDash([8, 5]);
    ctx.strokeRect(-length / 2, -width / 2, length, width);
    ctx.setLineDash([]);
    drawHandle(ctx, length / 2, width / 2, "#f2cf83");
    drawHandle(ctx, length / 2, -width / 2, "#f2cf83");
    drawHandle(ctx, -length / 2, width / 2, "#f2cf83");
    drawHandle(ctx, -length / 2, -width / 2, "#f2cf83");
    ctx.restore();
  }

  if (state.setupTool === "accessory") {
    const transform = getMapTransform(canvas);
    const resolution = mapResolution();
    const footprint = effectiveFootprint(setup);
    const front = (footprint.front / resolution) * transform.scale;
    const back = (footprint.back / resolution) * transform.scale;
    const left = (footprint.left / resolution) * transform.scale;
    const right = (footprint.right / resolution) * transform.scale;
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(-pose.yaw);
    ctx.strokeStyle = "#5aa6d6";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-back, 0);
    ctx.lineTo(front, 0);
    ctx.moveTo(0, -left);
    ctx.lineTo(0, right);
    ctx.stroke();
    drawHandle(ctx, front, 0, "#5aa6d6");
    drawHandle(ctx, -back, 0, "#5aa6d6");
    drawHandle(ctx, 0, -left, "#5aa6d6");
    drawHandle(ctx, 0, right, "#5aa6d6");
    ctx.restore();
  }
}

function drawSetupObject(ctx, canvas, setup, pose) {
  normalizeObstacles(setup.obstacles || []).forEach((obstacle, index) => {
    drawObstacleRect(ctx, canvas, obstacle, `#${index + 1}`, false);
  });
  if (state.setupObjectRect) drawObstacleRect(ctx, canvas, state.setupObjectRect, "new", true);
}

function drawAvoidanceRobots(ctx, canvas, setup) {
  for (const robot of avoidanceRobotObstacles(setup)) {
    const center = worldToCanvas(robot.x, robot.y, canvas);
    if (!center) continue;
    const transform = getMapTransform(canvas);
    const resolution = mapResolution();
    const length = Math.max(8, (robot.bodyLength / resolution) * transform.scale);
    const width = Math.max(8, (robot.bodyWidth / resolution) * transform.scale);
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(-(Number(robot.yaw) || 0));
    ctx.fillStyle = "rgba(232, 184, 95, 0.32)";
    ctx.strokeStyle = "#e8b85f";
    ctx.lineWidth = 2;
    ctx.fillRect(-length / 2, -width / 2, length, width);
    ctx.strokeRect(-length / 2, -width / 2, length, width);
    ctx.restore();
    ctx.save();
    ctx.fillStyle = "#f5dfab";
    ctx.font = "600 12px Inter, system-ui, sans-serif";
    ctx.fillText(robot.label, center.x + 8, center.y - 8);
    ctx.restore();
  }
}

function drawObstacleRect(ctx, canvas, rect, label, dashed) {
  const topLeft = worldToCanvas(rect.x - rect.width / 2, rect.y + rect.height / 2, canvas);
  const bottomRight = worldToCanvas(rect.x + rect.width / 2, rect.y - rect.height / 2, canvas);
  if (!topLeft || !bottomRight) return;
  const x = Math.min(topLeft.x, bottomRight.x);
  const y = Math.min(topLeft.y, bottomRight.y);
  const width = Math.abs(bottomRight.x - topLeft.x);
  const height = Math.abs(bottomRight.y - topLeft.y);

  ctx.save();
  ctx.fillStyle = "rgba(90, 166, 214, 0.22)";
  ctx.strokeStyle = "#5aa6d6";
  ctx.lineWidth = 2;
  ctx.setLineDash(dashed ? [7, 5] : []);
  ctx.fillRect(x, y, width, height);
  ctx.strokeRect(x, y, width, height);
  ctx.setLineDash([]);
  if (dashed) {
    drawHandle(ctx, x + width, y + height, "#5aa6d6");
  }
  ctx.fillStyle = "#e9f0f2";
  ctx.font = `${Math.max(11, Math.round(12 * (window.devicePixelRatio || 1)))}px sans-serif`;
  ctx.fillText(label, x + 8, y - 8);
  ctx.restore();
}

function drawHandle(ctx, x, y, color) {
  const size = 8 * (window.devicePixelRatio || 1);
  ctx.save();
  ctx.fillStyle = color;
  ctx.strokeStyle = "#07130c";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.rect(x - size / 2, y - size / 2, size, size);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawWaypointMarkers(ctx, canvas) {
  state.waypoints.forEach((waypoint, index) => {
    const point = worldToCanvas(waypoint.x, waypoint.y, canvas);
    if (!point) return;
    const radius = 10 * (window.devicePixelRatio || 1);
    ctx.save();
    ctx.fillStyle = "#e8b85f";
    ctx.strokeStyle = "#07130c";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#07130c";
    ctx.font = `${Math.max(10, Math.round(11 * (window.devicePixelRatio || 1)))}px sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(String(index + 1), point.x, point.y);
    ctx.restore();
  });
}

function drawGoal(ctx, canvas, goal) {
  const point = worldToCanvas(goal.x, goal.y, canvas);
  if (!point) return;
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.strokeStyle = "#5aa6d6";
  ctx.fillStyle = "#5aa6d6";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(0, 0, 12, 0, Math.PI * 2);
  ctx.stroke();
  ctx.rotate(-goal.yaw);
  ctx.beginPath();
  ctx.moveTo(24, 0);
  ctx.lineTo(8, -7);
  ctx.lineTo(8, 7);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function getMapTransform(canvas) {
  const imgWidth = state.mapLoaded ? state.mapImage.naturalWidth : 800;
  const imgHeight = state.mapLoaded ? state.mapImage.naturalHeight : 600;
  const margin = 24 * (window.devicePixelRatio || 1);
  const scale = Math.min((canvas.width - margin * 2) / imgWidth, (canvas.height - margin * 2) / imgHeight);
  return {
    scale: Math.max(0.05, scale),
    x: (canvas.width - imgWidth * Math.max(0.05, scale)) / 2,
    y: (canvas.height - imgHeight * Math.max(0.05, scale)) / 2,
  };
}

function mapResolution() {
  return Number(state.data?.setup?.map?.resolution) || 0.05;
}

function mapOrigin() {
  const map = state.data?.setup?.map || {};
  return {
    x: Number(map.originX) || 0,
    y: Number(map.originY) || 0,
  };
}

function canvasEventToWorld(event, canvas) {
  const point = canvasEventToCanvasPoint(event, canvas);
  return canvasToWorld(point.x, point.y, canvas);
}

function canvasEventToCanvasPoint(event, canvas) {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  return {
    x: (event.clientX - rect.left) * ratio,
    y: (event.clientY - rect.top) * ratio,
  };
}

function canvasToWorld(canvasX, canvasY, canvas) {
  const transform = getMapTransform(canvas);
  const imgWidth = state.mapLoaded ? state.mapImage.naturalWidth : 800;
  const imgHeight = state.mapLoaded ? state.mapImage.naturalHeight : 600;
  const px = (canvasX - transform.x) / transform.scale;
  const py = (canvasY - transform.y) / transform.scale;
  if (px < 0 || py < 0 || px > imgWidth || py > imgHeight) return null;
  const origin = mapOrigin();
  const resolution = mapResolution();
  return {
    x: origin.x + px * resolution,
    y: origin.y + (imgHeight - py) * resolution,
  };
}

function worldToCanvas(worldX, worldY, canvas) {
  const transform = getMapTransform(canvas);
  const imgHeight = state.mapLoaded ? state.mapImage.naturalHeight : 600;
  const origin = mapOrigin();
  const resolution = mapResolution();
  const px = (worldX - origin.x) / resolution;
  const py = imgHeight - (worldY - origin.y) / resolution;
  return {
    x: transform.x + px * transform.scale,
    y: transform.y + py * transform.scale,
  };
}

async function fetchJson(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url} ${response.status}`);
  return response.json();
}

async function postJson(url, payload) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await response.json();
  if (!response.ok || data.ok === false) {
    throw new Error(data.error || `${url} ${response.status}`);
  }
  return data;
}

function showResult(result) {
  toast(result?.transport ? `전송 완료: ${result.transport}` : "완료");
}

let toastTimer = null;
function toast(message) {
  els.toast.textContent = message;
  els.toast.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => els.toast.classList.remove("show"), 2200);
}
