var ws = null;              // The websocket object
var systemd_mode = null;    // Holds the global systemd mode which adjusts the UI rendering
var errorShown = false;     // Used so that when we close the websocket, the close message won't overtake other errors

var baseTheme = {
  foreground: '#F8F8F8',
  background: '#222',
  selectionBackground: '#8A6500',
  selectionInactiveBackground: '#8A6500',
  selectionForeground: '#FFFFFF',
  black: '#1E1E1D',
  brightBlack: '#262625',
  red: '#CE5C5C',
  brightRed: '#FF7272',
  green: '#5BCC5B',
  brightGreen: '#72FF72',
  yellow: '#CCCC5B',
  brightYellow: '#FFFF72',
  blue: '#5D5DD3',
  brightBlue: '#7279FF',
  magenta: '#BC5ED1',
  brightMagenta: '#E572FF',
  cyan: '#5DA5D5',
  brightCyan: '#72F0FF',
  white: '#F8F8F8',
  brightWhite: '#FFFFFF'
};
var logLevelColorMap = {
  starting: "\x1B[1;36m",
  running: "\x1B[1;32m",
  stopping: "\x1B[1;38:5:208m",
  global: "\x1B[1;35m",
  global_error: "\x1B[1;31m",
  notice: "\x1B[1;33m"
}
const term = new Terminal({
  fontFamily: '"Cascadia Code", Menlo, monospace',
  theme: baseTheme,
  disableStdin: true,
  convertEol: true,
  scrollback: 100000
});
const fitAddon = new FitAddon.FitAddon();
term.loadAddon(fitAddon);

globalThis.term = term;

// ========================================
// Rendering Helpers
// ========================================

function refreshLaunchState(launchData) {
  var launchId = launchData.id;
  var topicData = launchData.monitored_topics;

  var progressElement = document.getElementById("launchentry-" + launchId + "-center");
  var progressBar = document.getElementById("launchentry-" + launchId + "-bar");
  var progressTitle = document.getElementById("launchentry-" + launchId + "-info");
  var startBtn = document.getElementById("launchentry-" + launchId + "-start");
  var stopBtn = document.getElementById("launchentry-" + launchId + "-end");

  var wasError = false;
  if (launchData.state == "stopped") {
    progressElement.style.width = "0%";
    progressTitle.textContent = "Stopped";
    progressBar.title = "";
    startBtn.disabled = false;
    stopBtn.disabled = true;
  }
  else if (launchData.state == "running") {
    var tooltip = "Pending Topics:";
    var numCompleted = 0;
    for (var i = 0; i < topicData.length; i++) {
      if (topicData[i].seen) {
        numCompleted++;
      }
      else {
        tooltip += "\n - " + topicData[i].name;
      }
    }

    if (numCompleted == topicData.length) {
      progressElement.style.width = "100%";
      progressTitle.textContent = "Running";
      progressBar.title = "";
    }
    else {
      progressElement.style.width = (numCompleted / topicData.length) * 100 + "%";
      progressTitle.textContent = numCompleted + "/" + topicData.length + " completed";
      progressBar.title = tooltip;
    }

    startBtn.disabled = true;
    stopBtn.disabled = false;
  }
  else if (launchData.state == "starting") {
    progressElement.style.width = "0%";
    progressTitle.textContent = "Starting...";
    progressBar.title = "";
    startBtn.disabled = true;
    stopBtn.disabled = true;
  }
  else if (launchData.state == "stopping") {
    progressTitle.textContent = "Stopping...";
    startBtn.disabled = true;
    stopBtn.disabled = true;
  }
  else if (launchData.state == "stopping_error") {
    progressTitle.textContent = "ERROR (Cleaning Up...)";
    startBtn.disabled = false;
    stopBtn.disabled = false;
  } else {
    progressElement.style.width = "0%";
    progressTitle.textContent = "ERROR!";
    progressBar.title = "";
    startBtn.disabled = false;
    stopBtn.disabled = true;
    wasError = true;
  }

  if (wasError) {
    progressTitle.classList.add("progress-info-error");
    progressBar.classList.add("progress-bar-error");
  }
  else {
    progressTitle.classList.remove("progress-info-error");
    progressBar.classList.remove("progress-bar-error");
  }

  var detailsElement = document.getElementById("launchdetails-" + launchId + "-contents");
  detailsElement.innerHTML = "";

  if (systemd_mode && (launchData.launch_procinfo != null || launchData.monitor_procinfo != null)) {
    var processHdr = document.createElement("h4");
    processHdr.innerText = "Running Processes";
    processHdr.classList.add("launchdetails-info");
    detailsElement.appendChild(processHdr);

    var processList = document.createElement("ul");
    processList.classList.add("launchdetails-info");
    if (launchData.launch_procinfo != null) {
      addProcinfo(launchData.launch_procinfo, processList);
    }
    if (launchData.monitor_procinfo != null) {
      addProcinfo(launchData.monitor_procinfo, processList);
    }
    detailsElement.appendChild(processList);
  }

  var topicsHdr = document.createElement("h4");
  topicsHdr.innerText = "Monitored Topics";
  topicsHdr.classList.add("launchdetails-info");
  detailsElement.appendChild(topicsHdr);

  var topicsList = document.createElement("ul");
  topicsList.classList.add("launchdetails-info");
  if (topicData.length === 0) {
    var topicItem = document.createElement("li");
    topicItem.innerText = "None";
    topicsList.appendChild(topicItem);
  }
  else {
    for (var i = 0; i < topicData.length; i++) {
      var topicEntry = document.createElement("li");
      var topicName = document.createElement("span");
      topicName.classList.add("launchdetails-topic-name");
      topicName.innerText = topicData[i].name;
      topicEntry.appendChild(topicName);

      if (launchData.state !== "stopped" && launchData.state !== "error") {
        var topicSeen = document.createElement("span");
        if (topicData[i].seen) {
          topicSeen.classList.add("launchdetails-topic-seen");
          topicSeen.innerText = " [Present]";
        }
        else {
          topicSeen.classList.add("launchdetails-topic-unseen");
          topicSeen.innerText = " [Not Present]";
        }
        topicEntry.appendChild(topicSeen);
      }

      topicsList.appendChild(topicEntry);
    }
  }
  detailsElement.appendChild(topicsList);
}

function toggleDetailsVisibility(launchId) {
  var detailsRoot = document.getElementById("launchdetails-" + launchId);
  var btnElement = document.getElementById("launchentry-" + launchId + "-details-btn");
  if (detailsRoot.style.display === "none") {
    detailsRoot.style.display = null;
    btnElement.innerText = "-";
  }
  else {
    detailsRoot.style.display = "none";
    btnElement.innerText = "+";
  }
}

function addLaunchEntry(launchId, friendlyName, logKey) {
  var progressTable = document.getElementById("bringup_table");

  var entryRoot = document.createElement("tr");
  entryRoot.id = "launchentry-" + launchId;

  var entryTitleCell = document.createElement("td");
  var expandInfoBtn = document.createElement("span");
  expandInfoBtn.id = "launchentry-" + launchId + "-details-btn";
  expandInfoBtn.classList.add("expand-info-btn");
  expandInfoBtn.onclick = function () {toggleDetailsVisibility(launchId)};
  expandInfoBtn.innerText = "+";

  entryTitleCell.appendChild(expandInfoBtn);
  var entryTitle = document.createElement("span");
  entryTitle.classList.add("title-box");
  entryTitle.innerText = friendlyName;
  entryTitleCell.appendChild(entryTitle);
  entryTitleCell.appendChild(document.createTextNode(" "));
  var errorCount = document.createElement("span");
  errorCount.classList.add("launch-error-count");
  errorCount.dataset.logKey = logKey;
  showLaunchErrorCount(errorCount);
  entryTitleCell.appendChild(errorCount);
  entryRoot.appendChild(entryTitleCell);

  var entryButtons = document.createElement("td");
  entryButtons.classList.add("button-box");

  var startButton = document.createElement("button");
  startButton.classList.add("start-button");
  startButton.id = "launchentry-" + launchId + "-start";
  startButton.innerText = "Start";
  startButton.onclick = function () {
    startClicked(launchId);
  };
  entryButtons.appendChild(startButton);

  entryButtons.appendChild(document.createTextNode("\u00A0"));

  var stopButton = document.createElement("button");
  stopButton.classList.add("stop-button");
  stopButton.id = "launchentry-" + launchId + "-end";
  stopButton.innerText = "Stop";
  stopButton.onclick = function () {
    stopClicked(launchId);
  };
  entryButtons.appendChild(stopButton);

  entryButtons.appendChild(document.createTextNode("\u00A0"));

  var logLabel = document.createElement("label");
  logLabel.classList.add("log-checkbox-label");
  var logCheckbox = document.createElement("input");
  logCheckbox.type = "checkbox";
  logCheckbox.checked = isLogSourceShown(logKey);
  logCheckbox.dataset.logKey = logKey;
  logCheckbox.classList.add("log-checkbox-input");
  logCheckbox.onchange = function () {
    setLogSourcesShown([logKey], logCheckbox.checked);
  };
  logLabel.appendChild(logCheckbox);
  logLabel.appendChild(document.createTextNode("Log"));
  entryButtons.appendChild(logLabel);

  entryRoot.appendChild(entryButtons);

  var progressColumn = document.createElement("td");
  progressColumn.classList.add("progress-box");

  var progressBar = document.createElement("div");
  progressBar.id = "launchentry-" + launchId + "-bar";
  progressBar.classList.add("progress-bar");
  var progressCenter = document.createElement("div");
  progressCenter.classList.add("progress");
  progressCenter.id = "launchentry-" + launchId + "-center";
  progressBar.appendChild(progressCenter);

  var progressInfo = document.createElement("div");
  progressInfo.classList.add("progress-info");
  progressInfo.id = "launchentry-" + launchId + "-info";
  progressBar.appendChild(progressInfo);
  progressColumn.appendChild(progressBar);

  entryRoot.appendChild(progressColumn);

  var detailsRoot = document.createElement("tr");
  detailsRoot.id = "launchdetails-" + launchId;
  detailsRoot.style.display = "none";
  var detailsContents = document.createElement("td");
  detailsContents.id = "launchdetails-" + launchId + "-contents";
  detailsContents.setAttribute("colspan", "3");
  detailsContents.classList.add("launchdetails-contents");
  detailsRoot.appendChild(detailsContents);

  progressTable.appendChild(entryRoot);
  progressTable.appendChild(detailsRoot);
}

function showCriticalError(msg) {
  if (errorShown) {
    // First error shown gets priority. Once we restore the connection, this is cleared and new errors can appear
    return;
  }

  document.getElementById("zombies").innerHTML = "";
  var bringupTable = document.getElementById("bringup_table");
  bringupTable.innerHTML = "";

  var msgElement = document.createElement("td");
  msgElement.setAttribute("colspan", "3");
  msgElement.classList.add("msg-header");
  msgElement.id = "errmsg";
  msgElement.innerText = msg;
  bringupTable.appendChild(msgElement);

  // Disable all global elements on fatal error
  document.getElementById("shutDownCleanBtn").disabled = true;
  document.getElementById("restartImmediateBtn").disabled = true;
  document.getElementById("launches").disabled = true;
  document.getElementById('log-default-checkbox').disabled = true;

  if (document.getElementById("discovery-server-status").style.color != "red") {
    // Only hide discovery server info if we don't have an error (color red)
    document.getElementById("discovery-server-info").style.display = "none";
  }

  errorShown = true;
}

function addProcinfo(procInfo, listElement) {
  // Create child process info
  var listItem = document.createElement("li");
  listItem.classList.add("procinfo");
  listItem.title = procInfo.full_name;

  // Create PID
  var pidSpan = document.createElement("span");
  pidSpan.innerText = "PID " + procInfo.pid.toString() + " [" + procInfo.state + "]: ";
  pidSpan.classList.add("procinfo-pid");
  listItem.appendChild(pidSpan);

  // Create cmdline
  var cmdlineSpan = document.createElement("span");
  cmdlineSpan.classList.add("procinfo-cmdline");
  cmdlineSpan.innerText = procInfo.full_name;
  listItem.appendChild(cmdlineSpan);
  listElement.appendChild(listItem);

  if (procInfo.children.length > 0) {
    var childListElement = document.createElement("ul");
    listElement.appendChild(childListElement);
    for (var i = 0; i < procInfo.children.length; i++) {
      addProcinfo(procInfo.children[i], childListElement);
    }
  }
}

function addZombies(statusResp) {
  var zombieContainer = document.getElementById("zombies");
  zombieContainer.innerHTML = '';

  var listElement = null;
  for (var i = 0; i < statusResp.launches.length; i++) {
    var launchEntry = statusResp.launches[i];
    if (!launchEntry.is_zombie) {
      continue;
    }

    if (listElement === null) {
      var header = document.createElement("h3");
      header.innerText = "Abandoned Launches:";
      header.classList.add("zombie-list");
      zombieContainer.appendChild(header);

      listElement = document.createElement("ul");
      listElement.classList.add("zombie-list");
      zombieContainer.appendChild(listElement);
    }

    var listItem = document.createElement("li");
    listItem.innerText = launchEntry.friendly_name + " (" + launchEntry.state + ")";
    listElement.appendChild(listItem);

    if (launchEntry.launch_procinfo != null || launchEntry.monitor_procinfo != null) {
      var childListElement = document.createElement("ul");
      listElement.appendChild(childListElement);

      if (launchEntry.launch_procinfo != null) {
        addProcinfo(launchEntry.launch_procinfo, childListElement);
      }
      if (launchEntry.monitor_procinfo != null) {
        addProcinfo(launchEntry.monitor_procinfo, childListElement);
      }
    }
  }

  if (statusResp.orphans != null && statusResp.orphans.length > 0) {
    var header = document.createElement("h3");
    header.innerText = "Orphans:";
    header.classList.add("zombie-list");
    zombieContainer.appendChild(header);

    var orphanListElement = document.createElement("ul");
    orphanListElement.classList.add("zombie-list");
    zombieContainer.appendChild(orphanListElement);
    for (var i = 0; i < statusResp.orphans.length; i++) {
      addProcinfo(statusResp.orphans[i], orphanListElement);
    }
  }
}

// ========================================
// Websocket Message Callbacks
// ========================================

function statusCallback(resp) {
  // Grab list of all IDs in the progress table
  var progressTable = document.getElementById("bringup_table");
  var progressChildren = progressTable.children;
  var id_list = [];
  for (var i = 0; i < progressChildren.length; i++) {
    if (progressChildren[i].id.length > 0) {
      id_list.push(progressChildren[i].id);
    }
  }

  syncLogSources(resp.launches);

  // Update all launches that we are provided in status
  for (var i = 0; i < resp.launches.length; i++) {
    var entry = resp.launches[i];
    if (entry.is_zombie) {
      continue;
    }

    var existingIdx = id_list.indexOf("launchentry-" + entry["id"]);

    if (existingIdx < 0) {
      addLaunchEntry(entry.id, entry.friendly_name, entry.log_key);
    } else {
      id_list.splice(existingIdx, 1);
      id_list.splice(id_list.indexOf("launchdetails-" + entry["id"]), 1);
    }

    refreshLaunchState(entry);
  }

  // Remove any launch items that weren't in the list
  for (var i = 0; i < id_list.length; i++) {
    document.getElementById(id_list[i]).remove();
  }

  // Set selected launch to the one in the status message
  var selector = document.getElementById("launches");
  selector.value = resp.current_launchfile;
  // Save the launch file to local storage so it can be easily pulled up next time we run the remote launcher
  localStorage.last_launchfile = resp.current_launchfile;
  selector.disabled = false;

  // Handle zombies
  addZombies(resp);

  // Handle discovery server status
  document.getElementById("discovery-server-info").style.display = null;  // Clear the display: none
  var discovery_status = document.getElementById("discovery-server-status");
  if (discovery_status.innerText != resp.discovery_status) {
    discovery_status.innerText = resp.discovery_status;
  }
  if (discovery_status.style.color != resp.discovery_status_color) {
    discovery_status.style.color = resp.discovery_status_color;
  }
}

function firstConnectCallback(resp) {
  // Update list of launches
  var selector = document.getElementById("launches");
  while (selector.firstChild) {
    selector.removeChild(selector.firstChild);
  }
  for (var i = 0; i < resp.launch_files.length; i++) {
    var entry = resp.launch_files[i];
    var cleanName = entry.substr(entry.lastIndexOf("/") + 1);
    var option = new Option(cleanName, entry);
    selector.appendChild(option);
  }

  // Handle systemd mode, updating items accordingly
  systemd_mode = resp.systemd_mode;
  var restartImmBtn = document.getElementById("restartImmediateBtn");
  var shutdownBtn = document.getElementById("shutDownCleanBtn");
  if (systemd_mode) {
    // In systemd mode, the shut down acts more like a restart since systemd will restart the launch service for us
    shutdownBtn.innerText = "Restart Launcher";
    restartImmBtn.style.display = null;
    restartImmBtn.disabled = false;
  }
  else {
    shutdownBtn.innerText = "Shut Down Launcher";
    restartImmBtn.style.display = "none";
  }
  document.getElementById("shutDownCleanBtn").disabled = false;

  document.getElementById('log-default-checkbox').disabled = false;

  // Handle the bundled status message
  statusCallback(resp.status);

  // Show everything that was logged before we connected
  resp.log_history.forEach(addLogLine);
  rebuildLog();
}

// ========================================
// Launch Log
// ========================================

const LOG_MAX_LINES = 30000;            // Oldest lines are dropped past this, must fit in the terminal scrollback
const LOG_LAUNCHER_KEY = "launcher";    // Filter key for lines from the launcher itself
const LEVEL_NORMAL = 0, LEVEL_WARN = 1, LEVEL_ERROR = 2;
const LEVEL_COLORS = {[LEVEL_WARN]: "\x1B[33m", [LEVEL_ERROR]: "\x1B[31m"};
const SEARCH_HIGHLIGHT = "\x1B[30;43m";
const ansiRe = /\x1B\[[0-9;:?]*[ -\/]*[@-~]/g;
const rosLevelRe = /\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]/;

var logLines = [];          // Every line we hold, oldest first
var logShown = [];          // The lines that pass the filters, which is what the terminal holds
var logMatches = [];        // Indices in logShown of the lines matching the search
var logMatchPos = -1;       // Which of logMatches is selected
var logSources = new Map(); // Filter key -> {label, current, errors}, current means it has a Log checkbox
var logFilter = {};         // Filter key -> shown, for the sources that were changed from the default
var logSearchRe = null;
var logCountsQueued = false;

function isLogSourceShown(key) {
  // Filtering is done with the launch's Log checkbox, so anything without one can't be left hidden
  var source = logSources.get(key);
  if (source == null || !source.current) {
    return true;
  }
  if (key in logFilter) {
    return logFilter[key];
  }
  return JSON.parse(localStorage.logging_enable_default);
}

function setLogSourcesShown(keys, shown) {
  keys.forEach(function (key) { logFilter[key] = shown; });
  localStorage.log_filter = JSON.stringify(logFilter);
  updateLogCheckboxes();
  rebuildLog();
}

function syncLogSources(launches) {
  var active = launches.filter(function (entry) { return !entry.is_zombie; });
  var nameCounts = {};
  active.forEach(function (entry) {
    nameCounts[entry.friendly_name] = (nameCounts[entry.friendly_name] || 0) + 1;
  });

  var currentKeys = active.map(function (entry) { return entry.log_key; });
  var changed = false;
  logSources.forEach(function (source, key) {
    if (source.current != currentKeys.includes(key)) {
      source.current = !source.current;
      changed = true;
    }
  });

  active.forEach(function (entry) {
    // Some launch files use the same launch file name from two packages
    var label = nameCounts[entry.friendly_name] > 1 ? entry.package + "/" + entry.friendly_name : entry.friendly_name;
    var source = logSources.get(entry.log_key);
    if (source == null) {
      logSources.set(entry.log_key, {label: label, current: true, errors: 0});
    } else {
      source.label = label;
    }
  });

  if (changed) {
    rebuildLog();  // Whether a line can be filtered depends on the launch still having a checkbox
  }
}

function setAllLogSourcesShown(shown) {
  var keys = Array.from(logSources.keys()).filter(function (key) { return logSources.get(key).current; });
  setLogSourcesShown(keys, shown);
}

function updateLogCheckboxes() {
  document.querySelectorAll(".log-checkbox-input").forEach(function (checkbox) {
    checkbox.checked = isLogSourceShown(checkbox.dataset.logKey);
  });
}

function showLaunchErrorCount(element) {
  var errors = logSources.get(element.dataset.logKey).errors;
  element.innerText = errors > 0 ? errors : "";
  element.title = errors + (errors == 1 ? " error in the log" : " errors in the log");
}

function addLogLine(line) {
  line.key = line.key != null ? line.key : LOG_LAUNCHER_KEY;
  line.plain = line.data.replace(ansiRe, "");

  var rosLevel = line.plain.match(rosLevelRe);
  if (line.state == "global_error" || (rosLevel != null && ["ERROR", "FATAL"].includes(rosLevel[1]))) {
    line.level = LEVEL_ERROR;
  } else if (rosLevel != null && rosLevel[1].startsWith("WARN")) {
    line.level = LEVEL_WARN;
  } else {
    line.level = LEVEL_NORMAL;
  }

  if (!logSources.has(line.key)) {
    logSources.set(line.key, {label: line.name, current: false, errors: 0});
  }
  // Counts are per run, and nodes dying while we stop them isn't a problem (ros2 launch logs that as an error)
  var source = logSources.get(line.key);
  var newRun = source.run !== line.run;
  source.run = line.run;
  if (newRun) {
    source.errors = 0;
  }
  if (line.level == LEVEL_ERROR && line.state != "stopping") {
    source.errors += 1;
  }
  if (newRun || line.level == LEVEL_ERROR) {
    document.querySelectorAll(".launch-error-count").forEach(function (element) {
      if (element.dataset.logKey == line.key) {
        showLaunchErrorCount(element);
      }
    });
  }
  logLines.push(line);
}

function appendLogLine(line) {
  addLogLine(line);

  if (logLines.length > LOG_MAX_LINES + 1000) {
    logLines.splice(0, logLines.length - LOG_MAX_LINES);
    rebuildLog();
  } else if (isLogLineShown(line)) {
    logShown.push(line);
    if (logSearchRe != null && isLogSearchHit(line)) {
      logMatches.push(logShown.length - 1);
    }
    term.writeln(formatLogLine(line));
  }
  queueLogCounts();
}

function appendLogNotice(msg) {
  appendLogLine({time: Date.now() / 1000, name: "log", key: null, state: "notice", data: msg});
}

function isLogSearchHit(line) {
  return line.plain.search(logSearchRe) >= 0;
}

function isLogLineShown(line) {
  return isLogSourceShown(line.key)
    && line.level >= document.getElementById("log-level").value
    && (logSearchRe == null || !document.getElementById("log-only-matches").checked || isLogSearchHit(line));
}

function logLineLabel(line) {
  return line.key == LOG_LAUNCHER_KEY ? line.name : logSources.get(line.key).label;
}

function formatLogLine(line) {
  var prefix = "";
  if (document.getElementById("log-time").checked) {
    prefix = "\x1B[2m" + new Date(line.time * 1000).toTimeString().substring(0, 8) + "\x1B[0m ";
  }
  if (logLevelColorMap[line.state] != null) {
    prefix += logLevelColorMap[line.state];
  }
  prefix += logLineLabel(line) + ": \x1B[0m";

  var color = LEVEL_COLORS[line.level] || "";
  if (logSearchRe != null && isLogSearchHit(line)) {
    // Colors from the line itself are dropped here so the highlight doesn't have to be threaded through them
    return prefix + color + line.plain.replace(logSearchRe, function (hit) {
      return SEARCH_HIGHLIGHT + hit + "\x1B[0m" + color;
    }) + "\x1B[0m";
  }
  if (color != "" && line.plain == line.data) {
    return prefix + color + line.data + "\x1B[0m";
  }
  return prefix + line.data;
}

function rebuildLog() {
  logShown = logLines.filter(isLogLineShown);
  logMatches = [];
  logMatchPos = -1;
  if (logSearchRe != null) {
    logShown.forEach(function (line, idx) {
      if (isLogSearchHit(line)) {
        logMatches.push(idx);
      }
    });
  }

  // The reset goes through write so that it lands after any rebuild the terminal is still working through
  var text = "\x1Bc" + logShown.map(function (line) { return formatLogLine(line) + "\n"; }).join("");
  term.write(text, function () {
    if (logMatches.length > 0) {
      goToLogMatch(logMatches.length - 1);  // The newest match is the one you're most likely after
    }
  });
  queueLogCounts();
}

// Terminal rows where each line of logShown starts. Lines wrap, so this has to be read from the terminal.
function getLogShownRows() {
  var buffer = term.buffer.active;
  var rows = [];
  var idx = logShown.length - 1;
  for (var row = buffer.baseY + buffer.cursorY - 1; row >= 0 && idx >= 0; row--) {
    if (!buffer.getLine(row).isWrapped) {
      rows[idx--] = row;
    }
  }
  return rows;
}

function goToLogMatch(pos) {
  if (logMatches.length == 0) {
    return;
  }
  logMatchPos = (pos + logMatches.length) % logMatches.length;

  var rows = getLogShownRows();
  var line = logMatches[logMatchPos];
  if (rows[line] != null) {
    var lastRow = rows[line + 1] != null ? rows[line + 1] - 1 : rows[line];
    term.scrollToLine(Math.max(rows[line] - Math.floor(term.rows / 2), 0));
    term.selectLines(rows[line], lastRow);
  }
  queueLogCounts();
}

function queueLogCounts() {
  if (logCountsQueued) {
    return;
  }
  logCountsQueued = true;
  requestAnimationFrame(function () {
    logCountsQueued = false;
    var matchCount = "";
    if (logSearchRe != null) {
      matchCount = logMatches.length > 0 ? (logMatchPos + 1) + "/" + logMatches.length : "No matches";
    }
    document.getElementById("log-match-count").innerText = matchCount;
    document.getElementById("log-line-count").innerText = logShown.length + " of " + logLines.length + " lines";
  });
}

function logSearchChanged() {
  var input = document.getElementById("log-search");
  var text = input.value;
  var regexForm = text.match(/^\/(.+)\/$/);

  // Plain text by default, since log lines are full of brackets that a regex would trip over
  logSearchRe = null;
  input.classList.remove("log-search-invalid");
  if (text != "") {
    try {
      logSearchRe = new RegExp(regexForm != null ? regexForm[1] : text.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"), "gi");
    } catch (err) {
      input.classList.add("log-search-invalid");
    }
  }
  rebuildLog();
}

function saveLog() {
  var text = logShown.map(function (line) {
    return new Date(line.time * 1000).toISOString() + " " + logLineLabel(line) + ": " + line.plain + "\n";
  }).join("");

  var link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([text], {type: "text/plain"}));
  link.download = "launch-log-" + new Date().toISOString().replace(/[:.]/g, "-") + ".txt";
  link.click();
  URL.revokeObjectURL(link.href);
}

function installLogToolbarHooks() {
  logFilter = JSON.parse(localStorage.log_filter || "{}");
  var timeCheckbox = document.getElementById("log-time");
  timeCheckbox.checked = localStorage.log_show_time == "true";
  timeCheckbox.onchange = function () {
    localStorage.log_show_time = timeCheckbox.checked;
    rebuildLog();
  };
  document.getElementById("log-only-matches").onchange = rebuildLog;
  document.getElementById("log-level").onchange = rebuildLog;
  document.getElementById("log-prev").onclick = function () { goToLogMatch(logMatchPos - 1); };
  document.getElementById("log-next").onclick = function () { goToLogMatch(logMatchPos + 1); };
  document.getElementById("log-save").onclick = saveLog;
  document.getElementById("log-all").onclick = function () { setAllLogSourcesShown(true); };
  document.getElementById("log-none").onclick = function () { setAllLogSourcesShown(false); };

  // Wait for a pause in typing since this rebuilds the whole log
  var searchInput = document.getElementById("log-search");
  var searchTimer = null;
  searchInput.oninput = function () {
    clearTimeout(searchTimer);
    searchTimer = setTimeout(logSearchChanged, 150);
  };
  searchInput.onkeydown = function (e) {
    if (e.key == "Enter") {
      goToLogMatch(logMatchPos + (e.shiftKey ? -1 : 1));
    } else if (e.key == "Escape") {
      searchInput.value = "";
      searchInput.blur();
      logSearchChanged();
    }
  };

  // The browser's find can't see into the terminal
  document.addEventListener("keydown", function (e) {
    var logOpen = document.querySelector(".terminal-container").style.display != "none";
    if (logOpen && (e.ctrlKey || e.metaKey) && e.key == "f") {
      e.preventDefault();
      searchInput.focus();
      searchInput.select();
    }
  });
}

// ========================================
// Window Control Callbacks
// ========================================

function cleanShutDownClicked() {
  ws.send(JSON.stringify({
    cmd: "stop_server"
  }));
  showCriticalError("Remote Launch Shut Down");
}

function restartImmediateClicked() {
  ws.send(JSON.stringify({
    cmd: "kill_server_now"
  }));
  showCriticalError("Restarting...");
}

function startClicked(launchId) {
  // Disable start button so it can't be clicked until next status refresh
  // The status callback will update the disabled flags accordingly when it gets the launch status
  var startBtn = document.getElementById("launchentry-" + launchId + "-start");
  startBtn.disabled = true;

  ws.send(JSON.stringify({
    cmd: "start_launch",
    id: launchId
  }));
}

function stopClicked(launchId) {
  // Disable stop button so it can't be clicked until next status refresh
  // The status callback will update the disabled flags accordingly when it gets the launch status
  var stopBtn = document.getElementById("launchentry-" + launchId + "-end");
  stopBtn.disabled = true;

  ws.send(JSON.stringify({
    cmd: "stop_launch",
    id: launchId
  }));
}

function launchSelected() {
  var selector = document.getElementById("launches");
  selector.disabled = true;

  var launch_sel = selector.options[selector.selectedIndex].value;
  ws.send(JSON.stringify({
    cmd: "load_launch",
    file: launch_sel
  }));
}

function logByDefaultChanged() {
  var checkbox = document.getElementById('log-default-checkbox');
  localStorage.logging_enable_default = JSON.stringify(checkbox.checked);

  // Everything goes back to following the default
  logFilter = {};
  localStorage.log_filter = JSON.stringify(logFilter);
  updateLogCheckboxes();
  rebuildLog();
}

// ========================================
// Websocket Control Logic
// ========================================

function websocketConnect() {
  ws = new WebSocket('/ws');
  ws.onopen = function(e) {
    // subscribe to some channels
    ws.send(JSON.stringify({
      cmd: "connect",
      // Pull the last used launch file from local storage to reduce button clicks when loading the page
      last_launchfile: localStorage.last_launchfile,
      // Logs are filtered here rather than on the server, so filters also apply to lines that are already received
      logging_enable_default: true,
      log_history: true
    }));
    errorShown = false;
  };

  ws.onmessage = function(e) {
    try {
      var msg = JSON.parse(e.data);
      if (msg.type === "status") {
        statusCallback(msg);
      }
      else if (msg.type === "first_connect") {
        firstConnectCallback(msg);
      }
      else if (msg.type === "error") {
        alert("Error: " + msg.msg);
      }
      else if (msg.type === "log") {
        appendLogLine(msg);
      }
      else {
        throw new Error("Unexpected Message Type: " + msg.type);
      }
    } catch (error) {
      console.error("Failed to Decode Message:", error);
      showCriticalError("Client Error (See Dev Console)");
      ws.close();
    }
  };

  ws.onclose = function(e) {
    showCriticalError("Connection Lost");
    appendLogNotice("<Launch Log Ended: Connection Closed>");
  };
}

function installTerminalResizeHooks(showByDefault) {
  const resizeBar = document.querySelector('.term-resize-bar');
  const terminalContainer = document.querySelector('.terminal-container');
  const closeButton = document.querySelector('.termclose-btn');
  const openButton = document.querySelector('.termopen-btn');
  const minPaneSizePx = 150;

  let isResizing = false;

  resizeBar.addEventListener('mousedown', (e) => {
    // Only listen to left click events
    if (e.button != 0) return;

    isResizing = true;
    document.body.style.userSelect = 'none';
    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', dragDoneListener);
  });

  function handleMouseMove(e) {
    if (isResizing) {
      var containerBottom = terminalContainer.getBoundingClientRect().bottom;
      var targetY = e.clientY;
      // Make sure the top portion doesn't get too small
      if (targetY < minPaneSizePx) {
        targetY = minPaneSizePx;
      }
      // Make sure bottom doesn't get too small, but give priority to top portion
      var newHeight = containerBottom - targetY;
      var minHeight = (containerBottom < minPaneSizePx * 2 ? containerBottom - minPaneSizePx : minPaneSizePx);
      if (newHeight < minHeight) {
        newHeight = minHeight;
      }

      terminalContainer.style.height = `${newHeight}px`;
      fitAddon.fit();
    }
  }

  function dragDoneListener(e) {
    isResizing = false;
    document.body.style.userSelect = null;
    document.removeEventListener('mousemove', handleMouseMove);
    document.removeEventListener('mouseup', dragDoneListener);
  }

  // Close button functionality
  closeButton.addEventListener('click', () => {
    terminalContainer.style.display = 'none'; // Hide the entire resizable container
    openButton.style.display = null; // Hide the open button
  });

  // Open button functionality
  openButton.addEventListener('click', () => {
    terminalContainer.style.height = null;  // Clear the height attribute so it falls back to default on open
    terminalContainer.style.display = null;
    openButton.style.display = 'none'; // Hide the open button
    fitAddon.fit();
  });

  // Configure default terminal visibility depending on if show by default is set
  if (showByDefault) {
    terminalContainer.style.display = null;
    openButton.style.display = 'none';
  }
  else {
    terminalContainer.style.display = 'none';
    openButton.style.display = null;
  }
}

window.onload = function () {
  // Disable all elements before we connect to the websocket
  document.getElementById("shutDownCleanBtn").disabled = true;
  document.getElementById("restartImmediateBtn").disabled = true;
  document.getElementById("launches").disabled = true;
  document.getElementById("discovery-server-info").style.display = "none";

  var logDefaultCheckbox = document.getElementById('log-default-checkbox');
  logDefaultCheckbox.disabled = true;

  // Either set the log default checkbox to the value set by localstorage, or if
  // localstorage isn't set, set localstorage to the default checkbox value
  if (localStorage.logging_enable_default != null) {
    logDefaultCheckbox.checked = JSON.parse(localStorage.logging_enable_default);
  }
  else {
    localStorage.logging_enable_default = JSON.stringify(logDefaultCheckbox.checked);
  }

  // Initialize Terminal
  installTerminalResizeHooks(logDefaultCheckbox.checked);
  term.open(document.getElementById('terminal'));
  installLogToolbarHooks();
  appendLogNotice("<Launch Log Started>");

  // Connect to the websocket
  websocketConnect();
  fitAddon.fit();

  const xterm_resize_ob = new ResizeObserver(function (entries) {
    // since we are observing only a single element, so we access the first element in entries array
    try {
      fitAddon && fitAddon.fit();
    } catch (err) {
      console.log(err);
    }
  });

  // start observing for resize
  xterm_resize_ob.observe(document.getElementById('terminal'));
};
