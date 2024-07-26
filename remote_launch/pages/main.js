var ws = null;              // The websocket object
var systemd_mode = null;    // Holds the global systemd mode which adjusts the UI rendering
var errorShown = false;     // Used so that when we close the websocket, the close message won't overtake other errors

var baseTheme = {
  foreground: '#F8F8F8',
  background: '#222',
  selection: '#5DA5D533',
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
  stopping: "\x1B[1;91m",
  global: "\x1B[1;35m",
}
const term = new Terminal({
  fontFamily: '"Cascadia Code", Menlo, monospace',
  theme: baseTheme,
  disableStdin: true,
  convertEol: true,
  scrollback: 10000
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

function addLaunchEntry(launchId, friendlyName) {
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
  logCheckbox.checked = JSON.parse(localStorage.logging_enable_default);
  logCheckbox.id = "launchentry-" + launchId + "-log-en";
  logCheckbox.onchange = function () {
    logCheckboxChanged(launchId);
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

  document.getElementById("shutDownCleanBtn").disabled = true;
  document.getElementById("restartImmediateBtn").disabled = true;
  document.getElementById("launches").disabled = true;
  document.getElementById('log-default-checkbox').disabled = true;

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
  for (var i in statusResp.launches) {
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
    for (var i in statusResp.orphans) {
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

  // Update all launches that we are provided in status
  for (var i in resp.launches) {
    var entry = resp.launches[i];
    if (entry.is_zombie) {
      continue;
    }

    var existingIdx = id_list.indexOf("launchentry-" + entry["id"]);

    if (existingIdx < 0) {
      addLaunchEntry(entry.id, entry.friendly_name);
    } else {
      id_list.splice(existingIdx, 1);
      id_list.splice(id_list.indexOf("launchdetails-" + entry["id"]), 1);
    }

    refreshLaunchState(entry);
  }

  // Remove any launch items that weren't in the list
  for (var i in id_list) {
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
}

function firstConnectCallback(resp) {
  // Update list of launches
  var selector = document.getElementById("launches");
  while (selector.firstChild) {
    selector.removeChild(selector.firstChild);
  }
  for (var i in resp.launch_files) {
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

  // Handle the bundled status message
  statusCallback(resp.status);

  // Now that we've refreshed all the launches, update the log enrollment status
  logEnrollmentCallback(resp.log_enrollment);
}

function logEnrollmentCallback(resp) {
  // Update the checkboxes with the reported log enrollment from the server
  // Set default value
  var defaultCheckbox = document.getElementById('log-default-checkbox');
  defaultCheckbox.checked = resp.default;
  defaultCheckbox.disabled = false;

  // Save in local storage as well
  localStorage.logging_enable_default = JSON.stringify(resp.default);

  for (var launchId in resp.launches) {
    var checkbox = document.getElementById(`launchentry-${launchId}-log-en`);
    if (checkbox != null) {
      checkbox.checked = resp.launches[launchId];
      checkbox.disabled = false;
    }
  }
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
  checkbox.disabled = true;
  // TODO: Disable all other checkboxes as well
  ws.send(JSON.stringify({
    cmd: "set_logdefault",
    enable: checkbox.checked
  }));
}

function logCheckboxChanged(launchId) {
  var checkbox = document.getElementById(`launchentry-${launchId}-log-en`);
  checkbox.disabled = true;
  ws.send(JSON.stringify({
    cmd: "set_logenable",
    id: launchId,
    enable: checkbox.checked
  }));
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
      // If the websocket should be enrolled in logging by default
      logging_enable_default: JSON.parse(localStorage.logging_enable_default)  // Gaurenteed to be set in onload
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
        var prefix = "";
        if (logLevelColorMap[msg.state] != null) {
          prefix = logLevelColorMap[msg.state];
        }
        prefix += msg.name + ": \x1B[0m"
        term.writeln(prefix + msg.data);
      }
      else if (msg.type == "log_enrollment") {
        logEnrollmentCallback(msg);
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
    term.writeln("\x1b[1;33m<Launch Log Ended: Connection Closed>\x1b[0m");
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
  term.writeln("\x1b[1;33m<Launch Log Started>\x1b[0m");

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
