// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0


// ---------------------------------------------------------------------------
// DOM references
// ---------------------------------------------------------------------------

const recentDetectionsElement =
  document.getElementById('recentDetections');

const feedbackContentElement =
  document.getElementById('feedback-content');

const errorContainer =
  document.getElementById('error-container');


// ---------------------------------------------------------------------------
// Robot controls
// ---------------------------------------------------------------------------

const trackingToggle =
  document.getElementById('tracking-toggle');

const trackingToggleLabel =
  document.getElementById('tracking-toggle-label');

const trackingStatus =
  document.getElementById('tracking-status');

const robotStatusBadge =
  document.getElementById('robot-status-badge');

const centerRobotButton =
  document.getElementById('center-robot-button');

const emergencyStopButton =
  document.getElementById('emergency-stop-button');

const targetLabel =
  document.getElementById('target-label');

const targetConfidence =
  document.getElementById('target-confidence');

const targetPosition =
  document.getElementById('target-position');

const robotNotice =
  document.getElementById('robot-notice');


// ---------------------------------------------------------------------------
// Voice controls
// ---------------------------------------------------------------------------

const voiceToggle =
  document.getElementById('voice-toggle');

const voiceToggleLabel =
  document.getElementById('voice-toggle-label');

const voiceStatus =
  document.getElementById('voice-status');

const voiceStatusBadge =
  document.getElementById('voice-status-badge');

const voiceTranscript =
  document.getElementById('voice-transcript');

const voiceCommand =
  document.getElementById('voice-command');


// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------

const MAX_RECENT_SCANS = 5;

let scans = [];

let detectionTimeout;

let handVisible = false;

let trackingEnabled = false;

let voiceListening = false;


// ---------------------------------------------------------------------------
// Arduino WebUI
// ---------------------------------------------------------------------------

const ui = new WebUI();

ui.on_connect(
  onUIConnected,
);

ui.on_disconnect(
  onUIDisconnected,
);


// ---------------------------------------------------------------------------
// Existing backend messages
// ---------------------------------------------------------------------------

ui.on_message(
  'detection',
  handleDetection,
);

ui.on_message(
  'tracking_state',
  handleTrackingState,
);

ui.on_message(
  'target_state',
  handleTargetState,
);

ui.on_message(
  'robot_notice',
  handleRobotNotice,
);


// ---------------------------------------------------------------------------
// NEW: Voice backend messages
// ---------------------------------------------------------------------------

ui.on_message(
  'voice_state',
  handleVoiceState,
);

ui.on_message(
  'transcription',
  handleTranscription,
);

ui.on_message(
  'voice_transcription',
  handleVoiceTranscription,
);


// ---------------------------------------------------------------------------
// Application startup
// ---------------------------------------------------------------------------

initializeConfidenceSlider();

initializeRobotControls();

initializeVoiceControls();


if (feedbackContentElement) {
  feedbackContentElement.innerHTML = `
    <img src="img/stars.svg" alt="Stars">
    <p class="feedback-text">
      System response will appear here
    </p>
  `;
}


handVisible = false;

renderDetections();

updateTrackingDisplay(
  false,
);

updateVoiceDisplay(
  false,
);

clearTargetDisplay();


// ---------------------------------------------------------------------------
// Popover logic
// ---------------------------------------------------------------------------

const confidencePopoverText =
  'Minimum confidence score for detected faces. '
  + 'Lower values show more results but may include false positives.';


const feedbackPopoverText =
  'When the camera detects a target, an animation will appear here.';


document
  .querySelectorAll(
    '.info-btn.confidence',
  )
  .forEach(img => {

    const popover =
      img.nextElementSibling;


    img.addEventListener(
      'mouseenter',
      () => {

        popover.textContent =
          confidencePopoverText;

        popover.style.display =
          'block';
      },
    );


    img.addEventListener(
      'mouseleave',
      () => {

        popover.style.display =
          'none';
      },
    );
  });


document
  .querySelectorAll(
    '.info-btn.feedback',
  )
  .forEach(img => {

    const popover =
      img.nextElementSibling;


    img.addEventListener(
      'mouseenter',
      () => {

        popover.textContent =
          feedbackPopoverText;

        popover.style.display =
          'block';
      },
    );


    img.addEventListener(
      'mouseleave',
      () => {

        popover.style.display =
          'none';
      },
    );
  });


// ---------------------------------------------------------------------------
// WebUI connection callbacks
// ---------------------------------------------------------------------------

function onUIConnected() {

  console.log(
    '[WEBUI] Connected',
  );


  if (errorContainer) {

    errorContainer.style.display =
      'none';

    errorContainer.textContent =
      '';
  }


  setRobotNotice(
    'Connected to Arduino App Lab',
    'success',
  );


  // Ask ROS bridge for current status.
  ui.send_message(
    'request_robot_status',
    {},
  );
}


function onUIDisconnected() {

  console.log(
    '[WEBUI] Disconnected',
  );


  if (errorContainer) {

    errorContainer.textContent =
      'Connection to the board lost. '
      + 'Please check the connection.';

    errorContainer.style.display =
      'block';
  }


  updateTrackingDisplay(
    false,
  );


  updateVoiceDisplay(
    false,
  );


  setRobotNotice(
    'Connection to the board lost',
    'error',
  );
}


// ---------------------------------------------------------------------------
// Robot controls
// ---------------------------------------------------------------------------

function initializeRobotControls() {

  if (trackingToggle) {

    trackingToggle.addEventListener(
      'change',
      handleTrackingToggle,
    );
  }


  if (centerRobotButton) {

    centerRobotButton.addEventListener(
      'click',
      handleCenterRobot,
    );
  }


  if (emergencyStopButton) {

    emergencyStopButton.addEventListener(
      'click',
      handleEmergencyStop,
    );
  }
}


function handleTrackingToggle() {

  if (!trackingToggle) {
    return;
  }


  const enabled =
    Boolean(
      trackingToggle.checked,
    );


  trackingEnabled =
    enabled;


  updateTrackingDisplay(
    enabled,
  );


  ui.send_message(
    'set_tracking',
    {
      enabled:
        trackingToggle.checked,
    },
  );


  setRobotNotice(
    enabled
      ? 'Tracking enable command sent'
      : 'Tracking disable command sent',

    enabled
      ? 'success'
      : 'info',
  );
}


function handleCenterRobot() {

  trackingEnabled =
    false;


  updateTrackingDisplay(
    false,
  );


  ui.send_message(
    'center_robot',
    {},
  );


  setRobotNotice(
    'Centering robot...',
    'info',
  );
}


function handleEmergencyStop() {

  trackingEnabled =
    false;


  updateTrackingDisplay(
    false,
  );


  ui.send_message(
    'emergency_stop',
    {},
  );


  setRobotNotice(
    'Emergency stop command sent',
    'error',
  );
}


function handleTrackingState(
  message,
) {

  const enabled =
    Boolean(
      message
      && message.enabled,
    );


  trackingEnabled =
    enabled;


  updateTrackingDisplay(
    enabled,
  );
}


function updateTrackingDisplay(
  enabled,
) {

  if (trackingToggle) {

    trackingToggle.checked =
      enabled;
  }


  if (trackingToggleLabel) {

    trackingToggleLabel.textContent =
      enabled
        ? 'Tracking enabled'
        : 'Enable tracking';
  }


  if (trackingStatus) {

    trackingStatus.textContent =
      enabled
        ? 'Tracking enabled'
        : 'Tracking disabled';
  }


  if (robotStatusBadge) {

    robotStatusBadge.textContent =
      enabled
        ? 'TRACKING'
        : 'DISABLED';


    robotStatusBadge.classList.toggle(
      'status-enabled',
      enabled,
    );


    robotStatusBadge.classList.toggle(
      'status-disabled',
      !enabled,
    );
  }
}


// ---------------------------------------------------------------------------
// NEW: Voice control
// ---------------------------------------------------------------------------

function initializeVoiceControls() {

  if (!voiceToggle) {

    console.warn(
      '[VOICE UI] voice-toggle element not found',
    );

    return;
  }


  voiceToggle.addEventListener(
    'change',
    handleVoiceToggle,
  );
}


function handleVoiceToggle() {

  if (!voiceToggle) {
    return;
  }


  const enabled =
    Boolean(
      voiceToggle.checked,
    );


  console.log(
    `[VOICE UI] toggle=${enabled}`,
  );


  // Update immediately so the UI feels responsive.
  updateVoiceDisplay(
    enabled,
  );


  if (enabled) {

    if (voiceTranscript) {

      voiceTranscript.textContent =
        'Listening...';
    }


    if (voiceCommand) {

      voiceCommand.textContent =
        'Waiting for command';
    }


    console.log(
      '[VOICE UI] Sending start_dictation',
    );


    ui.send_message(
      'start_dictation',
      {},
    );

  } else {

    console.log(
      '[VOICE UI] Sending stop_dictation',
    );


    ui.send_message(
      'stop_dictation',
      {},
    );
  }
}


function handleVoiceState(
  message,
) {

  const enabled =
    Boolean(
      message
      && message.listening,
    );


  voiceListening =
    enabled;


  console.log(
    `[VOICE STATE] listening=${enabled}`,
  );


  updateVoiceDisplay(
    enabled,
  );
}


function updateVoiceDisplay(
  enabled,
) {

  voiceListening =
    Boolean(enabled);


  if (voiceToggle) {

    voiceToggle.checked =
      voiceListening;
  }


  if (voiceToggleLabel) {

    voiceToggleLabel.textContent =
      voiceListening
        ? 'Voice commands enabled'
        : 'Enable voice commands';
  }


  if (voiceStatus) {

    voiceStatus.textContent =
      voiceListening
        ? 'Listening'
        : 'Voice control off';
  }


  if (voiceStatusBadge) {

    voiceStatusBadge.textContent =
      voiceListening
        ? 'LISTENING'
        : 'OFF';


    voiceStatusBadge.classList.toggle(
      'status-enabled',
      voiceListening,
    );


    voiceStatusBadge.classList.toggle(
      'status-disabled',
      !voiceListening,
    );
  }
}


// ---------------------------------------------------------------------------
// ASR transcription
// ---------------------------------------------------------------------------

function handleTranscription(
  message,
) {

  if (!message) {
    return;
  }


  const text =
    String(
      message.text || '',
    ).trim();


  const type =
    String(
      message.type || '',
    );


  console.log(
    '[ASR TRANSCRIPTION]',
    {
      type,
      text,
    },
  );


  if (!text) {
    return;
  }


  if (voiceTranscript) {

    voiceTranscript.textContent =
      text;
  }


  // Try to present a useful command summary in the UI.
  updateVoiceCommandDisplay(
    text,
  );
}


function handleVoiceTranscription(
  message,
) {

  if (!message) {
    return;
  }


  const text =
    String(
      message.text || '',
    ).trim();


  if (!text) {
    return;
  }


  console.log(
    `[VOICE TEXT] ${text}`,
  );


  if (voiceTranscript) {

    voiceTranscript.textContent =
      text;
  }


  updateVoiceCommandDisplay(
    text,
  );
}


function updateVoiceCommandDisplay(
  rawText,
) {

  if (!voiceCommand) {
    return;
  }


  const text =
    String(
      rawText || '',
    )
      .trim()
      .toLowerCase();


  if (!text) {

    voiceCommand.textContent =
      'None';

    return;
  }


  // Emergency Stop
  if (
    text.includes(
      'emergency stop',
    )
    ||
    text.includes(
      'stop robot',
    )
    ||
    text.includes(
      'robot stop',
    )
  ) {

    voiceCommand.textContent =
      'Emergency Stop';

    return;
  }


  // Center
  if (
    text.includes(
      'center robot',
    )
    ||
    text.includes(
      'centre robot',
    )
    ||
    text.includes(
      'center the robot',
    )
    ||
    text.includes(
      'return to center',
    )
    ||
    text === 'center'
  ) {

    voiceCommand.textContent =
      'Center Robot';

    return;
  }


  // Disable tracking
  if (
    text.includes(
      'disable tracking',
    )
    ||
    text.includes(
      'stop tracking',
    )
    ||
    text.includes(
      'tracking off',
    )
    ||
    text.includes(
      'turn tracking off',
    )
  ) {

    voiceCommand.textContent =
      'Tracking OFF';

    return;
  }


  // Enable tracking
  if (
    text.includes(
      'enable tracking',
    )
    ||
    text.includes(
      'start tracking',
    )
    ||
    text.includes(
      'tracking on',
    )
    ||
    text.includes(
      'turn tracking on',
    )
    ||
    text.includes(
      'follow me',
    )
  ) {

    voiceCommand.textContent =
      'Tracking ON';

    return;
  }


  // Unknown text
  voiceCommand.textContent =
    text;
}


// ---------------------------------------------------------------------------
// Target telemetry
// ---------------------------------------------------------------------------

function handleTargetState(
  message,
) {

  if (
    !message
    || !message.detected
  ) {

    clearTargetDisplay();

    return;
  }


  if (targetLabel) {

    targetLabel.textContent =
      message.label
      || 'Target';
  }


  if (targetConfidence) {

    const confidence =
      Number(
        message.confidence
        || 0,
      );


    targetConfidence.textContent =
      `${Math.round(
        confidence * 100,
      )}%`;
  }


  if (targetPosition) {

    const centerX =
      Number(
        message.center_x
        || 0,
      );


    const centerY =
      Number(
        message.center_y
        || 0,
      );


    targetPosition.textContent =
      `${Math.round(centerX)}, `
      + `${Math.round(centerY)}`;
  }
}


function clearTargetDisplay() {

  if (targetLabel) {

    targetLabel.textContent =
      'None';
  }


  if (targetConfidence) {

    targetConfidence.textContent =
      '0%';
  }


  if (targetPosition) {

    targetPosition.textContent =
      '—';
  }
}


// ---------------------------------------------------------------------------
// Robot notices
// ---------------------------------------------------------------------------

function handleRobotNotice(
  message,
) {

  if (!message) {
    return;
  }


  setRobotNotice(
    message.text || '',
    message.level || 'info',
  );
}


function setRobotNotice(
  text,
  level = 'info',
) {

  if (!robotNotice) {
    return;
  }


  robotNotice.textContent =
    text;


  robotNotice.dataset.level =
    level;
}


// ---------------------------------------------------------------------------
// Detection feedback
// ---------------------------------------------------------------------------

function handleDetection(
  message,
) {

  if (detectionTimeout) {

    clearTimeout(
      detectionTimeout,
    );
  }


  printDetection(
    message,
  );


  renderDetections();


  if (
    !handVisible
    && feedbackContentElement
  ) {

    const greetings = [
      'Hello!',
      'Hi there!',
      'Hey!',
      'Nice to see you!',
      'Great to have you here!',
      'I see you',
      'Looking good!',
      'There you are!',
      'Howdy!',
      'Happy to see a face!',
      'Hi, friend!',
      'Face detected!',
      'Hello, human!',
    ];


    const randomGreeting =
      greetings[
        Math.floor(
          Math.random()
          * greetings.length,
        )
      ];


    feedbackContentElement.innerHTML = `
      <img
        src="img/hand.gif"
        alt="Hand"
      >
      <p>
        ${randomGreeting}
      </p>
    `;


    handVisible =
      true;
  }


  detectionTimeout =
    setTimeout(
      () => {

        if (feedbackContentElement) {

          feedbackContentElement.innerHTML = `
            <img
              src="img/stars.svg"
              alt="Stars"
            >

            <p class="feedback-text">
              System response will appear here
            </p>
          `;
        }


        handVisible =
          false;
      },

      3000,
    );
}


function printDetection(
  newDetection,
) {

  scans.unshift(
    newDetection,
  );


  if (
    scans.length
    > MAX_RECENT_SCANS
  ) {

    scans.pop();
  }
}


// ---------------------------------------------------------------------------
// Recent detections list
// ---------------------------------------------------------------------------

function renderDetections() {

  if (!recentDetectionsElement) {
    return;
  }


  recentDetectionsElement.innerHTML =
    '';


  if (
    scans.length === 0
  ) {

    recentDetectionsElement.innerHTML = `
      <div class="no-recent-scans">

        <img
          src="./img/no-face.svg"
          alt="No face"
        >

        No face detected yet

      </div>
    `;


    return;
  }


  scans.forEach(
    scan => {

      const row =
        document.createElement(
          'div',
        );


      row.className =
        'scan-container';


      const cellContainer =
        document.createElement(
          'span',
        );


      cellContainer.className =
        'scan-cell-container cell-border';


      const contentText =
        document.createElement(
          'span',
        );


      contentText.className =
        'scan-content';


      const value =
        Number(
          scan.confidence
          || 0,
        );


      const result =
        Math.floor(
          value * 1000,
        ) / 10;


      const label =
        scan.content
        || 'Face';


      contentText.innerHTML =
        `${result}% - ${label}`;


      const timeText =
        document.createElement(
          'span',
        );


      timeText.className =
        'scan-content-time';


      timeText.textContent =
        new Date(
          scan.timestamp,
        )
          .toLocaleString(
            'it-IT',
          )
          .replace(
            ',',
            ' -',
          );


      cellContainer.appendChild(
        contentText,
      );


      cellContainer.appendChild(
        timeText,
      );


      row.appendChild(
        cellContainer,
      );


      recentDetectionsElement.appendChild(
        row,
      );
    },
  );
}


// ---------------------------------------------------------------------------
// Confidence slider
// ---------------------------------------------------------------------------

function initializeConfidenceSlider() {

  const confidenceSlider =
    document.getElementById(
      'confidenceSlider',
    );


  const confidenceInput =
    document.getElementById(
      'confidenceInput',
    );


  const confidenceResetButton =
    document.getElementById(
      'confidenceResetButton',
    );


  if (
    !confidenceSlider
    || !confidenceInput
  ) {

    return;
  }


  confidenceSlider.addEventListener(
    'input',
    updateConfidenceDisplay,
  );


  confidenceInput.addEventListener(
    'input',
    handleConfidenceInputChange,
  );


  confidenceInput.addEventListener(
    'blur',
    validateConfidenceInput,
  );


  updateConfidenceDisplay();


  if (
    confidenceResetButton
  ) {

    confidenceResetButton.addEventListener(
      'click',
      event => {

        if (
          event.target.classList.contains(
            'reset-icon',
          )
          ||
          event.target.closest(
            '.reset-icon',
          )
        ) {

          resetConfidence();
        }
      },
    );
  }
}


function handleConfidenceInputChange() {

  const confidenceInput =
    document.getElementById(
      'confidenceInput',
    );


  const confidenceSlider =
    document.getElementById(
      'confidenceSlider',
    );


  if (
    !confidenceInput
    || !confidenceSlider
  ) {

    return;
  }


  let value =
    parseFloat(
      confidenceInput.value,
    );


  if (
    Number.isNaN(
      value,
    )
  ) {

    value =
      0.5;
  }


  if (
    value < 0
  ) {

    value =
      0;
  }


  if (
    value > 1
  ) {

    value =
      1;
  }


  confidenceSlider.value =
    value;


  updateConfidenceDisplay();
}


function validateConfidenceInput() {

  const confidenceInput =
    document.getElementById(
      'confidenceInput',
    );


  if (!confidenceInput) {

    return;
  }


  let value =
    parseFloat(
      confidenceInput.value,
    );


  if (
    Number.isNaN(
      value,
    )
  ) {

    value =
      0.5;
  }


  if (
    value < 0
  ) {

    value =
      0;
  }


  if (
    value > 1
  ) {

    value =
      1;
  }


  confidenceInput.value =
    value.toFixed(
      2,
    );


  handleConfidenceInputChange();
}


function updateConfidenceDisplay() {

  const confidenceSlider =
    document.getElementById(
      'confidenceSlider',
    );


  const confidenceInput =
    document.getElementById(
      'confidenceInput',
    );


  const confidenceValueDisplay =
    document.getElementById(
      'confidenceValueDisplay',
    );


  const sliderProgress =
    document.getElementById(
      'sliderProgress',
    );


  if (
    !confidenceSlider
    || !confidenceInput
    || !confidenceValueDisplay
    || !sliderProgress
  ) {

    return;
  }


  const value =
    parseFloat(
      confidenceSlider.value,
    );


  ui.send_message(
    'override_th',
    value,
  );


  const percentage =
    (
      (
        value
        -
        confidenceSlider.min
      )
      /
      (
        confidenceSlider.max
        -
        confidenceSlider.min
      )
    )
    *
    100;


  const displayValue =
    value.toFixed(
      2,
    );


  confidenceValueDisplay.textContent =
    displayValue;


  if (
    document.activeElement
    !== confidenceInput
  ) {

    confidenceInput.value =
      displayValue;
  }


  sliderProgress.style.width =
    `${percentage}%`;


  confidenceValueDisplay.style.left =
    `${percentage}%`;
}


function resetConfidence() {

  const confidenceSlider =
    document.getElementById(
      'confidenceSlider',
    );


  const confidenceInput =
    document.getElementById(
      'confidenceInput',
    );


  if (
    !confidenceSlider
    || !confidenceInput
  ) {

    return;
  }


  confidenceSlider.value =
    '0.5';


  confidenceInput.value =
    '0.50';


  updateConfidenceDisplay();
}