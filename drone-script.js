"use strict";

// Simulation settings and helper functions
const $ = (id) => document.getElementById(id);
const simulationTimeStep = 0.002;
const WINDOW = 12;
const SAMPLE_EVERY = 10;
const ANALYSIS_END = 30;
const numericIds = ["w", "z", "target", "fixed", "kp", "ki", "kd"];
const CONTROLLERS = ["P", "PD", "PI", "PID"];

const fmt = (value, places = 2) =>
  Number.isFinite(value) ? value.toFixed(places) : "—";

const initialState = () => ({
  t: 0,
  altitude: 0,
  verticalVelocity: 0,
  accumulatedError: 0,
  controlInput: 0,
});

const initialSample = () => ({ t: 0, y: 0, v: 0, u: 0 });

function on(id, event, handler, options) {
  $(id)?.addEventListener(event, handler, options);
}

function text(id, value) {
  if ($(id)) $(id).textContent = value;
}

function hide(id, value = true) {
  if ($(id)) $(id).hidden = value;
}

function typeset(elements) {
  const nodes = elements.filter(Boolean);
  if (nodes.length && window.MathJax?.typesetPromise) {
    window.MathJax.typesetPromise(nodes).catch(() => {});
  }
}

function equation(tex) {
  return `<div class="solution-equation">\\[${tex}\\]</div>`;
}

function showStatus(message) {
  text("status", message);
}

// Simulation state and controller variables
let state = initialState();
let samples = [initialSample()];
let analysisSamples = [initialSample()];
let currentConfig = null;

let running = false;
let frame = null;
let previousFrame = null;
let accumulator = 0;
let steps = 0;
let hasLaunched = false;
let flightWasDisturbed = false;
let analysisStart = 0;
let lastLivePlot = 0;

let droneApp = null;
let droneSprite = null;

let designChallenge = null;
let designHasLaunched = false;
let designAnswerCorrect = false;
let solutionRevealed = false;

const comparisonResults = {};
const GUST_DIRECTION = -1;
const GUST_DURATION = 0.2;
const CONSTANT_WIND_TARGET_PD_OFFSET = 1;

let constantWind = false;
let wind = freshWind();

// Wind disturbance state
function freshWind() {
  return {
    index: 0,
    value: 0,
    until: 0,
    baseline: 0,
    maxDeviation: 0,
    lastDeviation: null,
  };
}

// Check which controller terms are active
function hasIntegral(settings) {
  return ["PI", "PID"].includes(settings.type) && settings.ki > 0;
}

function hasDerivative(settings) {
  return ["PD", "PID"].includes(settings.type);
}

// Read and validate the controller settings
function config() {
  const settings = {
    mode: $("mode").value,
    type: $("type").value,
    plant: $("experiment").value,
  };

  for (const id of numericIds) {
    const raw = $(id).value;
    if (raw.trim() === "") {
      throw Error("Enter a value for every required setting.");
    }
    settings[id] = Number(raw);
  }

  settings.naturalFrequency = settings.w;
  settings.dampingRatio = settings.z;

  if (
    numericIds.some((id) => !Number.isFinite(settings[id])) ||
    settings.w <= 0 ||
    settings.z < 0 ||
    settings.z > 1 ||
    settings.target <= 0 ||
    ["kp", "ki", "kd"].some((id) => settings[id] < 0)
  ) {
    throw Error(
      "Use a positive ω₀ and target, ζ₀ between 0 and 1, and non-negative gains.",
    );
  }

  return settings;
}

// Check the closed-loop stability conditions
function isStable(settings) {
  if (!settings || settings.mode !== "closed") return false;

  const integral = hasIntegral(settings);
  const kd = hasDerivative(settings) ? settings.kd : 0;

  if (settings.plant === "compare") {
    if (!integral) return true;
    return settings.ki < (2 + kd) * (1 + settings.kp);
  }

  const a = 2 * settings.z * settings.w + settings.w ** 2 * kd;

  if (settings.kp <= 0 || a <= 0) return false;
  if (!integral) return true;

  return settings.ki < a * settings.kp;
}

// Display the mathematical equation for each controller
function mathLaw(type) {
  const p = String.raw`K_pe(t)`;
  const i = String.raw`K_i\int_0^t e(\tau)\,d\tau`;
  const d = String.raw`K_d\frac{de(t)}{dt}`;

  if (type === "P") return String.raw`\[u(t)=${p}\]`;
  if (type === "PI") {
    return String.raw`\[\begin{aligned}u(t)&=${p}\\&+${i}\end{aligned}\]`;
  }
  if (type === "PD") {
    return String.raw`\[\begin{aligned}u(t)&=${p}\\&+${d}\end{aligned}\]`;
  }

  return String.raw`\[\begin{aligned}u(t)&=${p}+${i}\\&+${d}\end{aligned}\]`;
}

// Calculate the theoretical steady-state error
function theoreticalError(settings) {
  if (!settings || !isStable(settings)) return null;

  if (settings.plant === "compare") {
    return hasIntegral(settings) ? 0 : settings.target / (1 + settings.kp);
  }

  return 0;
}

// Show the steady-state error derivation for the selected controller
function updateTheory() {
  const settings = currentConfig;
  const box = $("essDerivation");

  if (!settings) {
    text("theoreticalEss", "—");
    return;
  }

  if (settings.mode !== "closed") {
    text("theoreticalEss", "N/A");
    return;
  }

  const result = theoreticalError(settings);
  text(
    "theoreticalEss",
    result === null ? "No stable final value" : `${fmt(result, 3)} m`,
  );

  if (!box || settings.plant !== "compare") return;

  const type = settings.type;
  const integral = hasIntegral(settings);
  const derivative = hasDerivative(settings);
  const damping = derivative ? "(2+K_D)" : "2";
  const reference = fmt(settings.target, 2);

  const laws = {
    P: String.raw`K_P`,
    PD: String.raw`K_P+K_Ds`,
    PI: String.raw`K_P+\frac{K_I}{s}`,
    PID: String.raw`K_P+\frac{K_I}{s}+K_Ds`,
  };

  let working = equation(String.raw`C_{\mathrm{${type}}}(s)=${laws[type]}`);

  if (integral) {
    const characteristic = String.raw`s^3+${damping}s^2+(1+K_P)s+K_I`;

    working += equation(
      String.raw`E(s)=\frac{${reference}}{s}\frac{s(s^2+2s+1)}{${characteristic}}`,
    );
    working += equation(
      String.raw`e_{ss}=\lim_{s\to0}\frac{${reference}s(s^2+2s+1)}{${characteristic}}=0`,
    );
    working += `
      <p>
        ${type} includes integral action with
        Kᵢ = ${fmt(settings.ki, 3)}.
        For a stable closed loop, this removes the constant step-reference error.
      </p>
      <p class="note">
        Stability requires
        Kᵢ &lt; ${derivative ? "(2 + Kᴅ)" : "2"}(1 + Kₚ).
      </p>`;
  } else {
    if (["PI", "PID"].includes(type)) {
      working += `
        <p>
          Kᵢ is zero. This ${type} controller currently behaves as
          ${derivative ? "PD" : "P"}.
        </p>`;
    }

    working += equation(
      String.raw`E(s)=\frac{${reference}}{s}\frac{s^2+2s+1}{s^2+${damping}s+(1+K_P)}`,
    );
    working += equation(
      String.raw`e_{ss}=\frac{${reference}}{1+K_P}=\frac{${reference}}{1+${fmt(settings.kp, 3)}}=${fmt(settings.target / (1 + settings.kp), 3)}\,\mathrm{m}`,
    );

    working += derivative
      ? `<p>
          PD has the same theoretical step-reference error as P at the
          same Kₚ. At steady state, the derivative term is zero.
          It changes the transient response rather than removing this offset.
        </p>`
      : `<p>
          P needs a nonzero error to produce the constant control input
          that balances this plant’s restoring term.
        </p>`;
  }

  const warning = isStable(settings)
    ? ""
    : `<p class="note">
        These gains do not give a stable closed loop.
        The Final Value Theorem cannot be used to claim a settled error.
      </p>`;

  box.innerHTML = `
    <p><strong>Selected controller: ${type}</strong></p>
    <p><strong>1 · Write the error transfer function</strong></p>
    ${equation(
      String.raw`E(s)=\frac{R(s)}{1+C(s)G_1(s)},\qquad R(s)=\frac{${reference}}{s}`,
    )}
    <p><strong>2 · Substitute ${type} and apply the Final Value Theorem</strong></p>
    ${working}
    ${warning}
    <p><strong>3 · Test ${type}</strong></p>
    <p>
      Launch a fresh flight with disturbance OFF.
      Wait for the observed steady-state error, then record the result.
    </p>`;

  typeset([box]);
}

// Check whether the drone has reached steady state
function observedSettled() {
  if (
    !hasLaunched ||
    !currentConfig ||
    !isStable(currentConfig) ||
    state.t < 3 ||
    wind.value !== 0 ||
    wind.until > 0
  ) {
    return false;
  }

  const recent = samples.filter((point) => point.t >= state.t - 3);

  if (
    recent.length < 40 ||
    state.t - recent[0].t < 2.9 ||
    recent.some((point) => Math.abs(point.v) >= 0.015)
  ) {
    return false;
  }

  const values = recent.map((point) => point.y);
  return Math.max(...values) - Math.min(...values) < 0.015;
}

// Display the measured steady-state error
function updateSteadyError() {
  if (!hasLaunched) {
    text("ess", "—");
    return;
  }

  text(
    "ess",
    observedSettled()
      ? `${fmt(currentConfig.target - state.altitude, 3)} m`
      : "Not settled",
  );
}

// Update controller fields and displayed equations
function updateForm() {
  const type = $("type").value;

  hide("pidFields", false);
  hide("kiRow", !["PI", "PID"].includes(type));
  hide("kdRow", !["PD", "PID"].includes(type));

  if ($("controllerFormula")) {
    $("controllerFormula").innerHTML = mathLaw(type);
  }
  if ($("inputFormula")) {
    $("inputFormula").innerHTML = mathLaw(type);
  }

  typeset([$("controllerFormula"), $("inputFormula")]);

  text("modeLabel", `With ${type}`);
  text(
    "tip",
    $("experiment").value === "compare"
      ? "Compare P, PD, PI and PID at the same proportional gain. Record each settled, wind-free result."
      : "Calculate your PID gains, launch a fresh flight, then compare the full response with the design estimates.",
  );

  for (const id of numericIds) {
    text(`${id}O`, $(id).value);
  }

  try {
    currentConfig = config();
  } catch (error) {
    currentConfig = null;
    showStatus(error.message);
  }

  analysisStart = 0;

  if ($("analysisScroll")) $("analysisScroll").value = "0";
  text("analysisRange", "0.0–1.0 s");

  recomputeAnalysis();
  analysisMetrics();
  updateTheory();
  updateControllerButtons();

  const target = currentConfig?.target || 10;
  show(state, target);
  draw(target);
  drawAnalysis(target);
}

// Initialise the drone graphics using PIXI
function initDrone() {
  if (!window.PIXI || droneApp || !$("droneStage") || !$("sky")) return;

  droneApp = new PIXI.Application({
    resizeTo: $("sky"),
    backgroundAlpha: 0,
    antialias: true,
  });

  $("droneStage").appendChild(droneApp.view);

  const drone = new PIXI.Container();
  const arms = new PIXI.Graphics();
  const body = new PIXI.Graphics();
  const rotors = new PIXI.Graphics();

  arms.beginFill(0x243b4e).drawRect(-60, -2, 120, 5).endFill();

  body
    .beginFill(0xedf6fc)
    .lineStyle(2, 0x1a8bd0)
    .drawRoundedRect(-24, -16, 48, 33, 9)
    .endFill();

  body.beginFill(0x1a8bd0).drawPolygon([-5, -2, 0, 5, 5, -2, 0, -8]).endFill();

  rotors
    .lineStyle(2, 0x233d51)
    .drawEllipse(-43, -20, 32, 4)
    .drawEllipse(43, -20, 32, 4);

  drone.addChild(arms, rotors, body);
  droneApp.stage.addChild(drone);
  droneSprite = drone;
}

// Update the drone position based on its altitude
function moveDrone(altitude, target) {
  if (!droneSprite) initDrone();
  if (!droneSprite) return;

  const sky = $("sky");
  const maxAltitude = Math.max(target * 1.5, 1);
  const fraction = Math.max(0, Math.min(1, altitude / maxAltitude));

  droneSprite.x = sky.clientWidth / 2;
  droneSprite.y = sky.clientHeight - 28 - fraction * (sky.clientHeight - 56);
}

// Update the live flight readings and drone display
function show(point, target) {
  const p =
    point === state
      ? {
          t: state.t,
          y: state.altitude,
          v: state.verticalVelocity,
          u: state.controlInput,
        }
      : point;

  if (wind.until > 0) {
    wind.maxDeviation = Math.max(
      wind.maxDeviation,
      Math.abs(p.y - wind.baseline),
    );
  }

  text(
    "gustDeviation",
    wind.until > 0
      ? `Gust altitude change: ${fmt(p.y - wind.baseline, 3)} m · largest ${fmt(wind.maxDeviation, 3)} m`
      : wind.lastDeviation !== null
        ? `Last gust: largest ${fmt(wind.lastDeviation, 3)} m · finished`
        : "No gust yet",
  );

  const appliedWind =
    wind.value +
    (constantWind && currentConfig?.plant === "compare"
      ? comparisonWindAcceleration(currentConfig)
      : 0);

  text(
    "wind",
    constantWind
      ? `Constant downward · ${fmt(Math.abs(appliedWind), 1)} m/s²`
      : wind.value === 0
        ? "Calm"
        : `${wind.value > 0 ? "Upward" : "Downward"} gust · ${fmt(Math.abs(wind.value), 0)} m/s²`,
  );

  text("time", `${fmt(p.t)} s`);
  text("altitude", `${fmt(p.y)} m`);
  text("velocity", `${fmt(p.v)} m/s`);
  text("command", fmt(p.u));
  text("error", `${fmt(target - p.y)} m`);

  moveDrone(p.y, target);

  const sky = $("sky");
  const maxAltitude = Math.max(target * 1.5, 1);

  if (sky && $("targetLine")) {
    const fraction = Math.max(0, Math.min(1, target / maxAltitude));
    $("targetLine").style.top =
      `${sky.clientHeight - 28 - fraction * (sky.clientHeight - 56)}px`;
    $("targetLine").style.bottom = "auto";
  }

  text("targetLabel", `TARGET · ${fmt(target, 1)} m`);
  text("topScale", `${fmt(maxAltitude, 1)} m`);
}

// Calculate the constant wind disturbance for Experiment 1
function comparisonWindAcceleration(settings) {
  return (
    -(1 + Math.max(0, settings.kp)) *
    CONSTANT_WIND_TARGET_PD_OFFSET *
    Number($("gustStrength").value)
  );
}

// Update the constant wind button label
function syncWindButton() {
  text(
    "holdWind",
    constantWind ? "Stop constant disturbance" : "Start constant disturbance",
  );
}

// Reset all wind disturbance values
function resetWind() {
  wind = freshWind();
  constantWind = false;
  syncWindButton();
}

// Stop the gust when its duration has ended
function updateWind(settings) {
  if (wind.until > 0 && (settings.mode !== "closed" || state.t >= wind.until)) {
    wind.maxDeviation = Math.max(
      wind.maxDeviation,
      Math.abs(state.altitude - wind.baseline),
    );

    wind.lastDeviation = wind.maxDeviation;
    wind.value = 0;
    wind.until = 0;

    showStatus("Gust finished. Wind is calm; you can apply another gust.");
  }
}

// Apply a temporary downward wind gust during flight
function applyWind() {
  if (!running || !currentConfig || currentConfig.mode !== "closed") {
    showStatus("Launch the drone in feedback mode before applying wind.");
    return;
  }

  if (wind.until > 0) {
    showStatus("A gust is already active. Wait for it to finish.");
    return;
  }

  const stiffness =
    (currentConfig.plant === "compare" ? 1 : currentConfig.w ** 2) *
    Math.max(currentConfig.kp, 1);

  wind.value =
    GUST_DIRECTION *
    Math.max(120, Math.min(9000, stiffness * 1.5)) *
    Number($("gustStrength").value);

  wind.baseline = state.altitude;
  wind.maxDeviation = 0;
  wind.lastDeviation = null;
  wind.index++;
  wind.until = state.t + GUST_DURATION;
  flightWasDisturbed = true;

  showStatus(
    "Downward gust applied for 0.2 simulated seconds. Watch the LIVE response and recovery.",
  );
}

// Reset the response measurements
function clearMetrics() {
  for (const id of ["rise", "peak", "os", "settle", "sse", "ess"]) {
    text(id, "—");
  }
}

// Find the first time the response reaches a given level
function firstCrossing(points, level) {
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1];
    const b = points[i];

    if (a.y < level && b.y >= level) {
      return a.t + ((level - a.y) / (b.y - a.y)) * (b.t - a.t);
    }
  }

  return null;
}

// Find the first peak in the response
function firstPeak(points) {
  for (let i = 1; i < points.length - 1; i++) {
    if (points[i].y >= points[i - 1].y && points[i].y > points[i + 1].y) {
      return points[i];
    }
  }

  return null;
}

// Calculate settling time using the specified tolerance
function settlingTime(points, finalValue, tolerance) {
  if (points.length < 2) return null;

  let lastOutside = -1;

  for (let i = points.length - 1; i >= 0; i--) {
    if (Math.abs(points[i].y - finalValue) > tolerance) {
      lastOutside = i;
      break;
    }
  }

  if (lastOutside < 0) return 0;
  if (lastOutside >= points.length - 1) return null;

  const a = points[lastOutside];
  const b = points[lastOutside + 1];
  const boundary =
    a.y > finalValue ? finalValue + tolerance : finalValue - tolerance;
  const change = b.y - a.y;

  if (Math.abs(change) < 1e-12) return b.t;

  const fraction = Math.max(0, Math.min(1, (boundary - a.y) / change));

  return a.t + fraction * (b.t - a.t);
}

// Calculate rise time, peak time, overshoot and settling time
function responseMetrics() {
  if (!hasLaunched || !currentConfig || analysisSamples.length < 3) {
    return null;
  }

  const target = currentConfig.target;
  const tr = firstCrossing(analysisSamples, target);
  const candidate = firstPeak(analysisSamples);

  const finalAltitude =
    currentConfig.plant === "compare"
      ? target - (theoreticalError(currentConfig) ?? 0)
      : target;

  const peak =
    candidate && candidate.y > finalAltitude * 1.001 ? candidate : null;

  const maximum = Math.max(...analysisSamples.map((point) => point.y));
  const overshoot = Math.max(0, ((maximum - target) / target) * 100);
  const tolerance = Math.max(0.02, Math.abs(target) * 0.02);
  const measuredTs = settlingTime(analysisSamples, target, tolerance);
  const ts =
    measuredTs !== null && analysisSamples.at(-1).t - measuredTs >= 1
      ? measuredTs
      : null;

  return { tr, peak, overshoot, ts, tolerance };
}

// Display the calculated response measurements
function analysisMetrics() {
  const metrics = responseMetrics();

  if (!metrics) {
    clearMetrics();
    updateDesignResult();
    return;
  }

  text("rise", metrics.tr === null ? "—" : `${fmt(metrics.tr)} s`);
  text("peak", metrics.peak ? `${fmt(metrics.peak.t)} s` : "No overshoot peak");
  text("os", `${fmt(metrics.overshoot, 1)} %`);
  text("settle", metrics.ts === null ? "N/A" : `${fmt(metrics.ts)} s`);
  text("sse", `${fmt(currentConfig.target - state.altitude)} m`);

  updateSteadyError();
  updateDesignResult();
}

// Plot the live drone altitude response
function draw(target) {
  if (!window.Plotly || !$("chart")) return;

  const now = performance.now();

  if (state.t > 0 && now - lastLivePlot < 100) return;
  lastLivePlot = now;

  const start = Math.max(0, state.t - WINDOW);
  const visible = samples.filter((point) => point.t >= start);

  Plotly.react(
    "chart",
    [
      {
        x: visible.map((point) => point.t),
        y: visible.map((point) => point.y),
        type: "scatter",
        mode: "lines",
        name: "Altitude",
        line: { color: "#38bdf8", width: 3 },
      },
      {
        x: [start, start + WINDOW],
        y: [target, target],
        type: "scatter",
        mode: "lines",
        name: "Target",
        line: { color: "#ffb347", dash: "dash", width: 2 },
      },
    ],
    {
      paper_bgcolor: "#071a2f",
      plot_bgcolor: "#071a2f",
      font: { color: "#d7edff" },
      margin: { l: 65, r: 20, t: 20, b: 50 },
      xaxis: {
        title: { text: "Time (s)" },
        range: [start, start + WINDOW],
        gridcolor: "#244563",
      },
      yaxis: {
        title: { text: "Altitude (m)" },
        gridcolor: "#244563",
      },
      showlegend: true,
      legend: { orientation: "h" },
    },
    { responsive: true, displaylogo: false, scrollZoom: true },
  );
}

// Construct the full closed-loop transfer function model
// Polynomial coefficients are stored in ascending powers of s
function closedLoopModel(settings) {
  const b = settings.plant === "compare" ? 1 : settings.w ** 2;
  const a = settings.plant === "compare" ? 2 : 2 * settings.z * settings.w;
  const restoring = settings.plant === "compare" ? 1 : 0;
  const kd = hasDerivative(settings) ? settings.kd : 0;

  if (hasIntegral(settings)) {
    return {
      den: [b * settings.ki, restoring + b * settings.kp, a + b * kd],
      num: [b * settings.ki, b * settings.kp, b * kd],
    };
  }

  return {
    den: [restoring + b * settings.kp, a + b * kd],
    num: [b * settings.kp, b * kd],
  };
}

// Simulate the closed-loop response using the fourth-order Runge-Kutta method
function simulateClosedLoopModel(settings) {
  const model = closedLoopModel(settings);
  const order = model.den.length;
  let x = Array(order).fill(0);

  const derivative = (q) => {
    const result = Array(order).fill(0);

    for (let i = 0; i < order - 1; i++) {
      result[i] = q[i + 1];
    }

    let last = settings.target;

    for (let i = 0; i < order; i++) {
      last -= model.den[i] * q[i];
    }

    result[order - 1] = last;
    return result;
  };

  const output = (q) =>
    model.num.reduce((sum, coefficient, i) => sum + coefficient * q[i], 0);

  const result = [initialSample()];
  const totalSteps = Math.round(ANALYSIS_END / simulationTimeStep);

  for (let n = 0; n < totalSteps; n++) {
    const k1 = derivative(x);
    const k2 = derivative(
      x.map((value, i) => value + (k1[i] * simulationTimeStep) / 2),
    );
    const k3 = derivative(
      x.map((value, i) => value + (k2[i] * simulationTimeStep) / 2),
    );
    const k4 = derivative(
      x.map((value, i) => value + k3[i] * simulationTimeStep),
    );

    x = x.map(
      (value, i) =>
        value +
        (simulationTimeStep * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])) / 6,
    );

    const y = output(x);
    const t = (n + 1) * simulationTimeStep;

    if (!Number.isFinite(y) || Math.abs(y) > 1e7) return [];

    if ((n + 1) % SAMPLE_EVERY === 0) {
      result.push({ t, y, v: 0, u: 0 });
    }
  }

  return result;
}

// Generate the complete response for graph analysis
function recomputeAnalysis() {
  analysisSamples = [initialSample()];

  if (!currentConfig || !hasLaunched) return;

  if (currentConfig.mode === "closed") {
    analysisSamples = simulateClosedLoopModel(currentConfig);
    return;
  }

  const settings = currentConfig;
  let y = 0;
  let v = 0;

  for (let n = 0; n < Math.round(ANALYSIS_END / simulationTimeStep); n++) {
    const acceleration =
      settings.plant === "compare"
        ? settings.fixed - 2 * v - y
        : settings.w ** 2 * settings.fixed - 2 * settings.z * settings.w * v;

    v += acceleration * simulationTimeStep;
    y += v * simulationTimeStep;

    if (!Number.isFinite(y) || Math.abs(y) > 1e7) {
      analysisSamples = [];
      return;
    }

    if ((n + 1) % SAMPLE_EVERY === 0) {
      analysisSamples.push({
        t: (n + 1) * simulationTimeStep,
        y,
        v,
        u: settings.fixed,
      });
    }
  }
}

// Plot the detailed response with performance measurements
function drawAnalysis(target) {
  if (!window.Plotly || !$("analysisChart")) return;

  const start = analysisStart;
  const end = start + 1;
  const ready = hasLaunched && analysisSamples.length > 2;
  const metrics = ready ? responseMetrics() : null;
  const tolerance = metrics?.tolerance ?? Math.max(0.02, target * 0.02);

  if ($("analysisNote")) {
    $("analysisNote").innerHTML = `
      Browse the 30-second response using the slider.<br>
      <b>t<sub>r</sub> — Rise time:</b> First reaches 100% of the target.<br>
      <b>t<sub>p</sub> — Peak time:</b> Time of the first overshoot peak.<br>
      <b>t<sub>s</sub> — Settling time:</b> Remains within ±2% of the target
      for the rest of the analysed response.<br>
      <b>M<sub>p</sub> — Percentage overshoot:</b> Maximum amount above
      the target, expressed as a percentage of the target.
    `;
    $("analysisNote").style.lineHeight = "1.8";
  }

  const visible = ready
    ? analysisSamples.filter((p) => p.t >= start - 0.02 && p.t <= end + 0.02)
    : [];

  const shapes = [];
  const annotations = [];
  const traces = [];

  if (ready) {
    shapes.push({
      type: "rect",
      xref: "x",
      yref: "y",
      x0: start,
      x1: end,
      y0: target - tolerance,
      y1: target + tolerance,
      fillcolor: "rgba(127,228,160,.14)",
      line: { width: 0 },
      layer: "below",
    });

    traces.push(
      {
        x: visible.map((p) => p.t),
        y: visible.map((p) => p.y),
        type: "scatter",
        mode: "lines",
        name: "Wind-free model response",
        line: { color: "#38bdf8", width: 4 },
        hovertemplate:
          "Time: %{x:.3f} s<br>Altitude: %{y:.3f} m<extra></extra>",
      },
      {
        x: [start, end],
        y: [target, target],
        type: "scatter",
        mode: "lines",
        name: "Target",
        line: { color: "#ffb347", dash: "dash", width: 3 },
        hoverinfo: "skip",
      },
    );

    // Mark the response measurement times on the graph
    function mark(time, label, color, offset = 10) {
      if (!Number.isFinite(time) || time < start || time > end) return;

      shapes.push({
        type: "line",
        xref: "x",
        yref: "paper",
        x0: time,
        x1: time,
        y0: 0,
        y1: 1,
        line: { color, dash: "dot", width: 2 },
      });

      annotations.push({
        x: time,
        y: 1,
        xref: "x",
        yref: "paper",
        text: `<b>${label}</b>`,
        font: { color, size: 15 },
        showarrow: false,
        yanchor: "bottom",
        yshift: offset,
        bgcolor: "rgba(7,26,47,.85)",
      });
    }

    mark(metrics?.tr, "t<sub>r</sub>", "#38bdf8");
    mark(metrics?.peak?.t, "t<sub>p</sub>", "#fb7185", 32);
    mark(metrics?.ts, "t<sub>s</sub>", "#4ade80");

    // Highlight the first overshoot peak and its percentage
    const peak = metrics?.peak;

    if (peak && peak.t >= start && peak.t <= end) {
      const mp = Math.max(0, ((peak.y - target) / target) * 100);

      traces.push({
        x: [peak.t],
        y: [peak.y],
        type: "scatter",
        mode: "markers+text",
        name: "First peak",
        showlegend: false,
        marker: { size: 11, color: "#fb7185" },
        text: [`M<sub>p</sub> = ${fmt(mp, 2)}%`],
        textposition: peak.t > start + 0.7 ? "top left" : "top right",
        textfont: { color: "#ffd7df", size: 14 },
        cliponaxis: false,
        hovertemplate:
          `Peak time: ${fmt(peak.t, 3)} s<br>` +
          `Peak altitude: ${fmt(peak.y, 3)} m<br>` +
          `Mₚ: ${fmt(mp, 2)}%<extra></extra>`,
      });
    }
  } else {
    annotations.push({
      text: hasLaunched
        ? "Response could not be analysed. Check the gains."
        : "Launch the drone to analyse a fresh takeoff.",
      xref: "paper",
      yref: "paper",
      x: 0.5,
      y: 0.5,
      showarrow: false,
    });
  }

  const values = visible
    .map((p) => p.y)
    .concat(0, target - tolerance, target + tolerance);
  const minimum = Math.min(...values);
  const maximum = Math.max(...values);
  const padding = Math.max(0.5, (maximum - minimum) * 0.12);

  // Render the analysis graph using Plotly
  Plotly.react(
    "analysisChart",
    traces,
    {
      paper_bgcolor: "#071a2f",
      plot_bgcolor: "#071a2f",
      font: { color: "#e7f5ff", size: 15 },
      margin: { l: 78, r: 35, t: 125, b: 75 },

      xaxis: {
        title: { text: "Time (s)", standoff: 18 },
        range: [start, end],
        gridcolor: "#315a78",
        gridwidth: 1.3,
        fixedrange: true,
        visible: ready,
      },

      yaxis: {
        title: { text: "Altitude (m)", standoff: 15 },
        range: [minimum - padding, maximum + padding],
        gridcolor: "#315a78",
        gridwidth: 1.3,
        fixedrange: false,
        visible: ready,
      },

      shapes,
      annotations,
      showlegend: ready,
      legend: {
        orientation: "h",
        x: 0,
        y: 1.25,
        xanchor: "left",
        yanchor: "bottom",
        font: { size: 14 },
      },
      hovermode: "closest",
    },
    {
      responsive: true,
      displaylogo: false,
      displayModeBar: "hover",
      scrollZoom: false,
    },
  );
}

// Calculate the controller output using the selected PID gains
function controllerOutput(settings, error) {
  const proportional = settings.kp * error;
  const integral = ["PI", "PID"].includes(settings.type)
    ? settings.ki * state.accumulatedError
    : 0;
  const derivative = hasDerivative(settings)
    ? -settings.kd * state.verticalVelocity
    : 0;

  return proportional + integral + derivative;
}

// Update the drone motion using the plant equations
function advance(settings) {
  const error = settings.target - state.altitude;
  const input =
    settings.mode === "open"
      ? settings.fixed
      : controllerOutput(settings, error);

  updateWind(settings);

  const disturbance =
    wind.value +
    (constantWind && settings.plant === "compare"
      ? comparisonWindAcceleration(settings)
      : 0);

  const acceleration =
    (settings.plant === "compare"
      ? input - 2 * state.verticalVelocity - state.altitude
      : settings.w ** 2 * input -
        2 * settings.z * settings.w * state.verticalVelocity) + disturbance;

  state.verticalVelocity += acceleration * simulationTimeStep;
  state.altitude += state.verticalVelocity * simulationTimeStep;

  if (settings.mode === "closed" && ["PI", "PID"].includes(settings.type)) {
    state.accumulatedError += error * simulationTimeStep;
  }

  state.controlInput = input;
  state.t += simulationTimeStep;
  steps++;

  if (
    !Number.isFinite(state.altitude) ||
    !Number.isFinite(state.verticalVelocity) ||
    Math.abs(state.altitude) > 1e7 ||
    Math.abs(state.verticalVelocity) > 1e7
  ) {
    pause();
    showStatus(
      "Simulation stopped: the selected gains produced an unstable or excessively large response.",
    );
    return false;
  }

  // Store flight samples for the live response graph
  if (steps % SAMPLE_EVERY === 0) {
    samples.push({
      t: state.t,
      y: state.altitude,
      v: state.verticalVelocity,
      u: state.controlInput,
    });

    if (samples.length > 10000) {
      samples = samples.filter(
        (point) => point.t >= state.t - Math.max(WINDOW + 5, 30),
      );
    }
  }

  return true;
}

// Run the simulation according to the selected animation speed
function tick(now) {
  if (!running) return;

  if (previousFrame === null) previousFrame = now;

  accumulator +=
    Math.min(0.05, (now - previousFrame) / 1000) * Number($("simSpeed").value);

  previousFrame = now;

  while (currentConfig && accumulator >= simulationTimeStep) {
    if (!advance(currentConfig)) return;
    accumulator -= simulationTimeStep;
  }

  const target = currentConfig?.target || 10;

  show(state, target);
  draw(target);
  text("sse", `${fmt(target - state.altitude)} m`);
  updateSteadyError();

  frame = requestAnimationFrame(tick);
}

// Pause the current simulation
function pause() {
  running = false;

  if (frame !== null) cancelAnimationFrame(frame);

  frame = null;
  previousFrame = null;

  if ($("run")) $("run").disabled = false;
}

// Resume the simulation from its current state
function resumeFlight() {
  if (running || !currentConfig) return;

  running = true;
  previousFrame = null;
  accumulator = 0;

  if ($("run")) $("run").disabled = true;

  frame = requestAnimationFrame(tick);
}

// Clear the flight data and reset the simulation state
function clearFlight() {
  pause();
  resetWind();

  hasLaunched = false;
  designHasLaunched = false;
  flightWasDisturbed = false;
  state = initialState();
  samples = [initialSample()];
  analysisSamples = [initialSample()];
  steps = 0;
  accumulator = 0;
  lastLivePlot = 0;

  clearMetrics();
}

// Launch a new flight using the current controller settings
function launch() {
  try {
    currentConfig = config();
  } catch (error) {
    showStatus(error.message);
    return false;
  }

  pause();
  resetWind();

  state = initialState();
  samples = [initialSample()];
  steps = 0;
  accumulator = 0;
  lastLivePlot = 0;

  hasLaunched = true;
  flightWasDisturbed = false;
  designHasLaunched = currentConfig.plant === "design";

  recomputeAnalysis();
  analysisMetrics();
  updateTheory();
  drawAnalysis(currentConfig.target);
  show(state, currentConfig.target);
  draw(currentConfig.target);
  updateChallengeButtons();
  resumeFlight();

  showStatus(
    "Drone launched. The live graph shows the flight; response analysis uses the complete wind-free model.",
  );

  return true;
}

// Reset the drone to its initial position
function reset() {
  clearFlight();
  updateForm();
  updateChallengeButtons();
  showStatus("Reset to ground level. Click Launch to start a fresh flight.");
}

// Reset the simulation when controller settings are changed
function settingsChanged() {
  clearFlight();
  designAnswerCorrect = false;

  updateForm();
  syncGuideGainsFromMain();
  updateChallengeButtons();

  if (designChallenge) {
    text(
      "designAnswerFeedback",
      "Settings changed. Check your gains, or launch a trial flight.",
    );
  }

  showStatus("Settings changed. Launch a new flight to measure this controller.");
}

// Generate a new PID design challenge with random requirements
function makeDesignChallenge() {
  const z = Number((0.55 + Math.random() * 0.25).toFixed(2));
  const tr = Number((0.35 + Math.random() * 0.2).toFixed(2));
  const theta = Math.acos(z);
  const wn = (Math.PI - theta) / (tr * Math.sqrt(1 - z * z));
  const alpha = z * wn;
  const p = 10 * alpha;

  designChallenge = {
    z,
    tr,
    theta,
    wn,
    alpha,
    p,
    kd: (2 * z * wn + p - 5) / 25,
    kp: (wn * wn + 2 * z * wn * p) / 25,
    ki: wn * wn * p / 25,
    mp: Math.exp(-z * Math.PI / Math.sqrt(1 - z * z)) * 100,
    tp: Math.PI / (wn * Math.sqrt(1 - z * z)),
    ts: 4 / (z * wn)
  };

  text(
    "designChallenge",
    "Your mission: calculate Kₚ, Kᵢ and Kᴅ for the given damping ratio, rise time and zero steady-state error. Use p = 10ζωₙ and keep the plant at ω₀ = 5, ζ₀ = 0.5."
  );

  text("missionZ", fmt(z, 2));
  text("missionTr", `${fmt(tr, 2)} s`);
}

// Copy the main controller gains into the guided challenge
function syncGuideGainsFromMain() {
  for (const [guide, main] of [
    ["guideKp", "kp"],
    ["guideKi", "ki"],
    ["guideKd", "kd"]
  ]) {
    if ($(guide) && $(main)) $(guide).value = $(main).value;
  }
}

// Read and validate the gains entered in the challenge
function readGuideGains() {
  const result = {};

  for (const [guide, main] of [
    ["guideKp", "kp"],
    ["guideKi", "ki"],
    ["guideKd", "kd"]
  ]) {
    if (!$(guide) || $(guide).value.trim() === "") return null;

    const value = Number($(guide).value);
    if (!Number.isFinite(value) || value < 0) return null;

    result[main] = value;
  }

  return result;
}

// Apply the guided challenge gains to the main simulator
function syncMainGainsFromGuide() {
  const gains = readGuideGains();
  if (!gains) return false;

  for (const id of ["kp", "ki", "kd"]) {
    $(id).value = gains[id];
  }

  $("type").value = "PID";
  return true;
}

// Check whether the challenge is ready for a trial flight
function guideReady() {
  return (
    $("experiment").value === "design" &&
    designChallenge !== null &&
    readGuideGains() !== null
  );
}

// Enable or disable challenge buttons based on progress
function updateChallengeButtons() {
  if ($("guideLaunch")) {
    $("guideLaunch").disabled = !guideReady();
  }

  if ($("anotherQuestion")) {
    $("anotherQuestion").disabled = !(
      solutionRevealed ||
      (designAnswerCorrect && designHasLaunched)
    );
  }

  if ($("guideSolution")) {
    $("guideSolution").textContent = solutionRevealed
      ? "View worked solution"
      : "Reveal solution";
  }
}

// Navigate between the guided challenge stages
function goToChallengeStage(id) {
  const step = $(id);
  if (!step) return;

  document.querySelectorAll("#designGuide .guide-step").forEach((item) => {
    item.open = item.id === id;
  });

  document.querySelectorAll("#designGuide [data-stage]").forEach((button) => {
    if (button.dataset.stage === id) {
      button.setAttribute("aria-current", "step");
    } else {
      button.removeAttribute("aria-current");
    }
  });

  requestAnimationFrame(() => {
    step.scrollIntoView({ behavior: "smooth", block: "nearest" });
  });
}

// Open the design challenge at the requirements stage
function openDesignAtRequirements() {
  hide("experimentGuide", false);
  goToChallengeStage("designRequirementsStep");
}

// Reset the challenge and optionally generate a new question
function resetForDesignQuestion(makeNew) {
  clearFlight();

  $("type").value = "PID";
  $("mode").value = "closed";
  $("w").value = "5";
  $("z").value = "0.5";
  $("target").value = "10";

  if (makeNew || !designChallenge) makeDesignChallenge();

  for (const id of ["kp", "ki", "kd"]) $(id).value = "0";

  designAnswerCorrect = false;
  solutionRevealed = false;

  hide("manualSolution");
  hide("predictionReveal");
  hide("solutionLocked", false);

  if ($("solutionBody")) $("solutionBody").innerHTML = "";
  if ($("predictionValues")) $("predictionValues").innerHTML = "";

  text(
    "designAnswerFeedback",
    "Enter your calculated gains. Check answer for feedback, or launch a trial flight."
  );

  updateForm();
  syncGuideGainsFromMain();
  updateChallengeButtons();

  text("designResult", "Launch your design to see the response.");
  showStatus("New design question ready. Calculate Kₚ, Kᵢ and Kᴅ.");
}

// Set the accepted rounding tolerance for each controller gain
function designGainTolerance(expected, gain) {
  const absoluteTolerance = gain === "kd" ? 0.1 : 1;
  return Math.max(absoluteTolerance, Math.abs(expected) * 0.01);
}

// Compare the entered gain against the expected solution
function designGainMatches(entered, expected, gain) {
  return (
    Number.isFinite(entered) &&
    Math.abs(entered - expected) <= designGainTolerance(expected, gain) + 1e-10
  );
}

// Check the student's calculated PID gains and provide feedback
function checkDesignAnswer() {
  if (!designChallenge) return;

  const entered = readGuideGains();

  if (!entered) {
    designAnswerCorrect = false;
    text(
      "designAnswerFeedback",
      "Enter a valid non-negative number for all three gains."
    );
    updateChallengeButtons();
    return;
  }

  syncMainGainsFromGuide();
  clearFlight();
  updateForm();

  const matches = ["kp", "ki", "kd"].map(
    (gain) => designGainMatches(entered[gain], designChallenge[gain], gain)
  );

  designAnswerCorrect = matches.every(Boolean);

  const labels = { kp: "Kₚ", ki: "Kᵢ", kd: "Kᴅ" };
  const details = ["kp", "ki", "kd"].map(
    (gain, index) => `${labels[gain]} ${matches[index] ? "✓" : "needs checking"}`
  );

  text(
    "designAnswerFeedback",
    designAnswerCorrect
      ? "Correct ✓ Your gains are within the accepted rounding range. Launch your design when ready."
      : `${details.join(" · ")}. Review the coefficient matching and try again.`
  );

  updateChallengeButtons();
}

// Apply the correct PID gains directly to the simulator
function useSolutionGains() {
  if (!designChallenge) return;

  clearFlight();

  $("type").value = "PID";
  $("mode").value = "closed";
  $("w").value = "5";
  $("z").value = "0.5";
  $("target").value = "10";

  for (const gain of ["kp", "ki", "kd"]) {
    $(gain).value = Number(designChallenge[gain].toFixed(4));
  }

  designAnswerCorrect = true;

  updateForm();
  syncGuideGainsFromMain();
  updateChallengeButtons();

  text(
    "designAnswerFeedback",
    "Solution gains applied ✓ Click Launch my design. No additional answer check is needed."
  );

  text("designResult", "Solution gains are ready. Launch a fresh flight.");
  showStatus("Solution gains applied. You can launch directly.");
  goToChallengeStage("enterDesignStep");
}

// Generate a new PID design challenge with random requirements
function makeDesignChallenge() {
  const z = Number((0.55 + Math.random() * 0.25).toFixed(2));
  const tr = Number((0.35 + Math.random() * 0.2).toFixed(2));
  const theta = Math.acos(z);
  const wn = (Math.PI - theta) / (tr * Math.sqrt(1 - z * z));
  const alpha = z * wn;
  const p = 10 * alpha;

  designChallenge = {
    z,
    tr,
    theta,
    wn,
    alpha,
    p,
    kd: (2 * z * wn + p - 5) / 25,
    kp: (wn * wn + 2 * z * wn * p) / 25,
    ki: wn * wn * p / 25,
    mp: Math.exp(-z * Math.PI / Math.sqrt(1 - z * z)) * 100,
    tp: Math.PI / (wn * Math.sqrt(1 - z * z)),
    ts: 4 / (z * wn)
  };

  text(
    "designChallenge",
    "Your mission: calculate Kₚ, Kᵢ and Kᴅ for the given damping ratio, rise time and zero steady-state error. Use p = 10ζωₙ and keep the plant at ω₀ = 5, ζ₀ = 0.5."
  );

  text("missionZ", fmt(z, 2));
  text("missionTr", `${fmt(tr, 2)} s`);
}

// Copy the main controller gains into the guided challenge
function syncGuideGainsFromMain() {
  for (const [guide, main] of [
    ["guideKp", "kp"],
    ["guideKi", "ki"],
    ["guideKd", "kd"]
  ]) {
    if ($(guide) && $(main)) $(guide).value = $(main).value;
  }
}

// Read and validate the gains entered in the challenge
function readGuideGains() {
  const result = {};

  for (const [guide, main] of [
    ["guideKp", "kp"],
    ["guideKi", "ki"],
    ["guideKd", "kd"]
  ]) {
    if (!$(guide) || $(guide).value.trim() === "") return null;

    const value = Number($(guide).value);
    if (!Number.isFinite(value) || value < 0) return null;

    result[main] = value;
  }

  return result;
}

// Apply the guided challenge gains to the main simulator
function syncMainGainsFromGuide() {
  const gains = readGuideGains();
  if (!gains) return false;

  for (const id of ["kp", "ki", "kd"]) {
    $(id).value = gains[id];
  }

  $("type").value = "PID";
  return true;
}

// Check whether the challenge is ready for a trial flight
function guideReady() {
  return (
    $("experiment").value === "design" &&
    designChallenge !== null &&
    readGuideGains() !== null
  );
}

// Enable or disable challenge buttons based on progress
function updateChallengeButtons() {
  if ($("guideLaunch")) {
    $("guideLaunch").disabled = !guideReady();
  }

  if ($("anotherQuestion")) {
    $("anotherQuestion").disabled = !(
      solutionRevealed ||
      (designAnswerCorrect && designHasLaunched)
    );
  }

  if ($("guideSolution")) {
    $("guideSolution").textContent = solutionRevealed
      ? "View worked solution"
      : "Reveal solution";
  }
}

// Navigate between the guided challenge stages
function goToChallengeStage(id) {
  const step = $(id);
  if (!step) return;

  document.querySelectorAll("#designGuide .guide-step").forEach((item) => {
    item.open = item.id === id;
  });

  document.querySelectorAll("#designGuide [data-stage]").forEach((button) => {
    if (button.dataset.stage === id) {
      button.setAttribute("aria-current", "step");
    } else {
      button.removeAttribute("aria-current");
    }
  });

  requestAnimationFrame(() => {
    step.scrollIntoView({ behavior: "smooth", block: "nearest" });
  });
}

// Open the design challenge at the requirements stage
function openDesignAtRequirements() {
  hide("experimentGuide", false);
  goToChallengeStage("designRequirementsStep");
}

// Reset the challenge and optionally generate a new question
function resetForDesignQuestion(makeNew) {
  clearFlight();

  $("type").value = "PID";
  $("mode").value = "closed";
  $("w").value = "5";
  $("z").value = "0.5";
  $("target").value = "10";

  if (makeNew || !designChallenge) makeDesignChallenge();

  for (const id of ["kp", "ki", "kd"]) $(id).value = "0";

  designAnswerCorrect = false;
  solutionRevealed = false;

  hide("manualSolution");
  hide("predictionReveal");
  hide("solutionLocked", false);

  if ($("solutionBody")) $("solutionBody").innerHTML = "";
  if ($("predictionValues")) $("predictionValues").innerHTML = "";

  text(
    "designAnswerFeedback",
    "Enter your calculated gains. Check answer for feedback, or launch a trial flight."
  );

  updateForm();
  syncGuideGainsFromMain();
  updateChallengeButtons();

  text("designResult", "Launch your design to see the response.");
  showStatus("New design question ready. Calculate Kₚ, Kᵢ and Kᴅ.");
}

// Set the accepted rounding tolerance for each controller gain
function designGainTolerance(expected, gain) {
  const absoluteTolerance = gain === "kd" ? 0.1 : 1;
  return Math.max(absoluteTolerance, Math.abs(expected) * 0.01);
}

// Compare the entered gain against the expected solution
function designGainMatches(entered, expected, gain) {
  return (
    Number.isFinite(entered) &&
    Math.abs(entered - expected) <= designGainTolerance(expected, gain) + 1e-10
  );
}

// Check the student's calculated PID gains and provide feedback
function checkDesignAnswer() {
  if (!designChallenge) return;

  const entered = readGuideGains();

  if (!entered) {
    designAnswerCorrect = false;
    text(
      "designAnswerFeedback",
      "Enter a valid non-negative number for all three gains."
    );
    updateChallengeButtons();
    return;
  }

  syncMainGainsFromGuide();
  clearFlight();
  updateForm();

  const matches = ["kp", "ki", "kd"].map(
    (gain) => designGainMatches(entered[gain], designChallenge[gain], gain)
  );

  designAnswerCorrect = matches.every(Boolean);

  const labels = { kp: "Kₚ", ki: "Kᵢ", kd: "Kᴅ" };
  const details = ["kp", "ki", "kd"].map(
    (gain, index) => `${labels[gain]} ${matches[index] ? "✓" : "needs checking"}`
  );

  text(
    "designAnswerFeedback",
    designAnswerCorrect
      ? "Correct ✓ Your gains are within the accepted rounding range. Launch your design when ready."
      : `${details.join(" · ")}. Review the coefficient matching and try again.`
  );

  updateChallengeButtons();
}

// Apply the correct PID gains directly to the simulator
function useSolutionGains() {
  if (!designChallenge) return;

  clearFlight();

  $("type").value = "PID";
  $("mode").value = "closed";
  $("w").value = "5";
  $("z").value = "0.5";
  $("target").value = "10";

  for (const gain of ["kp", "ki", "kd"]) {
    $(gain).value = Number(designChallenge[gain].toFixed(4));
  }

  designAnswerCorrect = true;

  updateForm();
  syncGuideGainsFromMain();
  updateChallengeButtons();

  text(
    "designAnswerFeedback",
    "Solution gains applied ✓ Click Launch my design. No additional answer check is needed."
  );

  text("designResult", "Solution gains are ready. Launch a fresh flight.");
  showStatus("Solution gains applied. You can launch directly.");
  goToChallengeStage("enterDesignStep");
}

// Update the selected controller button in Experiment 1
function updateControllerButtons() {
  document.querySelectorAll("[data-compare-controller]").forEach((button) => {
    const selected = button.dataset.compareController === $("type").value;
    button.setAttribute("aria-pressed", String(selected));
    button.classList.toggle("selected", selected);
  });
}

// Select a controller for the steady-state comparison
function selectComparisonController(type, launchNow = false) {
  if (!CONTROLLERS.includes(type)) return;

  if ($("experiment").value !== "compare") {
    $("experiment").value = "compare";
    updateExperiment();
  }

  $("type").value = type;
  settingsChanged();

  text(
    "comparisonStatus",
    `${type} selected. Launch with disturbance OFF, wait until settled, then record the result.`
  );

  if (launchNow) {
    hide("experimentGuide");
    launch();
    $("chart")?.scrollIntoView({ behavior: "smooth", block: "center" });
  }
}

// Set up the Experiment 1 guided manual and controller buttons
function installComparisonManual() {
  const guide = $("compareGuide");
  if (!guide) return;

  const heading = guide.querySelector("h3");
  if (heading) {
    heading.textContent =
      "Experiment 1 · P, PD, PI and PID steady-state challenge";
  }

  const details = [...guide.querySelectorAll(".guide-step")];

  if (details[1] && !$("compareControllerButtons")) {
    const box = document.createElement("div");
    box.id = "compareControllerButtons";
    box.className = "guide-controller-buttons";
    box.innerHTML = `
      <p><strong>Choose the controller to investigate</strong></p>
      <div class="guide-check-actions">
        ${CONTROLLERS.map(
          (type) => `
            <button
              type="button"
              class="guide-action-button secondary"
              data-compare-controller="${type}"
              aria-pressed="false"
            >${type}</button>`
        ).join("")}
      </div>
      <p class="note">
        These buttons update the simulator and the derivation below.
        Keep the same Kₚ when comparing all four controllers.
      </p>`;

    const derivation = $("essDerivation");
    if (derivation) derivation.before(box);
    else details[1].appendChild(box);
  }

  if (details[2] && !$("manualLaunchComparison")) {
    const actions = document.createElement("div");
    actions.className = "guide-check-actions";
    actions.innerHTML = `
      <button
        id="manualLaunchComparison"
        class="guide-action-button"
        type="button"
      >Launch selected controller</button>`;

    details[2].appendChild(actions);
  }

  // Display the expected behaviour of all four controllers
  if (details[4]) {
    const summary = details[4].querySelector("summary");
    if (summary) {
      summary.textContent = "5 · Compare all four: P vs PD vs PI vs PID";
    }

    const tbody = details[4].querySelector("tbody");

    if (tbody) {
      tbody.innerHTML = `
        <tr>
          <td><strong>P</strong></td>
          <td>
            With Kₚ &gt; 0, settles below the target with
            eₛₛ = 10/(1 + Kₚ).
          </td>
          <td>
            A nonzero error is needed to supply the plant’s steady input.
          </td>
        </tr>
        <tr>
          <td><strong>PD</strong></td>
          <td>
            Has the same theoretical final error as P at the same Kₚ.
            Its transient response can differ.
          </td>
          <td>
            Derivative action changes the transient behaviour.
            It is zero once the error stops changing.
          </td>
        </tr>
        <tr>
          <td><strong>PI</strong></td>
          <td>
            With Kᵢ &gt; 0 and stable gains, the final altitude approaches
            10 m and the error approaches zero.
          </td>
          <td>
            Integral action removes the constant step-reference offset.
            Settling can take longer for some gain choices.
          </td>
        </tr>
        <tr>
          <td><strong>PID</strong></td>
          <td>
            With Kᵢ &gt; 0 and stable gains, the final error also approaches
            zero. Compare its transient response with PI.
          </td>
          <td>
            Integral action removes the offset; derivative action changes
            the transient response.
          </td>
        </tr>`;
    }
  }

  guide.querySelectorAll("[data-compare-controller]").forEach((button) => {
    button.addEventListener("click", () => {
      selectComparisonController(button.dataset.compareController);
    });
  });

  on("manualLaunchComparison", "click", () => {
    selectComparisonController($("type").value, true);
  });
}

// Display the recorded results for P, PD, PI and PID
function renderComparisonResults() {
  if (!$("comparisonRows")) return;

  $("comparisonRows").innerHTML = CONTROLLERS.map((type) => {
    const result = comparisonResults[type];

    if (!result) {
      return `
        <tr>
          <td><strong>${type}</strong></td>
          <td>—</td>
          <td>Not recorded</td>
          <td>—</td>
        </tr>`;
    }

    return `
      <tr>
        <td>
          <strong>${type}</strong>
          <small style="display:block">
            Kₚ=${fmt(result.kp, 2)},
            Kᵢ=${fmt(result.ki, 2)},
            Kᴅ=${fmt(result.kd, 2)}
          </small>
        </td>
        <td>${fmt(result.theory, 3)} m</td>
        <td>${fmt(result.error, 3)} m</td>
        <td>${fmt(result.alt, 3)} m</td>
      </tr>`;
  }).join("");
}

// Record the steady-state error after a wind-free flight has settled
function recordComparison() {
  if (
    $("experiment").value !== "compare" ||
    !currentConfig ||
    currentConfig.mode !== "closed" ||
    !hasLaunched
  ) {
    text(
      "comparisonStatus",
      "Select Experiment 1 and launch a fresh flight before recording."
    );
    return;
  }

  if (flightWasDisturbed || constantWind || wind.value !== 0) {
    text(
      "comparisonStatus",
      "Launch a fresh wind-free flight before recording the step-reference error."
    );
    return;
  }

  if (!observedSettled()) {
    text(
      "comparisonStatus",
      "The response has not settled. Wait until Observed steady-state error shows a value."
    );
    return;
  }

  const settings = currentConfig;

  comparisonResults[settings.type] = {
    alt: state.altitude,
    error: settings.target - state.altitude,
    theory: theoreticalError(settings),
    kp: settings.kp,
    ki: ["PI", "PID"].includes(settings.type) ? settings.ki : 0,
    kd: hasDerivative(settings) ? settings.kd : 0
  };

  renderComparisonResults();

  const count = Object.keys(comparisonResults).length;

  text(
    "comparisonStatus",
    count === 4
      ? "All four controllers recorded. Compare P, PD, PI and PID, checking that the runs used the same Kₚ."
      : `${settings.type} recorded · ${count}/4 complete. Choose another controller and launch a fresh flight.`
  );
}

// Switch between the steady-state comparison and PID design experiments
function updateExperiment() {
  clearFlight();

  const compare = $("experiment").value === "compare";

  hide("designGuide", compare);
  hide("compareGuide", !compare);
  hide("experimentGuide");
  hide("solutionModal");
  hide("holdWind", !compare);
  hide("applyWind", compare);
  hide("readyDesign", compare);

  // Keep the review and prediction inside the experiment manual
  hide("completeDesign");
  hide("designPlantSettings", compare);
  hide("controllerTypeRow", !compare);
  hide("pidLockedController", compare);

  $("plantFormula").innerHTML = compare
    ? String.raw`\(G_1(s)=\dfrac{1}{s^2+2s+1}\)`
    : String.raw`\(G_2(s)=\dfrac{\omega_0^2}{s(s+2\zeta_0\omega_0)}\)`;

  $("plantDynamics").innerHTML = compare
    ? String.raw`\(\ddot y=u-2\dot y-y+d(t)\)`
    : String.raw`\(\ddot y=\omega_0^2u-2\zeta_0\omega_0\dot y+d(t)\)`;

  text(
    "plantNote",
    compare
      ? "Experiment 1 uses a normalised plant with a restoring term to compare steady-state error."
      : "Experiment 2 uses the integrator plant for the PID design challenge."
  );

  text(
    "experimentIntro",
    compare
      ? "Compare P, PD, PI and PID for the same 10 m target. Predict, launch, observe and record."
      : "Calculate PID gains, test your design and review the worked solution."
  );

  text(
    "quickHelp",
    compare
      ? "Use the manual’s controller buttons. Launch without wind, wait until settled, then record each result."
      : "Open Ready to design. You can check your answer or launch a trial flight."
  );

  $("mode").value = "closed";
  $("target").value = "10";
  $("w").value = "5";
  $("z").value = "0.5";
  $("type").value = compare ? "P" : "PID";

  if (compare) {
    $("kp").value = "100";
    $("ki").value = "4";
    $("kd").value = "5";

    updateForm();
    syncGuideGainsFromMain();
  } else {
    resetForDesignQuestion(!designChallenge);
  }

  updateChallengeButtons();
  typeset([$("plantFormula"), $("plantDynamics")]);

  showStatus(
    compare
      ? "Experiment 1 ready. Open the manual or launch the selected controller."
      : "Experiment 2 ready. Open Ready to design to begin the challenge."
  );
}

// Add styling for the guided experiment manual
function installManualStyles() {
  if ($("droneManualEnhancements")) return;

  const style = document.createElement("style");
  style.id = "droneManualEnhancements";

  style.textContent = `
    #experimentGuide [hidden] {
      display: none !important;
    }

    #designGuide .guide-step,
    #compareGuide .guide-step {
      margin: 14px 0;
      border: 1px solid rgba(125, 211, 252, .22);
      border-radius: 15px;
      overflow: hidden;
      background: rgba(8, 30, 52, .55);
    }

    #designGuide .guide-step > summary,
    #compareGuide .guide-step > summary {
      padding: 17px 20px;
      cursor: pointer;
      font-weight: 700;
    }

    #designGuide .guide-step[open],
    #compareGuide .guide-step[open] {
      border-color: rgba(56, 189, 248, .65);
    }

    .guide-controller-buttons {
      margin: 16px 0;
      padding: 16px;
      border: 1px solid rgba(125, 211, 252, .24);
      border-radius: 12px;
    }

    .guide-check-actions,
    .guide-end-actions {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin: 16px 0;
    }

    #experimentGuide .guide-action-button {
      padding: 11px 17px;
      border-radius: 10px;
      cursor: pointer;
      font-weight: 700;
    }

    #experimentGuide button:disabled {
      opacity: .45;
      cursor: not-allowed;
    }

    #experimentGuide [data-compare-controller].selected {
      background: #38bdf8;
      color: #06233b;
      border-color: #7dd3fc;
      box-shadow: 0 0 0 2px rgba(56, 189, 248, .2);
    }

    #predictionReveal {
      margin-top: 25px;
      padding: 20px;
      border: 1px solid rgba(56, 189, 248, .35);
      border-radius: 14px;
      background: rgba(14, 65, 95, .3);
    }

    #predictionValues {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(145px, 1fr));
      gap: 12px;
    }

    #predictionValues > div {
      padding: 15px;
      border-radius: 10px;
      background: rgba(5, 24, 43, .65);
    }

    #predictionValues span,
    #predictionValues strong {
      display: block;
    }

    #predictionValues span {
      font-size: .85rem;
      margin-bottom: 8px;
    }

    #predictionValues strong {
      color: #7dd3fc;
      font-size: 1.35rem;
    }

    #designAnswerFeedback {
      padding: 14px 16px;
      border-left: 3px solid #38bdf8;
      border-radius: 8px;
      background: rgba(56, 189, 248, .08);
      line-height: 1.6;
    }

    #solutionBody .solution-equation {
      overflow-x: auto;
      padding: 8px 0;
    }

    #solutionBody h3 {
      margin-top: 25px;
    }
  `;

  document.head.appendChild(style);
}

// Prepare the experiment manual and its guided learning controls
function prepareManual() {
  installManualStyles();
  installComparisonManual();

  // Allow calculated design gains to exceed the original input limits
  for (const id of [
    "kp", "ki", "kd", "guideKp", "guideKi", "guideKd"
  ]) {
    if ($(id)) {
      $(id).removeAttribute("max");
      $(id).step = "any";
    }
  }

  const manual = $("manualSolution");
  const prediction = $("predictionReveal");

  if (manual && prediction && !manual.contains(prediction)) {
    manual.appendChild(prediction);
  }

  hide("manualSolution");
  hide("predictionReveal");

  const enterStep = $("enterDesignStep");

  if (enterStep) {
    const paragraph = enterStep.querySelector("p");
    if (paragraph) {
      paragraph.textContent =
        "Enter your calculated gains. Check answer gives feedback; Launch my design lets you test a valid set of gains directly.";
    }
  }

  const review = $("understandDesignStep");
  if (review) {
    const paragraph = review.querySelector("p");
    if (paragraph) {
      paragraph.textContent =
        "Reveal the working when ready. The second-order prediction appears below the solution, and Try another question becomes available immediately.";
    }
  }

  if ($("guideLaunch")) {
    $("guideLaunch").textContent = "Launch my design";
  }

  text(
    "analysisNote",
    "Browse the 30 s wind-free model response. tᵣ is the first 0–100% target crossing, tₚ is the first peak, and tₛ uses the ±2% target band."
  );
}

// Main simulator button and input events
on("run", "click", launch);
on("reset", "click", reset);
on("applyWind", "click", applyWind);
on("type", "change", settingsChanged);
on("mode", "change", settingsChanged);
on("experiment", "change", updateExperiment);
on("recordComparison", "click", recordComparison);

for (const id of numericIds) {
  on(id, "input", settingsChanged);
}

// Keep the damping ratio within its allowed range
on("z", "change", () => {
  const value = Number($("z").value);
  $("z").value = Number.isFinite(value)
    ? Math.max(0, Math.min(1, value))
    : 0.5;

  settingsChanged();
});

// Update the disturbance strength and simulation speed displays
on("gustStrength", "input", () => {
  text("gustStrengthO", `${$("gustStrength").value}×`);
});

on("simSpeed", "input", () => {
  text("simSpeedO", `${Number($("simSpeed").value).toFixed(2)}×`);
});

// Toggle the constant wind disturbance during flight
on("holdWind", "click", () => {
  if (!running || !currentConfig || currentConfig.mode !== "closed") {
    showStatus(
      "Launch the drone in feedback mode before starting the constant disturbance."
    );
    return;
  }

  constantWind = !constantWind;

  if (constantWind) flightWasDisturbed = true;

  wind.value = 0;
  wind.until = 0;

  syncWindButton();

  showStatus(
    constantWind
      ? `Constant disturbance ON. P and PD have an additional downward offset of approximately ${fmt(CONSTANT_WIND_TARGET_PD_OFFSET * Number($("gustStrength").value), 2)} m after settling. Stable PI/PID with integral action should reject the constant disturbance.`
      : "Constant disturbance OFF. Watch the controller recover."
  );

  show(state, currentConfig.target);
});

// Open the PID design challenge
on("readyDesign", "click", () => {
  if ($("experiment").value !== "design") return;
  if (!designChallenge) resetForDesignQuestion(true);
  openDesignAtRequirements();
});

// Generate another challenge after completing or revealing the solution
on("anotherQuestion", "click", () => {
  if (
    !solutionRevealed &&
    !(designAnswerCorrect && designHasLaunched)
  ) {
    return;
  }

  resetForDesignQuestion(true);
  openDesignAtRequirements();
});

// Update the controller gains when the student enters new answers
for (const id of ["guideKp", "guideKi", "guideKd"]) {
  on(id, "input", () => {
    clearFlight();
    designAnswerCorrect = false;

    const valid = syncMainGainsFromGuide();

    if (valid) updateForm();

    text(
      "designAnswerFeedback",
      valid
        ? "Gains updated. Check answer for feedback, or launch a trial flight."
        : "Enter a valid non-negative number for all three gains."
    );

    updateChallengeButtons();
  });
}

// Check the calculated gains
on("checkDesignAnswer", "click", checkDesignAnswer);

// Launch the drone using the student's design gains
on("guideLaunch", "click", () => {
  if (!syncMainGainsFromGuide()) {
    text(
      "designAnswerFeedback",
      "Enter valid non-negative values for all three gains before launching."
    );
    return;
  }

  updateForm();

  if (!launch()) return;

  hide("experimentGuide");

  requestAnimationFrame(() => {
    $("chart")?.scrollIntoView({
      behavior: "smooth",
      block: "center"
    });
  });
});

// Reveal the worked solution
on("guideSolution", "click", revealSolution);
on("completeDesign", "click", revealSolution);

// Open and close the experiment manual
on("openExperimentGuide", "click", () => {
  hide("experimentGuide", false);

  if ($("experiment").value === "design" && !designChallenge) {
    resetForDesignQuestion(true);
  }

  $("closeExperimentGuide")?.focus();
});

on("closeExperimentGuide", "click", () => {
  hide("experimentGuide");
  $("openExperimentGuide")?.focus();
});

// Close the manual when clicking outside its content
on("experimentGuide", "click", (event) => {
  if (event.target === $("experimentGuide")) {
    hide("experimentGuide");
    $("openExperimentGuide")?.focus();
  }
});

// Close the solution window
on("closeSolution", "click", () => hide("solutionModal"));

on("solutionModal", "click", (event) => {
  if (event.target === $("solutionModal")) hide("solutionModal");
});

// Navigate between the design challenge stages
document.querySelectorAll("#designGuide [data-stage]").forEach((button) => {
  button.addEventListener("click", () => {
    goToChallengeStage(button.dataset.stage);
  });
});

// Update the active stage when a challenge section is opened
document.querySelectorAll("#designGuide .guide-step").forEach((step) => {
  step.addEventListener("toggle", () => {
    if (!step.open) return;

    document.querySelectorAll("#designGuide [data-stage]").forEach((button) => {
      if (button.dataset.stage === step.id) {
        button.setAttribute("aria-current", "step");
      } else {
        button.removeAttribute("aria-current");
      }
    });
  });
});

// Support keyboard navigation within the experiment manual
on("experimentGuide", "keydown", (event) => {
  if (event.key === "Escape") {
    hide("experimentGuide");
    $("openExperimentGuide")?.focus();
    return;
  }

  if (event.key !== "Tab") return;

  const items = [
    ...$("experimentGuide").querySelectorAll(
      'button:not(:disabled), input:not(:disabled), select:not(:disabled), summary, a[href]'
    )
  ].filter((element) => element.getClientRects().length);

  if (!items.length) return;

  const first = items[0];
  const last = items[items.length - 1];

  if (event.shiftKey && document.activeElement === first) {
    event.preventDefault();
    last.focus();
  } else if (!event.shiftKey && document.activeElement === last) {
    event.preventDefault();
    first.focus();
  }
});

// Move between different sections of the analysis graph
function setAnalysisStart(value) {
  analysisStart = Math.max(0, Math.min(ANALYSIS_END - 1, value));

  if ($("analysisScroll")) {
    $("analysisScroll").value = String(analysisStart);
  }

  text(
    "analysisRange",
    `${analysisStart.toFixed(1)}–${(analysisStart + 1).toFixed(1)} s`
  );

  drawAnalysis(currentConfig?.target || 10);
}

// Navigate the graph using the slider and buttons
on("analysisScroll", "input", () => {
  setAnalysisStart(Number($("analysisScroll").value));
});

on("analysisBack", "click", () => {
  setAnalysisStart(analysisStart - 0.5);
});

on("analysisNext", "click", () => {
  setAnalysisStart(analysisStart + 0.5);
});

// Scroll through the analysis graph using the mouse wheel
on(
  "analysisScroll",
  "wheel",
  (event) => {
    event.preventDefault();
    setAnalysisStart(analysisStart + (event.deltaY > 0 ? 0.2 : -0.2));
  },
  { passive: false }
);

// Initialise the simulator and experiment manual
prepareManual();
renderComparisonResults();
updateExperiment();

// Refresh the drone and graphs when the page finishes loading
function refreshVisuals() {
  initDrone();

  const target = currentConfig?.target || 10;
  show(state, target);
  draw(target);
  drawAnalysis(target);
}

if (document.readyState === "complete") {
  refreshVisuals();
} else {
  window.addEventListener("load", refreshVisuals);
}

// Update the drone display when the browser window is resized
window.addEventListener("resize", () => {
  show(state, currentConfig?.target || 10);
});
