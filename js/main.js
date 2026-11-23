import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { buildUH60Transmission, buildTailDriveShaft, tiltTransmission, TEETH } from './uh60-transmission.js';
import { buildRotorHead } from './rotor-head.js';
import { buildTail, TAIL_TEETH } from './tail.js';
import { buildCockpit } from './cockpit.js';
import { buildR44, R44_DATA } from './r44.js';
import { addT700Internals, setHousingCutaway } from './engine-t700.js';

// ---------- HUD ----------
const viewPanel = document.querySelector('.views');
const compactHud = matchMedia('(max-width: 850px)');
if (compactHud.matches) viewPanel.open = false;
compactHud.addEventListener('change', () => { viewPanel.open = !compactHud.matches; });
const topbar = document.querySelector('.topbar');
const bottomHud = document.querySelector('.bottom-hud');
const viewHeading = document.querySelector('.view-heading');
const flightHud = document.getElementById('ctrl');
const hudInsets = new ResizeObserver(() => {
  document.body.style.setProperty('--heading-top', topbar.offsetHeight + 12 + 'px');
  document.body.style.setProperty('--footer-space', bottomHud.offsetHeight + 24 + 'px');
  const flightTop = flightHud.getBoundingClientRect().top;
  const headingBottom = Math.ceil(viewHeading.getBoundingClientRect().bottom);
  const pipTop = Math.max(topbar.offsetHeight + 10, Math.min(headingBottom + 10, flightTop - 130));
  document.body.style.setProperty('--pip-top', pipTop + 'px');
  document.body.style.setProperty('--pip-height', Math.floor(flightTop - pipTop - 74) + 'px');
});
hudInsets.observe(topbar);
hudInsets.observe(bottomHud);
hudInsets.observe(viewHeading);
hudInsets.observe(flightHud);
// The HUD can move on resize even if its elements keep the same dimensions.
hudInsets.observe(document.getElementById('app'));

const fullscreenButton = document.getElementById('fullscreen');
if (document.fullscreenEnabled) {
  fullscreenButton.hidden = false;
  fullscreenButton.onclick = async () => {
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen();
    } catch (error) {
      console.warn('Fullscreen unavailable', error);
    }
  };
  document.addEventListener('fullscreenchange', () => {
    const fullscreen = Boolean(document.fullscreenElement);
    fullscreenButton.textContent = fullscreen ? 'Exit fullscreen' : 'Enter fullscreen';
    fullscreenButton.setAttribute('aria-pressed', String(fullscreen));
  });
}

// ---------- renderer / scene ----------
const app = document.getElementById('app');
const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(app.clientWidth, app.clientHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
app.appendChild(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.setSize(app.clientWidth, app.clientHeight);
labelRenderer.domElement.className = 'labels';
app.appendChild(labelRenderer.domElement);

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.9;

const camera = new THREE.PerspectiveCamera(35, app.clientWidth / app.clientHeight, 0.05, 200);
camera.position.set(7.5, 6.5, 12);
// the label layer ignores the mouse (pointer-events: none), so input goes to the 3D canvas
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(-2.5, 1.2, 0);

const key = new THREE.DirectionalLight(0xffffff, 2.4);
key.position.set(6, 14, 9);
key.castShadow = true;
key.shadow.mapSize.set(4096, 4096);
Object.assign(key.shadow.camera, { left: -20, right: 20, top: 20, bottom: -20, near: 1, far: 60 });
key.shadow.bias = -0.0004;
key.shadow.normalBias = 0.02;
key.target.position.set(-3, 0, 0);
scene.add(key, key.target);
const rim = new THREE.DirectionalLight(0xbcd4ff, 1.1);
rim.position.set(-10, 6, -8);
scene.add(rim);
scene.add(new THREE.HemisphereLight(0xdfe9ff, 0x1a2333, 0.5));

// ---------- model ----------
// `air` = airframe axes (level). The main transmission group (gears, engines, mast, rotor head,
// swashplate, servos) sits inside it with the real 3° forward tilt; cockpit, mixing unit and
// tail are airframe-mounted.
const air = new THREE.Group();
air.name = 'UH-60 airframe';
scene.add(air);
const tx = buildUH60Transmission();
air.add(tx.root);
const rh = buildRotorHead(tx.root);
Object.assign(tx.anchors, rh.anchors);
tiltTransmission(tx, 3);
// T700 cutaway internals in both engines; the housings are cut along a plane through both engine
// axes (plane parallel to the transmission deck), keeping the lower half
renderer.localClippingEnabled = true;
const t700 = tx.debug.engines.map((e) => ({ e, sideKey: e.side, int: addT700Internals(e.group, e.outer, e.side === 'eng2' ? 1 : -1) }));
const cutPlane = (() => {
  tx.root.updateMatrixWorld(true);
  const up = new THREE.Vector3(0, 1, 0).applyQuaternion(tx.root.getWorldQuaternion(new THREE.Quaternion()));
  return new THREE.Plane().setFromNormalAndCoplanarPoint(up.negate(), t700[0].e.group.getWorldPosition(new THREE.Vector3()));
})();
function setCutaway(on) {
  for (const t of t700) { setHousingCutaway(t.e.group, t.int.internalMeshes, cutPlane, on); t.int.state.cutaway = on; }
}
{ // engine part labels (engine-local positions -> transmission frame), on the right (No.2) engine
  const eg = t700.find((t) => t.sideKey === 'eng2').e.group;
  const L = (y, z) => eg.localToWorld(new THREE.Vector3(0, y, z)).applyMatrix4(tx.root.matrixWorld.clone().invert());
  Object.assign(tx.anchors, {
    e_ips: L(1.05, -0.4), e_comp: L(0.95, -2.05), e_cent: L(1.1, -2.65), e_comb: L(1.35, -3.2),
    e_ggt: L(0.65, -3.78), e_pt: L(1.0, -4.25), e_shaft: L(0.75, -1.0), e_exh: L(0.45, -5.4), e_agb: L(1.35, -0.95),
  });
}
const td = buildTailDriveShaft(air, tx);
const tail = buildTail(air, tx, td);
const cp = buildCockpit(air, { rh, tail });
const airAnchors = { ...tail.anchors, ...cp.anchors };
air.updateMatrixWorld(true);
// second aircraft: Robinson R44 (2-blade teetering rotor); shares the pilot-controls state
const r44Root = new THREE.Group();
r44Root.name = 'R44';
r44Root.visible = false;
scene.add(r44Root);
const r44 = buildR44(r44Root, rh.state.controls);
let active = 'uh60';
const activeState = () => (active === 'uh60' ? rh.state : r44.state);
const W = (v) => air.localToWorld(v.clone());

// ---------- post-processing (ambient occlusion) ----------
const composer = new EffectComposer(renderer, new THREE.WebGLRenderTarget(1, 1, { samples: 4, type: THREE.HalfFloatType }));
composer.addPass(new RenderPass(scene, camera));
let gtao = null;
try {
  gtao = new GTAOPass(scene, camera, app.clientWidth, app.clientHeight);
  gtao.updateGtaoMaterial({ radius: 0.35, distanceExponent: 1, thickness: 1, scale: 1 });
  gtao.blendIntensity = 0.85;
  composer.addPass(gtao);
} catch (e) { console.warn('GTAO unavailable', e); }
composer.addPass(new OutputPass());
composer.setPixelRatio(renderer.getPixelRatio());
composer.setSize(app.clientWidth, app.clientHeight);

// ---------- labels ----------
const LABELS = [
  ['engL', 'No.1 engine — T700 turboshaft'],
  ['engR', 'No.2 engine — T700 turboshaft'],
  ['fwL', 'Freewheel unit (sprag clutch)'],
  ['fwR', 'Freewheel unit (sprag clutch)'],
  ['inL', `Input module — bevel ${TEETH.inputPinion}:${TEETH.inputGear}`],
  ['inR', `Input module — bevel ${TEETH.inputPinion}:${TEETH.inputGear}`],
  ['accL', 'Accessory module (generator + hydraulic pump)'],
  ['accR', 'Accessory module (generator + hydraulic pump)'],
  ['mpL', `Main bevel pinion (${TEETH.mainPinion}T)`],
  ['mpR', `Main bevel pinion (${TEETH.mainPinion}T)`],
  ['mainGear', `Main bevel gear (${TEETH.mainGear}T)`],
  ['planetary', `Planetary: sun ${TEETH.sun}T · ${TEETH.planets} planets ${TEETH.planet}T · fixed ring ${TEETH.ring}T`],
  ['mast', 'Main rotor mast — 258 rpm'],
  ['tailPinion', 'Tail rotor takeoff'],
  ['blower', 'Oil cooler blower'],
  ['tail', 'Tail drive shaft (6 sections, disc couplings)'],
  ['hub', 'One-piece titanium hub'],
  ['elasto', 'Elastomeric bearing (flap · lead-lag · pitch)'],
  ['damper', 'Lead-lag damper (hydraulic)'],
  ['bifilar', 'Bifilar vibration absorber'],
  ['pitchLink', 'Pitch control rod → pitch horn'],
  ['rotSwash', 'Rotating swashplate'],
  ['statSwash', 'Stationary swashplate'],
  ['servo', 'Primary servo (1 of 3)'],
  ['blade', 'Main rotor blade (swept tip)'],
  ['igb', `Intermediate gearbox — ~58° turn, ${TAIL_TEETH.igbIn}:${TAIL_TEETH.igbOut}`],
  ['pylonShaft', 'Pylon drive shaft'],
  ['tgb', `Tail gearbox — ~105° turn, ${TAIL_TEETH.tgbIn}:${TAIL_TEETH.tgbOut}`],
  ['trotor', 'Tail rotor — 4 blades, bearingless cross-beams, canted 20°'],
  ['pcb', 'Pitch change beam'],
  ['trservo', 'Tail rotor servo (on tail gearbox)'],
  ['stab', 'Stabilator — 39° TED … 9° TEU'],
  ['cyclic', 'Cyclic stick (drag me)'],
  ['collective', 'Collective lever (drag me)'],
  ['pedals', 'Pedals (drag a pedal)'],
  ['mixing', 'Mixing unit'],
  ['column', 'Push-pull rods + bellcranks'],
  ['cables', 'Tail rotor control cables'],
  ['assist', 'Pilot-assist servos (boost + SAS)'],
  ['copilot', 'Copilot cyclic (interconnected)'],
  ['e_ips', 'Inlet particle separator (swirl vanes → dirt to scroll)'],
  ['e_comp', 'Compressor: 5 axial stages'],
  ['e_cent', '+ 1 centrifugal stage'],
  ['e_comb', 'Annular combustor (fuel injectors, flame)'],
  ['e_ggt', 'Gas-generator turbine (2 stages) — drives compressor'],
  ['e_pt', 'Free power turbine (2 stages)'],
  ['e_shaft', 'Power-turbine shaft runs forward to the output'],
  ['e_exh', 'Exhaust'],
  ['e_agb', 'Accessory gearbox (radial drive shaft)'],
  ['r44engine', 'Lycoming IO-540 — 6 cylinders, horizontally opposed'],
  ...[1, 2, 3, 4, 5, 6].map((n) => [`r44e_c${n}`, `Cylinder ${n}`]),
  ['r44e_crank', 'Crankshaft — one throw per cylinder'],
  ['r44e_piston', 'Piston + connecting rod'],
  ['r44e_valves', 'Valves: intake (blue tag) · exhaust (red tag)'],
  ['r44e_plugs', '2 spark plugs per cylinder (dual magnetos)'],
  ['r44belts', 'Lower + upper sheaves, 4 double vee-belts'],
  ['r44clutch', 'Clutch actuator (tensions belts) · sprag clutch in upper sheave'],
  ['r44gearbox', 'Main gearbox — single spiral bevel 11:57'],
  ['r44hub', 'Teetering hub · coning hinges'],
  ['r44swash', 'Swashplate'],
  ['r44servo', 'Hydraulic servo (1 of 3) on push-pull tube'],
  ['r44tail', 'Tail gearbox (90°) · 2-blade teetering tail rotor'],
  ['r44shaft', 'Tail drive shaft (no support bearings)'],
  ['r44cyclic', 'Centre cyclic (T-bar) — drag me'],
  ['r44collective', 'Collective + twist-grip throttle — drag me'],
  ['r44pedals', 'Pedals — drag a pedal'],
];
const labelObjs = [];
const ROTOR_KEYS = new Set(['hub', 'elasto', 'damper', 'bifilar', 'pitchLink', 'rotSwash', 'statSwash', 'servo', 'blade']);
const TAIL_KEYS = new Set(['igb', 'pylonShaft', 'tgb', 'trotor', 'pcb', 'trservo', 'stab']);
const COCKPIT_KEYS = new Set(['cyclic', 'collective', 'pedals', 'mixing', 'column', 'cables', 'assist', 'copilot']);
let labelGroup = 'drive';
const labelsToggle = document.getElementById('lbls');
let labelsOn = labelsToggle.checked;
function applyLabels() {
  const groupOf = (k) => (k.startsWith('e_') ? 'engine' : k.startsWith('r44e_') ? 'r44eng' : k.startsWith('r44') ? 'r44' : ROTOR_KEYS.has(k) ? 'rotor' : TAIL_KEYS.has(k) ? 'tail' : COCKPIT_KEYS.has(k) ? 'cockpit' : 'drive');
  labelObjs.forEach((o) => (o.visible = labelsOn && labelGroup !== 'none' && groupOf(o.userData.key) === labelGroup));
}
for (const [key, text] of LABELS) {
  const el = document.createElement('div');
  el.className = 'lbl';
  el.innerHTML = `<span class="dot"></span><span class="txt">${text}</span>`;
  const o = new CSS2DObject(el);
  const isR44 = key.startsWith('r44');
  const inAir = key in airAnchors;                       // airframe parts vs. tilted transmission parts
  o.position.copy(isR44 ? r44.anchors[key] : inAir ? airAnchors[key] : tx.anchors[key]);
  o.userData.key = key;
  (isR44 ? r44Root : inAir ? air : tx.root).add(o);
  labelObjs.push(o);
}

// ---------- camera shots ----------
const SHOTS = [
  {
    id: 'overview', name: 'Overview', title: 'Helicopter Drive System', sub: 'UH-60 Black Hawk — power path from engines to rotors',
    cam: [4.5, 8.2, 15.5], tgt: [-4.2, 0.9, 0],
    text: 'Two T700 turboshaft engines drive the main transmission. Speed is reduced in three stages — input bevel (22:80), main bevel (17:81) and planetary (sun 62 / ring 228) — from about 20,900 rpm to 258 rpm at the rotor mast, a total of about 81:1. The whole transmission is tilted 3° forward in the airframe; the tail rotor drive shaft is taken off the main bevel gear and its first disc coupling takes up that 3° angle.',
  },
  {
    id: 'engine', name: 'Engine', title: 'T700 Turboshaft — Cutaway', sub: 'How the engine makes shaft power', cutaway: true, labels: 'engine', controls: true,
    cam: [-2.2, 5.2, 9.0], tgt: [-6.3, 0.35, 4.05],
    text: 'Air (blue) enters at the front; the inlet particle separator spins it so dirt (brown) is thrown out to a scroll and blown overboard. 5 axial stages + 1 centrifugal stage compress the air; fuel burns in the annular combustor (orange). The hot gas spins the 2-stage gas-generator turbine, which drives the compressor on the same hollow shaft — then the separate 2-stage power turbine, whose shaft runs forward through the middle of the engine to the output at the front (≈20,900 rpm, governed at 100 %). The two spools are not connected: raise the collective and the gas generator (Ng) speeds up for more power while the power turbine stays at 100 %. Try "Stop No.2 engine".',
  },
  {
    id: 'freewheel', name: 'Freewheel', title: 'Freewheel Unit', sub: 'Sprag clutch between each engine and its input module',
    cam: [-1.7, 1.25, 6.3], tgt: [-2.85, 0.45, 4.0],
    text: 'Sprags (orange) wedge between the outer race (engine side, transparent) and the inner race (gearbox side) when the engine drives. If the engine slows or stops, the sprags tip back and the gearbox overruns freely — so one engine failure, or an autorotation, never drags the rotor. Try the "Stop No.2 engine" button.',
  },
  {
    id: 'input', name: 'Input module', title: 'Input Module', sub: `First reduction — bevel ${TEETH.inputPinion}:${TEETH.inputGear} (3.64 : 1)`,
    cam: [-3.4, 3.0, 1.6], tgt: [-1.2, 0.4, 4.4],
    text: 'The 22-tooth input bevel pinion drives the 80-tooth input bevel gear, turning the drive 90° toward the main module and reducing ~20,900 rpm to ~5,750 rpm. The accessory module (generator + hydraulic pump) is driven from the same bevel gear, after the freewheel, so it keeps running from the rotor if the engine stops.',
  },
  {
    id: 'bevel', name: 'Main bevel', title: 'Main Bevel Stage', sub: `Both inputs drive one bevel gear — ${TEETH.mainPinion}:${TEETH.mainGear} (4.76 : 1)`,
    cam: [4.4, 3.0, 4.6], tgt: [0, 0.3, 1.1],
    text: 'The left and right 17-tooth spiral bevel pinions both mesh with the single 81-tooth main bevel gear, combining the power of both engines and turning it to a vertical shaft. Speed drops from ~5,750 rpm to ~1,206 rpm, which drives the sun gear of the planetary stage.',
  },
  {
    id: 'planetary', name: 'Planetary', title: 'Planetary Gear', sub: `Final reduction — 1 + ${TEETH.ring}/${TEETH.sun} = 4.68 : 1`,
    cam: [2.4, 5.4, 3.4], tgt: [0, 1.55, 0],
    text: 'The sun gear (62 T) drives five planet gears (83 T) that roll around the inside of the fixed ring gear (228 T). The planet carrier — the 5-arm plate — turns the mast. Carrier speed = sun ÷ (1 + 228/62): ~1,206 rpm → 258 rpm. The mast turns counter-clockwise seen from above.',
  },
  {
    id: 'tail', name: 'Tail takeoff', title: 'Tail Rotor Drive', sub: 'Taken off the main bevel gear',
    cam: [-8.2, 2.4, -4.2], tgt: [-3.4, 0.5, 0],
    text: 'A pinion on the main bevel gear drives the tail drive shaft aft at ~4,100 rpm. Six aluminium shaft sections joined by flexible disc couplings (no universal joints) carry it to the intermediate and tail gearboxes. The shaft also drives the oil-cooler blower. The gust-lock teeth are on this takeoff flange.',
  },
  {
    id: 'rotor', name: 'Rotor head', title: 'Main Rotor Head', sub: '4 blades · elastomeric bearings · no oil, no hinges', discGuide: true, fast: true,
    cam: [5.6, 10.6, 6.2], tgt: [0, 7.5, 0],
    text: 'Each blade attaches to a spindle held by an elastomeric bearing (laminated rubber and steel) in the one-piece titanium hub. The bearing lets the blade flap up and down, lead and lag, and twist in pitch. Hydraulic dampers control lead-lag; the bifilar absorber on top (4 arms with pendulum weights) cancels vibration. A pitch control rod from the swashplate twists each blade through its pitch horn.',
  },
  {
    id: 'swash', name: 'Swashplate', title: 'Swashplate', sub: 'Turns pilot inputs into blade pitch', discGuide: true, fast: true,
    cam: [5.4, 7.6, 4.8], tgt: [0, 6.0, 0],
    text: 'Three primary servos push the stationary swashplate (held still by the stationary scissors). Through a bearing it carries the rotating swashplate, which turns with the rotor (driven by the rotating scissors). Collective moves the whole swashplate up or down — all blades change pitch together. Cyclic tilts it — each blade\'s pitch rises and falls once per turn. Try moving the on-screen cyclic or collective.',
  },
  {
    id: 'disc', name: 'Rotor disc', title: 'Cyclic & Rotor Disc', sub: 'The disc tilts where the stick points', discGuide: true, fast: true, labels: 'none',
    cam: [24, 15, 22], tgt: [0, 6.5, 0],
    text: 'The orange ring is the tip-path plane — grab it with the mouse and tilt it: the cyclic stick, rods, servos and swashplate all move to match. Push the cyclic and the disc tilts the same way. The blades reach their lowest point about 90° after their lowest pitch, so the controls give each blade its minimum pitch 90° before the stick direction. The mixing unit adds small automatic corrections when collective or pedals move.',
  },
  {
    id: 'cockpit', name: 'Pilot controls', title: 'Pilot Controls', sub: 'Grab the cyclic, collective or pedals with the mouse', labels: 'cockpit', fast: true, controls: true, discGuide: true,
    cam: [10.6, -2.5, 4.6], tgt: [6.9, -3.95, 0.75],
    text: 'Drag the cyclic stick (tilts the rotor disc), the collective lever (all blades together) or a pedal (tail rotor pitch) — pilot\'s or copilot\'s: they are interconnected (cross rods under the floor, one collective torque tube). Each control drives push-pull rods under the floor; bellcranks turn the motion up the control column through the pilot-assist servos to the mixing unit.',
  },
  {
    id: 'controlpath', name: 'Control path', title: 'Control Path', sub: 'Cockpit → rods → mixing unit → servos → swashplate', labels: 'cockpit', fast: true, controls: true, discGuide: true,
    cam: [19.5, 3.0, 17.5], tgt: [3.6, -1.1, 0.3],
    text: 'Every rod keeps its length; each bellcrank and lever angle is solved from the rod that drives it. Follow a control from the cockpit, under the floor, up the column through the pilot-assist servos (boost: collective, pitch, yaw · SAS: pitch, roll, yaw) to the mixing unit, then to the pilot-valve lever on each primary servo — the servos move the swashplate, which pitches the blades. Pedal inputs leave the mixing unit as two cables that run aft to the tail rotor servo.',
  },
  // ------------------------------- Robinson R44 -------------------------------
  {
    id: 'r44overview', ac: 'r44', name: 'Overview', title: 'Robinson R44 — Drive System', sub: '2-blade teetering rotor · piston engine · belt drive', labels: 'r44', fast: true, controls: true,
    cam: [10.5, 6.5, 13.5], tgt: [-2.5, 2.2, 0],
    text: `A Lycoming IO-540 piston engine turns the lower sheave; four double vee-belts drive the upper sheave (ratio ${R44_DATA.SHEAVE_RATIO}, ≈2,665 → ≈2,073 rpm). An overrunning (sprag) clutch in the upper sheave hub drives the clutch shaft: forward to the single-stage spiral-bevel main gearbox (11:57 → 400 rpm), aft along the tail drive shaft to the 90° tail gearbox and the 2-blade tail rotor.`,
  },
  {
    id: 'r44engine', ac: 'r44', name: 'Engine', title: 'Lycoming IO-540 — Cutaway', sub: '4-stroke piston engine · firing order 1-4-5-2-3-6', labels: 'r44eng', fast: true, controls: true,
    cam: [-0.6, 3.3, 2.7], tgt: [-2.05, -0.15, 0],
    text: 'Each cylinder works in four strokes over two crank turns: INTAKE (blue) — the piston moves in and the intake valve lets the fuel-air mix in; COMPRESSION — both valves shut, the mix is squeezed; POWER — the two spark plugs fire just before top dead centre and the burning gas (orange) pushes the piston out, turning the crankshaft; EXHAUST — the exhaust valve opens and the piston pushes the burnt gas out. Opposed cylinders (1-2, 3-4, 5-6) reach the top together, one on compression and one on exhaust, and a cylinder fires every 120° of crank in the order 1-4-5-2-3-6. The crankshaft drives the lower belt sheave and the cooling fan.',
  },
  {
    id: 'r44belts', ac: 'r44', name: 'Belts + clutch', title: 'Belt Drive & Clutch', sub: 'Sheaves, vee-belts, clutch actuator, sprag clutch', labels: 'r44', fast: true, controls: true,
    cam: [2.9, 2.6, 4.4], tgt: [-0.1, 1.0, 0],
    text: 'After start, the clutch actuator between the sheaves raises the upper sheave until the vee-belts are tensioned, then switches off. The orange marks show the belts moving. The overrunning (sprag) clutch in the upper sheave hub lets the rotor keep turning if the engine stops (autorotation). The direct-drive cooling fan sits on the engine output shaft behind the lower sheave.',
  },
  {
    id: 'r44gearbox', ac: 'r44', name: 'Main gearbox', title: 'Main Gearbox', sub: 'Single-stage spiral bevel — 11 : 57 (5.18 : 1)', labels: 'r44', fast: true, controls: true,
    cam: [4.6, 3.6, 3.0], tgt: [2.6, 2.1, 0],
    text: 'The clutch shaft enters through a flexible coupling and drives the 11-tooth spiral bevel pinion, which turns the 57-tooth gear on the mast: ≈2,073 rpm → 400 rpm. The gearbox sits on four rubber mounts on the welded steel-tube frame and also drives the hydraulic pump for the three servos.',
  },
  {
    id: 'r44rotor', ac: 'r44', name: 'Rotor head', title: 'Teetering Rotor Head', sub: 'Teeter hinge + coning hinges · pitch bearings in the blade root housings', labels: 'r44', fast: true, controls: true,
    cam: [5.4, 7.0, 3.6], tgt: [2.6, 5.1, 0],
    text: 'The hub is mounted to the mast by a teeter hinge, so the two blades see-saw as one piece: when one rises the other falls. Each blade is mounted to the hub by a coning hinge (both blades cone up together with lift). Pitch-change bearings sit in an oil-filled housing at each blade root, twisted by a pitch link from the swashplate. Three servos on the push-pull tubes support the swashplate.',
  },
  {
    id: 'r44disc', ac: 'r44', name: 'Rotor disc', title: 'Cyclic on a Teetering Rotor', sub: 'The whole hub teeters so the disc tilts where the stick points', labels: 'none', fast: true, controls: true,
    cam: [17, 10.5, 15], tgt: [2.6, 4.4, 0],
    text: 'Push the cyclic: the swashplate tilts, each blade\'s pitch changes once per turn, and the disc tilts toward the stick. On a teetering rotor this is exactly 90° of phase: the hub see-saws on its teeter hinge instead of each blade flapping on its own hinge.',
  },
  {
    id: 'r44tail', ac: 'r44', name: 'Tail rotor', title: 'R44 Tail Rotor', sub: '2 blades · teetering hub · fixed coning angle · left side', labels: 'r44', fast: true, controls: true,
    cam: [-10.6, 3.0, -4.6], tgt: [-13.3, 2.0, -0.45],
    text: 'The long tail drive shaft (no support bearings, one damper bearing) drives a single 90° spiral-bevel tail gearbox. The 2-blade tail rotor has a teetering hub with a fixed coning angle. Pedals act through push-pull tubes directly on the pitch control, sliding a plate along the output shaft that twists both blades.',
  },
  {
    id: 'r44controls', ac: 'r44', name: 'Controls', title: 'R44 Controls', sub: 'Centre T-bar cyclic · collective with correlated throttle', labels: 'r44', fast: true, controls: true,
    cam: [8.4, 0.9, 3.6], tgt: [5.2, -0.45, 0.15],
    text: 'Drag the T-bar cyclic, the collective or a pedal. Raising the collective also opens the throttle through a mechanical correlator (watch the twist grip turn); the governor trims it. All controls work through push-pull tubes and bellcranks; the three main rotor servos are irreversible, so rotor forces do not feed back into the controls.',
  },
  {
    id: 'tailgear', name: 'Tail gearboxes', title: 'Tail Gearboxes', sub: 'Intermediate gearbox → pylon shaft → tail gearbox', labels: 'tail', fast: true, controls: true,
    cam: [-9.5, 6.0, 14.5], tgt: [-21.0, 3.8, 0.4],
    text: 'At the base of the tail pylon the intermediate gearbox turns the drive about 58° up the pylon and reduces speed slightly (~4,100 → ~3,300 rpm). At the top, the tail gearbox turns it again (~105°) toward the tail rotor, reduces it to ~1,190 rpm, carries the tail rotor and mounts the tail rotor servo. Tooth counts shown are chosen to match these published ratios.',
  },
  {
    id: 'tailrotor', name: 'Tail rotor', title: 'Tail Rotor', sub: 'Bearingless cross-beam — canted 20° up on the right side', labels: 'tail', fast: true, controls: true,
    cam: [-14.0, 10.0, 13.5], tgt: [-22.6, 6.4, 1.4],
    text: 'Two continuous graphite spars, each running tip to tip, form the four blades. There are no bearings: the spar itself twists (pitch) and bends (flap) near the hub. Pedals move the tail rotor servo, which pushes a rod through the hollow shaft to the pitch change beam — its links twist all four blades together. Canted 20° upward, the tail rotor also gives about 2.5 % of the lift in a hover. Try the Pedals slider.',
  },
  {
    id: 'stab', name: 'Stabilator', title: 'Stabilator', sub: 'Moving horizontal stabilizer — scheduled by airspeed', labels: 'tail', fast: true, controls: true, discGuide: true, downwash: true,
    cam: [-6.0, 4.6, 17.0], tgt: [-17.5, 3.0, 0],
    text: 'Two electric actuators move the whole stabilator from about 39° trailing-edge-down to 9° trailing-edge-up. In a hover and at low speed it hangs steeply down, lined up with the rotor downwash, so the downwash does not push the tail down and pitch the nose up. As airspeed rises the wake sweeps aft and the stabilator is programmed toward level for stability. Use the Airspeed slider. (Collective, pitch-rate and sideslip inputs also feed it — not modelled here.)',
  },
];

const shotBar = document.getElementById('shots');
let anim = null;
// ---------- free-fly camera ----------
let freeCam = false;
const keys = {};
const fly = { yaw: 0, pitch: 0, speed: 4, drag: false, lx: 0, ly: 0 };
const freeBtn = document.getElementById('free');
const freeHint = document.getElementById('freeHint');
const orbitHint = document.getElementById('orbitHint');
const cameraButtons = [...document.querySelectorAll('[data-camera]')];
const cameraSteps = {
  left: [-0.2, 0, 1], right: [0.2, 0, 1], up: [0, -0.2, 1], down: [0, 0.2, 1],
  in: [0, 0, 0.8], out: [0, 0, 1.25],
};
for (const button of cameraButtons) {
  button.onclick = () => {
    anim = null;
    const [azimuth, elevation, zoom] = cameraSteps[button.dataset.camera];
    const offset = camera.position.clone().sub(controls.target);
    const orbit = new THREE.Spherical().setFromVector3(offset);
    orbit.theta += azimuth;
    orbit.phi += elevation;
    orbit.makeSafe();
    orbit.radius = THREE.MathUtils.clamp(orbit.radius * zoom, 0.25, camera.far * 0.8);
    camera.position.copy(controls.target).add(offset.setFromSpherical(orbit));
    controls.update();
  };
}
document.getElementById('camera-reset').onclick = () => {
  const shotId = shotBar.querySelector('.on').dataset.id;
  goShot(SHOTS.find(({ id }) => id === shotId));
};
function setFree(on) {
  freeCam = on;
  controls.enabled = !on;
  anim = null;
  if (on) {
    const e = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ');
    fly.yaw = e.y; fly.pitch = e.x;
  } else {
    const f = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
    controls.target.copy(camera.position).addScaledVector(f, 6);
  }
  freeBtn.classList.toggle('on', on);
  freeBtn.setAttribute('aria-pressed', String(on));
  freeHint.hidden = !on;
  orbitHint.hidden = on;
  cameraButtons.forEach((button) => { button.disabled = on; });
}
freeBtn.onclick = () => setFree(!freeCam);
addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.code === 'KeyF' && !e.repeat) setFree(!freeCam);
  keys[e.code] = true;
});
addEventListener('keyup', (e) => (keys[e.code] = false));
addEventListener('blur', () => Object.keys(keys).forEach((k) => (keys[k] = false)));
const evEl = renderer.domElement;
// free camera mouse: any button drag = look · hold RIGHT = fly forward · hold MIDDLE = fly back
evEl.addEventListener('pointerdown', (e) => {
  if (!freeCam) return;
  e.preventDefault();
  fly.drag = true; fly.lx = e.clientX; fly.ly = e.clientY;
  if (e.button === 2) fly.fwd = 1;
  if (e.button === 1) fly.fwd = -1;
  try { evEl.setPointerCapture(e.pointerId); } catch { /* pointer already gone */ }
});
evEl.addEventListener('pointermove', (e) => {
  if (!freeCam || !fly.drag) return;
  fly.yaw -= (e.clientX - fly.lx) * 0.004;
  fly.pitch = THREE.MathUtils.clamp(fly.pitch - (e.clientY - fly.ly) * 0.004, -1.5, 1.5);
  fly.lx = e.clientX; fly.ly = e.clientY;
});
const endDrag = () => { fly.drag = false; fly.fwd = 0; };
evEl.addEventListener('pointerup', endDrag);
evEl.addEventListener('pointercancel', endDrag);
evEl.addEventListener('contextmenu', (e) => { if (freeCam) e.preventDefault(); });
evEl.addEventListener('wheel', (e) => {
  if (!freeCam) return;
  fly.speed = THREE.MathUtils.clamp(fly.speed * (e.deltaY > 0 ? 0.85 : 1.18), 0.3, 40);
  freeHint.querySelector('b').textContent = fly.speed.toFixed(1);
}, { passive: true });
// game controller (standard mapping, e.g. Xbox / PlayStation):
//   left stick = move · right stick = look · RT / LT = up / down · RB / LB = faster / slower
//   A held = boost · Y or Start = toggle free camera (moving a stick also switches it on)
const gpad = { prev: [], move: new THREE.Vector3(), boost: false };
const dz = (v) => (Math.abs(v) < 0.15 ? 0 : (v - Math.sign(v) * 0.15) / 0.85);
function pollGamepad(dt) {
  const gp = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find((g) => g && g.connected);
  gpad.move.set(0, 0, 0);
  if (!gp) return;
  const btn = (i) => gp.buttons[i] && (gp.buttons[i].pressed || gp.buttons[i].value > 0.5);
  const val = (i) => (gp.buttons[i] ? gp.buttons[i].value : 0);
  const edge = (i) => btn(i) && !gpad.prev[i];
  if (edge(3) || edge(9)) setFree(!freeCam);
  const lx = dz(gp.axes[0] || 0), ly = dz(gp.axes[1] || 0), rx = dz(gp.axes[2] || 0), ry = dz(gp.axes[3] || 0);
  if (!freeCam && (lx || ly || rx || ry)) setFree(true);
  if (freeCam) {
    fly.yaw -= rx * 2.2 * dt;
    fly.pitch = THREE.MathUtils.clamp(fly.pitch - ry * 1.8 * dt, -1.5, 1.5);
    gpad.move.set(lx, val(7) - val(6), -ly);              // x strafe, y up/down, z forward
    if (edge(5) || edge(4)) {
      fly.speed = THREE.MathUtils.clamp(fly.speed * (btn(5) ? 1.4 : 1 / 1.4), 0.3, 40);
      freeHint.querySelector('b').textContent = fly.speed.toFixed(1);
    }
    gpad.boost = btn(0);
  }
  gpad.prev = gp.buttons.map((b) => b.pressed);
}
addEventListener('gamepadconnected', (e) => console.info('Controller connected:', e.gamepad.id));

function updateFly(dt) {
  camera.quaternion.setFromEuler(new THREE.Euler(fly.pitch, fly.yaw, 0, 'YXZ'));
  if (gpad.move.lengthSq()) {
    const f = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
    const r = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion);
    const v = r.multiplyScalar(gpad.move.x).addScaledVector(f, gpad.move.z).add(new THREE.Vector3(0, gpad.move.y, 0));
    camera.position.addScaledVector(v, fly.speed * (gpad.boost ? 3 : 1) * dt);
  }
  const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
  const right = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion);
  const mv = new THREE.Vector3();
  if (keys.KeyW || keys.ArrowUp || fly.fwd > 0) mv.add(fwd);
  if (fly.fwd < 0) mv.sub(fwd);
  if (keys.KeyS || keys.ArrowDown) mv.sub(fwd);
  if (keys.KeyD || keys.ArrowRight) mv.add(right);
  if (keys.KeyA || keys.ArrowLeft) mv.sub(right);
  if (keys.KeyE || keys.Space) mv.y += 1;
  if (keys.KeyQ || keys.KeyC) mv.y -= 1;
  if (mv.lengthSq()) camera.position.addScaledVector(mv.normalize(), fly.speed * (keys.ShiftLeft || keys.ShiftRight ? 3 : 1) * dt);
}

function setAircraft(ac) {
  if (ac === active && air.visible === (ac === 'uh60')) return;
  active = ac;
  air.visible = ac === 'uh60';
  r44Root.visible = ac === 'r44';
  document.querySelectorAll('#acSel button').forEach((button) => {
    const selected = button.dataset.ac === ac;
    button.classList.toggle('on', selected);
    button.setAttribute('aria-pressed', String(selected));
  });
  document.querySelectorAll('#shots button').forEach((b) => (b.style.display = b.dataset.ac === ac ? '' : 'none'));
  document.querySelectorAll('.uh60only').forEach((el) => (el.style.display = ac === 'uh60' ? '' : 'none'));
}
function goShot(s, instant = false) {
  if (freeCam) setFree(false);
  const aircraft = s.ac || 'uh60';
  if (aircraft !== active) { stopDemo(); closePip(); }
  setAircraft(aircraft);
  document.querySelectorAll('#shots button').forEach((button) => {
    const selected = button.dataset.id === s.id;
    button.classList.toggle('on', selected);
    button.setAttribute('aria-pressed', String(selected));
  });
  document.getElementById('title').textContent = s.title;
  document.getElementById('subtitle').textContent = s.sub;
  document.getElementById('caption').textContent = s.text;
  rh.setTpp(!!s.discGuide);
  setCutaway(!!s.cutaway);
  labelGroup = s.labels ?? (s.discGuide ? 'rotor' : 'drive');
  applyLabels();
  tail.state.showDownwash = !!s.downwash;
  if (s.fast && slowIn.value < 3) { slowIn.value = 3; updSlow(); }
  if (!s.fast && slowIn.value > 0) { slowIn.value = 0; updSlow(); }
  const toPos = W(new THREE.Vector3(...s.cam));
  const toTgt = W(new THREE.Vector3(...s.tgt));
  if (instant) { camera.position.copy(toPos); controls.target.copy(toTgt); return; }
  anim = { t: 0, dur: 1.8, p0: camera.position.clone(), t0: controls.target.clone(), p1: toPos, t1: toTgt };
}
for (const s of SHOTS) {
  const b = document.createElement('button');
  b.textContent = s.name;
  b.dataset.id = s.id;
  b.dataset.ac = s.ac || 'uh60';
  b.onclick = () => {
    goShot(s);
    if (compactHud.matches) {
      viewPanel.open = false;
      viewPanel.querySelector("summary").focus();
    }
  };
  shotBar.appendChild(b);
}
document.querySelectorAll('#acSel button').forEach((b) => {
  b.onclick = () => goShot(SHOTS.find((s) => (s.ac || 'uh60') === b.dataset.ac));
});
document.querySelectorAll('#shots button').forEach((b) => (b.style.display = b.dataset.ac === 'uh60' ? '' : 'none'));

// ---------- controls ----------
let playing = true;
let slow = 1;
const BASE_REV_PER_S = 0.012; // mast revolutions per second at 1x (others scale by true ratios)
const playBtn = document.getElementById('play');
playBtn.onclick = () => { playing = !playing; playBtn.textContent = playing ? '❚❚ Pause' : '▶ Play'; };
const slowIn = document.getElementById('speed');
const slowOut = document.getElementById('speedOut');
function updSlow() {
  slow = Math.pow(2, parseFloat(slowIn.value));
  const secPerRev = 1 / (BASE_REV_PER_S * slow);
  slowOut.textContent = `mast: 1 turn / ${secPerRev.toFixed(secPerRev < 10 ? 1 : 0)} s`;
}
slowIn.oninput = updSlow;
updSlow();

// ---------- pilot controls (collective, cyclic, pedals, mixing unit) ----------
const C = rh.state.controls;
const collIn = document.getElementById('coll');
const pedIn = document.getElementById('ped');
const mixIn = document.getElementById('mix');
collIn.oninput = () => { C.collective = Number(collIn.value); pilotInput('collective'); };
pedIn.oninput = () => { C.pedal = -Number(pedIn.value); pilotInput('pedals'); };

// ---------- grab the 3D cockpit controls with the mouse ----------
const pickRay = new THREE.Raycaster();
let grab = null;
const ndc = (e) => {
  const r = renderer.domElement.getBoundingClientRect();
  return new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
};
const pickables = () => (active === 'r44' ? r44.pick : rh.discGrabbable() ? [...cp.pick, rh.discPick] : cp.pick);
const ctrlGeo = () => (active === 'r44' ? r44.geo : cp.geo);
const planeHit = (e, plane) => {
  pickRay.setFromCamera(ndc(e), camera);
  return pickRay.ray.intersectPlane(plane, new THREE.Vector3());
};
document.addEventListener('pointerdown', (e) => {
  if (e.target !== renderer.domElement || e.button !== 0) return;
  pickRay.setFromCamera(ndc(e), camera);
  const hit = pickRay.intersectObjects(pickables(), false)[0];
  if (!hit) return;
  const g = ctrlGeo(), kind = hit.object.userData.ctrl;
  e.stopImmediatePropagation();
  controls.enabled = false;
  stopDemo();
  if (kind === 'disc') {
    // drag the grabbed point of the ring up / down in the vertical plane through the mast
    // azimuth of the grabbed point: where the mouse ray meets the disc plane (exact, unlike the
    // surface of the thick invisible grab band)
    const upW = new THREE.Vector3(0, 1, 0).applyQuaternion(tx.root.getWorldQuaternion(new THREE.Quaternion()));
    const discPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(upW, tx.root.localToWorld(new THREE.Vector3(0, rh.discCentreY(), 0)));
    const onDisc = pickRay.ray.intersectPlane(discPlane, new THREE.Vector3()) || hit.point.clone();
    const p = tx.root.worldToLocal(onDisc);
    const phi = Math.atan2(-p.z, p.x);
    const n = tx.root.localToWorld(new THREE.Vector3(Math.sin(phi), 0, Math.cos(phi))).sub(tx.root.localToWorld(new THREE.Vector3()));
    // the plane contains the mast axis (not the grab-band surface), so heights read true
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(n.normalize(), tx.root.localToWorld(new THREE.Vector3(0, rh.discCentreY(), 0)));
    const ug = new THREE.Vector2(Math.cos(phi), Math.sin(phi)), up = new THREE.Vector2(-Math.sin(phi), Math.cos(phi));
    grab = { kind, plane, ug, up, perp0: rh.state.discTilt.dot(up) };
  } else if (kind === 'cyclic') {
    const pv = hit.object.userData.pivot || g.cycP;                     // pilot or copilot stick
    grab = { kind, pv, plane: new THREE.Plane(new THREE.Vector3(0, 1, 0), -(pv.y + g.gripH)) };
  } else if (kind === 'collective') {
    const pv = hit.object.userData.pivot || g.colP;
    grab = { kind, pv, plane: new THREE.Plane(new THREE.Vector3(0, 0, 1), -pv.z) };
  }
  else {
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -(g.pedP.y + 0.42));
    grab = { kind, plane, x0: planeHit(e, plane)?.x ?? 0, p0: C.pedal };
  }
  try { renderer.domElement.setPointerCapture(e.pointerId); } catch { /* pointer already gone */ }
}, true);
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!grab) return;
  const g = ctrlGeo(), p = planeHit(e, grab.plane);
  if (!p) return;
  if (grab.kind === 'disc') {
    // ring height at the grabbed azimuth: h = -R·(T·ug)  ->  along-component of the tilt
    const q = tx.root.worldToLocal(p.clone());
    const h = q.y - rh.discCentreY();
    const along = THREE.MathUtils.clamp(-h / rh.discRadius, -0.2, 0.2);    rh.setDiscTarget(grab.ug.clone().multiplyScalar(along).add(grab.up.clone().multiplyScalar(grab.perp0)));
  } else if (grab.kind === 'cyclic') {
    const R = g.gripH * Math.sin(g.CYC_TILT);
    // (R44 T-bar: the grip sits gripZ to the side of the centre post)
    let lon = (p.x - grab.pv.x) / R, lat = (p.z - grab.pv.z - (g.gripZ || 0)) / R;
    const m = Math.hypot(lon, lat);
    if (m > 1) { lon /= m; lat /= m; }
    C.lon = lon; C.lat = lat;
  } else if (grab.kind === 'collective') {
    const a = Math.atan2(p.y - grab.pv.y, p.x - grab.pv.x);
    C.collective = THREE.MathUtils.clamp((a - g.COLL_U0) / (g.COLL_U1 - g.COLL_U0), 0, 1);
  } else {
    const travel = (active === 'r44' ? 0.24 : 0.36) * Math.sin(THREE.MathUtils.degToRad(16));
    const s = grab.kind === 'pedalL' ? 1 : -1;
    C.pedal = THREE.MathUtils.clamp(grab.p0 + (s * (p.x - grab.x0)) / travel, -1, 1);
  }
  showPip(grab.kind.startsWith('pedal') ? 'pedals' : grab.kind);
});
const release = () => { if (grab) { grab = null; controls.enabled = !freeCam; } };
renderer.domElement.addEventListener('pointerup', release);
renderer.domElement.addEventListener('pointercancel', release);
renderer.domElement.addEventListener('pointermove', (e) => {
  if (grab || freeCam) return;
  pickRay.setFromCamera(ndc(e), camera);
  renderer.domElement.style.cursor = pickRay.intersectObjects(pickables(), false).length ? 'pointer' : '';
});
mixIn.onchange = () => { C.mixing = mixIn.checked; pilotInput('cyclic'); };
let airspeed = 0;
const spdIn = document.getElementById('kts');
spdIn.oninput = () => { airspeed = Number(spdIn.value); pilotInput('airspeed'); };
const pad = document.getElementById('pad');
const padCtx = pad.getContext('2d');
let dragging = false;
const setStick = (e) => {
  const r = pad.getBoundingClientRect();
  let x = ((e.clientX - r.left) / r.width) * 2 - 1;
  let y = ((e.clientY - r.top) / r.height) * 2 - 1;
  const m = Math.hypot(x, y);
  if (m > 1) { x /= m; y /= m; }
  C.lat = x; C.lon = -y;
  pilotInput('cyclic');
};
pad.onpointerdown = (e) => { dragging = true; pad.setPointerCapture(e.pointerId); setStick(e); stopDemo(); };
pad.onpointermove = (e) => { if (dragging) setStick(e); };
pad.onlostpointercapture = () => (dragging = false);
pad.ondblclick = () => { C.lon = 0; C.lat = 0; pilotInput('cyclic'); };
pad.onkeydown = (event) => {
  const delta = { ArrowUp: [0, 0.1], ArrowDown: [0, -0.1], ArrowRight: [0.1, 0], ArrowLeft: [-0.1, 0], Home: [0, 0] }[event.key];
  if (!delta) return;
  event.preventDefault();
  const x = event.key === 'Home' ? 0 : C.lat + delta[0];
  const y = event.key === 'Home' ? 0 : C.lon + delta[1];
  const scale = Math.max(1, Math.hypot(x, y));
  C.lat = x / scale; C.lon = y / scale;
  pilotInput('cyclic');
};
document.getElementById('center').onclick = () => { C.lon = 0; C.lat = 0; C.pedal = 0; C.collective = 0.35; pilotInput(pipKind || 'cyclic'); };

// automatic demo sequence (real seconds)
const DEMO = [
  { t: 0, coll: 0.35, lon: 0, lat: 0, ped: 0, say: 'Neutral: collective low, cyclic centred. All four blades have the same pitch.' },
  { t: 2, coll: 0.35, lon: 0, lat: 0, ped: 0, say: 'Collective UP: the servos raise the whole swashplate — every blade gets more pitch at the same time, so total lift increases.' },
  { t: 6, coll: 0.85, lon: 0, lat: 0, ped: 0 },
  { t: 9, coll: 0.85, lon: 0, lat: 0, ped: 0, say: 'Collective DOWN: swashplate lowers, all blades lose pitch together.' },
  { t: 12, coll: 0.35, lon: 0, lat: 0, ped: 0 },
  { t: 13, coll: 0.35, lon: 0, lat: 0, ped: 0, say: 'Cyclic FORWARD: the swashplate tilts, each blade\'s pitch now changes once per turn. Lowest pitch is on the right (advancing) side, so the blade is lowest over the nose — the disc tilts forward.' },
  { t: 16, coll: 0.35, lon: 1, lat: 0, ped: 0 },
  { t: 24, coll: 0.35, lon: 1, lat: 0, ped: 0, say: 'Cyclic RIGHT: lowest pitch moves to the tail side, the disc tilts to the right.' },
  { t: 27, coll: 0.35, lon: 0, lat: 1, ped: 0 },
  { t: 35, coll: 0.35, lon: 0, lat: 1, ped: 0, say: 'Back to neutral.' },
  { t: 38, coll: 0.35, lon: 0, lat: 0, ped: 0 },
  { t: 40, coll: 0.35, lon: 0, lat: 0, ped: 0 },
];
let demoT = -1;
const demoBtn = document.getElementById('demo');
function stopDemo() { demoT = -1; demoBtn.textContent = '▶ Auto demo'; }
demoBtn.onclick = () => {
  if (demoT >= 0) return stopDemo();
  demoT = 0; demoBtn.textContent = '■ Stop demo';
  if (!rh.state.__shown) goShot(SHOTS.find((s) => s.id === 'disc'));
};
function runDemo(dt) {
  if (demoT < 0) return;
  demoT += dt;
  if (demoT > DEMO[DEMO.length - 1].t) demoT = 0;
  let i = 0;
  while (i < DEMO.length - 2 && DEMO[i + 1].t <= demoT) i++;
  const a = DEMO[i], b = DEMO[i + 1];
  const u = THREE.MathUtils.smoothstep(demoT, a.t, b.t);
  C.collective = THREE.MathUtils.lerp(a.coll, b.coll, u);
  C.lon = THREE.MathUtils.lerp(a.lon, b.lon, u);
  C.lat = THREE.MathUtils.lerp(a.lat, b.lat, u);
  C.pedal = THREE.MathUtils.lerp(a.ped, b.ped, u);
  for (let k = i; k >= 0; k--) if (DEMO[k].say) { document.getElementById('caption').textContent = DEMO[k].say; break; }
}

const R2D = THREE.MathUtils.radToDeg;
function drawPad() {
  const w = pad.width / 2, h = pad.height / 2, g = padCtx;
  g.setTransform(2, 0, 0, 2, 0, 0);
  g.clearRect(0, 0, w, h);
  g.strokeStyle = 'rgba(255,255,255,0.35)'; g.lineWidth = 1.5;
  g.beginPath(); g.arc(w / 2, h / 2, w / 2 - 3, 0, Math.PI * 2); g.stroke();
  g.beginPath(); g.moveTo(w / 2, 6); g.lineTo(w / 2, h - 6); g.moveTo(6, h / 2); g.lineTo(w - 6, h / 2); g.stroke();
  g.fillStyle = 'rgba(255,255,255,0.6)'; g.font = '11px Segoe UI'; g.textAlign = 'center';
  g.fillText('FWD', w / 2, 16); g.fillText('AFT', w / 2, h - 7);
  const x = w / 2 + C.lat * (w / 2 - 12), y = h / 2 - C.lon * (h / 2 - 12);
  g.fillStyle = '#f0a83a'; g.beginPath(); g.arc(x, y, 9, 0, Math.PI * 2); g.fill();
}
const pip = document.getElementById('pip');
const pipTitle = document.getElementById('pip-title');
const pipValue = document.getElementById('pip-value');
const pipView = document.getElementById('pip-view');
const pipCanvas = document.createElement('canvas');
const pipContext = pipCanvas.getContext('2d');
pipView.appendChild(pipCanvas);
const pipCamera = new THREE.PerspectiveCamera(45, 1, camera.near, camera.far);
const mainViewport = new THREE.Vector4();
const pipShots = {
  cyclic: ['swash', 'r44rotor'],
  disc: ['swash', 'r44rotor'],
  collective: ['swash', 'r44rotor'],
  pedals: ['tailrotor', 'r44tail'],
  airspeed: ['stab'],
};
let pipKind;
const collValue = document.getElementById('coll-value');
const pedValue = document.getElementById('ped-value');
const ktsValue = document.getElementById('kts-value');

function syncFlightControls() {
  collIn.value = C.collective;
  pedIn.value = -C.pedal;
  collValue.value = Math.round(C.collective * 100) + '%';
  const pedalPercent = Math.round(Number(pedIn.value) * 100);
  pedValue.value = pedalPercent === 0 ? 'Neutral' : Math.abs(pedalPercent) + '% ' + (pedalPercent < 0 ? 'L' : 'R');
  ktsValue.value = airspeed + ' kt';
}

function pilotInput(kind) {
  stopDemo();
  showPip(kind);
}

function showPip(kind) {
  const shotId = pipShots[kind]?.[active === 'r44' ? 1 : 0];
  if (!shotId || pipKind === kind) return;
  const shot = SHOTS.find(({ id }) => id === shotId);
  pipKind = kind;
  pipTitle.textContent = shot.name === 'Swashplate' ? shot.name : shot.title;
  pipCamera.position.copy(W(new THREE.Vector3(...shot.cam)));
  pipCamera.lookAt(W(new THREE.Vector3(...shot.tgt)));
  pip.hidden = false;
}

function closePip() {
  pip.hidden = true;
  pipKind = undefined;
}
document.getElementById('pip-close').onclick = closePip;

function renderPip() {
  if (!pipKind) return;
  const width = pipView.clientWidth;
  const height = pipView.clientHeight;
  if (!width || !height) return;
  const ratio = renderer.getPixelRatio();
  const pixelWidth = Math.floor(width * ratio);
  const pixelHeight = Math.floor(height * ratio);
  if (pipCanvas.width !== pixelWidth || pipCanvas.height !== pixelHeight) {
    pipCanvas.width = pixelWidth;
    pipCanvas.height = pixelHeight;
    pipCamera.aspect = width / height;
    pipCamera.updateProjectionMatrix();
  }

  // Render into a small viewport and copy it before the main composer redraws the canvas.
  renderer.getViewport(mainViewport);
  renderer.setViewport(0, 0, width, height);
  renderer.shadowMap.autoUpdate = false;
  renderer.render(scene, pipCamera);
  renderer.shadowMap.autoUpdate = true;
  renderer.setViewport(mainViewport);
  pipContext.clearRect(0, 0, pixelWidth, pixelHeight);
  pipContext.drawImage(renderer.domElement, 0, renderer.domElement.height - pixelHeight,
    pixelWidth, pixelHeight, 0, 0, pixelWidth, pixelHeight);

  const state = activeState();
  if (pipKind === 'pedals') {
    const angle = active === 'r44' ? state.trPitch : tail.state.trPitch;
    pipValue.value = 'Tail rotor pitch ' + R2D(angle).toFixed(1) + '°';
  } else if (pipKind === 'airspeed') {
    pipValue.value = 'Stabilator ' + R2D(tail.state.stabTED).toFixed(1) + '°';
  } else {
    pipValue.value = pipKind === 'collective'
      ? 'Collective pitch ' + R2D(state.thetaColl).toFixed(1) + '°'
      : 'Cyclic pitch ±' + R2D(state.cyclicAmp).toFixed(1) + '°';
  }
}
labelsToggle.onchange = (e) => { labelsOn = e.target.checked; applyLabels(); };
// ---------- loop ----------
const clock = new THREE.Clock();
const ease = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
function frame() {
  const dt = Math.min(clock.getDelta(), 0.05);
  if (anim) {
    anim.t += dt / anim.dur;
    const e = ease(Math.min(anim.t, 1));
    camera.position.lerpVectors(anim.p0, anim.p1, e);
    controls.target.lerpVectors(anim.t0, anim.t1, e);
    if (anim.t >= 1) anim = null;
  }
  const dTheta = playing ? 2 * Math.PI * BASE_REV_PER_S * slow * dt : 0;
  runDemo(dt);
  if (active === 'uh60') {
    tx.update(dTheta, dt);
    rh.update(tx.debug.carrier.angle, dTheta, dt);
    tail.update(rh.state.tailPitch, airspeed, dt);
    cp.update(rh.state);
    for (const t of t700) t.int.update(dTheta, dt, tx.engineFactor[t.sideKey], C.collective);
  } else {
    r44.update(dTheta, dt);                       // same on-screen mast turn rate as the UH-60
  }
  drawPad();
  syncFlightControls();
  renderPip();
  pollGamepad(dt);
  if (freeCam) updateFly(dt);
  else controls.update();
  composer.render();
  labelRenderer.render(scene, camera);
  requestAnimationFrame(frame);
}
goShot(SHOTS[0], true);
frame();

new ResizeObserver(() => {
  const width = app.clientWidth;
  const height = app.clientHeight;
  if (!width || !height) return;
  camera.aspect = width / height;
  // Keep the horizontal field of view on portrait screens.
  camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(35 / 2)) / Math.min(1, camera.aspect)));
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
  composer.setSize(width, height);
  labelRenderer.setSize(width, height);
}).observe(app);

// debugging hooks
window.__tx = tx;
window.__air = air;
window.__rh = rh;
window.__tail = tail;
window.__camera = camera;
window.__cp = cp;
window.__r44 = r44;
window.__shot = (id) => goShot(SHOTS.find((s) => s.id === id), true);
window.__cam = (p, t) => { camera.position.copy(W(new THREE.Vector3(...p))); controls.target.copy(W(new THREE.Vector3(...t))); };
window.__pause = (v = true) => { playing = !v; };
