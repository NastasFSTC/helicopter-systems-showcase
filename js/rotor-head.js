// UH-60 style 4-blade main rotor head + swashplate + primary servos.
// Facts (RESEARCH.md A4/A6): one-piece hub, elastomeric bearings (flap + lead-lag + pitch),
// hydraulic lag dampers, bifilar absorber on top, pitch control rods driven by the swashplate,
// 3 primary servos pushing the stationary swashplate, scissors links.
//
// Coordinates are root-local (same frame as the drivetrain): +X forward, +Y up, +Z right.
// Azimuth phi is measured counter-clockwise seen from above, from +X (nose): dir = (cos, 0, -sin).
// The rotor turns counter-clockwise seen from above (advancing blade on the right).
import * as THREE from 'three';
import { MAT, shadow, cylZ, tubeZ, boltRing } from './parts.js';
import { Link, Damper, Servo, Joint, hexNut, latheX, stadiumPath, plateY } from './detail.js';
const tang = (a) => new THREE.Vector3(-Math.sin(a), 0, -Math.cos(a)); // horizontal tangent at azimuth a

const deg = THREE.MathUtils.degToRad;
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const Yax = V(0, 1, 0);
const azDir = (phi) => V(Math.cos(phi), 0, -Math.sin(phi));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

const RUBBER = new THREE.MeshPhysicalMaterial({ color: 0x15171a, metalness: 0, roughness: 0.75 });
const BLADE = new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 0.15, roughness: 0.55, clearcoat: 0.25, side: THREE.DoubleSide });
const TPP = new THREE.MeshBasicMaterial({ color: 0xf0a83a, transparent: true, opacity: 0.55, depthWrite: false });

// ---------- geometry constants ----------
export const ROTOR = {
  hubY: 7.95,            // hub centre height (root-local)
  hinge: 0.9,            // elastomeric bearing centre = flap / lag / pitch point
  tip: 17.5,             // blade tip radius (shortened for display — real ratio is larger)
  chord: 1.05,
  hornX: 1.04,           // pitch horn tip, measured from the hinge along the span (outboard of the housing)
  hornY: -0.25,          //   ... below the spindle axis
  hornZ: -0.6,           //   ... toward the leading edge (-Z in blade frame)
  swashY: 6.45,          // swashplate centre at neutral
  deckY: 3.45,           // fixed deck where the primary servos are anchored
  servoAz: [0, deg(120), deg(240)], // forward, left-aft, right-aft
};
const HOUSING_BORE = 0.33;   // bore of the hub-arm bearing housing (clears 13° flap at full inputs)
const SPINDLE_R = 0.15;      // spindle radius where it passes through the housing
const LUG_Y = 0.28;     // height of the pitch-rod bolts above the rotating swashplate centre
const HORN_R = Math.hypot(ROTOR.hinge + ROTOR.hornX, ROTOR.hornZ);
export const HORN_LEAD = Math.atan2(-ROTOR.hornZ, ROTOR.hinge + ROTOR.hornX); // link leads its blade
// horn tip height (relative to hinge) as a function of blade pitch theta (nose-up positive)
const hornH = (t) => ROTOR.hornY * Math.cos(t) - ROTOR.hornZ * Math.sin(t);
const HA = -ROTOR.hornZ, HB = ROTOR.hornY, HR = Math.hypot(HA, HB), H0 = Math.atan2(HB, HA);
const pitchFromHornH = (h) => Math.asin(THREE.MathUtils.clamp(h / HR, -1, 1)) - H0;

/** Thin rod between two points (pitch links, servo rods, scissors). */
class Rod {
  constructor(parent, r, mat, ends = 0) {
    this.m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, 1, 18), mat));
    parent.add(this.m);
    this.ends = [];
    if (ends) for (let i = 0; i < 2; i++) {
      const e = shadow(new THREE.Mesh(new THREE.SphereGeometry(ends, 16, 12), MAT.steel));
      parent.add(e);
      this.ends.push(e);
    }
  }
  set(a, b) {
    const d = b.clone().sub(a);
    const len = d.length();
    this.m.position.copy(a).addScaledVector(d, 0.5);
    this.m.quaternion.setFromUnitVectors(Yax, d.normalize());
    this.m.scale.set(1, len, 1);
    if (this.ends.length) { this.ends[0].position.copy(a); this.ends[1].position.copy(b); }
  }
  get visible() { return this.m.visible; }
  set visible(v) { this.m.visible = v; this.ends.forEach((e) => (e.visible = v)); }
}

/** Two-link scissors between a and b, knee pushed toward `out`. */
function knee(a, b, l, out) {
  const mid = a.clone().add(b).multiplyScalar(0.5);
  const d = b.clone().sub(a);
  const h = Math.sqrt(Math.max(l * l - d.lengthSq() / 4, 0));
  const perp = out.clone().addScaledVector(d, -out.dot(d) / d.lengthSq()).normalize();
  return mid.addScaledVector(perp, h);
}

/** Annulus lying in the XZ plane between y0 and y1. */
function ringY(rOut, rIn, y0, y1, mat, seg = 72) {
  const m = tubeZ(rOut, rIn, y0, y1, mat, seg);
  m.rotation.x = -Math.PI / 2;
  return m;
}
function cylY(r, y0, y1, mat, seg = 40) {
  const m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, y1 - y0, seg), mat));
  m.position.y = (y0 + y1) / 2;
  return m;
}
function latheY(profile, mat, seg = 64) {
  return shadow(new THREE.Mesh(new THREE.LatheGeometry(profile.map(([r, y]) => new THREE.Vector2(r, y)), seg), mat));
}
function box(sx, sy, sz, x, y, z, mat) {
  const m = shadow(new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat));
  m.position.set(x, y, z);
  return m;
}

/**
 * Blade with SC1095-like section (9.5 % thick, slight camber), -18 deg linear twist,
 * 20 deg swept tip and a titanium leading-edge strip. Built in blade frame:
 * +X span, +Y up, leading edge toward -Z.
 */
function bladeGeometry(r0, r1, chord) {
  const nS = 48, nA = 36;
  const pos = [], col = [], idx = [];
  const dark = new THREE.Color(0x30353b), le = new THREE.Color(0xa7adb4), tipC = new THREE.Color(0xc9a23a);
  const sweepStart = r1 - (r1 - r0) * 0.07;
  for (let i = 0; i <= nS; i++) {
    const s = i / nS;
    const r = r0 + (r1 - r0) * s;
    const twist = deg(9) - deg(18) * s;              // root +9, tip -9 (zero near 50 % span)
    const sweep = r > sweepStart ? (r - sweepStart) * Math.tan(deg(20)) : 0;
    const c = chord * (r > sweepStart ? 1 - 0.35 * (r - sweepStart) / (r1 - sweepStart) : 1);
    for (let j = 0; j <= nA; j++) {
      const u = j / nA;                                 // 0..1 around the section
      const x01 = 0.5 * (1 - Math.cos(2 * Math.PI * u)); // 0 at LE -> 1 at TE -> 0 at LE
      const upper = u < 0.5;
      const t = 0.095 * 5 * (0.2969 * Math.sqrt(x01) - 0.126 * x01 - 0.3516 * x01 ** 2 + 0.2843 * x01 ** 3 - 0.1036 * x01 ** 4);
      const camber = 0.012 * Math.sin(Math.PI * x01);
      const yy = (upper ? t : -t) * c + camber * c;
      const zz = (x01 - 0.25) * c;                      // quarter-chord on the pitch axis; LE at -Z
      const cz = Math.cos(twist), sz = Math.sin(twist);
      // nose-up twist raises the leading edge (-Z) : rotate about X
      const y2 = yy * cz - zz * sz;
      const z2 = yy * sz + zz * cz;
      pos.push(r, y2, z2 + sweep);
      const cc = x01 < 0.06 ? le : r > r1 - 0.25 ? tipC : dark;
      col.push(cc.r, cc.g, cc.b);
    }
  }
  for (let i = 0; i < nS; i++) for (let j = 0; j < nA; j++) {
    const a = i * (nA + 1) + j, b = a + nA + 1;
    idx.push(a, b, a + 1, b, b + 1, a + 1);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

export function buildRotorHead(root) {
  const anchors = {};
  const R = ROTOR;

  // ================= rotating hub assembly =================
  const head = new THREE.Group();
  head.position.y = R.hubY;
  root.add(head);

  // ---- one-piece hub: central body; each arm is a loop (two side bars + a bored housing at the
  //      outboard end). The spindle passes through the housing bore WITH CLEARANCE, so it can
  //      pitch, flap and lead-lag about the elastomeric bearing without touching the hub.
  {
    const core = new THREE.Shape();
    core.absarc(0, 0, 0.74, 0, Math.PI * 2, false);
    const hole = new THREE.Path();
    hole.absarc(0, 0, 0.3, 0, Math.PI * 2, true);
    core.holes.push(hole);
    head.add(plateY(core, -0.19, 0.19, MAT.paint, 0.05));
  }
  // hub barrel, upper and lower flanges, mast nut with lock plate
  head.add(latheY([[0.3, -0.42], [0.56, -0.42], [0.6, -0.36], [0.6, 0.36], [0.5, 0.42], [0.3, 0.42]], MAT.paint));
  head.add(cylY(0.42, 0.42, 0.5, MAT.steelDark, 48));
  const capBolts = boltRing(12, 0.48, 0.42, 0.032, MAT.steelDark, 1);
  capBolts.rotation.x = -Math.PI / 2;
  head.add(capBolts);
  const lowBolts = boltRing(12, 0.48, -0.42, 0.032, MAT.steelDark, -1);
  lowBolts.rotation.x = -Math.PI / 2;
  head.add(lowBolts);
  head.add(cylY(0.22, 0.5, 1.12, MAT.steel));           // mast end (splined) — carries the bifilar
  for (let k = 0; k < 16; k++) {
    const sp = box(0.03, 0.4, 0.03, 0, 0.72, 0, MAT.steelDark);
    const a = (k / 16) * Math.PI * 2;
    sp.position.set(Math.cos(a) * 0.22, 0.72, Math.sin(a) * 0.22);
    head.add(sp);
  }
  const mastNut = hexNut(0.33, 0.16);
  mastNut.position.y = 0.88;
  head.add(mastNut);
  for (let k = 0; k < 6; k++) {               // castellations + lock tabs
    const a = (k / 6) * Math.PI * 2 + Math.PI / 6;
    const tab = box(0.08, 0.06, 0.05, Math.cos(a) * 0.3, 0.98, Math.sin(a) * 0.3, MAT.steel);
    tab.rotation.y = -a;
    head.add(tab);
  }
  // mast collar under the hub (rotating scissors anchor) with lug
  head.add(latheY([[0.3, -1.0], [0.47, -1.0], [0.47, -0.62], [0.4, -0.55], [0.3, -0.42]], MAT.steel));
  // rotating-scissors anchor bolt on the collar (fork opens down and outward)
  const rsAz = deg(45) + HORN_LEAD;
  const jCollar = new Joint(head, root, azDir(rsAz).multiplyScalar(0.64).setY(-0.8), tang(rsAz),
    azDir(rsAz).multiplyScalar(0.5).add(V(0, -1, 0)), { mat: MAT.steel });
  {
    const boss = box(0.22, 0.22, 0.22, 0, 0, 0, MAT.steel);       // collar boss carrying the fork
    boss.position.copy(azDir(rsAz).multiplyScalar(0.5)).setY(-0.7);
    boss.rotation.y = rsAz;
    head.add(boss);
  }

  // ---- bifilar vibration absorber: 4-arm plate, each weight hung on TWO pins (bifilar) ----
  const bif = new THREE.Group();
  bif.position.y = 1.02;
  head.add(bif);
  {
    const s = new THREE.Shape();
    s.absarc(0, 0, 0.32, 0, Math.PI * 2, false);
    bif.add(plateY(s, 0, 0.12, MAT.paint, 0.03));
    for (let i = 0; i < 4; i++) {
      const a = deg(45) + (i * Math.PI) / 2;
      const armS = new THREE.Shape(stadiumPath(0.15, 0.95, 0.13).getPoints(20));
      const arm = plateY(armS, 0.01, 0.11, MAT.paint, 0.025);
      arm.rotation.y = a;
      bif.add(arm);
      const g = new THREE.Group();
      g.position.copy(azDir(a).multiplyScalar(0.95));
      g.rotation.y = a;
      bif.add(g);
      const w = shadow(new THREE.Mesh(new THREE.CapsuleGeometry(0.11, 0.26, 6, 16), MAT.steelDark));
      w.rotation.x = Math.PI / 2;     // long axis tangential
      w.position.y = -0.16;
      g.add(w);
      for (const t of [-0.12, 0.12]) {
        const pin = cylY(0.03, -0.2, 0.16, MAT.steel, 12);
        pin.position.z = t;
        g.add(pin);
        const bush = cylY(0.05, 0.11, 0.15, MAT.gold, 16);
        bush.position.z = t;
        g.add(bush);
      }
    }
  }

  const blades = [];
  const bladeGeo = bladeGeometry(2.45, R.tip, R.chord);
  for (let i = 0; i < 4; i++) {
    const phi = (i * Math.PI) / 2;
    const arm = new THREE.Group();
    arm.rotation.y = phi;
    head.add(arm);
    // ---- STATIONARY (hub) side of the arm ----
    // hub loop: two side bars from the hub body out to the bearing housing
    for (const s of [-1, 1]) {
      const bar = plateY(new THREE.Shape(stadiumPath(0.55, 1.26, 0.075).getPoints(20)), -0.17, 0.17, MAT.paint, 0.035);
      bar.position.z = s * 0.3;
      arm.add(bar);
    }
    // bearing housing at the outboard end of the loop: bored so the spindle passes with clearance
    arm.add(latheX([[HOUSING_BORE, 1.2], [0.4, 1.2], [0.44, 1.24], [0.44, 1.42], [0.4, 1.46], [HOUSING_BORE, 1.46], [HOUSING_BORE, 1.2]], MAT.paint, 56));
    // elastomeric bearing outer race seated against the inside of the housing (does not rotate)
    arm.add(latheX([[0.2, 1.11], [0.26, 1.11], [0.26, 1.2], [0.2, 1.2], [0.2, 1.11]], MAT.steel, 40));
    // retainer ring bolted to the outside of the housing
    arm.add(latheX([[HOUSING_BORE, 1.46], [0.37, 1.46], [0.37, 1.5], [HOUSING_BORE, 1.5], [HOUSING_BORE, 1.46]], MAT.steelDark, 48));
    for (let k = 0; k < 8; k++) {
      const a = (k / 8) * Math.PI * 2 + Math.PI / 8;
      const b = hexNut(0.026, 0.035);
      b.rotation.z = Math.PI / 2;
      b.position.set(1.515, Math.sin(a) * 0.33, Math.cos(a) * 0.33);
      arm.add(b);
    }
    // fixed pitch pointer on the retainer (reads against the index stripe on the turning spindle)
    // (tab screwed to the retainer face, hanging down in front of the bore toward the spindle)
    arm.add(box(0.03, 0.19, 0.03, 1.515, 0.27, 0, MAT.gold));
    // droop stop (below) and anti-flap stop (above) on the hub — the spindle rests on them when stopped
    arm.add(box(0.2, 0.08, 0.26, 0.6, -0.22, 0, MAT.steelDark));
    arm.add(box(0.2, 0.06, 0.2, 0.6, 0.215, 0, MAT.steelDark));
    // hub lug for the lead-lag damper (trailing side)
    // (bolt vertical so the damper swings in the lead-lag plane; fork opens toward the trailing side)
    arm.add(box(0.2, 0.2, 0.2, 0.3, 0.05, 0.68, MAT.paint));
    const jDampHub = new Joint(arm, root, V(0.3, 0.05, 0.94), V(0, 1, 0), V(0, 0, 1));

    // hinge group: lag (Y) -> flap (Z) -> pitch (X)
    const hinge = new THREE.Group();
    hinge.position.x = R.hinge;
    hinge.rotation.order = 'YZX';
    arm.add(hinge);
    // ---- ROTATING side: everything below turns with blade pitch (and flaps / lead-lags) ----
    // elastomeric bearing laminates: inner race on the spindle, rubber/steel shims (they shear
    // to allow pitch, flap and lead-lag about this point)
    for (let k = 0; k < 11; k++) {
      const rr = 0.2 - Math.abs(k - 5) * 0.006;
      const disc = cylZ(rr, -0.017, 0.017, k % 2 ? MAT.steel : RUBBER, { seg: 40 });
      disc.rotation.y = Math.PI / 2;
      disc.position.x = -0.04 + k * 0.022;
      hinge.add(disc);
    }
    // spindle: inboard retaining nut, inner race seat, shaft through the housing, horn collar, fork root
    hinge.add(latheX([
      [0.0, -0.36], [0.1, -0.36], [0.13, -0.3], [0.13, 0.22], [SPINDLE_R, 0.25], [SPINDLE_R, 0.72],
      [0.17, 0.74], [0.17, 1.12], [0.22, 1.17], [0.26, 1.24], [0.26, 1.32], [0.0, 1.32],
    ], MAT.steel, 48));
    const spNut = hexNut(0.17, 0.08);
    spNut.rotation.z = Math.PI / 2;
    spNut.position.x = -0.32;
    hinge.add(spNut);
    // pitch index stripe on the spindle, just outboard of the housing
    hinge.add(box(0.2, 0.012, 0.035, 0.84, 0.172, 0, MAT.gold));
    // blade fold clevis: upper + lower fork plates, root doubler, two quick-release expandable pins
    for (const yy of [0.16, -0.16]) {
      const fork = plateY(new THREE.Shape(stadiumPath(1.45, 2.0, 0.27).getPoints(24)), yy - 0.035, yy + 0.035, MAT.paint, 0.012);
      hinge.add(fork);
    }
    hinge.add(box(0.45, 0.26, 0.46, 1.86, 0, 0, MAT.steelBlack));          // blade root fitting (fills the fork)
    for (const x of [1.58, 1.9]) {
      const pin = cylY(0.055, -0.24, 0.24, MAT.steel, 16);
      pin.position.set(x, 0, 0);
      hinge.add(pin);
      const handle = cylZ(0.03, -0.13, 0.13, MAT.gold, { seg: 10 });    // T-handle (no tools needed)
      handle.position.set(x, 0.27, 0);
      hinge.add(handle);
      const stem = cylY(0.025, 0.2, 0.29, MAT.gold, 10);
      stem.position.set(x, 0, 0);
      hinge.add(stem);
      const nutB = hexNut(0.07, 0.05);
      nutB.position.set(x, -0.25, 0);
      hinge.add(nutB);
    }
    // pitch horn: collar on the spindle + forged arm to a clevis at the leading-edge side
    const hornTip = new THREE.Object3D();
    hornTip.position.set(R.hornX, R.hornY, R.hornZ);
    hinge.add(hornTip);
    {
      const collar = latheX([[0.17, 0.95], [0.25, 0.97], [0.25, 1.12], [0.17, 1.14]], MAT.paint, 40);
      hinge.add(collar);
      // forged arm ends exactly on the base of the horn fork (fork opens down, bolt spanwise)
      const a = V(R.hornX, -0.06, -0.2), b = V(R.hornX, R.hornY + 0.19, R.hornZ);
      const d = b.clone().sub(a);
      const armG = new THREE.CylinderGeometry(0.05, 0.085, d.length(), 16);
      const armM = shadow(new THREE.Mesh(armG, MAT.paint));
      armM.position.copy(a).addScaledVector(d, 0.5);
      armM.quaternion.setFromUnitVectors(Yax, d.normalize());
      hinge.add(armM);
    }
    const jHorn = new Joint(hinge, root, hornTip.position.clone(), V(1, 0, 0), V(0, -1, 0));
    // damper attachment lug on the trailing side of the spindle
    const damperSp = new THREE.Object3D();
    damperSp.position.set(0.82, 0.05, 0.52);
    hinge.add(damperSp);
    hinge.add(box(0.2, 0.2, 0.2, 0.82, 0.05, 0.26, MAT.paint));
    const jDampSp = new Joint(hinge, root, damperSp.position.clone(), V(0, 1, 0), V(0, 0, 1));
    // anti-flap / droop stop pads on the spindle
    // (on the retaining nut, directly over / under the hub stops)
    hinge.add(box(0.07, 0.05, 0.14, -0.3, -0.16, 0, MAT.steelDark));
    hinge.add(box(0.07, 0.05, 0.14, -0.3, 0.16, 0, MAT.steelDark));
    // blade
    const bm = shadow(new THREE.Mesh(bladeGeo, BLADE));
    bm.position.x = -R.hinge; // geometry radii are measured from the hub centre
    hinge.add(bm);
    // BIM (blade inspection method) indicator at the root trailing edge: shows red if the spar leaks
    const bim = cylY(0.05, 0.02, 0.12, MAT.steelDark, 16);
    bim.position.set(2.2 - 0, 0, 0.42);
    hinge.add(bim);
    const bimEye = cylY(0.035, 0.12, 0.15, MAT.gold, 16);
    bimEye.position.set(2.2, 0, 0.42);
    hinge.add(bimEye);

    // hub side of the lag damper
    const damperHub = new THREE.Object3D();
    damperHub.position.set(0.3, 0.05, 0.62);
    arm.add(damperHub);
    blades.push({ phi, hinge, hornTip, damperSp, damperHub, jHorn, jDampHub, jDampSp, pitch: 0, flap: 0, lag: 0 });
  }

  // ================= swashplate =================
  const stat = new THREE.Group();   // stationary (tilts, does not rotate)
  const rot = new THREE.Group();    // rotating (tilts + turns with the mast)
  root.add(stat, rot);
  stat.add(ringY(1.12, 0.56, -0.17, -0.05, MAT.paint));
  stat.add(ringY(1.02, 0.6, -0.05, -0.01, MAT.gold, 64));          // bearing race
  const statBolts = boltRing(18, 0.86, -0.17, 0.03, MAT.steelDark, -1);
  statBolts.rotation.x = -Math.PI / 2;
  stat.add(statBolts);
  // servo attach bolts under the stationary ring: bolt tangential, fork opens down
  const jServoTop = R.servoAz.map((a) => {
    const ear = box(0.3, 0.1, 0.26, 0, 0, 0, MAT.paint);
    ear.position.copy(azDir(a).multiplyScalar(1.15)).setY(-0.14);
    ear.rotation.y = a;
    stat.add(ear);
    return new Joint(stat, root, azDir(a).multiplyScalar(1.2).setY(-0.36), tang(a), V(0, -1, 0));
  });
  const statScissorAz = deg(60);
  {
    const ear = box(0.26, 0.1, 0.22, 0, 0, 0, MAT.paint);
    ear.position.copy(azDir(statScissorAz).multiplyScalar(1.1)).setY(-0.14);
    ear.rotation.y = statScissorAz;
    stat.add(ear);
  }
  const jStatSwash = new Joint(stat, root, azDir(statScissorAz).multiplyScalar(1.15).setY(-0.36), tang(statScissorAz), V(0, -1, 0));
  rot.add(ringY(1.25, 0.56, -0.01, 0.11, MAT.paint));
  const rotBolts = boltRing(16, 0.92, 0.11, 0.03, MAT.steelDark, 1);
  rotBolts.rotation.x = -Math.PI / 2;
  rot.add(rotBolts);
  const lugs = [];
  for (const b of blades) {
    const a = b.phi + HORN_LEAD;
    const arm = box(0.95, 0.1, 0.2, 0, 0.05, 0, MAT.paint);
    arm.position.copy(azDir(a).multiplyScalar(HORN_R - 0.55)).setY(0.05);
    arm.rotation.y = a;
    rot.add(arm);
    // pitch-rod bolt on the rotating ring: fork opens up, bolt radial (parallel to the horn bolt)
    lugs.push(new Joint(rot, root, azDir(a).multiplyScalar(HORN_R).setY(LUG_Y), azDir(a), V(0, 1, 0)));
  }
  const rotScissorAz = rsAz;
  {
    const ear = box(0.24, 0.1, 0.22, 0, 0, 0, MAT.paint);
    ear.position.copy(azDir(rotScissorAz).multiplyScalar(1.1)).setY(0.12);
    ear.rotation.y = rotScissorAz;
    rot.add(ear);
  }
  const jRotSwash = new Joint(rot, root, azDir(rotScissorAz).multiplyScalar(1.12).setY(0.32), tang(rotScissorAz), V(0, 1, 0));
  // uniball + stand-pipe guide (fixed)
  const uniball = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.62, 40, 24), MAT.steel));
  root.add(uniball);
  root.add(cylY(0.5, R.deckY, 7.0, MAT.steelDark, 48));

  // fixed deck (top of the main transmission)
  // main gearbox housing (see-through): carries the deck, servos and scissors at its top
  {
    const glass = MAT.glass.clone();
    glass.opacity = 0.09;
    const housing = shadow(new THREE.Mesh(new THREE.LatheGeometry([
      [0.0, -0.85], [2.2, -0.85], [2.85, -0.4], [2.95, 1.0], [2.95, 2.1], [2.3, 2.9], [1.8, R.deckY - 0.1], [0.5, R.deckY - 0.1],
    ].map(([r, y]) => new THREE.Vector2(r, y)), 96), glass));
    housing.userData.support = true;
    root.add(housing);
  }
  const deck = ringY(1.75, 0.52, R.deckY - 0.12, R.deckY, MAT.gold, 96);
  root.add(deck);
  const deckBolts = boltRing(40, 1.6, R.deckY, 0.035, MAT.steelDark, 1);
  deckBolts.rotation.x = -Math.PI / 2;
  root.add(deckBolts);

  // primary servos (barrel fixed to deck clevis, piston rod to the stationary swashplate)
  const servos = R.servoAz.map((a, k) => {
    const mount = box(0.44, 0.1, 0.44, 0, 0, 0, MAT.paint);
    mount.position.copy(azDir(a).multiplyScalar(1.2)).setY(R.deckY + 0.05);
    root.add(mount);
    const jBase = new Joint(root, root, azDir(a).multiplyScalar(1.2).setY(R.deckY + 0.27), tang(a), V(0, 1, 0));
    const sv = new Servo(root, jBase.p(), a); // manifold faces outward
    return { a, sv, jBase, jTop: jServoTop[k] };
  });

  // pitch control rods (adjustable: turnbuckle + jam nuts), lag dampers, scissors links
  const pitchLinks = blades.map(() => new Link(root, { r: 0.04, eye: 0.07 }));
  const dampers = blades.map(() => new Damper(root));
  const scissor = () => new Link(root, { r: 0.05, eye: 0.07, mat: MAT.paint, turnbuckle: false });
  const rs = { a: scissor(), b: scissor() };
  const ss = { a: scissor(), b: scissor() };
  {
    const m = box(0.36, 0.1, 0.36, 0, 0, 0, MAT.paint);
    m.position.copy(azDir(statScissorAz).multiplyScalar(1.15)).setY(R.deckY + 0.05);
    root.add(m);
  }
  const jDeckScissor = new Joint(root, root, azDir(statScissorAz).multiplyScalar(1.15).setY(R.deckY + 0.27), tang(statScissorAz), V(0, 1, 0));
  // knee bolts (shared by the two scissor links)
  const kneeBolt = () => {
    const g = new THREE.Group();
    const b = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.026, 0.3, 12), MAT.steel));
    g.add(b);
    for (const s of [-1, 1]) { const n = hexNut(0.05, 0.035); n.position.y = s * 0.13; g.add(n); }
    root.add(g);
    return g;
  };
  const rsKnee = kneeBolt(), ssKnee = kneeBolt();

  // tip-path plane indicator
  const tpp = new THREE.Mesh(new THREE.TorusGeometry(R.tip - 0.6, 0.05, 8, 200), TPP);
  tpp.userData.overlay = true;   // indicator, not a part
  tpp.rotation.x = Math.PI / 2;
  const tppG = new THREE.Group();
  tppG.add(tpp);
  // invisible, thicker grab band riding with the ring (so it is easy to pick with the mouse)
  const discPick = new THREE.Mesh(new THREE.TorusGeometry(R.tip - 0.6, 0.7, 6, 96), new THREE.MeshBasicMaterial({ visible: false }));
  discPick.rotation.x = Math.PI / 2;
  discPick.userData.ctrl = 'disc';
  discPick.userData.overlay = true;
  tppG.add(discPick);
  root.add(tppG);

  anchors.hub = V(0.2, R.hubY + 1.05, -0.2);
  anchors.elasto = azDir(deg(270)).multiplyScalar(1.0).setY(R.hubY + 0.45);
  anchors.damper = azDir(deg(300)).multiplyScalar(1.3).setY(R.hubY - 0.35);
  anchors.bifilar = V(-0.75, R.hubY + 1.0, 0.75);
  anchors.pitchLink = azDir(deg(20)).multiplyScalar(1.75).setY(R.hubY - 0.85);
  anchors.rotSwash = azDir(deg(200)).multiplyScalar(1.45).setY(R.swashY + 0.15);
  anchors.statSwash = azDir(deg(150)).multiplyScalar(1.4).setY(R.swashY - 0.3);
  anchors.servo = azDir(deg(240)).multiplyScalar(1.45).setY(R.deckY + 1.2);
  anchors.blade = azDir(deg(0)).multiplyScalar(9).setY(R.hubY + 0.6);

  // ================= state / kinematics =================
  const lugY0 = R.swashY + LUG_Y;
  const hornH0 = hornH(0);
  const st = {
    controls: { collective: 0.35, lon: 0, lat: 0, pedal: 0, mixing: true },
    eff: { lon: 0, lat: 0 },
    thetaColl: 0, cyclicAmp: 0, stickAz: 0, swashLowAz: 0, swashTilt: 0, swashH: R.swashY,
    discTilt: new THREE.Vector2(0, 0), // (x: a*cos(az), y: a*sin(az)) of the disc low point
    coning: 0, tailPitch: 0, bladePitch: [0, 0, 0, 0], rotorAngle: 0,
  };
  const COLL_MIN = deg(-2), COLL_MAX = deg(16), CYC_MAX = deg(8);
  const MIX = { collToPitch: 0.12, collToRoll: 0.12, collToYaw: 0.35, yawToPitch: 0.08 }; // illustrative gains

  const tmp = V(0, 0, 0);
  const W2R = (o) => root.worldToLocal(o.getWorldPosition(tmp.clone()));

  function update(rotorAngle, dTheta, dt = 0) {
    const c = st.controls;
    st.rotorAngle = rotorAngle;
    head.rotation.y = rotorAngle;

    // ---- mixing unit (TM 1-1520-237-10 2.37.3), gains illustrative ----
    const tailPitch = THREE.MathUtils.clamp(c.pedal + (c.mixing ? MIX.collToYaw * (c.collective - 0.35) : 0), -1, 1);
    let lon = c.lon, lat = c.lat;
    if (c.mixing) {
      lon += MIX.collToPitch * (c.collective - 0.35) - MIX.yawToPitch * tailPitch; // fwd with collective, aft with tail pitch
      lat += -MIX.collToRoll * (c.collective - 0.35);                               // left with collective
    }
    lon = THREE.MathUtils.clamp(lon, -1, 1);
    lat = THREE.MathUtils.clamp(lat, -1, 1);
    st.eff.lon = lon; st.eff.lat = lat; st.tailPitch = tailPitch;

    // ---- swashplate position ----
    st.thetaColl = COLL_MIN + (COLL_MAX - COLL_MIN) * c.collective;
    const mag = Math.min(Math.hypot(lon, lat), 1);
    st.cyclicAmp = CYC_MAX * mag;
    st.stickAz = Math.atan2(-lat, lon);                    // forward = 0, right = -90 deg
    // max pitch reduction must happen 90 deg before the stick direction (rotation sense);
    // the pitch link leads its blade by HORN_LEAD, so the swashplate low point is:
    st.swashLowAz = st.stickAz - Math.PI / 2 + HORN_LEAD;
    st.swashH = R.swashY + (hornH(st.thetaColl) - hornH0);
    const dh = (hornH(st.thetaColl + st.cyclicAmp) - hornH(st.thetaColl - st.cyclicAmp)) / 2;
    st.swashTilt = Math.atan2(dh, HORN_R);
    const n = azDir(st.swashLowAz).multiplyScalar(Math.sin(st.swashTilt)).add(V(0, Math.cos(st.swashTilt), 0)).normalize();
    const qTilt = new THREE.Quaternion().setFromUnitVectors(Yax, n);
    stat.position.set(0, st.swashH, 0);
    stat.quaternion.copy(qTilt);
    rot.position.set(0, st.swashH, 0);
    rot.quaternion.copy(qTilt).multiply(new THREE.Quaternion().setFromAxisAngle(Yax, rotorAngle));
    uniball.position.set(0, st.swashH - 0.05, 0);
    root.updateMatrixWorld(true);

    // ---- flapping response: disc low point 90 deg after minimum pitch (= stick direction) ----
    const minPitchAz = st.swashLowAz - HORN_LEAD;
    const discAz = minPitchAz + Math.PI / 2;
    const target = new THREE.Vector2(Math.cos(discAz), Math.sin(discAz)).multiplyScalar(st.cyclicAmp);
    // The real disc settles in ~1/5 of a rotor turn. In slow motion that can be many seconds,
    // which looks like a stuck tilt, so it also settles within ~0.5 s of screen time.
    const k = 1 - Math.exp(-Math.max(Math.abs(dTheta) / (2 * Math.PI * 0.2), dt / 0.12));
    st.discTilt.lerp(target, k);
    st.coning = deg(1.5) + deg(3.5) * c.collective;
    const dA = st.discTilt.length(), dAz = Math.atan2(st.discTilt.y, st.discTilt.x);

    // ---- blades: pitch from the actual pitch-link height, flap from disc tilt ----
    blades.forEach((b, i) => {
      const lugW = lugs[i].p();
      const hTip = hornH0 + (lugW.y - lugY0);
      b.pitch = pitchFromHornH(hTip);
      const az = rotorAngle + b.phi;
      b.flap = st.coning - dA * Math.cos(wrap(az - dAz));
      b.lag = -deg(1.2) - deg(1.5) * c.collective; // steady lag from drag / torque
      b.hinge.rotation.set(b.pitch, b.lag, b.flap);
      st.bladePitch[i] = b.pitch;
    });
    root.updateMatrixWorld(true);

    // ---- linkages ----
    // every rod end sits on its bolt: link.set(boltA, boltB, axisA, axisB)
    blades.forEach((b, i) => {
      pitchLinks[i].set(lugs[i].p(), b.jHorn.p(), lugs[i].ax(), b.jHorn.ax());
      dampers[i].set(b.jDampHub.p(), b.jDampSp.p(), b.jDampHub.ax(), b.jDampSp.ax());
    });
    for (const s of servos) s.sv.set(s.jTop.p(), s.jTop.ax(), s.jBase.ax());
    // scissors: two links sharing a knee bolt; the eyes sit side by side on that bolt
    const scissors = (L, kb, jA, jB, l, out) => {
      const a = jA.p(), b = jB.p();
      const kn = knee(a, b, l, out);
      const ax = jA.ax();
      L.a.set(a, kn.clone().addScaledVector(ax, 0.055), ax, ax);
      L.b.set(kn.clone().addScaledVector(ax, -0.055), b, ax, jB.ax());
      kb.position.copy(kn);
      kb.quaternion.setFromUnitVectors(Yax, ax);
    };
    scissors(rs, rsKnee, jCollar, jRotSwash, 0.62, azDir(rotorAngle + rotScissorAz));
    scissors(ss, ssKnee, jDeckScissor, jStatSwash, 1.62, azDir(statScissorAz));
    // tip-path plane
    tppG.position.set(0, R.hubY + (R.tip - 0.6) * Math.sin(st.coning), 0);
    const tn = azDir(dAz).multiplyScalar(Math.sin(dA)).add(V(0, Math.cos(dA), 0)).normalize();
    tppG.quaternion.setFromUnitVectors(Yax, tn);
  }

  // joint pairs for verification (developer tool)
  const jointPairs = [
    ...blades.map((b, i) => ({ name: `pitch rod ${i + 1}`, A: lugs[i], B: b.jHorn, link: pitchLinks[i] })),
    ...blades.map((b, i) => ({ name: `damper ${i + 1}`, A: b.jDampHub, B: b.jDampSp, link: dampers[i] })),
    ...servos.map((s, i) => ({ name: `servo ${i + 1}`, A: s.jBase, B: s.jTop, link: s.sv })),
    { name: 'rot. scissors', A: jCollar, B: jRotSwash, link: rs.a, link2: rs.b },
    { name: 'stat. scissors', A: jDeckScissor, B: jStatSwash, link: ss.a, link2: ss.b },
  ];

  return {
    head, anchors, update, state: st, jointPairs, servos,
    setTpp(v) { showTpp = v; tppG.visible = v; },
    discPick,
    discGrabbable: () => tppG.visible,
    discRadius: R.tip - 0.6,
    discCentreY: () => tppG.position.y,
    /**
     * Back-drive: given the wanted disc tilt (x: a·cos(lowAz), y: a·sin(lowAz), radians), set the
     * cyclic STICK so that stick + mixing-unit inputs produce exactly that tilt.
     */
    setDiscTarget(T) {
      const c = st.controls;
      const a = Math.min(T.length(), CYC_MAX);
      const az = Math.atan2(T.y, T.x);
      const m = a / CYC_MAX;
      let lon = m * Math.cos(az), lat = -m * Math.sin(az);          // effective cyclic wanted
      if (c.mixing) {
        lon -= MIX.collToPitch * (c.collective - 0.35) - MIX.yawToPitch * st.tailPitch;
        lat -= -MIX.collToRoll * (c.collective - 0.35);
      }
      const s = Math.hypot(lon, lat);
      if (s > 1) { lon /= s; lat /= s; }                             // stick travel limit
      c.lon = lon; c.lat = lat;
    },
  };
}
