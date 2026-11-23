// UH-60 Black Hawk main transmission + engines + tail takeoff.
// Tooth counts and speeds: see RESEARCH.md (section A2).
// World axes (before the 3-degree transmission tilt): +X forward, +Y up, +Z right.
import * as THREE from 'three';
import {
  externalRimGeometry, internalRingGeometry, bevelGeometry, gearInfo,
} from './gears.js';
import {
  MAT, shadow, cylZ, tubeZ, boltRing, flexCoupling, hangerBearing, engineHousing,
} from './parts.js';

const V = (x, y, z) => new THREE.Vector3(x, y, z);
const Z = V(0, 0, 1);
const deg = THREE.MathUtils.degToRad;

export const MAST_RPM = 258;

export const TEETH = {
  inputPinion: 22, inputGear: 80,     // input module bevel (3.636:1)
  mainPinion: 17, mainGear: 81,       // main module bevel (4.765:1)
  sun: 62, planet: 83, ring: 228,     // planetary (1 + 228/62 = 4.677:1)
  planets: 5,
  tailPinion: 24,                     // tail takeoff (assumed, see RESEARCH.md)
  accPinion: 22,                      // accessory drive (assumed)
};

/** A rotating part: a fixed frame (axis = local Z) and a spinner rotating about it. */
class Rotor {
  constructor(parent, pos, axis, { z = 1, info = null, side = 'rotor', name = '' } = {}) {
    this.frame = new THREE.Group();
    this.frame.position.copy(pos);
    this.frame.quaternion.setFromUnitVectors(Z, axis.clone().normalize());
    parent.add(this.frame);
    this.spinner = new THREE.Group();
    this.frame.add(this.spinner);
    this.z = z;
    this.info = info || gearInfo({ z, m: 1, kind: 'spur' });
    this.k = 0;       // angular speed relative to the mast (signed, about local Z)
    this.phase = 0;
    this.angle = 0;
    this.side = side; // 'rotor' | 'eng1' | 'eng2' | 'carrier-rel'
    this.name = name;
  }
  add(o) { this.spinner.add(o); return o; }
  axisWorld() { return Z.clone().applyQuaternion(this.frame.getWorldQuaternion(new THREE.Quaternion())); }
  centerWorld() { return this.frame.getWorldPosition(new THREE.Vector3()); }
  localPolar(P) {
    this.frame.updateWorldMatrix(true, false);
    const p = P.clone().applyMatrix4(this.frame.matrixWorld.clone().invert());
    return { phi: Math.atan2(p.y, p.x), zl: p.z };
  }
}

/** Solve speed (direction from contact velocity, magnitude from tooth ratio) and tooth phase. */
function mesh(A, B, P, kA = A.k) {
  const vA = A.axisWorld().cross(P.clone().sub(A.centerWorld())).multiplyScalar(kA);
  const u = B.axisWorld().cross(P.clone().sub(B.centerWorld()));
  const sign = Math.sign(vA.dot(u)) || 1;
  const kB = sign * Math.abs(kA) * (A.z / B.z);
  const a = A.localPolar(P), b = B.localPolar(P);
  const phiA = a.phi - A.info.twistAt(a.zl);
  const phiB = b.phi - B.info.twistAt(b.zl);
  const phase = phiB - Math.PI / B.z - (kB / kA) * (phiA - A.phase) + B.info.twistAt(0) * 0;
  return { k: kB, phase };
}

// Spiral bevel teeth: the same spiral angle on both gears of a pair, opposite hands.
// Angular twist rate = tan(spiral angle) / mean pitch radius, so the tangential sweep
// along the face is identical on gear and pinion.
const params = new URLSearchParams(location.search);
const SPIRAL = deg(Number(params.get('psi') ?? 30));
const GEAR_HAND = Number(params.get('gh') ?? 1);
const PINION_HAND = Number(params.get('ph') ?? -1);
function spiralRate(z, m, faceWidth, delta, hand) {
  const r = (m * z) / 2;
  const A = r / Math.sin(delta);
  return (hand * Math.tan(SPIRAL)) / (r * (1 - faceWidth / (2 * A)));
}

function bevelGear(rotor, { z, m, faceWidth, delta, rimDepth, spiral, mat = MAT.steel }) {
  spiral = spiralRate(z, m, faceWidth, delta, spiral > 0 ? GEAR_HAND : PINION_HAND);
  const geo = bevelGeometry({ z, m, faceWidth, delta, rimDepth, spiralRate: spiral });
  rotor.info = gearInfo({ z, m, kind: 'bevel', spiralRate: spiral, faceWidth, delta });
  rotor.z = z;
  const mesh = shadow(new THREE.Mesh(geo, mat));
  rotor.add(mesh);
  return mesh;
}

export { Rotor, mesh, bevelGear };

/**
 * Airframe part of the tail drive: starts at the first flexible disc coupling behind the tail
 * takeoff (that coupling takes up the 3° angle between the tilted transmission and the level
 * tail drive), runs aft to x = endX. Six sections, disc couplings, oil-cooler blower, four
 * viscous-damped hanger bearings (TM 2.49). Returns the shaft Rotor (axis +X, shaft along -Z).
 */
export function buildTailDriveShaft(air, tx, endX = -17.93) {
  const tp = tx.debug.tp;
  tx.root.updateMatrixWorld(true);
  air.updateMatrixWorld(true);
  const start = air.worldToLocal(tp.frame.localToWorld(V(0, 0, -0.95)));
  start.z = 0;
  const shaft = new Rotor(air, start, V(1, 0, 0), { name: 'Tail drive shaft', z: tp.z });
  shaft.k = tp.k;
  shaft.angle = tp.angle;
  const len = start.x - endX;
  shaft.add(cylZ(0.15, -0.11, -len, MAT.alu));
  for (const x of [-6.73, -10.33, -13.93, -17.53]) shaft.add(flexCoupling(-(start.x - x), 0.15, MAT.paint));
  const fan = new THREE.Group();
  for (let i = 0; i < 14; i++) {
    const a = (i / 14) * Math.PI * 2;
    const blade = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.42, 0.16), MAT.steel));
    blade.position.set(Math.cos(a) * 0.42, Math.sin(a) * 0.42, 0);
    blade.rotation.set(0, 0, a - Math.PI / 2);
    blade.rotateY(0.6);
    fan.add(blade);
  }
  fan.add(cylZ(0.22, -0.1, 0.1, MAT.steelDark));
  fan.position.z = -(start.x - -4.83);
  shaft.add(fan);
  const shroud = tubeZ(0.75, 0.68, -0.25, 0.25, MAT.glass);
  shroud.rotation.y = -Math.PI / 2;
  shroud.position.set(-4.83, start.y, 0);
  shroud.userData.support = true;
  air.add(shroud);
  for (const x of [-6.1, -9.7, -13.3, -16.9]) {
    const hb = hangerBearing(0, 0.15);
    hb.rotation.y = Math.PI / 2;
    hb.position.set(x, start.y, 0);
    air.add(hb);
  }
  tx.rotors.push(shaft);
  return {
    shaft, y: start.y,
    anchors: { tail: V(-9.0, start.y + 0.5, 0), blower: V(-4.83, start.y + 0.95, 0) },
  };
}

/** Apply the built-in forward tilt of the main transmission (rotation about +Z at `pivot`). */
export function tiltTransmission(tx, deg3 = 3, pivot = V(0, -0.85, 0)) {
  const q = new THREE.Quaternion().setFromAxisAngle(V(0, 0, 1), -deg(deg3));
  tx.root.quaternion.copy(q);
  tx.root.position.copy(pivot).sub(pivot.clone().applyQuaternion(q));
  tx.root.updateMatrixWorld(true);
}

/**
 * Bevel pair on any shaft angle. O = common cone apex, u1/u2 = unit directions from the apex
 * toward each gear (along its shaft). Returns the two Rotors plus the pitch contact point.
 * The caller sets g1.k / g1.phase, then calls pair.solve() to get g2's speed and tooth phase.
 */
export function bevelPair(parent, O, u1, u2, { z1, z2, m, faceWidth, rim1 = 0.2, rim2 = 0.14, names = [] }) {
  const sigma = u1.angleTo(u2);
  const d1 = Math.atan2(Math.sin(sigma), z2 / z1 + Math.cos(sigma));
  const d2 = sigma - d1;
  const r1 = (m * z1) / 2;
  const A = r1 / Math.sin(d1);
  const g1 = new Rotor(parent, O.clone().addScaledVector(u1, A * Math.cos(d1)), u1.clone().negate(), { name: names[0] || '' });
  const g2 = new Rotor(parent, O.clone().addScaledVector(u2, A * Math.cos(d2)), u2.clone().negate(), { name: names[1] || '' });
  bevelGear(g1, { z: z1, m, faceWidth, delta: d1, rimDepth: rim1, spiral: -1 });
  bevelGear(g2, { z: z2, m, faceWidth, delta: d2, rimDepth: rim2, spiral: 1 });
  const e = u2.clone().addScaledVector(u1, -u2.dot(u1)).normalize();
  const P = O.clone().addScaledVector(u1.clone().multiplyScalar(Math.cos(d1)).addScaledVector(e, Math.sin(d1)), A);
  return {
    g1, g2, P, A, d1, d2,
    solve() {
      parent.updateMatrixWorld(true);
      const s = mesh(g1, g2, P);
      g2.k = s.k; g2.phase = s.phase;
    },
  };
}

export function buildUH60Transmission() {
  const root = new THREE.Group();
  root.name = 'UH-60 drivetrain';
  const rotors = [];
  const R = (...a) => { const r = new Rotor(root, ...a); rotors.push(r); return r; };
  const anchors = {}; // label anchor points (root-local)

  // ---------------- main module: main bevel gear (81T) ----------------
  const mMain = 0.06;
  const rMainGear = (mMain * TEETH.mainGear) / 2;            // 2.43
  const dMainGear = Math.atan(TEETH.mainGear / TEETH.mainPinion);
  const dMainPin = Math.PI / 2 - dMainGear;
  const Amain = rMainGear / Math.sin(dMainGear);
  const apexY = Amain * Math.cos(dMainGear);                 // ~0.51
  const fwMain = 0.62;

  const mainGear = R(V(0, 0, 0), V(0, 1, 0), { name: 'Main bevel gear' });
  bevelGear(mainGear, { z: TEETH.mainGear, m: mMain, faceWidth: fwMain, delta: dMainGear, rimDepth: 0.16, spiral: 1 });
  // web, hub and bolts under the bevel gear
  mainGear.add(tubeZ(2.2, 0.55, -0.36, -0.26, MAT.steelDark));
  mainGear.add(cylZ(0.62, -0.62, 0.12, MAT.steel));
  mainGear.add(boltRing(16, 1.55, -0.26, 0.05, MAT.steelDark, 1));
  mainGear.add(boltRing(10, 0.45, 0.12, 0.04, MAT.steelDark, 1));
  mainGear.k = 1 + TEETH.ring / TEETH.sun; // same shaft as sun gear
  anchors.mainGear = V(1.6, -0.2, 1.7);

  // ---------------- planetary stage ----------------
  const mPl = 0.022;
  const ySun = 1.6;
  const wPl = 0.36;
  const sun = R(V(0, ySun, 0), V(0, 1, 0), { z: TEETH.sun, name: 'Sun gear' });
  sun.info = gearInfo({ z: TEETH.sun, m: mPl, kind: 'spur' });
  sun.add(shadow(new THREE.Mesh(externalRimGeometry({ z: TEETH.sun, m: mPl, width: wPl, innerR: 0.18 }), MAT.steel)));
  sun.add(cylZ(0.2, -ySun + 0.1, 0.05, MAT.steel));
  sun.k = mainGear.k;
  sun.phase = 0;

  const carrier = R(V(0, ySun, 0), V(0, 1, 0), { name: 'Planet carrier' });
  carrier.k = 1; // the mast
  // carrier plate: hub + 5 arms + pads
  const plate = new THREE.Group();
  const rOrbit = (mPl * (TEETH.sun + TEETH.planet)) / 2;
  plate.add(cylZ(0.62, 0.22, 0.36, MAT.paint));
  for (let i = 0; i < TEETH.planets; i++) {
    const a = (i / TEETH.planets) * Math.PI * 2;
    const arm = shadow(new THREE.Mesh(new THREE.BoxGeometry(rOrbit, 0.34, 0.12), MAT.paint));
    arm.position.set(Math.cos(a) * rOrbit / 2, Math.sin(a) * rOrbit / 2, 0.29);
    arm.rotation.z = a;
    plate.add(arm);
    const pad = cylZ(0.27, 0.22, 0.36, MAT.paint);
    pad.position.set(Math.cos(a) * rOrbit, Math.sin(a) * rOrbit, 0);
    plate.add(pad);
    const pin = cylZ(0.15, -0.22, 0.4, MAT.steelDark);   // planet pin = planet bore (bearing seat)
    pin.position.copy(pad.position);
    plate.add(pin);
  }
  carrier.add(plate);
  carrier.add(boltRing(10, 0.48, 0.36, 0.045, MAT.steelDark, 1));
  // mast (with spline ridges so rotation is visible)
  const mastTop = 6.2;
  carrier.add(cylZ(0.34, 0.36, mastTop, MAT.alu));
  carrier.add(cylZ(0.42, 0.36, 0.9, MAT.steel));
  for (let i = 0; i < 6; i++) {
    const a = (i / 6) * Math.PI * 2;
    const ridge = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 3.0), MAT.steelDark));
    ridge.position.set(Math.cos(a) * 0.345, Math.sin(a) * 0.345, 2.6);
    ridge.rotation.z = a;
    carrier.add(ridge);
  }
  carrier.add(cylZ(0.62, mastTop - 0.08, mastTop + 0.08, MAT.steel));
  carrier.add(boltRing(12, 0.52, mastTop + 0.08, 0.045, MAT.steelDark, 1));
  anchors.mast = V(0, ySun + 4.2, 0);
  anchors.planetary = V(-1.6, ySun + 0.3, -1.9);

  // planets: children of the carrier spinner, spinning relative to the carrier
  root.updateMatrixWorld(true);
  const planets = [];
  const sunRel = { k: sun.k - carrier.k };
  for (let i = 0; i < TEETH.planets; i++) {
    const a = (i / TEETH.planets) * Math.PI * 2;
    const world = V(Math.cos(a) * rOrbit, ySun, -Math.sin(a) * rOrbit);
    const local = carrier.spinner.worldToLocal(world.clone());
    const p = new Rotor(carrier.spinner, local, Z, { z: TEETH.planet, name: `Planet ${i + 1}` });
    p.info = gearInfo({ z: TEETH.planet, m: mPl, kind: 'spur' });
    p.add(shadow(new THREE.Mesh(externalRimGeometry({ z: TEETH.planet, m: mPl, width: wPl, innerR: 0.6 }), MAT.steel)));
    p.add(tubeZ(0.62, 0.15, -0.08, 0.08, MAT.steelDark));
    p.add(boltRing(8, 0.38, 0.08, 0.035, MAT.steel, 1));
    p.add(boltRing(8, 0.38, -0.08, 0.035, MAT.steel, -1));
    p.side = 'carrier-rel';
    root.updateMatrixWorld(true);
    const P = V(Math.cos(a) * (mPl * TEETH.sun) / 2, ySun, -Math.sin(a) * (mPl * TEETH.sun) / 2);
    const s = mesh(sun, p, P, sunRel.k);
    p.k = s.k;
    p.phase = s.phase;
    planets.push(p);
    rotors.push(p);
  }
  // fixed ring gear: phase solved from planet 1 (ring turns -1 relative to the carrier)
  const ringR = (mPl * TEETH.ring) / 2;
  const ring = new Rotor(root, V(0, ySun, 0), V(0, 1, 0), { z: TEETH.ring, name: 'Ring gear (fixed)' });
  ring.info = gearInfo({ z: TEETH.ring, m: mPl, kind: 'spur' });
  ring.add(shadow(new THREE.Mesh(internalRingGeometry({ z: TEETH.ring, m: mPl, width: wPl + 0.06, outerR: ringR + 0.16 }), MAT.steel)));
  ring.add(tubeZ(ringR + 0.3, ringR + 0.1, -0.24, -0.12, MAT.steel));
  ring.add(boltRing(36, ringR + 0.22, -0.12, 0.035, MAT.steelDark, 1));
  root.updateMatrixWorld(true);
  {
    const P = V(ringR, ySun, 0);
    const pr = planets[0];
    const vPlanet = pr.axisWorld().cross(P.clone().sub(pr.frame.getWorldPosition(V(0, 0, 0)))).multiplyScalar(pr.k);
    void vPlanet;
    const a = pr.localPolar(P), b = ring.localPolar(P);
    const kRing = -1; // relative to carrier
    ring.phase = b.phi - Math.PI / ring.z - (kRing / pr.k) * (a.phi - pr.phase);
    ring.k = 0;
    ring.angle = ring.phase;
    ring.spinner.rotation.z = ring.phase;
  }
  ring.side = 'fixed';

  // ---------------- main bevel pinions + input modules (left and right) ----------------
  const mIn = 0.045;
  const rInGear = (mIn * TEETH.inputGear) / 2;              // 1.8
  const dInGear = Math.atan(TEETH.inputGear / TEETH.inputPinion);
  const dInPin = Math.PI / 2 - dInGear;
  const Ain = rInGear / Math.sin(dInGear);
  const fwIn = 0.46;
  const zInGear = 4.55;                                      // back face of input bevel gear
  const zApexIn = zInGear - Ain * Math.cos(dInGear);         // engine centre-line
  const engines = [];

  for (const side of [1, -1]) {
    const tag = side > 0 ? 'R' : 'L';
    const engSide = side > 0 ? 'eng2' : 'eng1';    // No.1 engine = left, No.2 = right
    // main bevel pinion 17T
    const mp = R(V(0, apexY, side * rMainGear), V(0, 0, -side), { name: `Main bevel pinion ${tag}` });
    bevelGear(mp, { z: TEETH.mainPinion, m: mMain, faceWidth: fwMain, delta: dMainPin, rimDepth: 0.2, spiral: -1 });
    mp.add(cylZ(0.2, 0.25, -(zInGear - rMainGear) - 0.25, MAT.steel));
    root.updateMatrixWorld(true);
    let s = mesh(mainGear, mp, V(0, 0, side * rMainGear));
    mp.k = s.k; mp.phase = s.phase;

    // input bevel gear 80T (same shaft as the main pinion)
    const ig = R(V(0, apexY, side * zInGear), V(0, 0, -side), { name: `Input bevel gear ${tag}` });
    bevelGear(ig, { z: TEETH.inputGear, m: mIn, faceWidth: fwIn, delta: dInGear, rimDepth: 0.12, spiral: 1 });
    ig.add(tubeZ(1.55, 0.22, -0.3, -0.22, MAT.steelDark));
    ig.add(cylZ(0.32, -0.42, 0.1, MAT.steel));
    ig.add(boltRing(12, 1.05, -0.22, 0.04, MAT.steelDark, 1));
    ig.k = mp.k;
    ig.phase = 0;

    // input bevel pinion 22T (engine side, through the freewheel)
    const ip = R(V(-Ain * Math.cos(dInPin), apexY, side * zApexIn), V(1, 0, 0), { name: `Input bevel pinion ${tag}` });
    bevelGear(ip, { z: TEETH.inputPinion, m: mIn, faceWidth: fwIn, delta: dInPin, rimDepth: 0.2, spiral: -1 });
    root.updateMatrixWorld(true);
    s = mesh(ig, ip, V(-rInGear, apexY, side * zInGear));
    ip.k = s.k; ip.phase = s.phase;
    // freewheel inner race + sprags (rotor side)
    ip.add(cylZ(0.13, 0.1, -0.62, MAT.steel));
    ip.add(cylZ(0.17, -0.62, -1.38, MAT.steel));
    const sprags = [];
    for (let i = 0; i < 16; i++) {
      const a = (i / 16) * Math.PI * 2;
      const holder = new THREE.Group();
      holder.position.set(Math.cos(a) * 0.225, Math.sin(a) * 0.225, -1.0);
      holder.rotation.z = a;
      const sp = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.035, 0.58), MAT.sprag));
      holder.add(sp);
      ip.add(holder);
      sprags.push(sp);
    }
    // accessory drive pinion (forward side of the input gear) + accessory module
    const ap = R(V(Ain * Math.cos(dInPin), apexY, side * zApexIn), V(-1, 0, 0), { name: `Accessory drive ${tag}` });
    bevelGear(ap, { z: TEETH.accPinion, m: mIn, faceWidth: fwIn, delta: dInPin, rimDepth: 0.2, spiral: -1 });
    ap.add(cylZ(0.12, 0.1, -1.0, MAT.steel));
    root.updateMatrixWorld(true);
    s = mesh(ig, ap, V(rInGear, apexY, side * zInGear));
    ap.k = s.k; ap.phase = s.phase;
    const acc = new THREE.Group();
    const accBox = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.95, 0.9), MAT.paint));
    acc.add(accBox);
    const gen = cylZ(0.34, 0.45, 1.25, MAT.paintLight);
    gen.rotation.y = side > 0 ? 0 : Math.PI; // generator points outboard
    acc.add(gen);
    const pump = cylZ(0.22, 0.45, 0.95, MAT.steelDark);
    pump.rotation.x = -Math.PI / 2;
    acc.add(pump);
    acc.add(boltRing(14, 0.35, 0.45, 0.03, MAT.steelDark, 1));
    acc.position.set(Ain * Math.cos(dInPin) + 1.45, apexY, side * zApexIn);
    root.add(acc);
    anchors[`acc${tag}`] = acc.position.clone().add(V(0, 0.6, 0));

    // engine side of the freewheel (outer race), high-speed shaft, coupling
    const xEngFront = -3.75;
    const eo = R(V(-Ain * Math.cos(dInPin), apexY, side * zApexIn), V(1, 0, 0), { name: `Engine output ${tag}`, side: engSide });
    eo.add(cylZ(0.3, -0.6, -1.42, MAT.glass, { open: true, seg: 64 }));
    eo.add(tubeZ(0.32, 0.17, -0.6, -0.66, MAT.steelDark));
    eo.add(tubeZ(0.32, 0.17, -1.36, -1.42, MAT.steelDark));
    eo.add(boltRing(8, 0.27, -1.42, 0.03, MAT.steel, -1));
    const l0 = 1.42, l1 = -xEngFront - Ain * Math.cos(dInPin);
    eo.add(cylZ(0.12, -l0, -l1 - 0.1, MAT.steel));
    eo.add(flexCoupling(-(l0 + l1) / 2 - 0.05, 0.1, MAT.paint));
    eo.k = ip.k;
    eo.phase = 0;

    const eng = engineHousing();
    eng.rotation.y = Math.PI / 2; // build axis -Z -> world -X
    eng.position.set(xEngFront, apexY, side * zApexIn);
    root.add(eng);
    engines.push({ side: engSide, group: eng, sprags, inner: ip, outer: eo });

    anchors[`eng${tag}`] = V(xEngFront - 2.8, apexY + 1.05, side * zApexIn);
    anchors[`fw${tag}`] = V(-Ain * Math.cos(dInPin) - 1.0, apexY + 0.38, side * zApexIn);
    anchors[`in${tag}`] = V(-0.6, apexY + 1.6, side * (zInGear - 0.2));
    anchors[`mp${tag}`] = V(0, apexY + 0.55, side * (rMainGear + 0.8));
  }

  // ---------------- tail rotor takeoff ----------------
  const rTail = (mMain * TEETH.tailPinion) / 2;
  const dTail = Math.atan(TEETH.tailPinion / TEETH.mainGear);
  const tp = R(V(-rMainGear, rTail, 0), V(1, 0, 0), { name: 'Tail takeoff pinion' });
  bevelGear(tp, { z: TEETH.tailPinion, m: mMain, faceWidth: 0.5, delta: dTail, rimDepth: 0.22, spiral: -1 });
  root.updateMatrixWorld(true);
  {
    const s = mesh(mainGear, tp, V(-rMainGear, 0, 0));
    tp.k = s.k; tp.phase = s.phase;
  }
  // takeoff stub (tilts with the transmission) ending in the first flexible disc coupling;
  // the rest of the tail drive shaft belongs to the airframe (buildTailDriveShaft)
  tp.add(cylZ(0.15, 0.1, -0.55, MAT.steel));
  tp.add(tubeZ(0.42, 0.15, -0.55, -0.7, MAT.steel)); // takeoff flange (gust-lock teeth)
  for (let i = 0; i < 24; i++) {
    const a = (i / 24) * Math.PI * 2;
    const tooth = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.08, 0.12), MAT.steelDark));
    tooth.position.set(Math.cos(a) * 0.45, Math.sin(a) * 0.45, -0.62);
    tooth.rotation.z = a;
    tp.add(tooth);
  }
  tp.add(cylZ(0.15, -0.7, -0.95, MAT.alu));
  tp.add(flexCoupling(-0.95, 0.15, MAT.paint));
  anchors.tailPinion = V(-rMainGear + 0.2, rTail + 0.75, 0);

  // Built in transmission axes (mast vertical). main.js tilts this whole group 3° forward
  // relative to the airframe (TM 2.47: "built-in 3° forward tilt") with tiltTransmission().

  // ---------------- runtime ----------------
  const engineFactor = { eng1: 1, eng2: 1 };
  const engineTarget = { eng1: 1, eng2: 1 };
  for (const r of rotors) r.angle = r.phase;

  function update(dTheta, dt) {
    for (const k of ['eng1', 'eng2']) {
      const tgt = engineTarget[k];
      const rate = tgt > engineFactor[k] ? 0.6 : 0.35; // spool up / run down per second
      engineFactor[k] += Math.sign(tgt - engineFactor[k]) * Math.min(Math.abs(tgt - engineFactor[k]), rate * dt);
    }
    for (const r of rotors) {
      const f = r.side === 'eng1' || r.side === 'eng2' ? engineFactor[r.side] : 1;
      r.angle += r.k * dTheta * f;
      r.spinner.rotation.z = r.angle;
    }
    // sprags: wedged (driving) vs. tipped back (overrunning)
    for (const e of engines) {
      const f = engineFactor[e.side];
      const tilt = THREE.MathUtils.lerp(deg(4), deg(22), f >= 0.999 ? 1 : f * 0.4);
      for (const sp of e.sprags) sp.rotation.z = tilt;
    }
  }

  const k = (r) => Math.abs(r.k);
  const speeds = () => ({
    eng1: k(rotors.find((r) => r.side === 'eng1')) * MAST_RPM * engineFactor.eng1,
    eng2: k(rotors.find((r) => r.side === 'eng2')) * MAST_RPM * engineFactor.eng2,
    inputPinion: k(rotors.find((r) => r.name === 'Input bevel pinion R')) * MAST_RPM,
    inputOut: k(rotors.find((r) => r.name === 'Input bevel gear R')) * MAST_RPM,
    sun: k(sun) * MAST_RPM,
    mast: MAST_RPM,
    tail: k(tp) * MAST_RPM,
  });

  return {
    root, rotors, anchors, update, speeds,
    setEngine(which, on) { engineTarget[which] = on ? 1 : 0; },
    engineFactor,
    debug: { mainGear, sun, carrier, planets, ring, tp, engines },
  };
}
