// GE T700 turboshaft — cutaway internals (TM 1-1520-237-10 §2.19):
//   inlet particle separator (swirl vanes, scroll, blower) · 5 axial + 1 centrifugal compressor
//   (variable inlet guide vanes) · annular combustor · 2-stage gas-generator turbine (drives the
//   compressor) · 2-stage free power turbine (tip-shrouded) whose shaft runs FORWARD through the
//   gas-generator shaft to the front output · accessory gearbox driven by a radial shaft.
// The gas generator (Ng) and power turbine (Np) are not mechanically connected: Np is governed at
// 100 % (≈20,900 rpm), Ng rises and falls with power demand. Ng's absolute rpm was not found in the
// sources used, so it is shown as a percentage; on screen it spins ≈2.1× the power turbine.
// Built in the engine housing's local frame: axis = local Z, front face at z = 0, aft = -Z.
import * as THREE from 'three';
import { MAT, shadow, cylZ, tubeZ } from './parts.js';

const NG_VISUAL_RATIO = 2.14;    // on-screen spin of the gas generator relative to the power turbine (approx.)

function bladeRing(n, rHub, rTip, chord, twist, mat, thick = 0.012) {
  const g = new THREE.Group();
  const geo = new THREE.BoxGeometry(rTip - rHub, chord, thick);
  geo.translate((rHub + rTip) / 2, 0, 0);
  const im = new THREE.InstancedMesh(geo, mat, n);
  const m = new THREE.Matrix4(), q = new THREE.Quaternion(), e = new THREE.Euler();
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    e.set(twist, 0, a, 'ZYX');          // around the axis, then twisted about the blade's span
    q.setFromEuler(e);
    m.compose(new THREE.Vector3(), q, new THREE.Vector3(1, 1, 1));
    im.setMatrixAt(i, m);
  }
  im.castShadow = true;
  g.add(im);
  g.add(cylZ(rHub, -chord * 0.6, chord * 0.6, MAT.steel, { seg: 40 }));     // disk rim
  return g;
}
function bladeRingZ(n, rHub, rTip, z, chord, twist, mat) {
  const r = bladeRing(n, rHub, rTip, chord, twist, mat);
  // blades were built in the XY plane with chord along Y: rotate so chord runs along the axis (Z)
  r.children[0].geometry.rotateX(Math.PI / 2);
  r.position.z = z;
  return r;
}

/** Add the cutaway internals to one engine. `eng` = housing group, `out` = its output Rotor (eo). */
export function addT700Internals(eng, out, side) {
  const hot = new THREE.MeshPhysicalMaterial({ color: 0xff8a3a, emissive: 0xff5a10, emissiveIntensity: 1.6, transparent: true, opacity: 0.55, depthWrite: false });
  const blade = MAT.steel, bladeHot = new THREE.MeshPhysicalMaterial({ color: 0xb48d6a, metalness: 1, roughness: 0.35 });

  // ---------------- static internals (stators, combustor, frames) ----------------
  const stat = new THREE.Group();
  eng.add(stat);
  stat.add(bladeRingZ(22, 0.36, 0.6, -1.6, 0.06, 0.5, MAT.steelDark));                  // variable inlet guide vanes
  const AX = [-1.75, -1.92, -2.09, -2.26, -2.43];                                       // 5 axial rotor stages
  AX.forEach((z, i) => { if (i < 4) stat.add(bladeRingZ(34, 0.32 + i * 0.02, 0.62 - i * 0.025, z + 0.085, 0.05, -0.45, MAT.steelDark)); });
  // (outer shells of the internals are cut open together with the casing: userData.cut)
  const cutOpen = (m) => { m.userData.cut = true; return m; };
  // diffuser after the centrifugal impeller
  stat.add(cutOpen(tubeZ(0.74, 0.62, -2.62, -2.82, MAT.steelDark)));
  // annular combustor: inner and outer liners + dome with fuel injectors
  stat.add(cutOpen(tubeZ(0.74, 0.715, -2.85, -3.52, MAT.steelDark)));
  stat.add(tubeZ(0.41, 0.385, -2.85, -3.52, MAT.steelDark));
  stat.add(tubeZ(0.74, 0.385, -2.83, -2.87, MAT.steelDark));                            // dome
  for (let i = 0; i < 12; i++) {
    const a = (i / 12) * Math.PI * 2;
    const inj = cylZ(0.025, -2.75, -2.92, MAT.gold, { seg: 10 });
    inj.position.set(Math.cos(a) * 0.56, Math.sin(a) * 0.56, 0);
    stat.add(inj);
  }
  const flame = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.69, 0.69, 0.6, 48, 1, true), hot));
  flame.rotation.x = Math.PI / 2;
  flame.position.z = -3.2;
  stat.add(flame);
  const flameIn = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.43, 0.43, 0.6, 48, 1, true), hot));
  flameIn.rotation.x = Math.PI / 2;
  flameIn.position.z = -3.2;
  stat.add(flameIn);
  // turbine nozzle vanes (in front of each turbine rotor)
  for (const z of [-3.6, -3.79, -4.02, -4.22]) stat.add(bladeRingZ(30, 0.36, 0.66, z, 0.05, 0.55, MAT.steelDark));
  // exhaust frame struts + rear bearing support
  for (let i = 0; i < 6; i++) {
    const a = (i / 6) * Math.PI * 2;
    const s = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.56, 0.03, 0.3), MAT.steelDark));
    s.position.set(Math.cos(a) * 0.36, Math.sin(a) * 0.36, -4.85);
    s.rotation.z = a;
    stat.add(s);
  }
  stat.add(cylZ(0.13, -4.7, -5.0, MAT.steelDark));
  // front frame: output shaft bearing support, strut to the swirl frame hub
  stat.add(cylZ(0.12, -0.36, -0.6, MAT.steelDark));
  stat.add(tubeZ(0.42, 0.33, -1.4, -1.5, MAT.steelDark));
  // IPS collector scroll ring (dirt is thrown outward into it, then out through the blower)
  stat.add(cutOpen(tubeZ(0.94, 0.86, -0.7, -1.05, MAT.steelDark)));

  // ---------------- gas-generator spool (Ng): compressor + GG turbine on one hollow shaft ----------------
  const gg = new THREE.Group();
  eng.add(gg);
  gg.add(tubeZ(0.14, 0.085, -1.55, -3.95, MAT.steel));                                  // hollow GG shaft
  AX.forEach((z, i) => gg.add(bladeRingZ(30, 0.3 + i * 0.02, 0.6 - i * 0.025, z, 0.07, 0.6, blade)));
  {
    // centrifugal impeller: hub cone + radial vanes curving forward at the inducer
    const hub = new THREE.LatheGeometry([[0.12, -2.5], [0.3, -2.52], [0.5, -2.62], [0.62, -2.72], [0.62, -2.76], [0.12, -2.78]].map(([r, y]) => new THREE.Vector2(r, y)), 48);
    hub.rotateX(Math.PI / 2);                    // lathe axis Y -> engine axis Z (profile already at negative z)
    gg.add(shadow(new THREE.Mesh(hub, blade)));
    for (let i = 0; i < 18; i++) {
      const a = (i / 18) * Math.PI * 2;
      const v = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.012, 0.2), blade));
      v.position.set(Math.cos(a) * 0.44, Math.sin(a) * 0.44, -2.62);
      v.rotation.z = a;
      v.rotateX(0.35);
      gg.add(v);
    }
  }
  for (const z of [-3.69, -3.88]) gg.add(bladeRingZ(36, 0.34, 0.65, z, 0.07, -0.6, bladeHot));   // 2-stage GG turbine
  // radial drive shaft: from a bevel gear at the front of the GG spool straight up into the
  // accessory gearbox on top of the engine
  {
    const rds = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.75, 12), MAT.steel));
    rds.position.set(0, 0.5, -1.3);
    stat.add(rds);
    stat.add(cylZ(0.11, -1.25, -1.36, MAT.steelDark, { seg: 20 }));              // drive bevel on the GG shaft nose
  }

  // ---------------- power turbine spool (Np): 2 stages + shaft forward through the GG shaft ----------------
  const pt = new THREE.Group();
  eng.add(pt);
  pt.add(cylZ(0.065, 0.02, -4.45, MAT.alu));                                            // runs through the GG shaft
  for (const z of [-4.11, -4.31]) {
    const r = bladeRingZ(40, 0.34, 0.64, z, 0.07, -0.55, bladeHot);
    r.add(tubeZ(0.665, 0.64, -0.04, 0.04, MAT.steel));                                  // tip shroud
    pt.add(r);
  }

  // ---------------- airflow (cold -> compressed -> burning -> expanding) ----------------
  const N = 170;
  const path = (t) => {                                   // -> [z, r, colour heat 0..1]
    if (t < 0.12) return [THREE.MathUtils.lerp(0.25, -0.3, t / 0.12), 0.47, 0];
    if (t < 0.3) return [THREE.MathUtils.lerp(-0.3, -1.45, (t - 0.12) / 0.18), 0.5, 0];
    if (t < 0.48) return [THREE.MathUtils.lerp(-1.55, -2.45, (t - 0.3) / 0.18), THREE.MathUtils.lerp(0.47, 0.42, (t - 0.3) / 0.18), THREE.MathUtils.lerp(0, 0.25, (t - 0.3) / 0.18)];
    if (t < 0.56) return [THREE.MathUtils.lerp(-2.5, -2.72, (t - 0.48) / 0.08), THREE.MathUtils.lerp(0.25, 0.66, (t - 0.48) / 0.08), 0.35];
    if (t < 0.72) return [THREE.MathUtils.lerp(-2.85, -3.5, (t - 0.56) / 0.16), 0.56, THREE.MathUtils.lerp(0.5, 1, (t - 0.56) / 0.16)];
    if (t < 0.86) return [THREE.MathUtils.lerp(-3.55, -4.4, (t - 0.72) / 0.14), 0.5, THREE.MathUtils.lerp(1, 0.6, (t - 0.72) / 0.14)];
    return [THREE.MathUtils.lerp(-4.5, -5.7, (t - 0.86) / 0.14), THREE.MathUtils.lerp(0.48, 0.55, (t - 0.86) / 0.14), THREE.MathUtils.lerp(0.6, 0.3, (t - 0.86) / 0.14)];
  };
  const flow = new THREE.InstancedMesh(new THREE.SphereGeometry(0.03, 8, 6), new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.85, depthWrite: false }), N);
  flow.userData.overlay = true;
  flow.userData.cut = true;                // only the air in the open half is drawn
  flow.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(N * 3), 3);
  eng.add(flow);
  const seeds = Array.from({ length: N }, (_, i) => ({ t: i / N, a: Math.random() * Math.PI * 2, j: Math.random() * 0.06 - 0.03 }));
  const cold = new THREE.Color(0x7fc8ff), warm = new THREE.Color(0xfff1c0), fire = new THREE.Color(0xff5a1a), smoke = new THREE.Color(0x8a6f66);
  // dirt thrown out by the inlet particle separator
  const DUST = 18;
  const dust = new THREE.InstancedMesh(new THREE.SphereGeometry(0.022, 6, 4), new THREE.MeshBasicMaterial({ color: 0x9a6a3a }), DUST);
  dust.userData.overlay = true;
  dust.userData.cut = true;
  eng.add(dust);
  const dseed = Array.from({ length: DUST }, (_, i) => ({ t: i / DUST, a: Math.random() * Math.PI * 2 }));

  const m4 = new THREE.Matrix4(), col = new THREE.Color(), v = new THREE.Vector3(), ONE = new THREE.Vector3(1, 1, 1), q0 = new THREE.Quaternion();
  let ngAngle = 0, flowT = 0;
  const st = { ng: 0, cutaway: false };
  const internalMeshes = new Set();
  for (const g of [gg, pt, stat, flow, dust]) g.traverse((o) => { if (o.isMesh && !o.userData.cut) internalMeshes.add(o); });
  return {
    groups: { gg, pt, stat, flow, dust },
    internalMeshes,
    state: st,
    /** ef = engine running factor (0..1), demand = power demand 0..1 (collective), dTheta = mast angle step. */
    update(dTheta, dt, ef, demand) {
      st.ng = ef * (0.84 + 0.16 * demand);                           // Ng % / 100 (load-dependent)
      pt.rotation.z = out.angle - out.phase;                         // power turbine IS the output shaft
      ngAngle += out.k * NG_VISUAL_RATIO * dTheta * st.ng;
      gg.rotation.z = ngAngle;
      // airflow speed follows Ng (screen-time animation)
      flowT += dt * 0.16 * st.ng;
      for (let i = 0; i < N; i++) {
        const s = seeds[i];
        const t = (s.t + flowT) % 1;
        const [z, r, heat] = path(t);
        const sw = s.a + t * 9 * side;                               // swirl around the axis
        v.set(Math.cos(sw) * (r + s.j), Math.sin(sw) * (r + s.j), z);
        m4.compose(v, q0, st.ng > 0.02 ? ONE : new THREE.Vector3(0, 0, 0));
        flow.setMatrixAt(i, m4);
        if (heat < 0.35) col.copy(cold).lerp(warm, heat / 0.35);
        else if (heat < 0.85) col.copy(warm).lerp(fire, (heat - 0.35) / 0.5);
        else col.copy(fire);
        if (t > 0.86) col.lerp(smoke, (t - 0.86) / 0.14);
        flow.setColorAt(i, col);
      }
      flow.instanceMatrix.needsUpdate = true;
      flow.instanceColor.needsUpdate = true;
      for (let i = 0; i < DUST; i++) {
        const s = dseed[i];
        const t = (s.t + flowT * 1.3) % 1;
        const z = THREE.MathUtils.lerp(0.2, -0.88, Math.min(t * 1.6, 1));
        const r = t < 0.45 ? 0.5 : THREE.MathUtils.lerp(0.5, 0.9, (t - 0.45) / 0.55);
        const a = s.a + t * 6;
        v.set(Math.cos(a) * r, Math.sin(a) * r, z);
        m4.compose(v, q0, st.ng > 0.02 && st.cutaway ? ONE : new THREE.Vector3(0, 0, 0));
        dust.setMatrixAt(i, m4);
      }
      dust.instanceMatrix.needsUpdate = true;
      const flick = 0.8 + 0.25 * Math.sin(performance.now() * 0.02) * Math.sin(performance.now() * 0.013);
      hot.emissiveIntensity = 1.6 * flick * st.ng;
      hot.opacity = 0.55 * Math.min(1, st.ng * 1.2);
      flow.visible = st.cutaway;
    },
  };
}

/** Cut the housing open: clip everything in the housing group on one side of `plane` (world). */
export function setHousingCutaway(eng, internals, plane, on) {
  eng.traverse((o) => {
    if (!o.isMesh || internals.has(o)) return;
    if (!o.userData.cutMat) {
      o.userData.cutMat = o.material.clone();
      o.userData.cutMat.clippingPlanes = [plane];
      o.userData.cutMat.clipShadows = true;
      o.userData.cutMat.side = THREE.DoubleSide;
      o.userData.fullMat = o.material;
    }
    o.material = on ? o.userData.cutMat : o.userData.fullMat;
  });
}
