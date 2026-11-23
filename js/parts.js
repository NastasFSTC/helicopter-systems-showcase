// Materials and reusable mechanical parts (shafts, couplings, bolts, bearings, engine).
import * as THREE from 'three';

export const MAT = {
  steel: new THREE.MeshPhysicalMaterial({ color: 0xc8ccd2, metalness: 1, roughness: 0.27 }),
  steelDark: new THREE.MeshPhysicalMaterial({ color: 0x6b7078, metalness: 1, roughness: 0.42 }),
  steelBlack: new THREE.MeshPhysicalMaterial({ color: 0x2c2f34, metalness: 0.9, roughness: 0.45 }),
  alu: new THREE.MeshPhysicalMaterial({ color: 0xd9dce0, metalness: 1, roughness: 0.2 }),
  paint: new THREE.MeshPhysicalMaterial({ color: 0x5b6d7a, metalness: 0.25, roughness: 0.5, clearcoat: 0.35, clearcoatRoughness: 0.4 }),
  paintLight: new THREE.MeshPhysicalMaterial({ color: 0x8d9aa4, metalness: 0.3, roughness: 0.5, clearcoat: 0.3 }),
  engine: new THREE.MeshPhysicalMaterial({ color: 0x3e4247, metalness: 0.75, roughness: 0.42 }),
  engineDark: new THREE.MeshPhysicalMaterial({ color: 0x1d1f22, metalness: 0.6, roughness: 0.6 }),
  gold: new THREE.MeshPhysicalMaterial({ color: 0xc9a23a, metalness: 1, roughness: 0.32 }),
  glass: new THREE.MeshPhysicalMaterial({
    color: 0xa9c4e0, metalness: 0.2, roughness: 0.08, transparent: true, opacity: 0.22,
    depthWrite: false, side: THREE.DoubleSide,
  }),
  sprag: new THREE.MeshPhysicalMaterial({ color: 0xe0a33a, metalness: 0.9, roughness: 0.3 }),
};

export function shadow(o) {
  o.traverse((c) => { if (c.isMesh) { c.castShadow = true; c.receiveShadow = true; } });
  return o;
}

/** Cylinder along local Z from z0 to z1 (radius r0 at z0, r1 at z1). */
export function cylZ(r0, z0, z1, mat, { r1 = r0, seg = 48, open = false } = {}) {
  const g = new THREE.CylinderGeometry(r1, r0, Math.abs(z1 - z0), seg, 1, open);
  g.rotateX(Math.PI / 2);
  g.translate(0, 0, (z0 + z1) / 2);
  return shadow(new THREE.Mesh(g, mat));
}

/** Tube (hollow cylinder) along Z. */
export function tubeZ(rOut, rIn, z0, z1, mat, seg = 64) {
  const s = new THREE.Shape();
  s.absarc(0, 0, rOut, 0, Math.PI * 2, false);
  const h = new THREE.Path();
  h.absarc(0, 0, rIn, 0, Math.PI * 2, true);
  s.holes.push(h);
  const g = new THREE.ExtrudeGeometry(s, { depth: Math.abs(z1 - z0), curveSegments: seg, bevelEnabled: false });
  g.translate(0, 0, Math.min(z0, z1));
  return shadow(new THREE.Mesh(g, mat));
}

const boltGeo = new THREE.CylinderGeometry(1, 1, 1, 6);
boltGeo.rotateX(Math.PI / 2);
/** Ring of hex bolt heads on a face at height z, facing dir (+1 / -1). */
export function boltRing(n, R, z, headR, mat = MAT.steelDark, dir = 1, phase = 0) {
  const im = new THREE.InstancedMesh(boltGeo, mat, n);
  const m4 = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const h = headR * 0.7;
  for (let i = 0; i < n; i++) {
    const a = phase + (i / n) * Math.PI * 2;
    q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), a);
    m4.compose(new THREE.Vector3(R * Math.cos(a), R * Math.sin(a), z + (dir * h) / 2), q, new THREE.Vector3(headR, headR, h));
    im.setMatrixAt(i, m4);
  }
  im.castShadow = true;
  im.receiveShadow = true;
  return im;
}

/** Multiple-disc flexible coupling centred at z on a shaft of radius r. */
export function flexCoupling(z, r, mat = MAT.paint) {
  const g = new THREE.Group();
  const R = r * 2.6;
  g.add(cylZ(R, z - 0.11, z - 0.05, mat, { seg: 6 }));
  g.add(cylZ(R, z + 0.05, z + 0.11, mat, { seg: 6 }));
  g.children[1].rotation.z = Math.PI / 6;
  for (let i = -2; i <= 2; i++) g.add(cylZ(R * 0.92, z + i * 0.018 - 0.006, z + i * 0.018 + 0.006, MAT.steel, { seg: 40 }));
  g.add(cylZ(r * 1.5, z - 0.22, z - 0.11, mat));
  g.add(cylZ(r * 1.5, z + 0.11, z + 0.22, mat));
  g.add(boltRing(6, R * 0.78, z + 0.11, r * 0.35, MAT.steelDark, 1));
  g.add(boltRing(6, R * 0.78, z - 0.11, r * 0.35, MAT.steelDark, -1, Math.PI / 6));
  return shadow(g);
}

/** Static hanger bearing (viscous damped) around a shaft along Z. */
export function hangerBearing(z, r) {
  const g = new THREE.Group();
  g.add(tubeZ(r * 2.4, r, z - 0.12, z + 0.12, MAT.paint));       // bearing bore = shaft
  g.add(boltRing(8, r * 1.9, z + 0.12, r * 0.22, MAT.steelDark, 1));
  const plate = new THREE.Mesh(new THREE.BoxGeometry(r * 5.5, r * 1.2, 0.3), MAT.paintLight);
  plate.position.set(0, -r * 2.6, z);                              // overlaps the housing; bolts to the airframe
  g.add(plate);
  return shadow(g);
}

/**
 * GE T700-style front-drive turboshaft housing. Built along -Z from z=0 (front face).
 * Section stations are returned so labels/cutaways can reference them.
 */
export function engineHousing() {
  const g = new THREE.Group();
  const prof = [
    [0.62, 0.0], [0.78, 0.02], [0.9, 0.14], [0.96, 0.42], [0.96, 1.18], [0.86, 1.42],
    [0.7, 1.56], [0.66, 1.68], [0.66, 2.62], [0.74, 2.74], [0.82, 2.95], [0.82, 3.55],
    [0.74, 3.72], [0.71, 3.85], [0.71, 4.6], [0.63, 4.75], [0.63, 5.35], [0.58, 5.5],
    [0.5, 5.5], [0.5, 5.2],
  ].map(([r, y]) => new THREE.Vector2(r, y));
  const lathe = new THREE.LatheGeometry(prof, 96);
  lathe.rotateX(-Math.PI / 2); // +Y -> -Z
  g.add(new THREE.Mesh(lathe, MAT.engine));

  // annular inlet: dark inner wall + centre body + swirl vanes
  const inner = new THREE.LatheGeometry(
    [[0.6, 0.02], [0.6, 0.36]].map(([r, y]) => new THREE.Vector2(r, y)), 64, 0, Math.PI * 2);
  inner.rotateX(-Math.PI / 2);
  const innerM = new THREE.Mesh(inner, MAT.engineDark);
  innerM.material = MAT.engineDark.clone();
  innerM.material.side = THREE.BackSide;
  g.add(innerM);
  const hub = new THREE.LatheGeometry(
    [[0.0, -0.05], [0.18, -0.03], [0.3, 0.08], [0.33, 0.36]].map(([r, y]) => new THREE.Vector2(r, y)), 64);
  hub.rotateX(-Math.PI / 2);
  g.add(new THREE.Mesh(hub, MAT.engine));
  const back = cylZ(0.62, -0.36, -0.38, MAT.engineDark);
  g.add(back);
  for (let i = 0; i < 20; i++) {
    const a = (i / 20) * Math.PI * 2;
    const vane = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.27, 0.3), MAT.steelDark);
    vane.position.set(Math.cos(a) * 0.465, Math.sin(a) * 0.465, -0.2);
    vane.rotation.z = a - Math.PI / 2;
    vane.rotation.y = 0.5;
    vane.rotation.order = 'ZYX';
    g.add(vane);
  }
  // flanges with bolts
  for (const [y, R] of [[1.42, 0.86], [2.68, 0.72], [3.68, 0.76], [4.7, 0.66]]) {
    const t = new THREE.Mesh(new THREE.TorusGeometry(R + 0.015, 0.025, 10, 96), MAT.engine);
    t.position.z = -y;
    g.add(t);
    g.add(boltRing(28, R + 0.02, -y, 0.022, MAT.steelDark, 1));
  }
  // accessory gearbox + IPS blower on top (front)
  const agb = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.32, 0.85), MAT.engine);
  agb.position.set(0, 0.98, -0.95);
  g.add(agb);
  const blower = cylZ(0.17, -0.55, -0.85, MAT.engine);
  blower.rotation.x = Math.PI / 2;
  blower.position.set(0.22, 0.55, -0.95);   // sits on the accessory gearbox
  g.add(blower);
  return shadow(g);
}
