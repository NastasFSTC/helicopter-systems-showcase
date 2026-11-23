// Procedural gear geometry: involute spur/helical, internal ring, spiral bevel.
// Every gear is built along its local +Z axis; tooth #0 is centred on local angle 0.
import * as THREE from 'three';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';

const PA = THREE.MathUtils.degToRad(20); // pressure angle
const inv = (a) => Math.tan(a) - a;
const pol = (r, a) => new THREE.Vector2(r * Math.cos(a), r * Math.sin(a));

/** Closed CCW outline of an external involute gear (z teeth, module m). */
export function toothOutline(z, m) {
  const flankPts = z > 120 ? 3 : z > 60 ? 4 : 6;
  const r = (m * z) / 2;
  const rb = r * Math.cos(PA);
  const ra = r + m;
  const rf = Math.max(r - 1.25 * m, 1e-4);
  const base = Math.PI / (2 * z) + inv(PA); // half tooth angle at pitch + involute angle
  const tMax = Math.sqrt(Math.max((ra / rb) ** 2 - 1, 0));
  const tMin = rf > rb ? Math.sqrt((rf / rb) ** 2 - 1) : 0;
  const flank = [];
  for (let i = 0; i <= flankPts; i++) {
    const t = tMin + ((tMax - tMin) * i) / flankPts;
    flank.push([rb * Math.sqrt(1 + t * t), t - Math.atan(t)]);
  }
  const pts = [];
  const pitch = (2 * Math.PI) / z;
  for (let k = 0; k < z; k++) {
    const c = k * pitch;
    if (rf < rb) pts.push(pol(rf, c - base));
    for (const [rad, ang] of flank) pts.push(pol(rad, c - base + ang));
    const tipA = flank[flank.length - 1][1];
    pts.push(pol(ra, c)); // tip centre (base - tipA keeps it symmetric)
    for (let i = flank.length - 1; i >= 0; i--) pts.push(pol(flank[i][0], c + base - flank[i][1]));
    if (rf < rb) pts.push(pol(rf, c + base));
    const gap = pitch - 2 * base;
    pts.push(pol(rf, c + base + gap / 3));
    pts.push(pol(rf, c + base + (2 * gap) / 3));
    void tipA;
  }
  return pts;
}

function warp(geo, fn) {
  const p = geo.attributes.position;
  const v = new THREE.Vector3();
  for (let i = 0; i < p.count; i++) {
    v.fromBufferAttribute(p, i);
    fn(v);
    p.setXYZ(i, v.x, v.y, v.z);
  }
  p.needsUpdate = true;
}
function rotZ(v, a) {
  const c = Math.cos(a), s = Math.sin(a);
  const x = v.x * c - v.y * s;
  v.y = v.x * s + v.y * c;
  v.x = x;
}
function finalize(geo) {
  geo.deleteAttribute('uv');
  const out = toCreasedNormals(geo, THREE.MathUtils.degToRad(32));
  out.computeBoundingBox();
  out.computeBoundingSphere();
  return out;
}

/** External spur / helical toothed rim, centred on z=0. twistRate in rad per unit length. */
export function externalRimGeometry({ z, m, width, innerR, twistRate = 0 }) {
  const shape = new THREE.Shape(toothOutline(z, m));
  if (innerR > 1e-3) {
    const h = new THREE.Path();
    h.absarc(0, 0, innerR, 0, Math.PI * 2, true);
    shape.holes.push(h);
  }
  const b = m * 0.1;
  let geo = new THREE.ExtrudeGeometry(shape, {
    depth: width - 2 * b, steps: twistRate ? 6 : 1, curveSegments: 72,
    bevelEnabled: true, bevelThickness: b, bevelSize: b * 0.6, bevelSegments: 1,
  });
  geo.translate(0, 0, -(width - 2 * b) / 2);
  if (twistRate) warp(geo, (v) => rotZ(v, v.z * twistRate));
  return finalize(geo);
}

/** Internal (ring) gear centred on z=0. Teeth point inward, tooth #0 centred on angle 0. */
export function internalRingGeometry({ z, m, width, outerR }) {
  const r = (m * z) / 2;
  const pts = toothOutline(z, m)
    .map((p) => pol(2 * r - p.length(), Math.atan2(p.y, p.x)))
    .reverse();
  const shape = new THREE.Shape();
  shape.absarc(0, 0, outerR, 0, Math.PI * 2, false);
  shape.holes.push(new THREE.Path(pts));
  const b = m * 0.1;
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: width - 2 * b, steps: 1, curveSegments: 96,
    bevelEnabled: true, bevelThickness: b, bevelSize: b * 0.6, bevelSegments: 1,
  });
  geo.translate(0, 0, -(width - 2 * b) / 2);
  return finalize(geo);
}

/**
 * Spiral bevel gear. Local frame: z=0 is the plane of the back pitch circle (radius r),
 * teeth run toward the cone apex at z = A*cos(delta). Tooth height is laid along the
 * pitch-cone normal (Tredgold back-cone construction), so a large gear has its teeth
 * standing up on its face and a small pinion has them sticking out radially.
 */
export function bevelGeometry({ z, m, faceWidth, delta, rimDepth, spiralRate = 0 }) {
  const r = (m * z) / 2;
  const A = r / Math.sin(delta);
  const rf = r - 1.25 * m;
  const shape = new THREE.Shape(toothOutline(z, m));
  const innerR = Math.max(rf - rimDepth, 0.02);
  const h = new THREE.Path();
  h.absarc(0, 0, innerR, 0, Math.PI * 2, true);
  shape.holes.push(h);
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: faceWidth, steps: 8, curveSegments: 72, bevelEnabled: false,
  });
  const cd = Math.cos(delta), sd = Math.sin(delta);
  warp(geo, (v) => {
    const l = v.z;
    const s = 1 - l / A;
    const rho = Math.hypot(v.x, v.y);
    const a = Math.atan2(v.y, v.x) + (l - faceWidth / 2) * spiralRate;
    const hgt = rho - r;
    const R = r * s + hgt * s * cd;
    v.set(R * Math.cos(a), R * Math.sin(a), l * cd + hgt * s * sd);
  });
  return finalize(geo);
}

/** Returns a description object used by the kinematics solver. */
export function gearInfo({ z, m, kind, twistRate = 0, spiralRate = 0, faceWidth = 0, delta = 0 }) {
  return {
    z, m, kind, r: (m * z) / 2,
    // angular offset of the tooth pattern at local height zl (helix / spiral)
    twistAt(zl) {
      if (kind === 'bevel') return (zl / Math.cos(delta) - faceWidth / 2) * spiralRate;
      return zl * twistRate;
    },
  };
}
