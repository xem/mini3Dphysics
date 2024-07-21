// +===================================================================+
// |                         MINI 3D PHYSICS                           |
// +===================================================================+
// | This project aims to summarize and implement contents of the book |
// | "Game Physics Engine Development (2nd Edition)" by Ian Millington |
// | into a small 3D physics engine useable in golfed JS games & demos |
// +===================================================================+

// ===========================================
//  Part III: Rigid-Body Physics (p. 155-248)
// ===========================================

// Chapter 9. The Maths of Rotations (p. 157)
// ------------------------------------------

// Definitions:
// - Orientation = angle (in radians).
// - Rotation = angular velocity: rate of change of orientation (in radians/second).

// 2D rotations:
// - can be expressed between 0rad (0deg) and 2*Pi rad (360deg), or in (-Pi, Pi].
// - Wrap (Ex: 2*Pi is the same angle as 0, Pi/2 is the same angle as -3*Pi/2.)
// - Rotation In vector form: θ^ = [ cosθ, sin θ ]
// - Rotation matrix: r = [ cosθ, sinθ,
//                         -sinθ, cosθ ];
// Multiplying a vector or a point by r rotates it θ radians around the origin.
// If rotation is not around the origin: perform rotation, then translation.
// - Any number of translation ans rotation can be accumuled into a single 
// translation and a single rotation.

// 3D rotations "bad" representations:
// - Euler Angles (3 degrees of freedom, but too limited for most use cases).
// - Axis-angle: any combination of rotations can be represented by a single
// rotation along a specific axis. Works, compact, but impractical (complex maths).
// - Rotation Matrices: representation used bu GPUs internally. Combining two
// rotations is simple (matrix multiplication).
// Limitations of matrices: 9 or 16 values to handle, floating point numbers
// approximations and errors may stack up and become visible. 

// Quaternions:
// - 4 degrees of freedom, precise, and convertible into a matrix for rendering.
// - Vector form: q = [cos θ/2, x * sin θ/2, y * sin θ/2, z * sin θ/2],
// where [x, y, z] is the axis and θ the angle.
// - Numeric form: q = w + xi + yj + zk, where i, j, k are imaginary numbers:
// i² = j² = k² = -1 and ijk = -1.
// Thus (by definition): ij = -ji = k. jk = -kj = i. ki = -ik = j.
// - Combination by multiplication: (w1 + x1i + y1j + z1k) × (w2 + x2i + y2j + z2k)
// - Let's write quaternions as 4D vectors this way: θ = [w, x, y, z].
// - Ensure normalized magnitude: sqrt(w² + x² + y² + z²) = 1
// - Angular velocity quaternion: w = [0, θ˙x, θ˙y, θ˙z]. Not normalized.

// In JS, points, vectors and quaternions can be represented with DOMPoint.
// (WIP, NOT SURE IF I'LL KEEP DOMPOINT FOR QUATERNIONS)
// 4x4 matrices can be represented with DOMMatrix.
// Ex: new DOMPoint(x,y,z,1) is a point, new DOMPoint(i,j,k,r) is a quaternion,
// and new DOMPoint(1,2,3,0) is a vector.
// The 4th coordinate w is called a homogeneous coordinate and allows points and
// vectors to be propermy multiplied with 4x4 transformation matrices.
// When w=1, the point is sensitive to translations (points can be moved).
// When w=0, the vector is immune to translations (vectors don't move).
// Multiply a DOMPoint with a DOMMatrix: m.transformPoint(p)
// Inverse a matrix: m.inverse() (new) or m.invertSelf() (replaces the original)
// Transpose a matrix: usually not necessary because a pre-multiplication is 
// equivalent to a transpose plus a post-multiplication. Ex: M1 * M2^T = M2 * M1.

// A change of basis occurs if an object is moved or rotated from the origin.
// To perform a new transformation Mt independently from a change of basis Mb:
// M = Mb * Mt * Mb^-1 (Mb^-1 is the inverse of Mb)
// From right to left: the basis change is cancelled, the transformation is done,
// and finally the basis change is done again.

// Normalize a Quaternion

normQ = q => {
  var d = 1 / Math.hypot(q.x, q.y, q.z, q.w);
  if(d > 0){
    return new DOMPoint(q.x * d, q.y * d, q.z * d, q.w * d);
  }
  return q;
}

// Combine quaternions by multiplying them (NB: DOMPoint order is x,y,z,w)

mulQ = (a, b) => new DOMPoint(
  a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
  a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
  a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
  a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
);

// Quaternion rotation by a vector
// Q *= [x, y, z, 0];
// the vector values represent the amount of rotation in each axis I guess.

rotateByVector(q, v) {
  var q2 = new DOMPoint(v.x, v.y, v.z, 0);
  return mulQ(q, q2);
}

// Add scaled vector to a quaternion (to update its anguar velocity)

addScaledQ = (q, v, scale) => {
  var q2 = new DOMPoint(v.x * scale, v.y * scale, v.z * scale, 0);
  q2 = mulQ(q2, q);
  return new DOMPoint(q2.x / 2, q2.y / 2, q2.z / 2, q2.w / 2);
}

// Chapter 10: Laws of Motion for Rigid Bodies (p. 207)
// ----------------------------------------------------

// RigidBody constructor
// Options:
// - position (vec3, default: 0,0,0)
// - orientation (quaternion, default: 0,0,0,0)
// - rotation (vec3, angular velocity, default: 0,0,0)
// - velocity (vec3, default: 0,0,0)
// - acceleration (vec3 (default: 0,0,0)
// - linearDamping (number, default: 1 (no damping))
// - inverseMass (number, default: 1/1 = 1)

rigidBody = (options) => {
  
  // Default parameters
  options.position ??= vec3();
  options.orientation ??= new DOMPoint(0,0,0,0);
  options.rotation ??= vec3(),
  options.velocity ??= vec3(),
  acceleration ??= vec3(),
  linearDamping ??= 1,
  inverseMass ??= 1;
  
  // Create Object
  return {
    position,
    orientation,
    rotation,
    velocity,
    acceleration,
    damping,
    inverseMass,
    transformMatrix: new DOMMatrix(), // derived data, to update once per frame
    inverseInertiaTensor: new DOMMatrix() // goven in body space
  }
};

// Normalize orientation and update transformationMatrix of a rigidBody

calculateDerivedData = r => {
  
  // Normalize orientation
  r.orientation = normQ(orientation);
  
  // Calculate inertia tensor in world space (p. 219)
  transformInertiaTensor(
    r, inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix
  );
  
  // Calculate the body's transform matrix
  var x = r.x, y = r.y, z = r.z, w = r.w;
  r.transformMatrix = => new DOMMatrix([
    1-2*y**2-2*z**2,  2*x*y+2*z*w,      2*x*z-2*y*w,      r.position.x,
    2*x*y-2*z*w,      1-2*x**2-2*z**2,  2*y*z+2*x*w,      r.position.y,
    2*x*z+2*y*w,      2*y*z-2*x*w,      1-2*x**2-2*y**2,  r.position.z,
    0,                0,                0,                1
  ]);
}

// Torque (or "moments") is a twisting force.
// In 2D: τ = pf x f (point where force is applied x force)
// In 3D: τ = a*d (torque magnitude * normalized axis around which torque applies)

// Moment of inertia is to mass what torque is to rotation:
// It represents how difficult it is to change the rootation speed of an object.

// An inertia tensor is a 2D matrix representing a moment of inertia.

// In rigidBody, the inertia tensor is inverted, like mass, and given in body space.
// So, at each frame, we compute the transform matrix, transform the inverse inertia
// tensor into world coordinates, then perform rigidBody integration.

transformInertiaTensor = (r, iitWorld, q, iitBody, rotmat) => { /* TODO p. 218 */ }



// ===========================================
//  Part IV: Collision Detection (p. 251-331)
// ===========================================

// ======================================
//  Part V: Contact Physics (p. 333-461)
// ======================================