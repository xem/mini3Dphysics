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
// - angularDamping (number, default: 1 (no damping))
// - inverseMass (number, default: 1/1 = 1)

rigidBody = (options) => {
  
  // Default parameters
  options.position ??= vec3();
  options.orientation ??= new DOMPoint(0,0,0,0);
  options.rotation ??= vec3(),
  options.velocity ??= vec3(),
  acceleration ??= vec3(),
  linearDamping ??= 1,
  angularDamping ??= 1,
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
    inverseInertiaTensor: new DOMMatrix() // given in body space
    forceAccum: vec3(), // force accumulator
    torqueAccum: vec3(), // torque accumulator
    isAwake: false,
  }
};

// Normalize orientation and update transformationMatrix of a rigidBody

RBcalculateDerivedData = r => {
  
  // Normalize orientation
  r.orientation = normQ(orientation);
  
  // Calculate the body's transform matrix
  var x = r.x, y = r.y, z = r.z, w = r.w;
  r.transformMatrix = new DOMMatrix([
    1-2*y**2-2*z**2,  2*x*y+2*z*w,      2*x*z-2*y*w,      r.position.x,
    2*x*y-2*z*w,      1-2*x**2-2*z**2,  2*y*z+2*x*w,      r.position.y,
    2*x*z+2*y*w,      2*y*z-2*x*w,      1-2*x**2-2*y**2,  r.position.z,
    0,                0,                0,                1
  ]);

  // Calculate inertia tensor in world space (p. 219)
  RBtransformInertiaTensor(
    r, inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix
  );  
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

RBtransformInertiaTensor = (r, iitWorld, q, iitBody, rotmat) => {
  
  // P. 218 shows a complex code doing Mb * Mt * Mb^-1 (basis change) all at once.
  // TODO (maybe in a simpler way)
  // q seems usused
}

// Add a force to a rigidBody

RBaddForce = (r, f) => {
  r.forceAccum = add(r.forceAccum, f);
}

// Add a force at a rigidBody point

RBaddForceAtBodyPoint = (r, force, point) => {
  var pt = r.transformMatrix.transformPoint(point);
  RBaddForceAtPoint(r, force, point);
}

// Add a force at a point

RBaddForceAtPoint = (r, force, point) => {
  var pt = sub(point, r.position);
  r.forceAccum = add(r.forceAccum, force);
  r.torqueAccum = add(r.torqueAccum, cross(pt, force));
  r.isAwake = true;
}

// Force generators (TODO, p. 224)

// Integrate a rigidBody

inverseInertiaTensorWorld = new DOMMatrix();

RBintegrate = (r, duration) => {
  
  // Calculate linear acceleration from force inputs.
  var lastAcceleration = acceleration;
  lastAcceleration = addScaledVector(lastAcceleration, forceAccum, inverseMass);
  
  // Calculate angular acceleration from torque inputs.
  var angularAcceleration = inverseInertiaTensorWorld.transformPoint(r.torqueAccum);
  
  // Adjust velocities
  
  // Update linear velocity from both acceleration and impulse
  r.velocity = addScaled(r.velocity, lastAcceleration, duration);
  
  // Update angular velocity from both acceleration and impulse
  r.rotation = addScaled(r.rotation, angularAcceleration, duration);
  
  // Impose drag
  r.velocity = scale(r.velocity, r.linearDamping ** duration;
  r.rotation = scale(r.rotation, r.angularDamping ** duration;
  
  // Adjust positions
  
  // Update linear position
  r.position = addScaled(r.position, r.velocity, duration);
  
  // Update angular position
  r.orientation = addScaled(r.orientation, r.rotation, duration);
  
  // Normalize orientation, update matrices with new position and orientation.
  RBcalculateDerivedData(r);
  
  // Clear accumulators
  r.forceAccum = vec3();
  r.torqueAccum = new DOMMatrix();
}

// Chapter 11. The Rigid-Body Physics Engine (p. 231)
// --------------------------------------------------

// It contains:
// - The list of rigid bodies and their properties (position, angle, velocities...)
// - The force generators (can include gravity)

// World

world = {
  rigidBodies: [];
  
  // Beginning of each frame
  startFrame: () => {
    
    // For each rigid body
    for(var r of world.rigidBodies){
      
      // Clear accumulators
      r.forceAccum = vec3();
      r.torqueAccum = new DOMMatrix();
    
      // Calculate derived data
      RBcalculateDerivedData(r);
    }
  },
  
  // Integrate
  integrate: duration => {
    for(var r of world.rigidBodies){
      r.integrate(duration);
    }
  },
  
  // Process all physics
  runPhysics: duration => {
    
    // Update forces 
    // registry.updateForces(duration);
    
    // Integrate objects
    world.integrate(duration);
  }    
}

// TODO: plane, boat demos

// ===========================================
//  Part IV: Collision Detection (p. 251-331)
// ===========================================

// Chapter 12: Collision Detection (p.253)
// ---------------------------------------

// Entire books are dedicated to this topic
// (van den Bergen [2003], Ericson [2005], Eberly [2010])
// Optimized collision detection consists of:

// 1) Broad phase detection: reduce list of possible collisions to the minimum: 
// - Bounding spheres (in priority), sometimes bounding boxes (tall/long objects)
// - Bounding objects hierarchies
// - Space partitioning (quad-tree, oct-tree...)

// 2) Narrow-phase detection (final test saying if a pair of objects do collide)

// TODO: implement some of these algorithms if needed.

// Chapter 13: Generating Contacts (p. 291)
// ----------------------------------------

// Even at narrow-phase, fine collisions can be done with approaching primitives.
// We will also consider collisions with planes (often used in level geometry).
// An assembly of primitive shapes can be used to approach a shape's object.
// - Collision detection determines interpenetration (point + depth of collision).
// - Contact generation determines contact data including: point or area of contact
// on each object, direction and extent of deepest interpenetration, collision 
// normal, collision restitution (bounciness), friction...
// GJK algorithm: general-purpose (but incomplete data)
// SAT algorithm: more complete (but produces at most 1 contact per pair of objects)
// Contact coherence technique: used to complete such algorithms.

// Detection priority (p. 296)
// 1. Vertex–face and edge–edge (nonparallel edges)
// 2. Edge–face and face–face
// 3. Vertex–edge, vertex–vertex, and edge–edge (parallel edges) - generally ignored

// the most important ones:
// Vertex-face (p. 300)
// Edge-edge (p. 301)
// Edge-face (p. 301)

// Contact

contact = options => {
  options.contactNormal ??= vec3();
  options.contactPoint ??= vec3();
  options.penetration ??= 0;
  options.data ??= {}; // bodies, friction, restitution
  return options;
}

// Sphere

sphere = options => {
  options.position ??= vec3();
  options.radius ??= 1;
  return options;
}

// Sphere vs sphere

collisionDetectorSphereSphere = (one, two, data) => {
  var positionOne = one.position;
  var positionTwo = two.position;
  var midLine = sub(positionOne, positionTwo);
  var size = len(midline);
  if(size <= 0 || size > one.radius + two.radius){ return 0 }
  var normal = scale(midline, 1/size);
  var c = contact({
    contactNormal: normal,
    contactPoint: add(positionOne, scale(midline, 1/2)),
    penetration: (one.radius + two.radius - size),
    data
  });
  return 1;
}

// Plane

plane = options => {
  options.direction ??= vec3(); // normal
  options.offset ??= 0; // distance to origin
  options.transform ??= new DOMMatrix();
  return options;
}

// Sphere vs plane (half-space)

collisionDetectorSphereHalfSpace = (sphere, plane, data) => {
  var position = shere.position;
  var ballDistance = dot(plane.direction, position) - sphere.radius - plane.offset;
  if(ballDistance >= 0) return 0;
  var c = contact({
    contactNormal: plane.direction,
    contactPoint: dot(sub(position, plane.direction), ballDistance + sphere.radius),
    penetration: -ballDistance,
    data
  });
  return 1;
}

// Collision with true plane is rarely useful (p. 309)

// Collisions between a box and a half space produce 2 to 4 contact points.

// Box

box = options => {
  options.position ??= vec3();
  options.halfSize ??= vec3(1, 1, 1); // schema p. 313.
  options.transform ??= new DOMMatrix();
  return options;
};

// Distance between a point (vertex) and a plane (p. 312)

// Vertices positions (p. 313)

// Box vs plane (half-space)

/* =================== todo: replace with axis later ====================== 
collisionDetectorBoxHalfSpace = (box, plane, data) => {
  
  // Go through each combination of + and - for each half-size.
  var mults = [
    [1,1,1],[-1,1,1],[1,-1,1],[-1,-1,1],
    [1,1,-1],[-1,1,-1],[1,-1,-1],[-1,-1,-1]
  ];
  var contactsUsed = 0;
  var vertexPos, vertexDistance, c;
  for(var i = 0; i < 8; i++){
    
    // Calculate the position of each vertex
    vertexPos = vec3(
      box.position.x * mults[i][0],
      box.position.y * mults[i][1],
      box.position.z * mults[i][2]
    );
    
    vertexPos = mul(vertexPos, box.halfSize);
    vertexPos = box.transform.transformPoint(vertexPos);
    
    // Calculate the distance from the plane
    vertexDistance = dot(vertexPos, plane.direction);
  
    // Compare this to the plane’s distance
    if vertexDistance <= plane.offset){
  
      // Create the contact data.
      // contact point is halfway between vertex and plane.
      c = etc...
}
============================*/

// Sphere vs box (p. 316)

collisionDetectorBoxSphere(box, sphere, data){

  // Transform the centre of the sphere into box coordinates
  var centre = sphere.position;
  var relCentre = box.transform.inverse().transformPoint(centre);

  // Early out check to see if we can exclude the contact
  if(
    Math.abs(relCentre.x) - sphere.radius > box.halfSize.x ||
    Math.abs(relCentre.y) - sphere.radius > box.halfSize.y ||
    Math.abs(relCentre.z) - sphere.radius > box.halfSize.z)
  ){
    return 0;
  }

  var closestPt = vec3(0,0,0);
  var dist;

  // Clamp each coordinate to the box
  dist = relCentre.x;
  if(dist > box.halfSize.x) dist = box.halfSize.x;
  if(dist < -box.halfSize.x) dist = -box.halfSize.x;
  closestPt.x = dist;

  dist = relCentre.y;
  if(dist > box.halfSize.y) dist = box.halfSize.y;
  if(dist < -box.halfSize.y) dist = -box.halfSize.y;
  closestPt.y = dist;

  dist = relCentre.z;
  if(dist > box.halfSize.z) dist = box.halfSize.z;
  if(dist < -box.halfSize.z) dist = -box.halfSize.z;
  closestPt.z = dist;

  // Check we're in contact
  dist = squareMagnitude(sub(closestPt, relCentre));
  
  if(dist > sphere.radius * sphere.radius) return 0;

  // Compile the contact
  var losestPtWorld = box.transform.transformPoint(closestPt);

  var c = contact({
    contactNormal: norm(sub(closestPtWorld, centre)),
    contactPoint: closestPtWorld,
    penetration: sphere.radius - Math.sqrt(dist),
    data
  });

  return 1;
}




// ======================================
//  Part V: Contact Physics (p. 333-461)
// ======================================










// TODO: all .transformPoint() must be done on a DOMPoint.


