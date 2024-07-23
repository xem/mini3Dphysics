// +===================================================================+
// |                         MINI 3D PHYSICS                           |
// +===================================================================+
// | This project aims to summarize and implement contents of the book |
// | "Game Physics Engine Development (2nd Edition)" by Ian Millington |
// | into a small 3D physics engine useable in golfed JS games & demos |
// +===================================================================+

// Chapter 1: Introduction (p. 1 - 12)
// -----------------------------------

// - Intro to physics engines
// - Maths level required: basic trigonometry, 3D coordinates system
// - A "scalar" means that the value in question is a number.

//       /|                         Y
//      / |                         |   .(2,2,1)
//     /  |     b = a sinθ          | 
//  a /   | b   c = a cosθ        o |_ _ _ X
//   /    |     b = c tanθ         /
//  /_θ_ _|                       /
//     c                         Z 

// =====================================
//  Part I: Particle Physics (p. 15-73)
// =====================================

// Chapter 2. The Maths of Particles (p. 17-45)
// --------------------------------------------

// A 3D vector (a = {x, y, z}) can represent the coordinates of a point 
// in 3D space or a direction (offset) between two points.

// Constructor

vec3 = (x = 0, y = 0, z = 0) => ({x, y, z});

// Magnitude (length, operator: |v|)

len = v => Math.hypot(v.x, v.y, v.z);

// Square magnitude
squareMagnitude = v => v.x ** 2 + v.y ** 2 + v.z **2;

// Scale (set length without changing the direction, operator: *)

scale = (v, s) => ({ x: v.x * s, y: v.y * s, z: v.z * s });

// Flip

flip = v => scale(v, -1);

// Normalize (force length to 1 without changing the direction, operator: ||v||)
// In literature, a normalized vector is generally written with a hat ^ on top.

norm = (v, l) => { l = len(v); return (t > 0) ? scale(v, 1 / l) : v; };

// Add two vectors

add = (v, w) => ({ x: v.x + w.x, y: v.y + w.y, z: v.z + w.z });

// Subtract a vector from another

sub = (v, w) => ({ x: v.x - w.x, y: v.y - w.y, z: v.z - w.z });

// Add a scaled vector to another

addScaled = (v, w, s) => add(v, scale(w, s));

// Component product (operator: o)

mul = (v, w) => ({ x: v.x * w.x, y: v.y * w.y, z: v.z * w.z });

// Scalar (dot) product (operator: . or *)
// Represents how two vectors are aligned.
// Ex: 1: same direction / 0: perpendicular / -1: opposite

dot = (v, w) => v.x * w.x + v.y * w.y + v.z * w.z;

// Vector (cross) product (operator: x or %)
// Generates a third vector perpendicular to the first two based on 
// the right-hand rule. (cf. images/cross.jpg)

//         a x b
//           ^
//           | 
//           | 
//    a _ _ _|
//          /
//         /
//        b 

cross = (v, w) => ({ 
  x: v.y * w.z - v.z * w.y,
  y: v.z * w.x - v.x * w.z,
  z: v.x * w/y - v.y * w.x
}); 


// Chapter 3. The Laws of Motion (p. 47-59)
// ----------------------------------------

// Particle constructor
// A particle has position, velocity and acceleration along the world axis x, y, z.
// damping is an approximation of the drag force appied to the particle,
// reducing its velocity at each frame. (0: stop / 1: no damping)
// inverseMass (1/mass) is used instead of mass to ease computations and
// to represent immovable objects with infinite mass (inverseMass = 0).
// "speed" is a number, while "velocity" has a direction. |v| = s.
// In literature, vectors with a dot on top (˙) represent a velocity,
// and vectors with two dots on top (¨) represent an acceleration.
// In this book's equations, p = p.position, p˙ = p.velocity, p¨ = p.acceleration.

particle = (
  position = vec3(),
  velocity = vec3(),
  acceleration = vec3(),
  damping = 1,
  inverseMass = 1,
  forceAccum = vec3()
 ) => ({
  position,
  velocity,
  acceleration,
  damping,
  inverseMass,
  forceAccum
});

// Gravity is a downwards acceleration vector applied to all movable
// objects at each frame.
// (ex: position += velocity * time + acceleration * time * time * 0.5)

gravity = vec3(0, -2, 0);

// Integration (update forces and position at each frame)
// duration: time elapsed since last frame (in ms)

integrate = (p, duration) => {
  
  if(p.inverseMass != 0){
  
    // Update linear position
    p.position = addScaled(p.position, p.velocity, duration);
    
    // Sum forces applied to acceleration (user-defined + gravity * inverseMass)
    addForce(p, p.acceleration);
    addForce(p, scale(gravity, 1/p.inverseMass));

    // Update linear velocity
    p.velocity = addScaled(p.velocity, p.forceAccum, duration);
    
    // Apply drag force to the power of deltaTime to the velocity (see p.57)
    p.velocity = scale(p.velocity, p.damping ** duration);
    
    clearForceAccumulator(p);
  }
};


// Chapter 4. The Particles Physics Engine (p. 61-73)
// --------------------------------------------------

// This chapter shows how to implement projectiles and fireworks.
// More info in the readme file and the demos folder.

// =============================================
//  Part II: Mass Aggregate Physics (p. 75-154)
// =============================================

// Chapter 5. Adding General Forces (p. 77-88)
// -------------------------------------------

// D'Alembert principle: all the forces applied to an object,
// can be summed to make a single, equivalent force.
// To do this, we introduce forceAccum into the particle object.
// The accumulator is cleared after each integration.

clearForceAccumulator = p => p.forceAccum = vec3();

// Add a force to a particle
// For objects with non-infinite mass, the gravity will be added with this.
// Other forces include: drag, buoyancy, blast, thrust, spring

addForce = (p, force) => p.forceAccum = add(p.forceAccum, force);

// todo: updateForces


// Chapter 6. Springs and Spring-Like Things (p. 89-110)
// -----------------------------------------------------

// ꔛ ⅏
 
// Hook's law: the force of a spring depends only on its extension or 
// compression distance. Ex: doubling the extension doubles the force
// The spring stiffness is a constant k. Its rest length is l_0.
// Its force is felt at both ends.
// Force in 1D: f = -k * (l - l_0)
// Force in 2D: f = -k * (|d| - l_0) * ||d||.
// d is a vector between both ends, pointing to the object considered.
// Elasticity limits (min and max deformation) can be enforced in code.
// Spring-like things: bungee cord, buoyancy, 3rd person camera...

// Update spring between two particles

particleSpring = (p, other, springConstant = 1, restLength = 1) => {
  var v = sub(p.position, other.position);
  var magnitude = -springConstant * (Math.abs(len(v)) - restLength);
  var force = scale(norm(v), magnitude);
  addForce(p, force);
}

// Update spring between a particle and a fixed point
// To avoid using this, the anchor can be an immovable object.

particleAnchoredSpring = (p, anchor, springConstant = 1, restLength = 1) => {
  var v = sub(p.position, anchor);
  var magnitude = -springConstant * (Math.abs(len(v)) - restLength);
  var force = scale(norm(v), magnitude);
  addForce(p, force);
}

// If a camera is bound to the game's character / vehicle with a spring,
// be careful to avoid applying a force to the character, only to the camera.

// A bungee rope only acts as a spring when it's extended

particleBungee = (p, other, springConstant = 1, restLength = 1) => {
  var v = sub(p.position, other.position);
  if(len(v) <= restLength) return;
  var magnitude = -springConstant * (len(v) - restLength);
  var force = scale(norm(v), magnitude);
  addForce(p, force);
}

// Springs can approximate buoyancy by pulling the center of mass of an object
// to the surface of the liquid it is into. (Gravity is still present)
// Buoyancy force:
// - when the object is not submerged: f = 0
// - when it is totally submerged: f = vp (liquid density * object volume)
// - when it is partially submerged: f = dvp (d = submersion level, 0 < d < 1)
// d=(yo-yw-s)/2s (yo: object height, yw: liquid height, s: submersion depth)

particleBuoyancy = (p, maxDepth, volume, waterHeight, liquidDensity = 1) => {
  
  var depth = p.position.y;
  var force;
  
  // Out of water
  if(depth >= waterHeight + maxDepth){
    return;
  }
  
  // Max depth (fully submerged)
  if(depth <= waterHeight - maxDepth){
    force = vec3(0, liquidDensity * volume, 0);
    addForce(p, force);
    return;
  }
  // Partial submersion
  force = vec3(
    0,
    - liquidDensity * volume * ((depth - maxDepth - waterHeight) / (2 * maxDepth)),
    0
  );
  addForce(p, force);
}

// Unfortunately, springs stiffness can be very inaccurate or send objects to 
// infinity when constants and time intervals get too large.
// Damping can be used to limit this, but more precise approaches will be
// implemented in the rest of the book.

// Chapter 7. Hard Constraints (p. 113-142)
// ----------------------------------------

// Collision: when two objects touch or interpenetrate by moving towards each other.
// Collision resolution: objects movement after the collision.
// Closing velocity (vc): total speed at which two objects are moving together.
// vc is a number. vc > 0: objects are closing / vc < 0: ojects are going apart. 
// vs = -(pa.velocity - pb.velocity) . (||pa - pb||)
// Separation velocity (vs = -vc) will be used from now on.
// vs = (pa.velocity - pb.velocity) . (||pa - pb||)
// Contact normal / collision normal n: direction in which objects are colliding.
// For object a: n = ||pa - pb||. For object b, it's -n.
// If object a collides with leveled ground, n = [0, 1, 0].
// Impulse: offset immediately applied to the object's velocity, noted g.

// Collision processing
// restitution is the coefficient at which the objects react to the collision.

resolveVelocity = (p1, p2, restitution, duration) => {
  
  // Compute separating velocity
  var relativeVelocity = p1.velocity;
  if(p2) relativeVelocity = sub(relativeVelocity, p2.velocity); // vec3
  var contactNormal = norm(sub(p1.position, p2.position)); // normalized vec3
  var separatingVelocity = dot(relativeVelocity, contactNormal); // Number
  
  // Separating: nothing to do
  if(separatingVelocity > 0){
    return;
  }
  
  // Closing: compute new separating velocity
  var newSepVelocity = -separatingVelocity * restitution;
  
  // ======== added from page 128 ========
  
  // Check velocity caused by acceleration only
  var accCausedVelocity = p1.acceleration; // vec3
  if(p2) accCausedVelocity = sub(accCausedVelocity, p2.acceleration);
  var accCausedSepVelocity = dot(accCausedVelocity, scale(contactNormal, duration));
  
  // Remove closing velocity due to acceeration buildup
  if(accCausedSepVelocity < 0){
    newSepVelocity += restitution * accCausedSepVelocity;
    if(newSepVelocity < 0) newSepVelocity = 0;
  }
  
  // =====================================
  
  var deltaVelocity = newSepVelocity - separatingVelocity; // Number
  
  // Apply changes in proportion to the inverse masses
  var totalInverseMass = p1.inverseMass;
  if(p2) totalInverseMass += p2.inverseMass;
  if(totalInverseMass <= 0) return; // both are immovable
  
  // Compute impulse per unit of inverse mass
  var impulse = deltaVelocity / totalInverseMass;
  var impulsePerIMass = scale(contactNormal, impulse);
  
  // Apply impulse
  p1.velocity = addScaled(p1.velocity, impulsePerIMass, p1.inverseMass);
  if(p2){
    p2.velocity = addScaled(p2.velocity, impulsePerIMass, -p2.inverseMass);
  }
}

// Collision detection and resolution
// The goal is to separate interpenetrating objects immediately,
// By moving them along the contact normal, proportionally to their masses,
// in parallel of updating their velocities (see code above).

resolveInterpenetration = (p1, p2, penetration) => {
  
  // No penetration
  if(penetration <= 0) return;
  
  // Total inverse mass
  var totalInverseMass = p1.inverseMass;
  if(p2) totalInverseMass += p2.inverseMass;
  if(totalInverseMass <= 0) return; // both are immovable
  
  // Compute penetration per unit of inverse mass
  var movePerIMass = scale(contactNormal, penetration / totalInverseMass); // vec3
  
  // Compute movement
  var p1movement = scale(movePerIMass, p1.inverseMass);
  p1.position = add(p1.position, p1movement);
  var p2movement = vec3();
  if(p2){
    p2movement = scale(movePerIMass, -p2.inverseMass);
    p2.position = add(p2.position, p2movement);
  }
}

// Contact resolver function (TODO, p. 133)

// Contact resolver algorithm (at each frame):
// 1. Calculate all separating velocities, consider the lowest (negative) one.
// 2. If all separating velocities are >= 0: exit.
// 3. Process collision response for contact with lowest separating velocity.
// 4. If there are more iterations, return to Step 1

// Cables (links) prevent two object from getting apart further than a given 
// distance, but they can get closer.
// It works like the opposite of an interpenetration collisions.
// (TODO, p. 139)

// Rods are a mix of collision anc cables: objects are kept at an exact distance.
// (TODO: p. 140)

// Chapter 8. The Mass Aggregate Physics Engine (p. 145-154)
// ----------------------------------------------------------

// TODO: ParticleWorldGenerateContacts, ParticleWorldIntegrate, ParticleWorldRunPhysics, loop