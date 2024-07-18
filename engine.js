// +===================================================================+
// |                         MINI 3D PHYSICS                           |
// +===================================================================+
// | This project aims to summarize and implement contents of the book |
// | "Game Physics Engine Development (2nd Edition)" by Ian Millington |
// | into a small 3D physics engine useable in golfed JS games & demos |
// +===================================================================+

// =====================================
//  Part I: Particle Physics (p. 15-73)
// =====================================

// 2. The Maths of Particles (p. 17-45)
// ------------------------------------

// World coordinates axis (X, Y, Z)

//     Y
//     |
//     |
//   o |_ _ _ X
//     /
//    /
//   Z 


// A 3D vector (a = {x, y, z}) can represent the coordinates of a point 
// in 3D space or a direction (offset) between two points.

// Constructor

vec3 = (x = 0, y = 0, z = 0) => ({x, y, z});

// Flip

flip = v => ({x: -v.x, y: -v.y, z: -v.z});

// Magnitude (length)

len = v => Math.hypot(v.x, v.y, v.z);

// Scale (set length without changing the direction)

scale = (v, s) => ({ x: v.x * s, y: v.y * s, z: v.z * s });

// Normalize (force length to 1 without changing the direction)

norm = (v, l) => { l = len(v); return (t > 0) ? scale(v, 1 / l) : v; };

// Add two vectors

add = (v, w) => ({ x: v.x + w.x, y: v.y + w.y, z: v.z + w.z });

// Subtract a vector from another

sub = (v, w) => ({ x: v.x - w.x, y: v.y - w.y, z: v.z - w.z });

// Add a scaled vector to another

addScaled = (v, w, s) => add(v, scale(w, s));

// Component product (operator: o or *)

mul = (v, w) => ({ x: v.x * w.x, y: v.y * w.y, z: v.z * w.z });

// Scalar (dot) product (operator: .)
// Represents how two vectors are aligned.
// Ex: 1: same direction / 0: perpendicular / -1: opposite

dot = (v, w) => v.x * w.x + v.y * w.y + v.z * w.z;

// Vector (cross) product (operator: x or %)
// Generates a third vector perpendicular to the first two based on 
// the right-hand rule. (cf. images/cross.jpg)

cross = (v, w) => ({ 
  x: v.y * w.z - v.z * w.y,
  y: v.z * w.x - v.x * w.z,
  z: v.x * w/y - v.y * w.x
}); 


// 3. The Laws of Motion (p. 47-59)
// --------------------------------

// Particle constructor
// A particle has position, velocity and acceleration along the 3 
// world axis (x, y, z).
// damping is an approximation of the drag force appied to the particle,
// reducing its velocity at each frame. (0: stop / 1: no damping)
// inverseMass (1/mass) is used instead of mass to ease computations and
// to represent immovable objects with infinite mass (inverseMass = 0).

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

G = 0.01;
g = vec3(0, -G, 0);

// Integration (update forces and position at each frame)
// duration: time elapsed since last frame (in ms)
integrate = (p, duration) => {
  
  if(p.inverseMass > 0){
  
    // Update linear position
    p.position = addScaled(p.position, p.velocity, duration);
    
    // Work out acceleration
    var resultingAcc = p.acceleration;
    resultingAcc = addScaled(resultingAcc, p.forceAccum, p.inverseMass);

    // Update linear velocity
    p.velocity = addScaled(p.velocity, resultingAcc, duration);
    
    // Drag (velocity times damping to the power of the duration, see p.57)
    p.velocity = scale(p.velocity, p.damping ** duration);
    
    clearForceAccumulator(p);
  }
};


// 4. The Particles Physics Engine (p. 61-73)
// -----------------------------------------

// This chapter shows how to implement projectiles and fireworks.
// More infos in the demos folder

// =============================================
//  Part II: Mass Aggregate Physics (p. 75-154)
// =============================================

// 5. Adding General Forces (p. 77-88)
// -----------------------------------

// D'Alembert principle: all the forces applied to an object,
// can be summed to make a single, equivalent force.
// To do this, we introduce forceAccum into the particle object.
// The accumulator is cleared after each integration.

clearForceAccumulator = p => p.forceAccum = vec3();

// Add a force to a particle
// For objects with non-infinite mass, the gravity will be added with this.
// Other forces include: drag, buoyancy, blast, thrust, spring

addForce = (p, force) => p.forceAccum = add(p.forceAccum, force);



// 6. Springs and Spring-Like Things (p. 89-110)
// ---------------------------------------------

// 7. Hard Constraints (p. 113-142)
// --------------------------------

// 8. The Mass Aggregate Physics Engine (p. 145-154)
// -------------------------------------------------

// ===========================================
//  Part III: Rigid-Body Physics (p. 155-248)
// ===========================================

// ===========================================
//  Part IV: Collision Detection (p. 251-331)
// ===========================================

// ======================================
//  Part V: Contact Physics (p. 333-461)
// ======================================