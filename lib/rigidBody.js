// RigidBody
// Options:
// - type (0: none, 1: particle, 2: sphere, 3: box)
// - friction
// - bounce
// - gravity
// - mass

/* class rigidBody {

  constructor(options){
    for(var i in options){
      this[i] = options[i];
    }
  }

  update(deltaTime) {}
  
  render() {}
  
  applyForces() {}
  
  solveConstraints(constraints) {}
} */


// Global

GRAVITY_CONST = new DOMPoint(0, -9.82, 0, 0);
RIGIDBODY_SPHERE = 2;
RIGIDBODY_BOX = 3;

// rigidBodyVolume

class rigidBodyVolume{
  type; // 2: sphere, 3: box
  position;
  velocity;
  forces; // sum of all forces
  mass; // 0: immovable
  cor; // coefficient of restitution (bounce)
  friction;
  box = obb();
  sphere = sphere();
  orientation = new DOMPoint();
  angVel = new DOMPoint();
  torques = new DOMPoint(); // sum
  
  // Constructor
  
  constructor(type = 0){
    this.cor = 0.5;
    this.mass = 1;
    this.friction = 0.6;
    this.type = type;
  }
  
  // Functions
  
  render(){
    if(this.type == RIGIDBODY_SPHERE){
      render(sphere);
    }
    else if(type == RIGIDBODY_BOX){
      render(box);
    }
  };
  
  update(dt){
    var damping = 0.98;
    var acceleration = scale(forces, this.InvMass());
    this.velocity = add(this.velocity, scale(this.acceleration, dt));
    this.velocity = scale(this.velocity, damping);
    var angAccel;
    if(type == RIGIDBODY_BOX){
      angAccel = mul(torques, InvTensor());
      this.angVel = add(this.angVel, scale(angAccel, dt);
      this.angVel = scale(this.angVel, damping);
    }
    this.position = add(this.position, scale(this.velocity, dt));
    if(type == RIGIDBODY_BOX){
      this.orientation = add(this.orientation, scale(this.angVel, dt);
    }
    this.synchCollisionVolumes();
  }
  
  applyForces(){
    this.forces = scale(GRAVITY_CONST, this.mass);
  };
  
  synchCollisionVolumes(){
    this.sphere.position = this.position;
    this.box.position = this.position;
    this.box.orientation = new DOMMatrix().rotate(this.orientation.x, this.orientation.y, this.orientation.z);
  }
  
  invMass(){
    if(mass == 0) return 0;
    return 1 / mass;
  };
  
  addLinearImpulse(impulse){
    this.velocity = add(this.velocity, impulse);
  };
  
  FindCollisionFeatures(ra, rb){
    var result = collisionManifold();
    
    // Between a sphere and...
    if(ra.type == RIGIDBODY_SPHERE){
    
      // ... a sphere
      if(rb.type == RIGIDBODY_SPHERE){
        result = findCollisionFeaturesSphereSphere(ra.sphere, rb.sphere);
      }
      
      // ... a box 
      else if(rb.type == RIGIDBODY_BOX){
        result = findCollisionFeaturesSphereOBB(ra.sphere, rb.box);
      }
    }
    
    // Between a box and...
    if(ra.type == RIGIDBODY_BOX){
    
      // ... a sphere (inverse the normal compared to a sphere-box test)
      if(rb.type == RIGIDBODY_SPHERE){
        result = findCollisionFeaturesSphereOBB(rb.sphere, ra.box);
        result.normal = scale(result.normal, -1);
      }
      
      // ... a box 
      else if(rb.type == RIGIDBODY_BOX){
        result = findCollisionFeaturesOBBOBB(ra.box, rb.box);
      }
    }
    
    return result;
  }
  
  applyImpulse (A, B, M, c){
  
    // Linear velocity
    var invMass1 = A.invMass();
    var invMass2 = B.invMass();
    var invMassSum = invMass1 + invMass2;
    if(invMassSum == 0){ return; }
    
    // Contact point relative to centers of mass (new)
    var r1 = sub(M.contacts[c], A.position); 
    var r2 = sub(M.contacts[c], B.position); 
   
    // Inverse inertia tensors (new)
    var i1 = A.invTensor();
    var i2 = B.invTensor();
    
    // Relative velocity (updated)
    var relativeVel = sub(
      add(B.velocity, cross(B.angVel, r2)),
      add(A.velocity, cross(A.angVel, r1))
    );
    
    // Relative collision normal
    var relativeNorm = norm(M.normal);
    
    // Do nothing if they are moving away from each other
    if(dot(relativeVel, relativeNorm) > 0){ return; }
    
    var e = Math.min(A.cor, B.cor);
    var numerator = (-(1 + e) * dot(relativeVel, relativeNorm));
    
    // Calculate magnitude of impulse (new)
    var d1 = this.invMassSum;
    var d2 = cross(mul(cross(r1, relativeNorm), i1), r1);
    var d3 = cross(mul(cross(r2, relativeNorm), i2), r2);
    var denominator = d1 + dot(relativeNorm, add(d2, d3));
    var j = denominator == 0 ? 0 : numerator / denominator;
    
    if(M.contacts.length > 0 && j != 0){
      j /= M.contacts.length;
    }
    
    var impulse = scale(relativeNorm, j);
    
    // Linear velocity
    A.velocity = sub(A.velocity, scale(impulse, invMass1));
    B.velocity = add(B.velocity, scale(impulse, invMass2));
    
    // Angular velocity (new)
    A.angVel = sub(A.angVel, mul(cross(r1, impulse), i1));
    B.angVel = add(B.angVel, mul(cross(r2, impulse), i2));
    
    // Friction
    var t = sub(relativeVel, scale(relativeNorm, dot(relativeVel, relativeNorm)));
    if(mags(t) == 0){ return; }
    t = norm(t);
    numerator = -dot(relativeVel, t);
    
    //  Tangential impulse (new)
    d1 = invMassSum;
    d2 = cross(mul(cross(r1, t), i1), r1);
    d3 = cross(mul(cross(r2, t), i2), r2);
    denominator = d1 + dot(t, add(d2, d3));
    if(denominator == 0) { return }
    
    // Find tangential force (updated)
    var jt = numerator / denominator;
    if(M.contacts.length > 0 && jt != 0){
      jt /= M.contacts.length;
    }
    if(jt == 0){ return; }
    var friction = Math.sqrt(A.friction * B.friction);
    if(jt > j * friction) {
      jt = j * friction;
    }
    else if(jt &lt; -j * friction){
      jt = -j * friction;
    }
    var tangentImpuse = scale(t, jt);
    A.velocity = sub(A.velocity, scale(tangentImpuse, invMass1));
    B.velocity = add(B.velocity, scale(tangentImpuse, invMass2));
    A.angVel = sub(A.angVel, mul(cross(r1, tangentImpuse), i1));
    B.angVel = add(B.angVel, mul(cross(r2, tangentImpuse), i2));
  }
  
  invTensor(){
    var ix = 0, iy = 0, iz = 0, iw = 0;
    var r2, fraction, size, x2, y2, z2;
    if(this.type == RIGIDBODY_SPHERE){
      r2 = this.sphere.radius ** 2;
      fraction = 2/5;
      ix = r2 * this.mass * fraction;
      iy = r2 * this.mass * fraction;
      iz = r2 * this.mass * fraction;
      iw = 1;
    } 
    else if(this.type == RIGIDBODY_BOX){
      size = [this.box.size[0] * 2, this.box.size[1] * 2, this.box.size[2] * 2];
      fraction = 1/12;
      x2 = this.size[0] ** 2;
      x2 = this.size[1] ** 2;
      x2 = this.size[2] ** 2;
      ix = (y2 + z2) * mass * fraction;
      ix = (x2 + z2) * mass * fraction;
      ix = (x2 + y2) * mass * fraction;
      iw = 1;
    }
    return new DOMMatrix([
      ix, 0, 0, 0,
      0, iy, 0, 0,
      0, 0, iz, 0,
      0, 0, 0, iw)
    ]);
  }
  
  addRotationalImpulse(point, impulse){
    var centerOfMass = this.position;
    var torque = cross(sub(point, centerOfMass), impulse);
    var angAccel = this.invTensor().transformPoint(torque);
    this.angvel = add(angVel, angAccel);
  }
  
  
}




// Physics system

physicsSystem = {
  bodies: [],       // Array of rigidBodies
  constraints: [],  // Array of constraints (OBBs for now)
  colliders1: [],   // Colliding pairs of objects
  colliders2: [],
  results: [],
  linearProjectionPercent: 0.5, // Penetration correction level (0.2 to 0.8)
  linearProjectionSlack: 0.1,   // Penetration allowed (0.01 to 0.1, avoids jitter)
  impulseIteration: 6,          // Tests per frame, 6 to 8 works well
    
  update: deltaTime => {
    physicsSystem.colliders1 = [];
    physicsSystem.colliders2 = [];
    physicsSystem.results = [];
    var result;
    var m1, m2, size, i, j, k, jSize, totalMass, depth, scalar, correction;
    for(i = 0, size = physicsSystem.bodies.length; i< size; ++i){
      for(var j = i; j < size; ++j){
        if(i == j){ continue; }
        result = collisionManifold();
        m1 = physicsSystem.bodies[i];
        m2 = physicsSystem.bodies[j];
        result = FindCollisionFeatures(m1, m2);
        if(result.colliding){
          physicsSystem.colliders1.push(bodies[i]);
          physicsSystem.colliders2.push(bodies[j]);
          physicsSystem.results.push(result);
        }
      }
    }
    
    // Calculate foces acting on the object
    for(i = 0, size = bodies.length; i < size; ++i){
      bodies[i].applyForces();
    }
    for(int k = 0; k < ImpulseIteration; ++k){
      for(int i = 0; i < results.length; ++i){
        jSize = results[i].contacts.length;
        for(j = 0; j < jSize; ++j){
          m1 = physicsSystem.colliders1[i];
          m2 = physicsSystem.colliders2[i];
          physicsSystem.applyImpulse(m1, m2, results[i], j);
        }
      }
    }
    for(i = 0, size = bodies.length i < size; ++i){
      bodies[i].update(deltaTime);
    }
    for(i = 0, size = results.length; i < size; ++i){
      m1 = physicsSystem.colliders1[i];
      m2 = physicsSystem.colliders2[i];
      totalMass = m1.invMass() + m2.invMass();
      if(totalMass == 0){
        continue;
      }
      depth = Math.max(results[i].depth - PenetrationSlack, 0);
      scalar = depth / totalMass;
      correction = results[i].normal * scalar * LinearProjectionPercent;
      m1.position = sub(m1.position, scale(correction, m1.InvMass()));
      m2.position = add(m2.position, scale(correction, m2.InvMass()));
      m1.synchCollisionVolumes();
      m2.synchCollisionVolumes();
    }
    for(int i = 0, size = bodies.size(); i < size; ++i){
      bodies[i].solveConstraints(constraints);
    }
  },
  
  render: () => {
    for(var i of physicsSystem.bodies){
      i.render();
    }
    for(i of physicsSystem.constraints){
      i.render();
    }
  },
  
  addRigidBody: body => physicsSystem.bodies.push(body),
  
  addConstraint: constraint => physicsSystem.constraints.push(constraint),
  
  clear: () => {
    physicsSystem.bodies = [];
    physicsSystem.constraints = [];
  }
}