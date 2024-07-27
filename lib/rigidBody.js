// RigidBody (todo: use class?)
// Options:
// - type (0: none, 1: particle, 2: sphere, 3: box
// - friction
// - bounce
// - gravity
// - mass

class rigidBody {

  constructor(options){
    for(var i in options){
      this[i] = options[i];
    }
  }

  update(deltaTime) {}
  
  render() {}
  
  applyForces() {}
  
  solveConstraints(constraints) {}
}

// Physics system

physicsSystem = {

  bodies: [],       // Array of rigidBodies
  
  constraints: [],  // Array of constraints (OBBs for now)
  
  update: deltaTime => {
    for(var i of physicsSystem.bodies){
      i.applyForces();
    }
    for(i of physicsSystem.bodies){
      i.update(deltaTime);
    }
    for(i of physicsSystem.bodies){
      i.solveConstraints(physicsSystem.constraints);
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