// RigidBody (todo: use class?)

rigidBody = options => {
  return options;
}

rigidBody.update = deltaTime => {}
rigidBody.render = () => {}
rigidBody.applyForces = () => {}
rigidBody.solveConstraints = constraints => {}

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