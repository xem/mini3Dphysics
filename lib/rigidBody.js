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


// Particle

class particle extends rigidBody {

  // Globals

  gravity = new DOMPoint(0,-9.82,0,0);
  friction = 0.95;

  // Specific
  
  mass = 1;
  bounce = 0.7;
  
  // Others
  
  position = new DOMPoint();
  oldPosition = new DOMPoint();
  forces = new DOMPoint(0,0,0,0);
  velocity = new DOMPoint(0,0,0,0);
  
  constructor(options){
    super(options);
  }
  
  // Common
  
  update(deltaTime){
    this.oldPosition = position;
    var acceleration = scale(this.forces, 1/mass);
    this.velocity = add(scale(this.velocity, friction), scale(acceleration, deltaTime));
    this.position = add(this.position, scale(this.velocity, deltaTime);
  }
  
  render(){ W.sphere({pos: this.position, size: 0.1}) }
  
  applyForces(){ this.forces = this.gravity }
  
  solveConstraints(constraints){ /* todo */ }
  
  // Specific
  
  setPosition(pos){ this.position = this.oldPosition = pos }
  
  getPosition(){ return this.position }
  
  setBounce(b){ this.bounce = b }
  
  getBounce(){ return this.bounce }
}