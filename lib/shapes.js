// sphere

sphere = options => {
  options.position ??= new DOMPoint();
  options.radius ??= 1;
  return options;
}

// OBB

obb = options => {
  options.position ??= new DOMPoint();
  options.size ??= new DOMPoint();
  options.orientation ??= new DOMMatrix();
  return options;
}

// Plane

plane = options => {
  options.normal ??= new DOMPoint();
  options.distance ??= 0;
  options.normal = norm(options.normal);
  return options;
}
