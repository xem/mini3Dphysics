// Sphere to sphere

collisionSphereSphere = (s1, s2) => dist(s1.position, s2.position) < (s1.radius + s2.radius);

// Sphere to OBB

collisionSphereOBB = (sphere, obb) => dist(sphere.position, closestPointOBB(obb, sphere.position)) < sphere.radius;

// OBB to OBB

interval = options => {
  options.min ??= 0;
  options.max ??= 0;
}

getInterval = (obb, axis) => {
  var C = obb.position;     // center
  var E = obb.size;         // Extents
  var o = obb.orientation;
  var A = [                 // Axis
    new DOMPoint(o.m11, o.m12, o.m13),
    new DOMPoint(o.m21, o.m22, o.m23),
    new DOMPoint(o.m31, o.m32, o.m33)
  ];
  
  // Find all vertices of the OBB
  var vertex = [];
  for(var a of [-1, 1]){
    for(var b of [-1, 1]){
      for(var c of [-1, 1]){
        vertex.push(
          add(add(add(C, scale(A[0], a * E[0])), scale(A[1], b * E[1])), scale(A[2], c * E[2]))
        );
      }
    }
  }
  
  // Result
  var result = interval();
  result.min = result.max = dot(axis, vertex[0]);
  var projection;
  for(i = 0; i < 8; i++){
    projection = dot(axis, vertex[i]);
    if(projection < result.min) result.min = projection;
    if(projection > result.max) result.max = projection;
  }
  return result;
}

overlapOnAxis = (obb1, obb2, axis) => {
  var a = getInterval(obb1, axis);
  var b = getInterval(obb2, axis); // possible typo in the book, it says obb1 here
  return (b.min <= a.max) && (a.min <= b.max);
}

collisionOBBOBB = (obb1, obb2) => {
  var o1 = obb1.orientation;
  var o2 = obb2.orientation;
  var test = [
    new DOMPoint(o1.m11, o1.m12, o1.m13),
    new DOMPoint(o1.m21, o1.m22, o1.m23),
    new DOMPoint(o1.m31, o1.m32, o1.m33),
    new DOMPoint(o2.m11, o2.m12, o2.m13),
    new DOMPoint(o2.m21, o2.m22, o2.m23),
    new DOMPoint(o2.m31, o2.m32, o2.m33)
  ];
  
  // Create the other axis  
  for(var i = 0; i < 3; ++i){
    test[6 + i * 3] = cross(test[i], test[0]);
    test[6 + i * 3 + 1] = cross(test[i], test[1]);
    test[6 + i * 3 + 2] = cross(test[i], test[2]);
  }
  
  // Test tall the axis
  for(i = 0; i < 15; ++i){
    if(!overlapOnAxis(obb1, obb2, test[i])) return false;
  }
  return true;
}