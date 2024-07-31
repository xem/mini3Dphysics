// Closest point from a point on a sphere

closestPointSphere = (sphere, point) => {
  var sphereToPoint = scale(norm(sub(point, sphere.position)), sphere.radius);
  return add(sphereToPoint, sphere.position);
}

// Closest point from a point on a OBB

closestPointOBB = (obb, point) => {
  var result = new DOMPoint();
  result = add(result, obb.position);
  var dir = sub(point, obb.position);
  var axis = [
    new DOMPoint(obb.orientation.m11, obb.orientation.m12, obb.orientation.m13),
    new DOMPoint(obb.orientation.m21, obb.orientation.m22, obb.orientation.m23),
    new DOMPoint(obb.orientation.m31, obb.orientation.m32, obb.orientation.m33)
  ];
  var dist;
  for(var i = 0; i < 3; ++i){
    dist = dot(dir, axis[i]);
    if(dist > obb.size[i]){
      dist = obb.size[i];
    }
    if(dist < -obb.size[i]){
      dist = -obb.size[i];
    }
    result = add(result, scale(axis[i], dist));
  }
  return result;
}


pointInOBB = (obb, point) => {
  var dir = sub(point, obb.position);
  var axis = [
    new DOMPoint(obb.orientation.m11, obb.orientation.m21, obb.orientation.m31),
    new DOMPoint(obb.orientation.m12, obb.orientation.m22, obb.orientation.m32),
    new DOMPoint(obb.orientation.m13, obb.orientation.m23, obb.orientation.m33)
  ];
  var dist;
  for(var i = 0; i < 3; i++){
    dist = dot(dir, axis[i]);
    if(dist > obb.size[i]){
      return 0;
    }
    if(dist < -obb.size[i]){
      return 0;
    }
  }
  return 1;
}
