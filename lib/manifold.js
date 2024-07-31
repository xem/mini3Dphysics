// Collision manifold

collisionManifold = options => {
  options ??= {};
  resetCollisionManifold(options);
  return options;
}

resetCollisionManifold = m => {
  m.colliding = 0;
  m.normal = new DOMPoint(0,0,1);
  m.depth = Infinity;
  m.contacts = [];
}

// Sphere to sphere

findCollisionFeaturesSphereSphere = (a, b) => {
  var result = collisionManifold();
  var r = a.radius + b.radius;
  var d = sub(b.position, a.position);
  var m = mag(d);
  
  // No collision
  if(m >= r || m == 0) {
    return result;
  }
  
  d = norm(d);
  result.colliding = 1;
  result.normal = d; // towards a
  result.depth = Math.abs(m - r) / 2;
  result.dtp = a.radius - result.depth; // distance to intersection point
  var contact = add(a.position, scale(d, dtp));
  result.contacts.push(contact);
  return result;
}

// Sphere to OBB

findCollisionFeaturesSphereOBB = (a, b) => { // a: obb, b: sphere
  var result = collisionManifold();
  var closestPoint = closestPointOBB(a, b.position);
  var d = dist(closestPoint, B.position);
  
  // No collision
  if(d >= B.radius || d == 0){
    return result();
  }
  
  var normal = norm(sub(b.position, closestPoint));
  var outsidePoint = sub(b.position, scale(normal, b.radius));
  var distance = dist(closestPoint, outsidePoint);
  result.colliding = 1;
  result.contacts.push(add(closestPoint, scale(sub(outsidePoint, closestPoint), 0.5);
  result.normal = normal;
  result.depth = distance / 2;
  return result;
}

// Declare lines
line = (start, end) => ({start, end});

// Declare planes

plane = options => {normal, distance) => ({normal, distance});

// Get the vertices of an OBB

getVertices = obb => {
  var v = [];
  var C = obb.position;     // Center
  var E = obb.size;         // Extents
  var o = obb.orientation;
  var A = [                 // OBB axis
    new DOMPoint(o.m11, o.m12, o.m13),
    new DOMPoint(o.m21, o.m22, o.m23),
    new DOMPoint(o.m31, o.m32, o.m33)
  ];
  
  for(var a of [-1, 1]){
    for(var b of [-1, 1]){
      for(var c of [-1, 1]){
        v.push(
          add(add(add(C, scale(A[0], a * E[0])), scale(A[1], b * E[1])), scale(A[2], c * E[2]))
        );
      }
    }
  }
  return v;
}

// Get the edges of an OBB

getEdges = obb => {
  var result;
  var v = getVertices(obb);
  var index = [  // Indices of edge-vertices
    [6,1],[6,3],[6,4],[2,7],[2,5],[2,0],
    [0,1],[0,3],[7,1],[7,4],[4,5],[5,3]
  ];
  for(var j = 0; j < 12; ++j){
    result.push(line(v[index[j][0]], v[index[j][1]));
  }
  return result;
}

// Get the planes of the OBB

getPlanes = obb => {
  var result = [];
  var C = obb.position; // center
  var E = obb.size;     // extents
  var o = obb.orientation;
  var A = [             // axis
    new DOMPoint(o.m11, o.m12, o.m13),
    new DOMPoint(o.m21, o.m22, o.m23),
    new DOMPoint(o.m31, o.m32, o.m33)
  ];
  for(var a of [0, 1, 2]){
    for(var b of [-1, 1]){
        result.push(
          plane(scale(A[a], b), b * dot(A[a], add(C, scale(mul(A[a] * E[A]), b)))); 
        );
      }
    }
  }
  return result();
}

// Clip to plane

// Checks if a line intersects a plane and if it does, clip it to the plane

clipToPlane = (plane, line, outPoint) => {
  var ab = sub(line.end, line.start);
  var nab = dot(plane.normal, ab);
  if(nab == 0) return 0;
  var na = dot(plane.normal, line.start);
  var t = scale(sub(plane.distance, na), 1 / nab);
  if(t >= 0 && t <= 1){
    if(outPoint.value != 0){
      outpoint.value = add(line.start, scale(ab,t));
    }
    return 1;
  }
  return 0;
}

// Clip edges to OBB

clipEdgesToOBB = (edges, obb) => {
  var result = [];
  var intersection = { value: 0 };
  var planes = getPlanes(obb);
  for(var i = 0; i < planes.size; i++){
    for(var j = 0; j < edges.size; j++){
      if(clipToPlane(planes[i], edges[j], intersection)){
        if(pointInOBB(intersection, obb)){
          result.push(intersection);
        }
      }
    }
  }
  return result;
}

// Penetration depth

penetrationDepth = (o1, o2, axis, outShouldFlip) => {
  var i1 = getInterval(o1, norm(axis));
  var i2 = getInterval(o2, norm(axis));
  if(!((i2.min <= i1.max) && (i1.min < i2.max))){
    return 0;
  }
  var len1 = i1.max - i1.min;
  var len2 = i2.max - i2.min;
  var min = Math.min(i1.min, i2.min);
  var max = Math.max(i1.max, i2.max);
  var length = max - min;
  if(outShouldFlip.value != 0){
    outShouldFlip.value = (i2.min < i1.min);
  }
  return (len1 + len2) - length;
}

// Find collision features

findCollisionFeaturesOBBOBB = (A, B) => {
  var j;
  var result = collisionManifold();
  o1 = A.orientation;
  o2 = B.orientation;
  var test = [ // face axis
    new DOMPoint(o1.m11, o1.m12, o1.m13),
    new DOMPoint(o1.m21, o1.m22, o1.m23),
    new DOMPoint(o1.m31, o1.m32, o1.m33),
    new DOMPoint(o2.m11, o2.m12, o2.m13),
    new DOMPoint(o2.m21, o2.m22, o2.m23),
    new DOMPoint(o2.m31, o2.m32, o2.m33);
  ];
  
  // Other axis
  for(var i = 0; i < 3; i++){
    test[6 + i * 3] = cross(test[i], test[0]);
    test[6 + i * 3 + 1] = cross(test[i], test[1]);
    test[6 + i * 3 + 2] = cross(test[i], test[2]);
  }
  var hitNormal = 0;
  var shouldFlip = { value: 0 };
  var depth;
  for(i = 0; i < 15; i++){
    if(mags(test[i]) < 0.001){
      continue;
    }
    depth = penetrationDepth(A, B, test[i], shouldFlip);
    if(depth <= 0){
      return result; // no collision
    }
    else if(depth < result.depth){
      if(shouldFlip.value){
        test[i] = scale(test[i], -1);
      }
      result.depth = depth;
      hitNormal = test[i];
    }
  }
  
  // No collision
  if(hitNormal == 0){
    return result;
  }
  
  // Intersection points
  var c1 = clipEdgesToOBB(getEdges(B), A);
  var c2 = clipEdgesToOBB(getEdges(A), B);
  result.contacts.push([result.contacts.at(-1), c1[0], c1.at(-1));
  result.contacts.push([result.contacts.at(-1), c2[0], c2.at(-1));
  var i = getInterval(A, axis);
  var distance = (i.max - i.min) / 2 - result.depth / 2;
  var pointOnPlane = add(A.position, scale(axis, distance));
  var contact;
  for(i = result.contacts.length - 1; i >= 0; i--){
    contact = result.contacts[i]; 
    result.contacts[i] = add(contact, scale(axis, dot(axis, sub(pointOnPlane, contact)))); 
    
    // Remove duplicates
    for(j = result.contacts.size - 1; j > i; j--){
      if(dist(result.contacts[j], result.contacts[i]) < 0.0001){
        result.contacts.splice(j, 1);
        break;
      }
    }
  }
  result.colliding = 1;
  result.normal = axis;
  return result;
}