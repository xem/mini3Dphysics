// Sphere to sphere

collisionSphereSphere = (s1, s2) => {
  return mag(sub(s1.position, s2.position)) < (s1.radius + s2.radius);
}

// Sphere to OBB

collisionSphereOBB = (sphere, obb) => {
  var closest = closestPointOBB(obb, sphere.position);
  console.log(sphere.radius, sphere.position, closest);
  console.log(sphere.radius, sphere.position.x, closest.x);
  var d = dists(sphere.position, closest);
  //console.log(d, sphere.radius ** 2);
  return d < (sphere.radius ** 2);
}