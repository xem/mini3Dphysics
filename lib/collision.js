// Sphere to sphere

collisionSphereSphere = (s1, s2) => {
  return dist(s1.position, s2.position) < (s1.radius + s2.radius);
}

// Sphere to OBB

collisionSphereOBB = (sphere, obb) => {
  var closest = closestPointOBB(obb, sphere.position);
  var d = dists(sphere.position, closest);
  return d < (sphere.radius ** 2);
}