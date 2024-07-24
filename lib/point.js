// Closest point from a point on a sphere

closestPointSphere = (sphere, point) => {
  //console.log(sphere, point, sub(point, sphere.position), norm(sub(point, sphere.position)));
  var sphereToPoint = scale(norm(sub(point, sphere.position)), sphere.radius);
  return add(sphereToPoint, sphere.position);
}