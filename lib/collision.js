// Sphere to sphere

collisionSphereSphere = (s1, s2) => dist(s1.position, s2.position) < (s1.radius + s2.radius);

// Sphere to OBB

collisionSphereOBB = (sphere, obb) => dist(sphere.position, closestPointOBB(obb, sphere.position)) < sphere.radius;