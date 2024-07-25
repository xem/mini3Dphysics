// Addition
add = (u, v) => new DOMPoint(u.x + v.x, u.y + v.y, u.z + v.z, u.w)// + v.w);

// Subtraction
sub = (u, v) => new DOMPoint(u.x - v.x, u.y - v.y, u.z - v.z, u.w)// - v.w);

// Multiplication
mul = (u, v) => new DOMPoint(u.x * v.x, u.y * v.y, u.z * v.z, u.w)// * v.w);

// Scaling (s is a number)
scale = (u, s) => new DOMPoint(u.x * s, u.y * s, u.z * s)//, u.w * s);

// Dot product
dot = (u, v) => u.x * v.x + u.y * v.y + u.z * v.z// + u.w * v.w;

// Magnitude / magnitude squared
mag = v => Math.sqrt(dot(v, v));
mags = v => dot(v, v);

// Distance / distance squared
dist = (u, v) => mag(sub(u, v));
dists = (u, v) => mags(sub(u, v));

// Normalize
norm = v => scale(v, 1 / mag(v));

// Cross product
cross = (u, v) => new DOMPoint(
  u.y * v.z - u.z * v.y,
  u.x * v.z - u.z * v.x,
  u.x * v.y - u.y * v.x,
  0
);

// Angle
angle = (u, v) => Math.acos(dot(u, v) / (norm(u) * norm(v)))

// Projection
proj = (u, v) => mul(dot(u, v) / mags(v)) * v;
perp = (u, v) => sub(u, proj(u, v));

// Reflection
reflect = (v, n) => sub(v, scale(proj(v,n), 2));
