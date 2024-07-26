# Mini3DPhysics

Tiny 3D physics engine inspired by the book "Game Physics Cookbook" by Gabor Szauer.

The engine computes everything but does not render the scene.

The relevant chapters of the book are summarized in the pages below.

A custom WebGL renderer is included, based on [W.js](https://xem.github.io/W)

# Engine source code

See the [lib folder](https://github.com/xem/mini3Dphysics/tree/gh-pages/lib)

# Renderer API (todo)

Scene:
- W.reset(webglContext)
- W.light({x, y, z})
- W.camera({x, y, z, rx, ry, rz, fov})
- W.wclearColor("#rgb");

Shapes:
- W.sphere(options)
- W.box(options)
- W.add(model_name, {vertices, uv, indices}) // register new model, then it can be rendered using <b>W.model_name(options)</b>

Transformations:
- W.move(options)
- W.delete(id)

options: 
- id (string)
- p: position (DOMPoint)
- r: rotation (DOMPoint)
- s: size/radius ([w, h d] for boxes, number for spheres)
- c: color (string, optional)
- t: texture (image, optional)
- M: transformation matrix (DOMMatrix)
- s: smoothing (0/1)

(contrary to W.js, this renderer does not include groups, delays, animations, planes, billboards and pyramids)


# Chapters summarized

[1. Vectors](https://xem.github.io/mini3Dphysics/1/)

[2. Matrices + 3. Matrix Transformations](https://xem.github.io/mini3Dphysics/2/)

4. 2D Primitive Shapes (skipped)

5. 2D Collisions (skipped)

6. 2D Optimizations (skipped)

[7. 3D Primitive Shapes](https://xem.github.io/mini3Dphysics/7/)

[8. 3D Point Tests](https://xem.github.io/mini3Dphysics/8/)

[9. 3D Shape Intersections](https://xem.github.io/mini3Dphysics/9/)

10. 3D Line Intersections (skipped)

11. Triangles and Meshes (skipped)

12. Models and Scenes (skipped)

13. Camera and Frustum (skipped)

14. Constraint Solving

<!--

15. Manifolds and Impulses

16. Springs and Joints

-->