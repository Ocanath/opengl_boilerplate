#pragma once
#include <string>

class Model;

// Process-lifetime cache of renderable Model instances, keyed by source.
// Mirrors the shape-cache idiom in collision_box.cpp: callers get back a
// stable, non-owning Model* that stays valid for the life of the program,
// so many instances referencing the same mesh file or the same primitive
// dimensions share one set of GPU buffers.
//
// This exists so that anything which needs to draw an arbitrary, data-driven
// set of meshes (e.g. a URDF robot's per-link visual geometry) doesn't have
// to hand-roll assimp loading or mesh generation itself.

// Loads (and caches by resolved path) a mesh file via assimp. Repeated calls
// with the same path return the same Model.
Model* getCachedMeshModel(const std::string& path);

// Procedurally generated unit primitives, cached by shape parameters, for
// geometry types that don't come from a mesh file (URDF <box>/<cylinder>/
// <sphere>). Each is centered at the origin and sized so that scaling its
// model matrix by the primitive's full dimensions (not half-extents) fits
// it exactly, consistent with how CollisionBox scales assets/cube.obj:
//   box:      unit cube spanning [-0.5, 0.5]^3 (same asset as assets/cube.obj)
//   cylinder: radius 0.5, height 1, axis along Z, centered at origin
//   sphere:   radius 0.5, centered at origin
Model* getCachedBoxModel();
Model* getCachedCylinderModel(int segments = 24);
Model* getCachedSphereModel(int rings = 16, int sectors = 24);

// Releases every cached Model (and, transitively, its GPU buffers via
// ~Mesh()) right now. The caches above are otherwise only torn down at true
// process exit (after main() returns) — call this before destroying the GL
// context (glfwDestroyWindow/glfwTerminate), or ~Mesh()'s glDelete* calls
// run against a dead context.
void clearAssetCache();
