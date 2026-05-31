Dog2 symmetric meshes placeholder.

Visual meshes (STL): to be generated later via Blender mirror of the real-model
  meshes in ../meshes/.  Until then, dog2_symmetric.urdf.xacro still references
  the original meshes/ path.

Collision meshes (STL): to be replaced with mirrored collision primitives or
  mirrored collision STL; current symmetric URDF reuses original
  meshes/collision/ STL.

Priority: dynamics / IK / WPC / MPC symmetry is enforced at the URDF joint,
  inertial, foot, and control-parameter levels first.
