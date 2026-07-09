#!/usr/bin/env python3
"""Generate Dog2 symmetric STL meshes with Blender.

The symmetric URDF keeps the same link names and per-link mesh file names as the
real model, but reads them from ``meshes_symmetric``.  This script uses the
front-left CAD-exported link meshes as the canonical leg geometry and mirrors
each link type through the axes expected by the existing URDF link frames.

Do not mirror a whole leg with one global rule here. The URDF already applies
per-side joint-frame rotations, and the CAD STL files are in each link's local
visual frame. A single leg-level X/Y mirror double-flips several links and makes
RViz look broken. Collision meshes use the same per-link rules.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import bpy


REPO_ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION_DIR = REPO_ROOT / "src" / "dog2_description"
SRC_MESH_DIR = DESCRIPTION_DIR / "meshes"
DST_MESH_DIR = DESCRIPTION_DIR / "meshes_symmetric"
SRC_COLLISION_DIR = SRC_MESH_DIR / "collision"
DST_COLLISION_DIR = DST_MESH_DIR / "collision"

LEG_SUFFIXES = ("", "1", "11", "111")

# STL suffix -> leg number -> local mesh scale.
#
# suffix ""    rail: rear rails use the opposite local X side.
# suffix "1"   coxa: all four coxa meshes share the same local visual frame.
# suffix "11"  femur: right-hind mirrors local X; right-front mirrors local Z
#               because dog2_leg_macro.xacro already applies the rf visual
#               rpy flip that the original CAD export expects.
# suffix "111" tibia: right-side tibias mirror local X.
LINK_MIRRORS = {
    "": {
        "1": (1.0, 1.0, 1.0),
        "2": (-1.0, 1.0, 1.0),
        "3": (1.0, 1.0, 1.0),
        "4": (-1.0, 1.0, 1.0),
    },
    "1": {
        "1": (1.0, 1.0, 1.0),
        "2": (1.0, 1.0, 1.0),
        "3": (1.0, 1.0, 1.0),
        "4": (1.0, 1.0, 1.0),
    },
    "11": {
        "1": (1.0, 1.0, 1.0),
        "2": (1.0, 1.0, 1.0),
        "3": (-1.0, 1.0, 1.0),
        "4": (1.0, 1.0, -1.0),
    },
    "111": {
        "1": (1.0, 1.0, 1.0),
        "2": (1.0, 1.0, 1.0),
        "3": (-1.0, 1.0, 1.0),
        "4": (-1.0, 1.0, 1.0),
    },
}


def _clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def _active_mesh_objects() -> list[bpy.types.Object]:
    return [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]


def _import_stl(path: Path) -> list[bpy.types.Object]:
    _clear_scene()
    bpy.ops.import_mesh.stl(filepath=str(path))
    objects = _active_mesh_objects()
    if not objects:
        raise RuntimeError(f"No mesh objects imported from {path}")
    return objects


def _apply_mirror(objects: list[bpy.types.Object], mirror: tuple[float, float, float]) -> None:
    for obj in objects:
        obj.scale = mirror
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        obj.select_set(False)

        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.normals_make_consistent(inside=False)
        bpy.ops.object.mode_set(mode="OBJECT")
        obj.select_set(False)


def _export_stl(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in _active_mesh_objects():
        obj.select_set(True)
    bpy.ops.export_mesh.stl(filepath=str(path), use_selection=True, ascii=False)


def _mirror_stl(source: Path, destination: Path, mirror: tuple[float, float, float]) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    objects = _import_stl(source)
    _apply_mirror(objects, mirror)
    _export_stl(destination)


def _generate_visual_meshes() -> None:
    _mirror_stl(SRC_MESH_DIR / "base_link.STL", DST_MESH_DIR / "base_link.STL", (1.0, 1.0, 1.0))

    for suffix in LEG_SUFFIXES:
        for leg_num, mirror in LINK_MIRRORS[suffix].items():
            source = SRC_MESH_DIR / f"l1{suffix}.STL"
            destination = DST_MESH_DIR / f"l{leg_num}{suffix}.STL"
            _mirror_stl(source, destination, mirror)


def _generate_collision_meshes() -> None:
    base_collision = SRC_COLLISION_DIR / "base_link_collision.STL"
    if base_collision.is_file():
        _mirror_stl(
            base_collision,
            DST_COLLISION_DIR / "base_link_collision.STL",
            (1.0, 1.0, 1.0),
        )

    for suffix in LEG_SUFFIXES:
        for leg_num, mirror in LINK_MIRRORS[suffix].items():
            source = SRC_COLLISION_DIR / f"l1{suffix}_collision.STL"
            destination = DST_COLLISION_DIR / f"l{leg_num}{suffix}_collision.STL"
            _mirror_stl(source, destination, mirror)


def _copy_readme() -> None:
    readme = DST_MESH_DIR / "README.md"
    readme.write_text(
        "# Dog2 Symmetric Meshes\n\n"
        "Generated by `src/dog2_description/scripts/generate_symmetric_meshes_blender.py`.\n\n"
        "The leg visual and collision STL files are generated from the front-left "
        "`l1*` source meshes in Blender with per-link local-frame mirror rules. "
        "This avoids double-mirroring the URDF side-specific joint-frame rotations.\n\n"
        "- rail: leg2/leg4 mirror local X\n"
        "- coxa: all legs keep the front-left local frame\n"
        "- femur: leg3 mirrors local X; leg4 mirrors local Z for the existing rf visual flip\n"
        "- tibia: leg3/leg4 mirror local X\n\n"
        "The real model continues to use `meshes/`; only `dog2_symmetric.urdf.xacro` "
        "uses this directory.\n",
        encoding="utf-8",
    )


def main() -> int:
    if not SRC_MESH_DIR.is_dir():
        raise RuntimeError(f"Source mesh directory missing: {SRC_MESH_DIR}")

    if DST_MESH_DIR.exists():
        for child in DST_MESH_DIR.iterdir():
            if child.is_dir():
                shutil.rmtree(child)
            else:
                child.unlink()

    DST_MESH_DIR.mkdir(parents=True, exist_ok=True)
    DST_COLLISION_DIR.mkdir(parents=True, exist_ok=True)

    _generate_visual_meshes()
    _generate_collision_meshes()
    _copy_readme()
    print(f"Generated symmetric meshes under {DST_MESH_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
