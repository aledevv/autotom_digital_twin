"""Shared USD materials for ExporterV2 scenes."""

from pxr import Gf, Sdf, UsdGeom, UsdShade


TOMATO_LEAF_MATERIAL_PATH = "/World/Looks/TomatoLeaf"
TOMATO_LEAF_OMNISURFACE_MATERIAL_PATH = "/World/Looks/TomatoLeafOmniSurface"


_TOMATO_LEAF_PRESETS = {
    "realtime": {
        "diffuseColor": Gf.Vec3f(0.10, 0.30, 0.075),
        "roughness": 0.62,
        "metallic": 0.0,
        "specularColor": Gf.Vec3f(0.22, 0.28, 0.16),
    },
    "realistic": {
        "diffuse_reflection_weight": 0.85,
        "diffuse_reflection_color": Gf.Vec3f(0.10, 0.30, 0.055),
        "diffuse_reflection_roughness": 0.12,
        "metalness": 0.0,
        "specular_reflection_weight": 1.0,
        "specular_reflection_roughness": 0.50,
        "specular_reflection_ior": 1.42,
        "thin_walled": True,
        "enable_diffuse_transmission": True,
        "subsurface_weight": 0.30,
        "subsurface_transmission_color": Gf.Vec3f(0.30, 0.55, 0.08),
        "specular_retro_reflection_weight": 0.07,
        "specular_retro_reflection_color": Gf.Vec3f(0.35, 0.50, 0.22),
        "specular_retro_reflection_roughness": 0.50,
    },
}


def _material_path_for_preset(preset: str) -> str:
    if preset == "realistic":
        return TOMATO_LEAF_OMNISURFACE_MATERIAL_PATH
    return TOMATO_LEAF_MATERIAL_PATH


def _set_shader_input(shader, name: str, value) -> None:
    if isinstance(value, bool):
        value_type = Sdf.ValueTypeNames.Bool
    elif isinstance(value, Gf.Vec3f):
        value_type = Sdf.ValueTypeNames.Color3f
    else:
        value_type = Sdf.ValueTypeNames.Float

    shader.CreateInput(name, value_type).Set(value)


def _create_realtime_tomato_leaf_material(stage, material_path: str):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")

    for name, value in _TOMATO_LEAF_PRESETS["realtime"].items():
        _set_shader_input(shader, name, value)

    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(),
        "surface",
    )
    return material


def _create_realistic_tomato_leaf_material(stage, material_path: str):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset("OmniSurface.mdl", "mdl")
    shader.SetSourceAssetSubIdentifier("OmniSurface", "mdl")
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)

    material.CreateSurfaceOutput("mdl").ConnectToSource(
        shader.ConnectableAPI(),
        "out",
    )

    for name, value in _TOMATO_LEAF_PRESETS["realistic"].items():
        _set_shader_input(shader, name, value)

    return material


def get_or_create_tomato_leaf_material(stage, preset: str = "realtime"):
    """Return the shared tomato leaf material for a stage.

    The default uses UsdPreviewSurface to keep full-plant scenes responsive.
    The heavier OmniSurface preset remains available for isolated look-dev.
    """
    if preset not in _TOMATO_LEAF_PRESETS:
        raise ValueError(f"Unknown tomato leaf material preset: {preset!r}")

    material_path = _material_path_for_preset(preset)
    existing = stage.GetPrimAtPath(material_path)
    if existing.IsValid():
        return UsdShade.Material(existing)

    UsdGeom.Scope.Define(stage, "/World/Looks")

    if preset == "realistic":
        return _create_realistic_tomato_leaf_material(stage, material_path)
    return _create_realtime_tomato_leaf_material(stage, material_path)
