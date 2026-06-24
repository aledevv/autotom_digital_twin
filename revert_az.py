import re

with open('src/plant_model/usd_exporter.py', 'r') as f:
    content = f.read()

old_azimuth = """    def get_azimuth(n) -> float:
        if n is None or not isinstance(n, InternodeNode): return 0.0
        if hasattr(n, 'world_azimuth'): return n.world_azimuth
        if n.key.order == 0:
            az = (n.key.rank * PHYLLOTAXIS) % 360.0
        else:
            p_az = get_azimuth(n.parent)
            sign = 1 if (n.key.rank % 2 == 0) else -1
            az = (p_az + sign * 90.0) % 360.0
        n.world_azimuth = az
        return az"""

new_azimuth = """    def get_azimuth(n) -> float:
        if n is None or not isinstance(n, InternodeNode): return 0.0
        if hasattr(n, 'world_azimuth'): return n.world_azimuth
        # Revert to original behavior: no automatic phyllotaxis rotation
        n.world_azimuth = 0.0
        return 0.0"""

content = content.replace(old_azimuth, new_azimuth)

old_fruits = """        if node.parent and isinstance(node.parent, InternodeNode):
            tip_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
            truss_az = math.radians(getattr(node.parent, 'world_azimuth', 0.0))
        else:
            tip_z = 0.0
            truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)"""

new_fruits = """        if node.parent and isinstance(node.parent, InternodeNode):
            tip_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
        else:
            tip_z = 0.0
        truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)"""

content = content.replace(old_fruits, new_fruits)

with open('src/plant_model/usd_exporter.py', 'w') as f:
    f.write(content)

print("Applied revert patch")
