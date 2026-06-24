import re

with open('src/plant_model/usd_exporter.py', 'r') as f:
    content = f.read()

# 1. Update Separate organs logic
old_separate = """    # ── Separate organs ──────────────────────────────────────────────────────
    root_node = next((n for n in snapshot.organs if isinstance(n, RootNode)), None)
    internodes = sorted(
        [n for n in snapshot.organs if isinstance(n, InternodeNode)],
        key=lambda n: n.key.rank   # rank 0 → 1 → 2 ... bottom to top
    )"""
new_separate = """    # ── Separate organs ──────────────────────────────────────────────────────
    root_node = next((n for n in snapshot.organs if isinstance(n, RootNode)), None)
    
    def get_base_z(n) -> float:
        if n is None or not isinstance(n, InternodeNode): return 0.0
        if hasattr(n, 'world_base_z'): return n.world_base_z
        z = get_base_z(n.parent) + n.parent.length if (n.parent and isinstance(n.parent, InternodeNode)) else 0.0
        n.world_base_z = z
        return z

    for n in snapshot.organs:
        if isinstance(n, InternodeNode):
            get_base_z(n)

    internodes = sorted(
        [n for n in snapshot.organs if isinstance(n, InternodeNode)],
        key=lambda n: (n.key.order, n.key.rank)
    )"""
content = content.replace(old_separate, new_separate)

# 2. Update Internode chain rendering
old_internode_chain = """    # ── Internode chain — each starts where the previous ended ────────────────
    current_z = 0.0   # base of first internode = ground level z=0

    for node in internodes:
        L = node.length
        R = node.width_m / 2.0
        rank = node.key.rank

        print(f"\\n[INTERNODE rank={rank}]")
        cyl = _make_cylinder(
            stage,
            path=f"{stem_path}/Internode_r{rank}",
            height=L,
            radius=R,
            base_z=current_z,
        )
        current_z += L   # tip of this internode = base of next
        
        _bind_material(cyl, materials["stem"])"""
new_internode_chain = """    # ── Internode hierarchical rendering ──────────────────────────────────────
    max_z = 0.0
    for node in internodes:
        L = node.length
        R = node.width_m / 2.0
        rank = node.key.rank
        order = node.key.order
        base_z = getattr(node, 'world_base_z', 0.0)
        max_z = max(max_z, base_z + L)

        print(f"\\n[INTERNODE order={order} rank={rank}]")
        cyl = _make_cylinder(
            stage,
            path=f"{stem_path}/Internode_o{order}_r{rank}",
            height=L,
            radius=R,
            base_z=base_z,
        )
        _bind_material(cyl, materials["stem"])"""
content = content.replace(old_internode_chain, new_internode_chain)

# 3. Update anchor
content = content.replace(
    'anchor_path = f"{stem_path}/Internode_r{internodes[0].key.rank}"',
    'anchor_path = f"{stem_path}/Internode_o{internodes[0].key.order}_r{internodes[0].key.rank}"'
)

# 4. Update physics loop
old_physics_path = """    for i, node in enumerate(internodes):
        L = node.length
        R = node.width_m / 2.0
        path = f"{stem_path}/Internode_r{node.key.rank}"

        # Mass: cylinder volume * density
        mass = math.pi * R**2 * L * STEM_DENSITY_KG_M3
        _apply_rigid_body(stage, path, mass)

        # Stiffness linearly decreases from bottom to the top
        t = i / max(n_ranks - 1, 1)
        stiffness = JOINT_STIFFNESS_BASE + t * (JOINT_STIFFNESS_TIP - JOINT_STIFFNESS_BASE)

        # Joint with previous
        if i > 0:
            prev_path = f"{stem_path}/Internode_r{internodes[i-1].key.rank}"
            _make_stem_joint(
                stage,
                joint_path=f"{joints_path}/Joint_r{node.key.rank}",
                body0_path=prev_path,
                body1_path=path,
                pivot_z=internodes[i-1].length / 2.0,
                stiffness=stiffness,
            )"""
new_physics_path = """    for i, node in enumerate(internodes):
        L = node.length
        R = node.width_m / 2.0
        path = f"{stem_path}/Internode_o{node.key.order}_r{node.key.rank}"

        # Mass: cylinder volume * density
        mass = math.pi * R**2 * L * STEM_DENSITY_KG_M3
        _apply_rigid_body(stage, path, mass)

        # Stiffness linearly decreases from bottom to the top
        t = i / max(n_ranks - 1, 1)
        stiffness = JOINT_STIFFNESS_BASE + t * (JOINT_STIFFNESS_TIP - JOINT_STIFFNESS_BASE)

        # Joint with parent
        if node.parent and isinstance(node.parent, InternodeNode):
            prev_path = f"{stem_path}/Internode_o{node.parent.key.order}_r{node.parent.key.rank}"
            _make_stem_joint(
                stage,
                joint_path=f"{joints_path}/Joint_o{node.key.order}_r{node.key.rank}",
                body0_path=prev_path,
                body1_path=path,
                pivot_z=node.parent.length / 2.0,
                stiffness=stiffness,
            )"""
content = content.replace(old_physics_path, new_physics_path)

# 5. Update Leaves loop
old_leaves = """    # ── Leaves ───────────────────────────────────────────────────────────────
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode)
                and n.key.order == 0]

    # Build rank→tip_z map from the internode chain
    rank_tip_z = {}
    z = 0.0
    for node in internodes:          # already sorted by rank
        z += node.length
        rank_tip_z[node.key.rank] = z

    leaves_path = f"{plant_path}/Leaves"
    UsdGeom.Xform.Define(stage, leaves_path)

    for node in leaves:
        tip_z = rank_tip_z.get(node.key.rank, 0.0)
        leaf_id = f"r{node.key.rank}_i{node.key.organ_index}"
        leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
        UsdGeom.Xform.Define(stage, leaf_group)

        print(f"\\n[LEAF rank={node.key.rank} idx={node.key.organ_index}] "
                f"attaches at z={tip_z:.4f}m")
        _make_leaf(stage, leaf_group, node, tip_z, materials)"""
new_leaves = """    # ── Leaves ───────────────────────────────────────────────────────────────
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode)]

    leaves_path = f"{plant_path}/Leaves"
    UsdGeom.Xform.Define(stage, leaves_path)

    for node in leaves:
        if node.parent and isinstance(node.parent, InternodeNode):
            tip_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
        else:
            tip_z = 0.0

        leaf_id = f"o{node.key.order}_r{node.key.rank}_i{node.key.organ_index}"
        leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
        UsdGeom.Xform.Define(stage, leaf_group)

        print(f"\\n[LEAF order={node.key.order} rank={node.key.rank} idx={node.key.organ_index}] "
                f"attaches at z={tip_z:.4f}m")
        _make_leaf(stage, leaf_group, node, tip_z, materials)"""
content = content.replace(old_leaves, new_leaves)

# 6. Update Fruits loop
old_fruits = """    for node in fruits_nodes:
        tip_z    = rank_tip_z.get(node.key.rank, 0.0)
        truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)
        tilt     = math.radians(90 - node.truss_angle)   # from stem (Z), small angle"""
new_fruits = """    for node in fruits_nodes:
        if node.parent and isinstance(node.parent, InternodeNode):
            tip_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
        else:
            tip_z = 0.0
        truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)
        tilt     = math.radians(90 - node.truss_angle)   # from stem (Z), small angle"""
content = content.replace(old_fruits, new_fruits)

# 7. Update final print
content = content.replace('print(f"\\n  Total stem height: {current_z:.6f} m\\n")', 'print(f"\\n  Max stem height: {max_z:.6f} m\\n")')

with open('src/plant_model/usd_exporter.py', 'w') as f:
    f.write(content)

print("Applied Z-only patch")
