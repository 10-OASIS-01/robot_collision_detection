import xml.etree.ElementTree as ET
import numpy as np
import os
from ..core.robot import Robot
from ..core.stl_loader import load_stl


def _parse_origin(element):
    origin = element.find("origin")
    if origin is None:
        return np.eye(4)

    xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
    rpy = np.array([float(v) for v in origin.get("rpy", "0 0 0").split()])

    cr, sr = np.cos(rpy[0]), np.sin(rpy[0])
    cp, sp = np.cos(rpy[1]), np.sin(rpy[1])
    cy, sy = np.cos(rpy[2]), np.sin(rpy[2])

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def _axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(4)
    axis = axis / norm

    c, s = np.cos(angle), np.sin(angle)
    t = 1 - c
    x, y, z = axis

    R = np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])

    T = np.eye(4)
    T[:3, :3] = R
    return T


class URDFRobot(Robot):
    """Robot loaded from a URDF file.

    Parses the kinematic chain of revolute joints and automatically generates
    capsule collision geometry (one capsule per link connecting parent and child
    joint origins) and sphere geometry (one sphere per joint).
    """

    def __init__(self, urdf_path, base_transform=np.eye(4), name=None,
                 capsule_radius=0.05, auto_radius=False):
        self._urdf_joints = []
        self._joint_names = []
        self._joint_limits = []
        self._capsule_radius = capsule_radius
        self._auto_radius = auto_radius
        self._link_meshes = {}
        self._urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        self._parse_urdf(urdf_path)

        dummy_dh = []
        if name is None:
            name = self._robot_name

        self.dh_params = dummy_dh
        self.base_transform = base_transform
        self.name = name
        self.init_geometric_model()

    def _resolve_mesh_path(self, filename):
        if filename.startswith("package://"):
            rel = filename[len("package://"):]
            return os.path.join(self._urdf_dir, rel)
        if os.path.isabs(filename):
            return filename
        return os.path.join(self._urdf_dir, filename)

    def _parse_link_meshes(self, link_elem, link_name):
        entry = {"visual": [], "collision": []}
        for mesh_type in ("visual", "collision"):
            for geom_parent in link_elem.findall(mesh_type):
                origin_T = _parse_origin(geom_parent)
                geom = geom_parent.find("geometry")
                if geom is None:
                    continue
                mesh_elem = geom.find("mesh")
                if mesh_elem is None:
                    continue
                filename = mesh_elem.get("filename", "")
                resolved = self._resolve_mesh_path(filename)
                if os.path.isfile(resolved) and resolved.lower().endswith(".stl"):
                    try:
                        verts, faces = load_stl(resolved)
                        entry[mesh_type].append({
                            "vertices": verts,
                            "faces": faces,
                            "origin": origin_T,
                        })
                    except Exception:
                        pass
        self._link_meshes[link_name] = entry

    def _parse_urdf(self, urdf_path):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        self._robot_name = root.get("name", "URDFRobot")

        links = {}
        for link in root.findall("link"):
            lname = link.get("name")
            links[lname] = link
            self._parse_link_meshes(link, lname)

        joints_by_parent = {}
        all_joints = []
        for joint in root.findall("joint"):
            jtype = joint.get("type")
            jname = joint.get("name")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin_T = _parse_origin(joint)

            axis_elem = joint.find("axis")
            axis = np.array([0, 0, 1.0])
            if axis_elem is not None:
                axis = np.array([float(v) for v in axis_elem.get("xyz", "0 0 1").split()])

            limit_elem = joint.find("limit")
            lower, upper = None, None
            if limit_elem is not None:
                lower = float(limit_elem.get("lower", 0))
                upper = float(limit_elem.get("upper", 0))

            jinfo = {
                "name": jname,
                "type": jtype,
                "parent": parent,
                "child": child,
                "origin": origin_T,
                "axis": axis,
                "lower": lower,
                "upper": upper,
            }
            all_joints.append(jinfo)
            joints_by_parent.setdefault(parent, []).append(jinfo)

        child_links = {j["child"] for j in all_joints}
        root_links = [l for l in links if l not in child_links]
        if not root_links:
            root_links = [all_joints[0]["parent"]] if all_joints else []

        chain = []
        current_link = root_links[0]
        visited = set()
        while current_link in joints_by_parent and current_link not in visited:
            visited.add(current_link)
            candidates = joints_by_parent[current_link]
            best = None
            for c in candidates:
                if c["type"] in ("revolute", "continuous", "prismatic"):
                    best = c
                    break
            if best is None:
                for c in candidates:
                    if c["type"] == "fixed":
                        best = c
                        break
            if best is None:
                break
            chain.append(best)
            current_link = best["child"]

        self._urdf_joints = chain
        self._joint_names = [j["name"] for j in chain if j["type"] in ("revolute", "continuous", "prismatic")]
        self._joint_limits = [
            (j["lower"], j["upper"])
            for j in chain
            if j["type"] in ("revolute", "continuous", "prismatic")
        ]

        # Build link name -> chain index mapping
        # Index 0 = root link, index i+1 = child of chain[i]
        self._link_chain_index = {}
        if root_links:
            self._link_chain_index[root_links[0]] = 0
        for i, joint in enumerate(chain):
            self._link_chain_index[joint["child"]] = i + 1

    @property
    def num_joints(self):
        return len(self._joint_names)

    @property
    def joint_names(self):
        return list(self._joint_names)

    @property
    def joint_limits(self):
        return list(self._joint_limits)

    def _compute_link_radius(self, link_name):
        entry = self._link_meshes.get(link_name, {})
        meshes = entry.get("collision", []) or entry.get("visual", [])
        if not meshes:
            return None
        all_verts = np.vstack([m["vertices"] for m in meshes])
        extents = all_verts.max(axis=0) - all_verts.min(axis=0)
        sorted_ext = np.sort(extents)
        return float(max(sorted_ext[0], sorted_ext[1]) / 2.0)

    def _get_chain_link_names(self):
        names = []
        for lname, idx in sorted(self._link_chain_index.items(), key=lambda x: x[1]):
            names.append(lname)
        return names

    def init_geometric_model(self):
        n = len(self._urdf_joints)

        if self._auto_radius:
            chain_links = self._get_chain_link_names()
            link_radii = []
            for lname in chain_links:
                r = self._compute_link_radius(lname)
                link_radii.append(r if r is not None else self._capsule_radius)
        else:
            link_radii = [self._capsule_radius] * (n + 1)

        self.spheres = [
            [[0.0, 0.0, 0.0], link_radii[i] * 1.2] for i in range(n + 1)
        ]
        self.capsules = [
            [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], link_radii[i]]
            for i in range(n)
        ]

    def forward_kinematics(self, joint_angles):
        transforms = [self.base_transform.copy()]
        q_idx = 0

        for joint in self._urdf_joints:
            T_parent = transforms[-1]
            T_joint = joint["origin"]

            if joint["type"] in ("revolute", "continuous"):
                if q_idx < len(joint_angles):
                    T_rot = _axis_angle_matrix(joint["axis"], joint_angles[q_idx])
                    T_child = T_parent @ T_joint @ T_rot
                    q_idx += 1
                else:
                    T_child = T_parent @ T_joint
            elif joint["type"] == "prismatic":
                if q_idx < len(joint_angles):
                    T_trans = np.eye(4)
                    T_trans[:3, 3] = joint["axis"] * joint_angles[q_idx]
                    T_child = T_parent @ T_joint @ T_trans
                    q_idx += 1
                else:
                    T_child = T_parent @ T_joint
            else:
                T_child = T_parent @ T_joint

            transforms.append(T_child)

        return transforms

    def update_geometric_model(self, transforms):
        for i in range(min(len(self.spheres), len(transforms))):
            self.spheres[i][0] = transforms[i][:3, 3]

        for j in range(min(len(self.capsules), len(transforms) - 1)):
            self.capsules[j][0] = transforms[j][:3, 3]
            self.capsules[j][1] = transforms[j + 1][:3, 3]

    def get_transformed_meshes(self, transforms, mesh_type="collision"):
        """Return world-frame mesh data for each link.

        Args:
            transforms: List of 4x4 transforms from forward_kinematics
            mesh_type: "collision" or "visual"

        Returns:
            List of (vertices_world, faces) tuples
        """
        result = []
        for link_name, meshes in self._link_meshes.items():
            idx = self._link_chain_index.get(link_name)
            if idx is None or idx >= len(transforms):
                continue
            T_world = transforms[idx]
            for mesh_data in meshes.get(mesh_type, []):
                verts_local = mesh_data["vertices"]
                T_mesh_origin = mesh_data["origin"]
                T_full = T_world @ T_mesh_origin
                ones = np.ones((len(verts_local), 1))
                verts_h = np.hstack([verts_local, ones])
                verts_world = (T_full @ verts_h.T).T[:, :3]
                result.append((verts_world, mesh_data["faces"]))
        return result
