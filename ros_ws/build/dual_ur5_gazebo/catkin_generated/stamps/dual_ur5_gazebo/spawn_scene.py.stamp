#!/usr/bin/env python3
import glob
import math
import os

import rospkg
import rospy
import xacro
import yaml
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler


def _iter_entities(raw, default_prefix):
    if raw is None:
        return []

    entities = []
    if isinstance(raw, dict):
        for name, cfg in raw.items():
            entry = cfg or {}
            entry["name"] = name
            entities.append(entry)
        return entities

    if isinstance(raw, list):
        for idx, cfg in enumerate(raw):
            entry = cfg or {}
            entry.setdefault("name", f"{default_prefix}_{idx}")
            entities.append(entry)
        return entities

    raise ValueError("Expected a dict or list for scene entities.")


def _load_scene(scene_path):
    with open(scene_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("Scene config must be a YAML mapping.")
    return data


def _unit_factor(units):
    normalized = (units or "m").strip().lower()
    factors = {
        "m": 1.0,
        "meter": 1.0,
        "meters": 1.0,
        "cm": 0.01,
        "centimeter": 0.01,
        "centimeters": 0.01,
        "mm": 0.001,
        "millimeter": 0.001,
        "millimeters": 0.001,
    }
    if normalized not in factors:
        raise ValueError(f"Unsupported units '{units}'. Use m, cm, or mm.")
    return factors[normalized]


def _angle_value(pose_cfg, axis):
    deg_key = f"{axis}_deg"
    if deg_key in pose_cfg:
        return math.radians(float(pose_cfg[deg_key]))
    return float(pose_cfg.get(axis, 0.0))


def _pose_from_cfg(pose_cfg, factor):
    cfg = pose_cfg or {}
    x = float(cfg.get("x", 0.0)) * factor
    y = float(cfg.get("y", 0.0)) * factor
    z = float(cfg.get("z", 0.0)) * factor

    roll = _angle_value(cfg, "roll")
    pitch = _angle_value(cfg, "pitch")
    yaw = _angle_value(cfg, "yaw")
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

    return Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))


def _delete_if_exists(existing_models, delete_model, model_name):
    if model_name not in existing_models:
        return
    try:
        delete_model(model_name)
        existing_models.discard(model_name)
    except rospy.ServiceException as exc:
        rospy.logdebug("Delete model '%s' skipped: %s", model_name, str(exc))


def _get_existing_models(get_world_properties):
    response = get_world_properties()
    return set(response.model_names)


def _xacro_discovery_roots(rospack):
    roots = []
    for pkg in ("ur_description", "ur_gazebo"):
        try:
            roots.append((pkg, rospack.get_path(pkg)))
        except rospkg.ResourceNotFound:
            continue
    return roots


def _xacro_priority(path):
    basename = os.path.basename(path)
    priorities = {
        "ur5_robot.urdf.xacro": 0,
        "ur5_joint_limited_robot.urdf.xacro": 1,
        "ur5.urdf.xacro": 2,
        "ur5.xacro": 3,
        "ur.urdf.xacro": 4,
    }
    return priorities.get(basename, 10)


def _is_candidate_ur5_xacro(path):
    normalized = path.lower()
    basename = os.path.basename(normalized)
    if "/inc/" in normalized:
        return False
    if "macro" in basename and "robot" not in basename:
        return False
    if "ur5" in basename:
        return True
    if basename == "ur.urdf.xacro":
        return True
    return False


def _find_ur5_xacro():
    rospack = rospkg.RosPack()
    roots = _xacro_discovery_roots(rospack)
    if not roots:
        raise RuntimeError("Could not find ur_description or ur_gazebo packages.")

    discovered = []
    for _, root in roots:
        for rel in (
            "urdf/ur5_robot.urdf.xacro",
            "urdf/ur5_joint_limited_robot.urdf.xacro",
            "urdf/ur5.urdf.xacro",
            "urdf/ur5.xacro",
            "urdf/ur.urdf.xacro",
        ):
            discovered.append(os.path.join(root, rel))

    for _, root in roots:
        discovered.extend(glob.glob(os.path.join(root, "urdf", "**", "*.xacro"), recursive=True))

    candidates = []
    seen = set()
    for path in sorted(discovered, key=_xacro_priority):
        if not os.path.exists(path):
            continue
        if not _is_candidate_ur5_xacro(path):
            continue
        if path in seen:
            continue
        seen.add(path)
        candidates.append(path)

    if candidates:
        return candidates[0]

    roots_str = ", ".join(root for _, root in roots)
    raise RuntimeError(f"Could not find a UR5 xacro under: {roots_str}")


def _render_urdf(xacro_path, name, prefix):
    full_mapping = {
        "name": name,
        "prefix": prefix,
        "ur_type": "ur5",
        "limited": "true",
        "joint_limited": "true",
        "safety_limits": "false",
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "transmission_hw_interface": "hardware_interface/PositionJointInterface",
        "robot_ip": "0.0.0.0",
    }
    mappings_to_try = [
        full_mapping,
        {"name": name, "prefix": prefix, "ur_type": "ur5", "limited": "true", "joint_limited": "true"},
        {"name": name, "prefix": prefix},
        {"prefix": prefix, "ur_type": "ur5"},
        {"ur_type": "ur5"},
        {"prefix": prefix},
        {"name": name},
        {},
    ]

    last_exc = None
    for mapping in mappings_to_try:
        try:
            doc = xacro.process_file(xacro_path, mappings=mapping)
            return doc.toxml()
        except Exception as exc:  # pylint: disable=broad-except
            last_exc = exc

    raise RuntimeError(f"Unable to render UR5 URDF from {xacro_path}: {last_exc}")


def _spawn_objects(scene, pkg_path, factor, spawn_sdf, delete_model, existing_models):
    objects = _iter_entities(scene.get("objects"), "object")
    for obj in objects:
        name = obj["name"]
        model_key = obj.get("model", name)
        model_path = os.path.join(pkg_path, "models", model_key, "model.sdf")
        if not os.path.exists(model_path):
            raise RuntimeError(f"Object '{name}' references missing model '{model_key}' at {model_path}")

        with open(model_path, "r", encoding="utf-8") as f:
            model_xml = f.read()

        pose = _pose_from_cfg(obj.get("pose"), factor)
        _delete_if_exists(existing_models, delete_model, name)
        response = spawn_sdf(name, model_xml, f"/{name}", pose, "world")
        if not response.success:
            raise RuntimeError(f"Failed to spawn object '{name}': {response.status_message}")
        existing_models.add(name)

        rospy.loginfo("Spawned object '%s' from model '%s'.", name, model_key)


def _spawn_robots(scene, factor, spawn_urdf, delete_model, existing_models):
    robots = _iter_entities(scene.get("robots"), "robot")
    if not robots:
        rospy.logwarn("No robots defined in scene config.")
        return

    xacro_path = scene.get("ur5_xacro") or _find_ur5_xacro()
    rospy.loginfo("Using UR5 xacro: %s", xacro_path)

    for robot in robots:
        name = robot["name"]
        namespace = robot.get("namespace", f"/{name}")
        prefix = robot.get("prefix", f"{name}_")
        pose = _pose_from_cfg(robot.get("pose"), factor)

        urdf_xml = _render_urdf(xacro_path, name=name, prefix=prefix)
        _delete_if_exists(existing_models, delete_model, name)
        response = spawn_urdf(name, urdf_xml, namespace, pose, "world")
        if not response.success:
            raise RuntimeError(f"Failed to spawn robot '{name}': {response.status_message}")
        existing_models.add(name)

        rospy.loginfo("Spawned UR5 '%s' in namespace '%s'.", name, namespace)


def main():
    rospy.init_node("spawn_scene")

    scene_config = rospy.get_param("~scene_config")
    if not scene_config:
        raise RuntimeError("Parameter '~scene_config' is required.")
    if not os.path.exists(scene_config):
        raise RuntimeError(f"Scene config not found: {scene_config}")

    scene = _load_scene(scene_config)
    factor = _unit_factor(scene.get("units", "m"))

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("dual_ur5_gazebo")

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/get_world_properties")

    spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

    existing_models = _get_existing_models(get_world_properties)

    _spawn_objects(scene, pkg_path, factor, spawn_sdf, delete_model, existing_models)
    _spawn_robots(scene, factor, spawn_urdf, delete_model, existing_models)
    rospy.loginfo("Scene spawn complete from %s", scene_config)


if __name__ == "__main__":
    main()
