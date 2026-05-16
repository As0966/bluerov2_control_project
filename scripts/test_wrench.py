#!/usr/bin/env python3
"""
Standalone test: Apply a constant force to BlueROV2 base_link via gz-transport.
Run this WHILE Gazebo is running to see if the robot moves.

Usage:
  1. Start Gazebo in one terminal:
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     source /opt/ros/humble/setup.bash && source ~/bluerov2_control_project/ros2_ws/install/setup.bash
     ros2 launch bluerov2_gazebo start_simulation.launch.py

  2. Run this test in another terminal:
     python3 ~/bluerov2_control_project/test_wrench.py
"""
import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import time
import sys

# ── Try gz-transport first ──────────────────────────────────────
print("=" * 60)
print("  BlueROV2 Wrench Test — Direct gz-transport")
print("=" * 60)

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.entity_wrench_pb2 import EntityWrench
    from gz.msgs10.entity_pb2 import Entity
    print("[OK] gz-transport13 imported successfully")
except ImportError as e:
    print(f"[FAIL] Cannot import gz-transport: {e}")
    sys.exit(1)

# ── Create publisher ────────────────────────────────────────────
gz_node = GzNode()
topic = '/world/underwater_world/wrench'
pub = gz_node.advertise(topic, EntityWrench)

print(f"[OK] Publishing on: {topic}")
print()

# ── Test different entity names ─────────────────────────────────
test_configs = [
    ('bluerov2::base_link', Entity.LINK),
    ('base_link', Entity.LINK),
    ('bluerov2', Entity.MODEL),
]

for entity_name, entity_type in test_configs:
    type_str = 'LINK' if entity_type == Entity.LINK else 'MODEL'
    print(f"\n--- Testing: name='{entity_name}', type={type_str} ---")
    print(f"    Applying Fx=20N for 3 seconds...")

    for i in range(150):  # 3 seconds at 50Hz
        ew = EntityWrench()
        ew.entity.name = entity_name
        ew.entity.type = entity_type

        ew.wrench.force.x = 20.0  # Push forward
        ew.wrench.force.y = 0.0
        ew.wrench.force.z = 0.0
        ew.wrench.torque.x = 0.0
        ew.wrench.torque.y = 0.0
        ew.wrench.torque.z = 0.0

        pub.publish(ew)
        time.sleep(0.02)

    print(f"    Done. Check Gazebo — did the robot move?")
    print(f"    Waiting 3s before next test...")
    time.sleep(3)

print("\n" + "=" * 60)
print("  Test complete!")
print("  If the robot moved during ANY test, note which entity name worked.")
print("=" * 60)
