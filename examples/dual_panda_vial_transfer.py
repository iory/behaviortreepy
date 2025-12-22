#!/usr/bin/env python
"""
Dual Panda Robot Vial Transfer Example using BehaviorTree.

This example demonstrates transferring flies from vial B to vial A
using two Panda robots coordinated by a behavior tree.

Procedure:
1. Remove cap from vial A
2. Tap vial B on table to move flies down
3. Remove cap from vial B
4. Join vial A and B (B on bottom)
5. Flip A and B upside down
6. Tap vial A on table to transfer flies
7. Separate vials A and B
8. Cap vial A
9. Cap vial B
"""

import argparse
import time

import numpy as np
import skrobot
from skrobot.coordinates import Coordinates
from skrobot.coordinates.geo import midcoords
from skrobot.coordinates.math import rotation_matrix
from skrobot.model.primitives import Box
from skrobot.model.primitives import Cylinder

import behaviortreepy as bt


# Rotation matrix for end effector pointing downward (rotate 180 deg around X-axis)
DOWNWARD_ROT = rotation_matrix(np.pi, [1, 0, 0])[:3, :3]


# Global state
viewer = None
robot_a = None  # Robot holding vial A
robot_b = None  # Robot holding vial B
vial_a = None
vial_b = None
cap_a = None
cap_b = None
table = None

# Vial parameters
VIAL_RADIUS = 0.012
VIAL_HEIGHT = 0.095
CAP_RADIUS = 0.013
CAP_HEIGHT = 0.015

# Robot positions
ROBOT_A_POS = np.array([0.0, -0.3, 0.0])
ROBOT_B_POS = np.array([0.0, 0.3, 0.0])

# Work positions
TABLE_HEIGHT = 0.4
WORK_HEIGHT = 0.6


def create_vial(name, color):
    """Create a vial (cylinder) with specified color."""
    vial = Cylinder(
        radius=VIAL_RADIUS,
        height=VIAL_HEIGHT,
        face_colors=color,
        name=name
    )
    return vial


def create_cap(name, color):
    """Create a cap (cylinder) with specified color."""
    cap = Cylinder(
        radius=CAP_RADIUS,
        height=CAP_HEIGHT,
        face_colors=color,
        name=name
    )
    return cap


def setup_scene():
    """Setup the scene with robots, vials, and table."""
    global viewer, robot_a, robot_b, vial_a, vial_b, cap_a, cap_b, table

    # Create robots
    robot_a = skrobot.models.Panda()
    robot_a.translate(ROBOT_A_POS)
    robot_a.reset_pose()

    robot_b = skrobot.models.Panda()
    robot_b.translate(ROBOT_B_POS)
    robot_b.reset_pose()

    # Create table
    table = Box(extents=[0.8, 0.8, 0.02], face_colors=[139, 90, 43, 255])
    table.translate([0.5, 0.0, TABLE_HEIGHT - 0.01])

    # Create vials
    vial_a = create_vial("vial_a", [200, 200, 255, 200])  # Light blue
    vial_a.translate([0.4, -0.15, TABLE_HEIGHT + VIAL_HEIGHT / 2])

    vial_b = create_vial("vial_b", [255, 200, 200, 200])  # Light red
    vial_b.translate([0.4, 0.15, TABLE_HEIGHT + VIAL_HEIGHT / 2])

    # Create caps
    cap_a = create_cap("cap_a", [100, 100, 200, 255])  # Dark blue
    cap_a.translate([0.4, -0.15, TABLE_HEIGHT + VIAL_HEIGHT + CAP_HEIGHT / 2])

    cap_b = create_cap("cap_b", [200, 100, 100, 255])  # Dark red
    cap_b.translate([0.4, 0.15, TABLE_HEIGHT + VIAL_HEIGHT + CAP_HEIGHT / 2])

    # Setup viewer
    viewer = skrobot.viewers.PyrenderViewer(resolution=(1024, 768), update_interval=1 / 30.0)
    viewer.add(robot_a)
    viewer.add(robot_b)
    viewer.add(table)
    viewer.add(vial_a)
    viewer.add(vial_b)
    viewer.add(cap_a)
    viewer.add(cap_b)

    viewer.show()
    viewer.set_camera([np.deg2rad(30), np.deg2rad(0), np.deg2rad(90)], distance=1.5)


def move_robot_to_target(robot, target_coords, steps=20, sleep_time=0.05):
    """Move robot end effector to target coordinates."""
    link_list = robot.rarm.link_list
    move_target = robot.rarm.end_coords
    start_coords = move_target.copy_worldcoords()

    for i in range(steps):
        t = (i + 1) / steps
        intermediate = midcoords(t, start_coords, target_coords)
        robot.inverse_kinematics(
            intermediate,
            link_list=link_list,
            move_target=move_target,
            rotation_axis=True
        )
        viewer.redraw()
        time.sleep(sleep_time)


def attach_object_to_robot(robot, obj):
    """Attach object to robot's end effector."""
    robot.rarm.end_coords.assoc(obj)


def detach_object_from_robot(robot, obj):
    """Detach object from robot's end effector."""
    robot.rarm.end_coords.dissoc(obj)


# ============================================================================
# Behavior Tree Action Callbacks
# ============================================================================

def action_remove_cap_a(node):
    """Remove cap from vial A."""
    print("[Action] Removing cap from vial A...")

    # Move robot A to cap A position (offset by half cap height to reach top)
    cap_pos = cap_a.worldpos()
    grasp_z = cap_pos[2] + CAP_HEIGHT / 2
    target = Coordinates(pos=[cap_pos[0], cap_pos[1], grasp_z + 0.05], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    # Grasp cap
    target = Coordinates(pos=[cap_pos[0], cap_pos[1], grasp_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target, steps=10)
    attach_object_to_robot(robot_a, cap_a)

    # Lift cap
    target = Coordinates(pos=[cap_pos[0], cap_pos[1] - 0.1, cap_pos[2] + 0.1], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    print("[Action] Cap A removed")
    return bt.NodeStatus.SUCCESS


def action_tap_vial_b(node):
    """Tap vial B on table to move flies down."""
    print("[Action] Tapping vial B on table...")

    # Move robot B to vial B (offset by half vial height to reach top)
    vial_pos = vial_b.worldpos()
    grasp_z = vial_pos[2] + VIAL_HEIGHT / 2
    target = Coordinates(pos=[vial_pos[0], vial_pos[1], grasp_z + 0.05], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target)

    # Grasp vial
    target = Coordinates(pos=[vial_pos[0], vial_pos[1], grasp_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target, steps=10)
    attach_object_to_robot(robot_b, vial_b)
    attach_object_to_robot(robot_b, cap_b)

    # Tap motion (up and down)
    for _ in range(3):
        target = Coordinates(pos=[vial_pos[0], vial_pos[1], grasp_z + 0.1], rot=DOWNWARD_ROT)
        move_robot_to_target(robot_b, target, steps=5)
        target = Coordinates(pos=[vial_pos[0], vial_pos[1], TABLE_HEIGHT + VIAL_HEIGHT / 2 + 0.05], rot=DOWNWARD_ROT)
        move_robot_to_target(robot_b, target, steps=5)

    # Return to original position
    target = Coordinates(pos=[vial_pos[0], vial_pos[1], grasp_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target, steps=10)

    print("[Action] Vial B tapped")
    return bt.NodeStatus.SUCCESS


def action_remove_cap_b(node):
    """Remove cap from vial B (robot A helps)."""
    print("[Action] Removing cap from vial B...")

    # First place cap A aside
    aside_pos = [0.3, -0.2, WORK_HEIGHT]
    target = Coordinates(pos=aside_pos, rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)
    detach_object_from_robot(robot_a, cap_a)
    cap_a.translate(aside_pos - cap_a.worldpos())

    # Move to cap B (offset by half cap height to reach top)
    cap_pos = cap_b.worldpos()
    grasp_z = cap_pos[2] + CAP_HEIGHT / 2
    target = Coordinates(pos=[cap_pos[0], cap_pos[1], grasp_z + 0.05], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    target = Coordinates(pos=[cap_pos[0], cap_pos[1], grasp_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target, steps=10)

    # Detach cap from vial B (robot B's grasp) and attach to robot A
    detach_object_from_robot(robot_b, cap_b)
    attach_object_to_robot(robot_a, cap_b)

    # Lift cap
    target = Coordinates(pos=[cap_pos[0], cap_pos[1] + 0.1, cap_pos[2] + 0.1], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    print("[Action] Cap B removed")
    return bt.NodeStatus.SUCCESS


def action_join_vials(node):
    """Join vial A and B with B on bottom."""
    print("[Action] Joining vials A and B...")

    # Robot A picks up vial A
    vial_a_pos = vial_a.worldpos()
    grasp_z = vial_a_pos[2] + VIAL_HEIGHT / 2

    # Place cap B aside first
    aside_pos = [0.3, 0.2, WORK_HEIGHT]
    target = Coordinates(pos=aside_pos, rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)
    detach_object_from_robot(robot_a, cap_b)
    cap_b.translate(aside_pos - cap_b.worldpos())

    # Move to vial A (offset by half vial height to reach top)
    target = Coordinates(pos=[vial_a_pos[0], vial_a_pos[1], grasp_z + 0.05], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    target = Coordinates(pos=[vial_a_pos[0], vial_a_pos[1], grasp_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target, steps=10)
    attach_object_to_robot(robot_a, vial_a)

    # Lift vial A
    target = Coordinates(pos=[vial_a_pos[0], vial_a_pos[1], WORK_HEIGHT + 0.1], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    # Robot B lifts vial B
    target = Coordinates(pos=[0.45, 0.0, WORK_HEIGHT], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target)

    # Robot A moves vial A above vial B
    target = Coordinates(pos=[0.45, 0.0, WORK_HEIGHT + VIAL_HEIGHT + 0.02], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    print("[Action] Vials joined (B on bottom, A on top)")
    return bt.NodeStatus.SUCCESS


def action_flip_vials(node):
    """Flip vials upside down (A becomes bottom)."""
    print("[Action] Flipping vials...")

    # Coordinated rotation (simplified - just swap positions)
    # In reality, this would require coordinated dual-arm motion

    # Move both robots up
    current_a = robot_a.rarm.end_coords.worldpos()
    current_b = robot_b.rarm.end_coords.worldpos()

    # Rotate 180 degrees around horizontal axis
    # For simplicity, we simulate by swapping vertical positions
    target_a = Coordinates(pos=[0.45, 0.0, WORK_HEIGHT], rot=DOWNWARD_ROT)
    target_b = Coordinates(pos=[0.45, 0.0, WORK_HEIGHT + VIAL_HEIGHT + 0.02], rot=DOWNWARD_ROT)

    # Move simultaneously (simplified)
    for i in range(20):
        t = (i + 1) / 20
        coords_a = midcoords(t, Coordinates(pos=current_a, rot=DOWNWARD_ROT), target_a)
        coords_b = midcoords(t, Coordinates(pos=current_b, rot=DOWNWARD_ROT), target_b)
        move_robot_to_target(robot_a, coords_a, steps=1, sleep_time=0)
        move_robot_to_target(robot_b, coords_b, steps=1, sleep_time=0)
        viewer.redraw()
        time.sleep(0.05)

    print("[Action] Vials flipped (A now on bottom)")
    return bt.NodeStatus.SUCCESS


def action_tap_vial_a(node):
    """Tap vial A on table to transfer flies."""
    print("[Action] Tapping vial A to transfer flies...")

    # Tap motion
    current_pos = robot_a.rarm.end_coords.worldpos()

    for _ in range(3):
        target = Coordinates(pos=[current_pos[0], current_pos[1], current_pos[2] + 0.05], rot=DOWNWARD_ROT)
        move_robot_to_target(robot_a, target, steps=5)
        target = Coordinates(pos=[current_pos[0], current_pos[1], TABLE_HEIGHT + 0.05], rot=DOWNWARD_ROT)
        move_robot_to_target(robot_a, target, steps=5)

    # Return to position
    target = Coordinates(pos=current_pos, rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target, steps=10)

    print("[Action] Flies transferred to vial A")
    return bt.NodeStatus.SUCCESS


def action_separate_vials(node):
    """Separate vials A and B."""
    print("[Action] Separating vials...")

    # Robot B moves vial B away
    target = Coordinates(pos=[0.4, 0.15, WORK_HEIGHT], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target)

    # Robot A moves vial A away
    target = Coordinates(pos=[0.4, -0.15, WORK_HEIGHT], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    print("[Action] Vials separated")
    return bt.NodeStatus.SUCCESS


def action_cap_vial_a(node):
    """Put cap back on vial A."""
    print("[Action] Capping vial A...")

    # Robot B picks up cap A (it was placed aside earlier)
    detach_object_from_robot(robot_b, vial_b)
    vial_b_pos = robot_b.rarm.end_coords.worldpos()
    vial_b.translate(vial_b_pos - vial_b.worldpos())

    # Offset by half cap height to reach top
    cap_a_pos = cap_a.worldpos()
    grasp_z = cap_a_pos[2] + CAP_HEIGHT / 2
    target = Coordinates(pos=[cap_a_pos[0], cap_a_pos[1], grasp_z + 0.05], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target)

    target = Coordinates(pos=[cap_a_pos[0], cap_a_pos[1], grasp_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target, steps=10)
    attach_object_to_robot(robot_b, cap_a)

    # Move cap to vial A
    vial_a_pos = vial_a.worldpos()
    cap_place_z = vial_a_pos[2] + VIAL_HEIGHT / 2 + CAP_HEIGHT / 2
    target = Coordinates(pos=[vial_a_pos[0], vial_a_pos[1], cap_place_z + 0.02], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target)

    # Place cap
    target = Coordinates(pos=[vial_a_pos[0], vial_a_pos[1], cap_place_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_b, target, steps=10)
    detach_object_from_robot(robot_b, cap_a)

    print("[Action] Vial A capped")
    return bt.NodeStatus.SUCCESS


def action_cap_vial_b(node):
    """Put cap back on vial B."""
    print("[Action] Capping vial B...")

    # Robot A releases vial A and picks up cap B
    detach_object_from_robot(robot_a, vial_a)
    vial_a_pos = robot_a.rarm.end_coords.worldpos()
    vial_a.translate(vial_a_pos - vial_a.worldpos())

    # Offset by half cap height to reach top
    cap_b_pos = cap_b.worldpos()
    grasp_z = cap_b_pos[2] + CAP_HEIGHT / 2
    target = Coordinates(pos=[cap_b_pos[0], cap_b_pos[1], grasp_z + 0.05], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    target = Coordinates(pos=[cap_b_pos[0], cap_b_pos[1], grasp_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target, steps=10)
    attach_object_to_robot(robot_a, cap_b)

    # Move cap to vial B
    vial_b_pos = vial_b.worldpos()
    cap_place_z = vial_b_pos[2] + VIAL_HEIGHT / 2 + CAP_HEIGHT / 2
    target = Coordinates(pos=[vial_b_pos[0], vial_b_pos[1], cap_place_z + 0.02], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target)

    # Place cap
    target = Coordinates(pos=[vial_b_pos[0], vial_b_pos[1], cap_place_z], rot=DOWNWARD_ROT)
    move_robot_to_target(robot_a, target, steps=10)
    detach_object_from_robot(robot_a, cap_b)

    print("[Action] Vial B capped")
    return bt.NodeStatus.SUCCESS


def action_return_to_home(node):
    """Return robots to home position."""
    print("[Action] Returning to home position...")

    robot_a.reset_pose()
    robot_b.reset_pose()
    viewer.redraw()

    print("[Action] Robots returned to home")
    return bt.NodeStatus.SUCCESS


def main():
    parser = argparse.ArgumentParser(
        description='Dual Panda vial transfer with BehaviorTree')
    parser.add_argument(
        '--no-interactive',
        action='store_true',
        help='Run without waiting for user input'
    )
    parser.add_argument(
        '--groot',
        action='store_true',
        help='Enable Groot2 visualization (requires ZeroMQ)'
    )
    parser.add_argument(
        '--groot-port',
        type=int,
        default=1667,
        help='Port for Groot2 publisher (default: 1667)'
    )
    args = parser.parse_args()

    # Setup scene
    print("Setting up scene...")
    setup_scene()
    time.sleep(1.0)

    # Create behavior tree factory
    factory = bt.BehaviorTreeFactory()

    # Register action nodes
    factory.register_simple_action("RemoveCapA", action_remove_cap_a)
    factory.register_simple_action("TapVialB", action_tap_vial_b)
    factory.register_simple_action("RemoveCapB", action_remove_cap_b)
    factory.register_simple_action("JoinVials", action_join_vials)
    factory.register_simple_action("FlipVials", action_flip_vials)
    factory.register_simple_action("TapVialA", action_tap_vial_a)
    factory.register_simple_action("SeparateVials", action_separate_vials)
    factory.register_simple_action("CapVialA", action_cap_vial_a)
    factory.register_simple_action("CapVialB", action_cap_vial_b)
    factory.register_simple_action("ReturnHome", action_return_to_home)

    # Define behavior tree XML
    tree_xml = """
    <root BTCPP_format="4" main_tree_to_execute="VialTransfer">
        <BehaviorTree ID="VialTransfer">
            <Sequence name="TransferSequence">
                <RemoveCapA name="Step1_RemoveCapA"/>
                <TapVialB name="Step2_TapVialB"/>
                <RemoveCapB name="Step3_RemoveCapB"/>
                <JoinVials name="Step4_JoinVials"/>
                <FlipVials name="Step5_FlipVials"/>
                <TapVialA name="Step6_TapVialA"/>
                <SeparateVials name="Step7_SeparateVials"/>
                <CapVialA name="Step8_CapVialA"/>
                <CapVialB name="Step9_CapVialB"/>
                <ReturnHome name="Step10_ReturnHome"/>
            </Sequence>
        </BehaviorTree>
    </root>
    """

    # Create and execute tree
    print("\n" + "=" * 60)
    print("Starting Vial Transfer Behavior Tree")
    print("=" * 60 + "\n")

    blackboard = bt.Blackboard.create()
    tree = factory.create_tree_from_text(tree_xml, blackboard)

    # Add logger (must keep reference alive for logging to work)
    _logger = bt.StdCoutLogger(tree)  # noqa: F841

    # Setup Groot2 publisher if requested (must keep reference alive)
    _groot_publisher = None
    if args.groot:
        if bt.GROOT2_AVAILABLE:
            _groot_publisher = bt.Groot2Publisher(tree, args.groot_port)  # noqa: F841
            print(f"Groot2 publisher started on port {args.groot_port}")
            print("Connect Groot2 to monitor the behavior tree in real-time")
        else:
            print("Warning: Groot2 is not available (ZeroMQ not found during build)")

    # Execute tree
    status = tree.tick_while_running()
    print(f"\nBehavior tree completed with status: {bt.to_string(status)}")

    # Keep viewer open
    if not args.no_interactive:
        print("\n==> Press [q] to close window")
        while viewer.is_active:
            time.sleep(0.1)
            viewer.redraw()

    viewer.close()
    time.sleep(0.5)


if __name__ == '__main__':
    main()
