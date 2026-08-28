#!/usr/bin/env python3
"""Replay C++ ASR-SDM simulation CSV data in Pinocchio + Meshcat."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import signal
import time

from meshcat_primitives import set_arrow, set_polyline
import numpy as np
import pinocchio as pin
from viewer_config import (
    COLOR_BUOYANCY,
    COLOR_DAMPING,
    COLOR_TOTAL_FLUID,
    COLOR_TRAJECTORY,
    FORCE_SCALE,
    LINK_FRAMES,
    MAX_TRAJECTORY_POINTS,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--mesh-root', type=Path, default=None)
    parser.add_argument('--stride', type=int, default=20)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--force-scale', type=float, default=FORCE_SCALE)
    parser.add_argument('--show-damping', action='store_true')
    parser.add_argument('--show-buoyancy', action='store_true')
    parser.add_argument('--show-total', action='store_true')
    parser.add_argument('--no-z-to-x-map', action='store_true')
    parser.add_argument('--follow', action='store_true')
    parser.add_argument('--replay-existing', action='store_true')
    parser.add_argument('--poll-period', type=float, default=0.05)
    parser.add_argument('--full-model', action='store_true')
    parser.add_argument('--no-open', action='store_true')
    return parser.parse_args()


def load_rows(path: Path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f'CSV contains no data rows: {path}')
    required = [
        'time_s',
        'q_base_x_m',
        'q_base_y_m',
        'q_base_z_m',
        'q_quat_x',
        'q_quat_y',
        'q_quat_z',
        'q_quat_w',
    ]
    required += [f'q_joint_{i}_rad' for i in range(6)]
    missing = [field for field in required if field not in rows[0]]
    if missing:
        missing_text = ', '.join(missing)
        raise RuntimeError(f'CSV is missing fields: {missing_text}')
    return rows


def configuration_from_row(row):
    values = [
        float(row['q_base_x_m']),
        float(row['q_base_y_m']),
        float(row['q_base_z_m']),
        float(row['q_quat_x']),
        float(row['q_quat_y']),
        float(row['q_quat_z']),
        float(row['q_quat_w']),
    ]
    values.extend(float(row[f'q_joint_{i}_rad']) for i in range(6))
    q = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(q)):
        raise RuntimeError('CSV contains a non-finite configuration')
    quaternion_norm = np.linalg.norm(q[3:7])
    if quaternion_norm < 1.0e-10 or abs(quaternion_norm - 1.0) > 1.0e-3:
        raise RuntimeError(f'Invalid quaternion norm: {quaternion_norm}')
    q[3:7] /= quaternion_norm
    return q


def force_from_row(row, prefix):
    names = [f'{prefix}_f{axis}_local_N' for axis in ('x', 'y', 'z')]
    force = np.asarray([float(row[name]) for name in names], dtype=float)
    if not np.all(np.isfinite(force)):
        raise RuntimeError(f'CSV contains a non-finite force: {prefix}')
    return force


def world_frame_data(model, data, q):
    pin.framesForwardKinematics(model, data, q)
    return [data.oMf[model.getFrameId(name)] for name in LINK_FRAMES]


def display_axis_map(use_z_to_x_map):
    rotation = np.eye(3)
    if use_z_to_x_map:
        # Rotate URDF +Z onto display +X without changing the URDF.
        rotation = np.asarray(
            [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
            dtype=float,
        )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return rotation, transform


def follow_rows(path: Path, poll_period: float, replay_existing: bool = False):
    while not path.exists():
        time.sleep(poll_period)

    with path.open(newline='') as stream:
        header_line = stream.readline()
        while not header_line:
            time.sleep(poll_period)
            header_line = stream.readline()
        fields = next(csv.reader([header_line]))
        if not replay_existing:
            stream.seek(0, 2)
        pending = ''
        while True:
            line = stream.readline()
            if not line:
                time.sleep(poll_period)
                continue
            pending += line
            if not pending.endswith(('\n', '\r')):
                continue
            complete_line = pending.strip('\r\n')
            pending = ''
            if not complete_line:
                continue
            values = next(csv.reader([complete_line]))
            if len(values) == len(fields):
                yield dict(zip(fields, values))


def make_simplified_robot(viewer, geometry):
    material = geometry.MeshPhongMaterial(color=0x2F80ED)
    base_material = geometry.MeshPhongMaterial(color=0xF2C94C)
    viewer['underwater_robot/simplified/base'].set_object(
        geometry.Sphere(0.10), base_material)
    nodes = []
    for link in range(len(LINK_FRAMES)):
        node = viewer[f'underwater_robot/simplified/segment{link}']
        node.set_object(geometry.Box([0.24, 0.08, 0.08]), material)
        nodes.append(node)
    return nodes


def placement_matrix(placement):
    transform = np.eye(4)
    transform[:3, :3] = placement.rotation
    transform[:3, 3] = placement.translation
    return transform


def display_row(row, args, model, viz, data, geometry, axis_map, history, simplified_nodes):
    q = configuration_from_row(row)
    sim_time = float(row['time_s'])
    viz.display(q)
    placements = world_frame_data(model, data, q)
    if simplified_nodes:
        viz.viewer['underwater_robot/simplified/base'].set_transform(
            placement_matrix(placements[0]))
        for node, placement in zip(simplified_nodes, placements):
            node.set_transform(placement_matrix(placement))
        set_polyline(
            viz.viewer,
            'underwater_robot/simplified/skeleton',
            [placement.translation for placement in placements],
            COLOR_TRAJECTORY,
            geometry,
        )
    link0_origin = axis_map @ placements[0].translation.copy()
    history.append(link0_origin)
    if len(history) > MAX_TRAJECTORY_POINTS:
        del history[:-MAX_TRAJECTORY_POINTS]
    set_polyline(viz.viewer, 'trajectory/link0', history, COLOR_TRAJECTORY, geometry)

    for link, placement in enumerate(placements):
        origin = axis_map @ placement.translation
        rotation = axis_map @ placement.rotation
        if args.show_total:
            force = force_from_row(row, f'total_fluid_l{link}')
            set_arrow(
                viz.viewer,
                f'hydro/link{link}/total',
                origin,
                rotation @ force,
                args.force_scale,
                COLOR_TOTAL_FLUID,
                geometry,
            )
        if args.show_damping:
            force = force_from_row(row, f'damping_l{link}')
            set_arrow(
                viz.viewer,
                f'hydro/link{link}/damping',
                origin,
                rotation @ force,
                args.force_scale,
                COLOR_DAMPING,
                geometry,
            )
        if args.show_buoyancy:
            force = force_from_row(row, f'buoyancy_l{link}')
            set_arrow(
                viz.viewer,
                f'hydro/link{link}/buoyancy',
                origin,
                rotation @ force,
                args.force_scale,
                COLOR_BUOYANCY,
                geometry,
            )
    return sim_time


def close_meshcat(viz):
    window = getattr(viz.viewer, 'window', None)
    if window is None:
        return
    socket = getattr(window, 'zmq_socket', None)
    if socket is not None:
        socket.close(linger=0)
    server_process = getattr(window, 'server_proc', None)
    if server_process is None or server_process.poll() is not None:
        return
    server_process.terminate()
    try:
        server_process.wait(timeout=2.0)
    except Exception:
        server_process.kill()
        server_process.wait(timeout=2.0)


def stop_on_signal(signum, frame):
    del signum, frame
    raise KeyboardInterrupt


def main():
    args = parse_args()
    if args.stride < 1 or args.speed <= 0.0 or args.force_scale < 0.0:
        raise ValueError('stride must be positive; speed and force scale must be nonnegative')

    try:
        import meshcat.geometry as geometry
        from pinocchio.visualize import MeshcatVisualizer
    except ImportError as error:
        raise RuntimeError(
            'Meshcat is not installed. Install python3-venv, create a venv, '
            'then run `pip install meshcat`, or use the project visualization environment.'
        ) from error

    if args.poll_period <= 0.0:
        raise ValueError('poll period must be positive')

    rows = None if args.follow else load_rows(args.csv)
    mesh_root = args.mesh_root or args.urdf.parent.parent.parent.parent
    full_model = pin.buildModelFromUrdf(str(args.urdf), pin.JointModelFreeFlyer())
    visual_model = pin.GeometryModel()
    if args.full_model:
        _, visual_model = pin.buildModelsFromUrdf(
            str(args.urdf),
            [str(mesh_root)],
            pin.JointModelFreeFlyer(),
            geometry_types=pin.GeometryType.VISUAL,
        )
    locked_joints = [
        joint_id
        for joint_id in range(1, full_model.njoints)
        if 'screw_rotor' in full_model.names[joint_id]
    ]
    if len(locked_joints) != 8:
        raise RuntimeError(
            f'Expected eight rotor joints to lock, found {len(locked_joints)}'
        )
    model, visual_model = pin.buildReducedModel(
        full_model,
        visual_model,
        locked_joints,
        pin.neutral(full_model),
    )
    if model.nq != 13 or model.nv != 12:
        raise RuntimeError(f'Expected reduced model nq=13,nv=12, got {model.nq},{model.nv}')

    collision_model = pin.GeometryModel()
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=not args.no_open)
    viz.loadViewerModel(rootNodeName='underwater_robot')
    axis_map, display_transform = display_axis_map(not args.no_z_to_x_map)
    viz.viewer['underwater_robot'].set_transform(display_transform)
    simplified_nodes = [] if args.full_model else make_simplified_robot(viz.viewer, geometry)
    data = model.createData()
    if simplified_nodes:
        viz.display(pin.neutral(model))
        initial_placements = world_frame_data(model, data, pin.neutral(model))
        viz.viewer['underwater_robot/simplified/base'].set_transform(
            placement_matrix(initial_placements[0]))
        for node, placement in zip(simplified_nodes, initial_placements):
            node.set_transform(placement_matrix(placement))
        set_polyline(
            viz.viewer,
            'underwater_robot/simplified/skeleton',
            [placement.translation for placement in initial_placements],
            COLOR_TRAJECTORY,
            geometry,
        )
    signal.signal(signal.SIGINT, stop_on_signal)
    signal.signal(signal.SIGTERM, stop_on_signal)
    history = []
    try:
        if args.follow:
            sim_start = None
            wall_start = None
            displayed = 0
            for row in follow_rows(args.csv, args.poll_period, args.replay_existing):
                sim_time = float(row['time_s'])
                if sim_start is None:
                    sim_start = sim_time
                    wall_start = time.perf_counter()
                if displayed % args.stride == 0:
                    target_wall = wall_start + (sim_time - sim_start) / args.speed
                    delay = target_wall - time.perf_counter()
                    if delay > 0.0:
                        time.sleep(delay)
                    display_row(
                        row, args, model, viz, data, geometry, axis_map, history, simplified_nodes)
                displayed += 1
        else:
            sim_start = float(rows[0]['time_s'])
            wall_start = time.perf_counter()
            displayed = 0
            for index in range(0, len(rows), args.stride):
                row = rows[index]
                sim_time = float(row['time_s'])
                target_wall = wall_start + (sim_time - sim_start) / args.speed
                delay = target_wall - time.perf_counter()
                if delay > 0.0:
                    time.sleep(delay)
                display_row(
                    row, args, model, viz, data, geometry, axis_map, history, simplified_nodes)
                displayed += 1

            print(f'Displayed {displayed} frames from {args.csv}')
            print(f'Meshcat URL: {viz.viewer.url()}')
            input('Press Enter to exit... ')
    except KeyboardInterrupt:
        print('Meshcat viewer stopped')
    finally:
        close_meshcat(viz)


if __name__ == '__main__':
    main()
