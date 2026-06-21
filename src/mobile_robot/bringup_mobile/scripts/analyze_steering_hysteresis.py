#!/usr/bin/env python3
"""Measure FR/RL steering hysteresis between CW and CCW MCAP trials."""

import argparse
import json
import math
from pathlib import Path
import statistics

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def degrees(angle_rad):
    return math.degrees(angle_rad)


def shortest_angle(from_rad, to_rad):
    return math.atan2(math.sin(to_rad - from_rad), math.cos(to_rad - from_rad))


def circular_mean(values):
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def find_bag_uri(path):
    candidate = Path(path).expanduser().resolve()
    if candidate.is_file() and candidate.suffix == '.mcap':
        return str(candidate)
    if candidate.is_dir() and (candidate / 'metadata.yaml').exists():
        return str(candidate)
    raise argparse.ArgumentTypeError(f'{candidate} is not a rosbag2 MCAP path')


def read_topics(bag_uri, command_topic, joint_topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = [
        topic for topic in (command_topic, joint_topic)
        if topic not in topic_types
    ]
    if missing:
        raise RuntimeError(f'missing required topic(s): {", ".join(missing)}')

    commands = []
    joints = []
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in (command_topic, joint_topic):
            continue
        message = deserialize_message(raw, get_message(topic_types[topic]))
        timestamp_s = timestamp_ns * 1e-9
        if topic == command_topic:
            commands.append((timestamp_s, message.twist.angular.z))
        else:
            positions = dict(zip(message.name, message.position))
            joints.append((timestamp_s, positions))
    return commands, joints


def command_segments(commands, threshold, min_stop_duration):
    segments = []
    start = None
    end = None
    direction = None
    values = []
    zero_started_at = None
    for timestamp, value in commands:
        if abs(value) < threshold:
            if start is not None and zero_started_at is None:
                zero_started_at = timestamp
            continue

        new_direction = 1 if value > 0.0 else -1
        stopped_long_enough = (
            zero_started_at is not None
            and timestamp - zero_started_at >= min_stop_duration
        )
        if start is not None and (
            new_direction != direction or stopped_long_enough
        ):
            segments.append((start, end, direction, values))
            start = timestamp
            direction = new_direction
            values = []
        elif start is None:
            start = timestamp
            direction = new_direction
        end = timestamp
        values.append(value)
        zero_started_at = None
    if start is not None:
        segments.append((start, end, direction, values))
    return segments


def summarize_trial(index, segment, joints, modules, grace, tail_trim):
    start, end, direction, commands = segment
    analysis_start = start + grace
    analysis_end = end - tail_trim
    if analysis_end <= analysis_start:
        raise RuntimeError(
            f'trial {index} is too short after grace/tail trimming'
        )

    result = {
        'index': index,
        'direction': 'ccw' if direction > 0 else 'cw',
        'start_s': start,
        'duration_s': end - start,
        'median_command_rad_s': statistics.median(commands),
        'modules': {},
    }
    for module in modules:
        joint_name = f'{module}_steering_joint'
        values = [
            positions[joint_name]
            for timestamp, positions in joints
            if (
                analysis_start <= timestamp <= analysis_end
                and joint_name in positions
            )
        ]
        if not values:
            result['modules'][module] = {'sample_count': 0}
            continue
        mean = circular_mean(values)
        errors = [shortest_angle(mean, value) for value in values]
        result['modules'][module] = {
            'sample_count': len(values),
            'mean_rad': mean,
            'mean_deg': degrees(mean),
            'median_deg': degrees(statistics.median(values)),
            'stddev_deg': degrees(
                math.sqrt(sum(error * error for error in errors) / len(errors))
            ),
            'min_deg': degrees(min(values)),
            'max_deg': degrees(max(values)),
        }
    return result


def summarize_hysteresis(trials, modules, target):
    result = {}
    for module in modules:
        direction_means = {}
        for direction in ('cw', 'ccw'):
            values = [
                trial['modules'][module]['mean_rad']
                for trial in trials
                if (
                    trial['direction'] == direction
                    and trial['modules'][module].get('sample_count', 0) > 0
                )
            ]
            if values:
                direction_means[direction] = circular_mean(values)

        if set(direction_means) != {'cw', 'ccw'}:
            result[module] = {'available': False}
            continue
        cw_mean = direction_means['cw']
        ccw_mean = direction_means['ccw']
        signed = shortest_angle(ccw_mean, cw_mean)
        result[module] = {
            'available': True,
            'cw_mean_deg': degrees(cw_mean),
            'ccw_mean_deg': degrees(ccw_mean),
            'cw_target_offset_deg': degrees(shortest_angle(target, cw_mean)),
            'ccw_target_offset_deg': degrees(shortest_angle(target, ccw_mean)),
            'cw_minus_ccw_deg': degrees(signed),
            'absolute_hysteresis_deg': abs(degrees(signed)),
        }
    return result


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=find_bag_uri)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--command-topic', default='swerve_controller/cmd_vel')
    parser.add_argument('--joint-topic', default='joint_states')
    parser.add_argument('--modules', nargs='+', default=['FR', 'RL'])
    parser.add_argument('--command-threshold', type=float, default=0.05)
    parser.add_argument('--min-stop-duration', type=float, default=1.0)
    parser.add_argument('--grace-time', type=float, default=3.0)
    parser.add_argument('--tail-trim', type=float, default=1.0)
    parser.add_argument('--steering-target-deg', type=float, default=45.0)
    parser.add_argument('--json-output')
    args = parser.parse_args()
    namespace = args.namespace.strip().strip('/')
    prefix = f'/{namespace}/' if namespace else '/'
    if not args.command_topic.startswith('/'):
        args.command_topic = prefix + args.command_topic
    if not args.joint_topic.startswith('/'):
        args.joint_topic = prefix + args.joint_topic
    return args


def main():
    args = parse_args()
    commands, joints = read_topics(
        args.bag,
        args.command_topic,
        args.joint_topic,
    )
    segments = command_segments(
        commands,
        args.command_threshold,
        args.min_stop_duration,
    )
    trials = [
        summarize_trial(
            index + 1,
            segment,
            joints,
            args.modules,
            args.grace_time,
            args.tail_trim,
        )
        for index, segment in enumerate(segments)
    ]
    target = math.radians(args.steering_target_deg)
    hysteresis = summarize_hysteresis(trials, args.modules, target)
    report = {
        'bag': args.bag,
        'command_topic': args.command_topic,
        'joint_topic': args.joint_topic,
        'steering_target_deg': args.steering_target_deg,
        'trials': trials,
        'hysteresis': hysteresis,
    }

    for trial in trials:
        module_text = ', '.join(
            f'{module}={trial["modules"][module].get("mean_deg", math.nan):+.2f} deg'
            for module in args.modules
        )
        print(
            f'trial {trial["index"]}: {trial["direction"].upper()} '
            f'{trial["median_command_rad_s"]:+.3f} rad/s, {module_text}'
        )
    for module in args.modules:
        summary = hysteresis[module]
        if not summary['available']:
            print(f'{module}: CW/CCW hysteresis unavailable')
            continue
        print(
            f'{module}: CW={summary["cw_mean_deg"]:+.2f} deg, '
            f'CCW={summary["ccw_mean_deg"]:+.2f} deg, '
            f'CW-CCW={summary["cw_minus_ccw_deg"]:+.2f} deg, '
            f'|hysteresis|={summary["absolute_hysteresis_deg"]:.2f} deg'
        )

    if args.json_output:
        output = Path(args.json_output).expanduser().resolve()
        output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(f'JSON report: {output}')


if __name__ == '__main__':
    main()
