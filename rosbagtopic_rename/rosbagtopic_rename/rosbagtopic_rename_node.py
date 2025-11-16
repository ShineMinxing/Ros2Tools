#!/usr/bin/env python3
import os
import shutil   # ★ 新增：用于删除已存在的输出目录
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node

from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)

# ----------------------------------------------------------
# 1. 读取 YAML 配置（和 bt_imu_node 同风格）
#    从 Ros2Tools/config.yaml 里取：
#    rosbagtopic_rename_node:
#      ros__parameters:
#        input_bag: "..."
#        output_bag: "..."
#        topic_mappings: {...}
#        overwrite: true/false
# ----------------------------------------------------------
CFG_PATH = Path('/home/unitree/ros2_ws/LeggedRobot/src/Ros2Tools/config.yaml')
if not CFG_PATH.is_file():
    raise FileNotFoundError(f'配置文件不存在: {CFG_PATH}')

with CFG_PATH.open('r', encoding='utf-8') as f:
    YAML_DATA = yaml.safe_load(f) or {}

RENAME_PARAMS = (
    YAML_DATA
    .get('rosbagtopic_rename_node', {})
    .get('ros__parameters', {})
) or {}


class RosbagTopicRenameNode(Node):
    def __init__(self):
        # 不用参数覆盖，纯离线工具
        super().__init__('rosbagtopic_rename_node')
        self.cfg = RENAME_PARAMS

        self.get_logger().info("rosbagtopic_rename_node started.")
        try:
            self.run()
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            raise

    def run(self):
        cfg = self.cfg

        input_bag = cfg.get('input_bag')
        output_bag = cfg.get('output_bag')
        topic_mappings = cfg.get('topic_mappings', {}) or {}
        overwrite = bool(cfg.get('overwrite', False))   # ★ 新增：是否允许覆盖

        if not input_bag or not output_bag:
            raise ValueError(
                "config.yaml 中 rosbagtopic_rename_node 下必须包含 "
                "ros__parameters: { input_bag, output_bag, ... } 字段。"
            )

        self.get_logger().info(f"Input bag directory : {input_bag}")
        self.get_logger().info(f"Output bag directory: {output_bag}")
        self.get_logger().info(f"Topic mappings      : {topic_mappings}")
        self.get_logger().info(f"Overwrite enabled   : {overwrite}")

        # 1) 检查输入 bag 目录
        if not os.path.isdir(input_bag):
            raise FileNotFoundError(
                f"Input bag directory does not exist: {input_bag}"
            )

        metadata_yaml = os.path.join(input_bag, "metadata.yaml")
        if not os.path.exists(metadata_yaml):
            raise FileNotFoundError(
                f"metadata.yaml not found in input bag directory: {metadata_yaml}"
            )

        # 2) 输出目录处理：如果存在，根据 overwrite 决定是否删掉重建
        if os.path.exists(output_bag):
            if overwrite:
                self.get_logger().warn(
                    f"Output bag directory already exists, removing it: {output_bag}"
                )
                shutil.rmtree(output_bag)
            else:
                raise FileExistsError(
                    f"Output bag directory already exists: {output_bag}"
                )

        # 3) 读取 metadata.yaml 获取 storage_id（一般是 sqlite3）
        with open(metadata_yaml, 'r') as f:
            meta = yaml.safe_load(f) or {}
        info = meta.get('rosbag2_bagfile_information', {})
        storage_id = info.get('storage_identifier', 'sqlite3')

        self.get_logger().info(f"Detected storage_id: {storage_id}")

        # 4) 打开 reader & writer
        reader = SequentialReader()
        reader.open(
            StorageOptions(
                uri=input_bag,
                storage_id=storage_id,
            ),
            ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            ),
        )

        writer = SequentialWriter()
        writer.open(
            StorageOptions(
                uri=output_bag,
                storage_id=storage_id,
            ),
            ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            ),
        )

        # 5) 获取所有 topic，在 writer 里创建新 topic（有映射则用新名）
        all_topics = reader.get_all_topics_and_types()
        self.get_logger().info("Existing topics in input bag:")
        for t in all_topics:
            self.get_logger().info(f"  - {t.name} [{t.type}]")

        for t in all_topics:
            new_name = topic_mappings.get(t.name, t.name)
            new_metadata = TopicMetadata(
                name=new_name,
                type=t.type,
                serialization_format=t.serialization_format,
                offered_qos_profiles=t.offered_qos_profiles,
            )
            writer.create_topic(new_metadata)
            if new_name != t.name:
                self.get_logger().info(f"Remap topic: {t.name} -> {new_name}")
            else:
                self.get_logger().info(f"Keep topic : {t.name}")

        # 6) 遍历消息并写入新包
        count = 0
        while reader.has_next():
            topic, data, t = reader.read_next()
            new_topic = topic_mappings.get(topic, topic)
            writer.write(new_topic, data, t)
            count += 1
            if count % 10000 == 0:
                self.get_logger().info(f"Processed {count} messages...")

        self.get_logger().info(f"Finished. Total messages processed: {count}")
        self.get_logger().info(f"New bag saved to: {output_bag}")


def main(args=None):
    rclpy.init(args=args)
    node = RosbagTopicRenameNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
