#!/usr/bin/env python3
import os, yaml, threading, asyncio, math
from pathlib import Path

from bleak import BleakScanner, BleakClient
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

# ----------------------------------------------------------
# 1. 读取 YAML 并生成 parameter_overrides
# ----------------------------------------------------------
CFG_PATH = Path('/home/unitree/ros2_ws/LeggedRobot/src/Ros2Tools/config.yaml')
if not CFG_PATH.is_file():
    raise FileNotFoundError(f'配置文件不存在: {CFG_PATH}')

with CFG_PATH.open('r', encoding='utf-8') as f:
    yaml_data = yaml.safe_load(f)

yaml_params = yaml_data.get('bt_imu_node', {}).get('ros__parameters', {})
param_overrides = [Parameter(k, value=v) for k, v in yaml_params.items()]

# ----------------------------------------------------------
# 2. 节点实现
# ----------------------------------------------------------
class BTIMUNode(Node):
    def __init__(self, **kwargs):
        # 自动根据 overrides 声明参数
        kwargs.setdefault('parameter_overrides', param_overrides)
        kwargs.setdefault('automatically_declare_parameters_from_overrides', True)
        super().__init__('bt_imu_node', **kwargs)

        # 把参数读进 cfg
        p = self.get_parameter
        self.cfg = {name: p(name).value for name in yaml_params}

        # 发布器
        self.pub = self.create_publisher(Imu, self.cfg['SMX/BTIMU'], 10)

        # BLE 循环线程
        self._buf = bytearray()
        ble_loop = asyncio.new_event_loop()
        threading.Thread(target=ble_loop.run_forever, daemon=True).start()
        asyncio.run_coroutine_threadsafe(self.run_ble(ble_loop), ble_loop)

    # ---------- BLE 逻辑 ----------
    async def run_ble(self, loop):
        dev_name = self.cfg['WTSDCL']
        devices = await BleakScanner.discover()
        target  = next((d for d in devices if d.name and dev_name in d.name), None)
        if not target:
            self.get_logger().error(f'未找到设备: {dev_name}')
            return

        self.get_logger().info(f'连接 {target.address}')
        async with BleakClient(target, loop=loop) as client:
            if not client.is_connected:
                self.get_logger().error('BLE 连接失败'); return

            # 找到 notify 特征
            notify_uuid = self.cfg['NOTIFY_UUID']
            notify_char = client.services.get_characteristic(notify_uuid)
            if notify_char is None:
                # 回退：自动遍历
                for svc in await client.get_services():
                    notify_char = next((c for c in svc.characteristics
                                        if 'notify' in c.properties), None)
                    if notify_char:
                        break
            if notify_char is None:
                self.get_logger().error('未找到 Notify 特征'); return

            self.get_logger().info(f'订阅 {notify_char.uuid}')
            await client.start_notify(notify_char.uuid, self.handle_raw)

            while rclpy.ok():
                await asyncio.sleep(1)

    # ---------- 数据组帧 ----------
    def handle_raw(self, _sender: int, data: bytes):
        cfg = self.cfg
        self._buf += data
        while len(self._buf) >= cfg['FRAME_LEN']:
            if self._buf[0] != cfg['START_BYTE']:
                self._buf.pop(0); continue
            if len(self._buf) < cfg['FRAME_LEN'] or self._buf[1] != cfg['FLAG_IMU']:
                break
            frame = self._buf[:cfg['FRAME_LEN']]
            self._buf = self._buf[cfg['FRAME_LEN']:]
            self.parse_frame(frame)

    # ---------- 解析 IMU 帧 ----------
    def parse_frame(self, d: bytes):
        c = self.cfg
        s16 = lambda o: int.from_bytes(d[o:o+2], 'little', signed=True)
        ax, ay, az = (s16(2)/32768*c['G_SCA'],
                      s16(4)/32768*c['G_SCA'],
                      s16(6)/32768*c['G_SCA'])
        wx = s16( 8)/32768*c['GYRO_SCA'] * math.pi/180
        wy = s16(10)/32768*c['GYRO_SCA'] * math.pi/180
        wz = s16(12)/32768*c['GYRO_SCA'] * math.pi/180
        roll_deg  = s16(14)/32768*c['EULER_SCA']
        pitch_deg = s16(16)/32768*c['EULER_SCA']
        yaw_deg   = s16(18)/32768*c['EULER_SCA']

        self.get_logger().info(f"R:{roll_deg:.2f} P:{pitch_deg:.2f} Y:{yaw_deg:.2f}")

        roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = c['bt_imu_link']
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = ax, ay, az
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = wx, wy, wz
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = qw, qx, qy, qz
        imu.orientation_covariance[0] = imu.angular_velocity_covariance[0] = imu.linear_acceleration_covariance[0] = -1
        self.pub.publish(imu)

# ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BTIMUNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
