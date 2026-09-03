"""恢复到 HOME_POSITION = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398] 的测试程序。"""

from __future__ import annotations

from robots_franka_fr3 import FrankaFR3Driver
from robots_franka_fr3.contract import ACTUATOR_ORDER, HOME_POSITION


TARGET_POSITION = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]


def test_recover_to_home() -> None:
    """测试恢复到指定 joint 位置。"""
    driver = FrankaFR3Driver(backend=None, require_homing=False, auto_connect=False)
    driver.connect()

    try:
        # 获取当前状态
        state = driver.get_state()
        print(f"当前关节位置: {state.position[:7]}")
        print(f"当前夹爪宽度: {state.position[7]}")

        # 发送目标位置命令
        from forge_msgs import JointCommand
        command = JointCommand(
            name=list(ACTUATOR_ORDER),
            position=TARGET_POSITION + [0.08],  # 加上夹爪宽度
            mode="position",
        )
        driver.set_command(command)
        print(f"已发送目标位置: {TARGET_POSITION}")

        # 等待状态更新
        import time
        time.sleep(0.2)

        # 再次获取状态验证
        state = driver.get_state()
        print(f"更新后关节位置: {state.position[:7]}")
        print(f"更新后夹爪宽度: {state.position[7]}")

        # 验证位置（使用近似比较）
        import math
        for i, (actual, target) in enumerate(zip(state.position[:7], TARGET_POSITION)):
            assert math.isclose(actual, target, rel_tol=1e-5), f"关节{i+1}位置不匹配: {actual} != {target}"
        print("✓ 位置验证通过")

    finally:
        driver.disconnect()


if __name__ == "__main__":
    test_recover_to_home()
