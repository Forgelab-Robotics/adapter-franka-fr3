"""验证 action 以 fresh HOME_POSITION 为参考零点。"""

from __future__ import annotations

from robots_franka_fr3 import FrankaFR3Driver
from robots_franka_fr3.contract import ACTUATOR_ORDER, HOME_POSITION


TARGET_OFFSET = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_recover_to_home() -> None:
    """测试恢复到指定 joint 位置。"""
    driver = FrankaFR3Driver(backend=None, require_homing=False, auto_connect=False)
    driver.connect()

    try:
        # 获取当前状态
        state = driver.get_state()
        print(f"当前关节位置: {state.position[:7]}")
        print(f"当前夹爪宽度: {state.position[7]}")

        # 发送相对于 connect fresh state 的偏移命令。
        from forge_msgs import JointCommand
        command = JointCommand(
            name=list(ACTUATOR_ORDER),
            position=TARGET_OFFSET + [0.0],
            mode="position",
        )
        driver.set_command(command)
        print(f"已发送相对偏移: {TARGET_OFFSET}")

        # 等待状态更新
        import time
        time.sleep(0.2)

        # 再次获取状态验证
        state = driver.get_state()
        print(f"更新后关节位置: {state.position[:7]}")
        print(f"更新后夹爪宽度: {state.position[7]}")

        # 验证位置（使用近似比较）
        import math
        expected = [value + offset for value, offset in zip(HOME_POSITION[:7], TARGET_OFFSET)]
        for i, (actual, target) in enumerate(zip(state.position[:7], expected)):
            assert math.isclose(actual, target, rel_tol=1e-5), f"关节{i+1}位置不匹配: {actual} != {target}"
        print("✓ 位置验证通过")

    finally:
        driver.disconnect()


if __name__ == "__main__":
    test_recover_to_home()
