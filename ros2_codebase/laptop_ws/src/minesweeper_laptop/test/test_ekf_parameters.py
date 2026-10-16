import rclpy
from rclpy.parameter import Parameter

from minesweeper_laptop.ekf import EkfNode


def test_covariance_updates_are_atomic_and_require_positive_variances():
    rclpy.init()
    node = EkfNode()
    try:
        values = [0.01, 0.02, 0.03, 0.04, 0.05]
        result = node.set_parameters_atomically([Parameter('odom_variances', value=values)])
        assert result.successful, result.reason
        assert list(node.odom_variances) == values
        imu_before = list(node.imu_variances)
        result = node.set_parameters_atomically([
            Parameter('imu_variances', value=[0.2, 0.3]),
            Parameter('odom_variances', value=[0.0]*5),
        ])
        assert not result.successful
        assert list(node.odom_variances) == values
        assert list(node.imu_variances) == imu_before
    finally:
        node.destroy_node()
        rclpy.shutdown()
