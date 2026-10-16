"""Optional Tk sliders that update the running ROS 2 EKF parameters atomically."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from .runtime import param


def main(args=None):
    import tkinter as tk
    rclpy.init(args=args)
    node = Node('covariance_gui')
    client = AsyncParameterClient(node, param(node, 'ekf_node', '/ekf'))
    window = tk.Tk()
    window.title('Minesweepers EKF variances')
    sliders = {}
    for name, labels, defaults in [('odom_variances', ['x', 'y', 'yaw', 'v', 'omega'], [0.03, 0.03, 0.05, 0.04, 0.04]),
                                    ('imu_variances', ['yaw', 'omega'], [0.1, 0.01])]:
        sliders[name] = []
        for label, default in zip(labels, defaults):
            scale = tk.Scale(window, label=f'{name}: {label}', from_=0.001, to=1., resolution=0.001,
                             orient=tk.HORIZONTAL, length=500)
            scale.set(default)
            scale.pack()
            sliders[name].append(scale)
    status = tk.StringVar(value='Adjust variances and apply to the EKF')

    def finished(future):
        result = future.result().result
        status.set('Applied' if result.successful else result.reason)

    def apply():
        if not client.services_are_ready():
            status.set('EKF parameter service unavailable')
            return
        parameters = [Parameter(name, value=[float(scale.get()) for scale in values])
                      for name, values in sliders.items()]
        client.set_parameters_atomically(parameters).add_done_callback(finished)

    tk.Button(window, text='Apply to EKF', command=apply).pack()
    tk.Label(window, textvariable=status).pack()

    def poll():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            window.after(30, poll)
        else:
            window.quit()

    window.after(30, poll)
    try:
        window.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
