from typing import List
import threading

import rclpy


from routers.detector_param_router import PrintController


rclpy.init()

# ROS2 nodes tied to the API defined and added to the executor here
print_controller = PrintController()

# Define executor
executor = rclpy.executors.MultiThreadedExecutor()

# Add client nodes
executor.add_node(print_controller.publisher)

spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()


def get_nodes() -> List:
    return executor.get_nodes()