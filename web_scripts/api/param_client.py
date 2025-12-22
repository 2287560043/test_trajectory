import rcl_interfaces.msg
import rclpy
import rclpy.client
import rcl_interfaces
from rclpy.node import Node
import rclpy.qos

class ParamsClient(Node):
    def __init__(self, target_node_name:str, params: list):
        super().__init__(node_name='params_client')
        self.async_client = self.create_client(rcl_interfaces.srv.GetParameters, target_node_name + "/get_parameters",
                                               qos_profile=rclpy.qos.qos_profile_parameters)
        self.values = list()
        self.params = params
        
        
    def get_params(self):
        self.get_logger().info('Getting parameters')
        if self.async_client.wait_for_service(timeout_sec=1.0):
            request = rcl_interfaces.srv.GetParameters.Request()
            request.names = self.params
            future = self.async_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is not None:
                self.values = future.result().values      
                self.get_logger().info('Params Got')      
                return self.values    
            else:
                self.get_logger().info('Service call failed %r' % (future.exception(),))
                return None
        else:
            self.get_logger().info('Service not available')
            return None
