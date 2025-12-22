
import api.topic_publisher as topic_publisher
import api.param_client as ParamsClient
import api.topic_subscriber as topic_subscriber
import rclpy
import rclpy.executors
import threading


from autoaim_interfaces.msg import Armors
from std_msgs.msg import Float32



rclpy.init()

class PrintController:
    def __init__(self):
        self.publisher = topic_publisher.PrintPublisher()

    async def publish_print(self) -> int:
        return self.publisher.publish_print()
    
class ParamsGetterController:
    def __init__(self) -> None:
        self.params_client = ParamsClient.ParamsClient("/detector_node", 
            ['is_blue', 'debug', 'armor.use_traditional', 'energy.use_traditional', 'autoaim_mode'])
        
    # def get_params(self):
    #     return self.params_client.get_params()    
    
def callback(msg):
    pass
    
class SubscriptionController:
    def __init__(self):
        self.subscription = topic_subscriber.TopicSubscriber(Armors, callback, "/detector/armors")

    
print_controller = PrintController()
params_getter_controller = ParamsGetterController()
subscriber_controller = SubscriptionController()
    
from fastapi import APIRouter

from pydantic import BaseModel

router = APIRouter()

ros_thread = threading.Thread(target=lambda: rclpy.spin(subscriber_controller.subscription), daemon=True)
ros_thread.start()

class PrintResponse(BaseModel):
    is_blue: int
    debug: bool
    armor_use_traditional: int
    energy_use_traditional: int
    autoaim_mode: int
    
    def __init__(self, is_blue: int, debug:bool,
                armor_use_traditional: int, 
                energy_use_traditional: int,
                autoaim_mode: int):
        super().__init__(is_blue=is_blue, debug=debug, armor_use_traditional=armor_use_traditional,
                         energy_use_traditional=energy_use_traditional, autoaim_mode=autoaim_mode)


@router.get("/detector", response_model=PrintResponse)
async def sum_service():
    response = PrintResponse(-1, False, -1, -1, -1)
    result_print = await print_controller.publish_print()
    values = params_getter_controller.params_client.get_params()
    if values is not None:
        response.is_blue = values[0].integer_value
        response.debug = values[1].bool_value
        response.armor_use_traditional = values[2].integer_value
        response.energy_use_traditional = values[3].integer_value
        response.autoaim_mode = values[4].integer_value
    return response