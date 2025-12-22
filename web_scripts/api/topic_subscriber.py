from rclpy.node import Node


class TopicSubscriber(Node):
    def __init__(self, 
                 msg_type, 
                 callback,
                 topic_name: str,
                 name: str = "topic_subscriber"):
        super().__init__(node_name=name)
        self.msg_type = type(msg_type)
        self.subscription = self.create_subscription(msg_type, topic_name, callback, 10)