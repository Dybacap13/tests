#!/usr/bin/env python3
from rclpy.node import Node
from example_interfaces.srv import Trigger
from typing import Callable

class TriggerService:
    def __init__(
        self,
        node: Node,
        service_name: str,
        callback: Callable[[Trigger.Request, Trigger.Response], Trigger.Response]
    ):
        self.node_ = node
        self.srv = self.node.create_service(
            Trigger,
            service_name,
            callback
        )
        self.node.get_logger().info(f"Service '{service_name}' is ready!")
