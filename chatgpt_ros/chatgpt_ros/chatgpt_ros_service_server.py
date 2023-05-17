import rclpy
from rclpy.node import Node
from chatgpt_ros import chatgpt
from chatgpt_ros_interfaces.srv import ChatGptService

import os


class ChatGptServiceServer(Node):
    def __init__(self):
        """
        constructer
        """
        super().__init__("chatgpt_service_server")
        self.srv = self.create_service(
            ChatGptService, "chatgpt_service", self.gpt_service_callback
        )
        self.gpt = chatgpt.ChatGPT(api_key=os.environ["ChatGPT_API"])

        self.init_param()

    def init_param(self):
        self.declare_parameter("system_role.use_system_role", False)
        self.declare_parameter("system_role.content", "")
        self.declare_parameter("assistant_role.hold_passed_response", False)
        self.declare_parameter("assistant_role.num_passed_response", 0)

        use_system_role = (
            self.get_parameter("system_role.use_system_role")
            .get_parameter_value()
            .bool_value
        )
        system_content = (
            self.get_parameter("system_role.content").get_parameter_value().string_value
        )

        hold_passed_response = (
            self.get_parameter("assistant_role.hold_passed_response")
            .get_parameter_value()
            .bool_value
        )
        num_passed_response = (
            self.get_parameter("assistant_role.num_passed_response")
            .get_parameter_value()
            .integer_value
        )

        if use_system_role:
            self.gpt.set_system_content(system_content)

        if hold_passed_response:
            self.gpt.num_hold_pass_res = num_passed_response

    def gpt_service_callback(self, request, response):
        """
        Service callback function

        Parameters
        ----------
        requests : ChatGptService.requests
            requests for ros service
        response : ChatGptService.response
            response for ros service
        """
        text = request.text
        length = request.length
        response.response = self.gpt.generate_text(text, length)
        return response


def main(args=None):
    """
    main function
    """
    rclpy.init(args=args)
    node = ChatGptServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()
