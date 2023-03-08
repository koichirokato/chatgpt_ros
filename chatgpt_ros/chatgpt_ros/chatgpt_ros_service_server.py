import rclpy
from rclpy.node import Node
from chatgpt_ros import chatgpt
from chatgpt_ros_interfaces.srv import ChatGptService

import os

class ChatGptServiceServer(Node):
    def __init__(self):
        super().__init__('chatgpt_service_server')
        self.srv = self.create_service(ChatGptService, 'chatgpt_service', self.gpt_service_callback)

    def gpt_service_callback(self, request, response):
        text = request.text
        length = request.length

        gpt = chatgpt.ChatGPT(api_key=os.environ['ChatGPT_API'])
        response.response = gpt.generate_text(text, length)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ChatGptServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()