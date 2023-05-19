import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from chatgpt_ros import chatgpt

import os


class ChatGPTNode(Node):
    def __init__(self):
        """
        constructer
        """
        super().__init__("chatgpt_node")
        self.sub = self.create_subscription(
            String, "/input_text", self.listener_callback, 10
        )
        self.pub = self.create_publisher(String, "/output_text", 10)
        self.chatgpt = chatgpt.ChatGPT(api_key=os.environ["ChatGPT_API"])

    def listener_callback(self, msg):
        """
        Subscribe callback function

        Parameters
        ----------
        msg : std_msgs.msg.String
            subscribe message
        """
        self.get_logger().info(f"Subscribed Text: {msg.data}")
        prompt = msg.data
        response = self.chatgpt.generate_text(prompt)
        self.get_logger().info(response)
        output_msg = String()
        output_msg.data = response
        self.pub.publish(output_msg)


def main(args=None):
    """
    main function
    """
    rclpy.init(args=args)
    node = ChatGPTNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
