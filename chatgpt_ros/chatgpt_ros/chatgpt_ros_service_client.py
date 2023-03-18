import rclpy
from chatgpt_ros_interfaces.srv import ChatGptService


def main(args=None):
    '''
    sample service client
    '''
    rclpy.init(args=args)

    node = rclpy.create_node('chat_gpt_service_client')

    client = node.create_client(ChatGptService, 'chatgpt_service')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for service')

    node.get_logger().info('Please enter your prompt for ChatGPT API. \
                            If you want to exit, please enter "quit"')
    while True:
        request = ChatGptService.Request()
        prompt = input('>')
        if prompt == 'quit':
            node.get_logger().info('Exit')
            break
        elif len(prompt) == 0:
            continue
        request.text = prompt
        request.length = 50

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()

        if response is not None:
            print('Response: %s' % response.response)
        else:
            node.get_logger().info('Service call failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
