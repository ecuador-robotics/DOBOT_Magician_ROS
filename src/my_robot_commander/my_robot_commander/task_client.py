import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_msgs.action import Task


class TaskClient(Node):
    def __init__(self):
        super().__init__(node_name='task_client')
        self._action_client = ActionClient(self, Task, 'task_server')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server connected!')
        
        self._send_goal_future = None
        self._get_result_future = None

    def send_goal(self, task_number):
        goal_msg = Task.Goal()
        goal_msg.task_number = task_number

        self.get_logger().info(f'Sending goal: Task {task_number}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        if result.success:
            self.get_logger().info("Task completed successfully")
        else:
            self.get_logger().info("Task failed")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.percentage}% complete')


def main(args=None):
    rclpy.init(args=args)
    client = TaskClient()

    try:
        print("\n" + "="*60)
        print("TASK CLIENT")
        print("="*60)
        print("\nTasks:")
        print("  1. Home (0, 0, 0)")
        print("  2. Rotate 90° (1.57, 0, 0)")
        print("  3. Extend arm (0, 0.78, 0)")
        print("  4. Custom position 1 (-1.57, 0.5, 0.3)")
        print("  5. Custom position 2 (0.78, 1.0, -0.1)")
        print("\n" + "="*60)

        while True:
            try:
                task_num = int(input("\n(1-5, 0 to leave): "))
                
                if task_num == 0:
                    print("Leaving")
                    break
                
                if task_num < 1 or task_num > 5:
                    print("Invalid number, use a number from 1-5.")
                    continue

                send_goal_future = client.send_goal(task_num)
                rclpy.spin_until_future_complete(client, send_goal_future)
                
                if client._get_result_future is not None:
                    rclpy.spin_until_future_complete(client, client._get_result_future)

            except ValueError:
                print("Enter a valid number")
            except KeyboardInterrupt:
                print("\n\nLeaving")
                break

    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()