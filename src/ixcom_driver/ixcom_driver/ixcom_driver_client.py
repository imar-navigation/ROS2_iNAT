#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from interfaces.action import Example, Stop, Realign, Reboot


class Client(Node):

    def __init__(self):
        super().__init__('ixcom_driver_cli')
        self._get_reboot_result_future = None
        self._send_reboot_future = None
        self._send_realign_future = None
        self._get_stop_result_future = None
        self._send_stop_future = None
        self._get_result_future = None
        self._send_goal_future = None
        self._action_client_ex = ActionClient(self, Example, 'example')
        self._action_client_stop = ActionClient(self, Stop, 'stop')
        self._action_client_realign = ActionClient(self, Realign, 'realign')
        self._action_client_reboot = ActionClient(self, Reboot, 'reboot')

    def show_example(self, order):
        goal_msg = Example.Goal()
        goal_msg.order = order

        self._action_client_ex.wait_for_server()

        # return self._action_client_ex.send_goal_async(goal_msg)
        self._send_goal_future = self._action_client_ex.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('rejected...')
            return

        self.get_logger().info('accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def stop_ixcom_driver(self, opt):
        goal_msg = Stop.Goal()
        goal_msg.opt = int(opt)

        self._action_client_stop.wait_for_server()

        self._send_stop_future = self._action_client_stop.send_goal_async(goal_msg)
        self._send_stop_future.add_done_callback(self.stop_response_callback)

    def stop_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('stop request rejected...')
            return

        self.get_logger().info('stop request accepted')

        self._get_stop_result_future = goal_handle.get_result_async()
        self._get_stop_result_future.add_done_callback(self.get_stop_result_callback)

    def get_stop_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'response: {result.resp}')
        rclpy.shutdown()

    def realign_inat(self, opt):
        goal_msg = Realign.Goal()
        goal_msg.opt = int(opt)

        self._action_client_realign.wait_for_server()

        self._send_realign_future = self._action_client_realign.send_goal_async(goal_msg)
        self._send_realign_future.add_done_callback(self.realign_response_callback)

    def realign_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('realignment request rejected...')
            return

        self.get_logger().info('realignment request accepted')

        self._get_realign_result_future = goal_handle.get_result_async()
        self._get_realign_result_future.add_done_callback(self.get_realign_result_callback)

    def get_realign_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'response: {result.resp}')
        rclpy.shutdown()

    def reboot_inat(self, opt):
        goal_msg = Reboot.Goal()
        goal_msg.opt = int(opt)

        self._action_client_reboot.wait_for_server()

        self._send_reboot_future = self._action_client_reboot.send_goal_async(goal_msg)
        self._send_reboot_future.add_done_callback(self.reboot_response_callback)

    def reboot_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('reboot request rejected...')
            return

        self.get_logger().info('reboot request accepted')

        self._get_reboot_result_future = goal_handle.get_result_async()
        self._get_reboot_result_future.add_done_callback(self.get_reboot_result_callback)

    def get_reboot_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'response: {result.resp}')
        rclpy.shutdown()


def main():
    rclpy.init()
    try:
        par = sys.argv[1]
    except:
        print('missing argument!')
        print('q:  stops ixcom_driver')
        print('xr: starts realignment')
        print('xb: starts device reboot')
        # print('ex: shows an example')
        exit(1)

    cli = Client()

    # future = action_client.send_goal(10)
    # rclpy.spin_until_future_complete(action_client, future)
    if par.lower() == 'ex':
        cli.show_example(10)
    elif par.lower() == 'q':
        cli.stop_ixcom_driver(0)
    elif par.lower() == 'xr':
        cli.realign_inat(0)
    elif par.lower() == 'xb':
        cli.reboot_inat(0)
    else:
        print(f'wrong argument: {par}')
        print('q:  stops ixcom_driver')
        print('xr: starts realignment')
        print('xb: starts device reboot')
        # print('ex: shows an example')
        exit(1)
    rclpy.spin(cli)


if __name__ == '__main__':
    main()