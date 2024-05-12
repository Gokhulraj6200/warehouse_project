#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from attach_shelf.srv import GoToLoading
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Client for shelf lifting service
class ClientAsync(Node):
    def __init__(self):
        super().__init__('go_to_loading')
        self.client = self.create_client(GoToLoading, 'approach_shelf')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        req = GoToLoading.Request()
        req.attach_to_shelf = True
        self.future = self.client.call_async(req)

class ElevatorPublisher(Node):
    def __init__(self):
        super().__init__('elevator_publisher')
        self.publisher_ = self.create_publisher(String, '/elevator_down', 10)
        self.lift_publisher_ = self.create_publisher(String, '/elevator_up', 10)

    def drop(self):
        msg = String()
        self.publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Shelf unloaded')
        return None

    def lift(self):
        msg = String()
        self.lift_publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Shelf loaded')
        return None

    def wait(self):
        duration = Duration(seconds=10)
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            rate.sleep
# Shelf positions for picking
shelf_positions = {
    "init_position": [0.0, 0.0, 0.0, 1.0],
    "loading_position": [5.65071, -0.303011, -0.701128, 0.713036],
    }

# Shipping destination for picked products
shipping_destinations = {}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''


def main():
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = 'loading_position'
    request_destination = ''
    ####################

    rclpy.init()

    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = shelf_positions['init_position'][0]
    initial_pose.pose.position.y = shelf_positions['init_position'][1]
    initial_pose.pose.orientation.z = shelf_positions['init_position'][2]
    initial_pose.pose.orientation.w = shelf_positions['init_position'][3]
    

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()
    navigator.setInitialPose(initial_pose)

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination + ')...')

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    # Instance the elevator publisher
    elevator_publisher = ElevatorPublisher()
    
    # Instance the service client for shelf lifting
    for n in range(5):
        client = ClientAsync()
        print(f'Attempt {n+1}: Calling shelf lifting service.')
        client.send_request()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info(f'Result of service call: {response.complete}')
                    
                break

        client.destroy_node()
        if response.complete == True:
            elevator_publisher.lift()
            break

    if n + 1 == 5: exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)



if __name__ == '__main__':
    main()
