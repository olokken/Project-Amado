
import rclpy
from rclpy.node import Node
from fastapi import FastAPI
import asyncio
import websockets
import rclpy
import cv2
from typing import Optional
from pydantic import BaseModel

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist


class Response (BaseModel):
    msg:  str


class API(Node):
    def __init__(self):
        super().__init__('server')
        self.publisher_land = self.create_publisher(Empty, 'land', 1)
        self.publisher_flip = self.create_publisher(String, 'flip', 1)
        self.publisher_takeoff = self.create_publisher(Empty, 'takeoff', 10)
        self.publisher_velocity = self.create_publisher(Twist, 'control', 1)
        self.publisher_emergency = self.create_publisher(Empty, 'emergency', 1)

        self.manual_speed = 50.0  # Speed of the drone in manual control mode
        self.timer = self.create_timer(0.001, self.timer_callback)

    def manual_control(self, command):
        msg = Twist()
        if command == "rotate_left":
            msg.angular.z = -self.manual_speed
        elif command == "rotate_right":
            msg.angular.z = self.manual_speed
        elif command == "up":
            msg.linear.z = self.manual_speed
        elif command == "down":
            msg.linear.z = -self.manual_speed
        elif command == "left":  # LEFT arrow
            msg.linear.x = -self.manual_speed
        elif command == "right":  # RIGHT arrow
            msg.linear.x = self.manual_speed
        elif command == "forward":  # UP arrow
            msg.linear.y = self.manual_speed
        elif command == "backward":  # DOWN arrow
            msg.linear.y = -self.manual_speed

        self.publisher_velocity.publish(msg)

    def timer_callback(self, command):
        if command == "takeoff":
            # Perform the action corresponding to "takeoff"
            self.publisher_takeoff.publish(Empty())
        elif command == "land":
            # Perform the action corresponding to "land"
            self.publisher_land.publish(Empty())
        elif command == "flip":
            # Perform the action corresponding to "flip"
            self.publisher_flip.publish(String(data="f"))
        elif command == "e":
            # Perform the action corresponding to "emergency"
            self.publisher_emergency.publish(Empty())
        else:
            self.manual_control(command)

    def shutdown(self):
        asyncio.get_event_loop().close()
