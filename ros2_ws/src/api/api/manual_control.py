import rclpy
from rclpy.node import Node
from fastapi import FastAPI
import asyncio
import websockets
import rclpy 
from  pydantic  import  BaseModel 

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist

# Converted logic from tello-ros2 tello_controll

app = FastAPI()

class  Response ( BaseModel ): 
    msg :  str 

class ManualControl(Node):

    def __init__(self):
        super().__init__('api_manual_control_node')
        self.publisher_land = self.create_publisher(Empty, 'land', 1)
        self.publisher_flip = self.create_publisher(String, 'flip', 1)
        self.publisher_takeoff = self.create_publisher(Empty, 'takeoff', 10)
        self.publisher_velocity = self.create_publisher(Twist, 'control', 1)
        self.publisher_emergency = self.create_publisher(Empty, 'emergency', 1)

        self.manual_speed = 50.0  # Speed of the drone in manual control mode

        self.timer = self.create_timer(0.001, self.timer_callback)
        self.run()

    async def websocket_handler(self, websocket, path):
        self.websocket = websocket
        async for message in websocket:
            # Parse the command from the message
            self.get_logger().info(message + " in path: " + path)
            self.timer_callback(message)
            await self.websocket.send(f"{message} command sendt")
            
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
        elif command == 81:  # LEFT arrow
            msg.linear.x = -self.manual_speed
        elif command == 83:  # RIGHT arrow
            msg.linear.x = self.manual_speed
        elif command == 82:  # UP arrow
            msg.linear.y = self.manual_speed
        elif command == 84:  # DOWN arrow
            msg.linear.y = -self.manual_speed

        self.publisher_velocity.publish(msg)

    def timer_callback(self, command):
        if command == "takeoff":
            # Perform the action corresponding to "start"
            self.publisher_takeoff.publish(Empty())
        elif command == "land":
            # Perform the action corresponding to "start"
            self.publisher_land.publish(Empty())
        elif command == "flip":
            # Perform the action corresponding to "start"
            self.publisher_flip.publish(String(data="f"))
        elif command == "e":
            self.publisher_emergency.publish(Empty())
        else:
            self.manual_control(command)
        
    async def main(self):
        async with websockets.serve(self.websocket_handler, '0.0.0.0', 5683):
            await asyncio.Future()  # Run forever

    def run(self):
        asyncio.run(self.main())

    def shutdown(self):
        asyncio.get_event_loop().close()

def main(args=None):
    rclpy.init()
    node = ManualControl()
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()