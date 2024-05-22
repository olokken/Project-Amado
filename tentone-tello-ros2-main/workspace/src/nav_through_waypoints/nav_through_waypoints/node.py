import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Pose, PoseArray  # Import necessary message types

class FirestoreNode(Node):
    
    def __init__(self):
        super().__init__("FirestoreNode")
        self.publisher = self.create_publisher(Empty, 'takeoff', 1)
        self.mission_publisher = self.create_publisher(PoseArray, 'new_mission', 10)
        self.subscription = self.create_subscription(
            String,
            'ID',
            self.id_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.setup_database_listener()
    
    def id_callback(self, msg):
        id_data = msg.data
        self.get_logger().info(f'Received ID: {id_data}')
        self.check_and_create_record(id_data)

    def check_and_create_record(self, id_data):
        ref = db.reference(f'drones/{id_data}')
        record = ref.get()
        if record is None:
            self.get_logger().info(f'ID {id_data} does not exist in the database. Creating a new record.')
            new_record = {
                "batteryPercentage": 100,
                "id": id_data,
                "name": id_data,
                "operation": "",
                "parkingFloorId": "",
                "status": "IDLE"
            }
            ref.set(new_record)
            self.get_logger().info(f'Created new record for ID {id_data}')
        else:
            self.get_logger().info(f'ID {id_data} already exists in the database.')
            if 'missionId' in record and record['missionId']:
                mission_id = record['missionId']
                self.get_logger().info(f'ID {id_data} has a missionId: {mission_id}')
                self.retrieve_and_process_mission(mission_id)
            else:
                self.get_logger().info(f'ID {id_data} does not have a missionId.')

    def retrieve_and_process_mission(self, mission_id):
        mission_ref = db.reference(f'missions/{mission_id}')
        mission = mission_ref.get()
        if mission:
            self.get_logger().info(f'Retrieved mission: {mission}')
            if mission['type'] == 'MAPPER':
                self.process_mapper_mission(mission)
            elif mission['type'] == 'GUIDER':
                self.process_guider_mission(mission)
        else:
            self.get_logger().info(f'Mission ID {mission_id} does not exist.')

    def process_mapper_mission(self, mission):
        current_spot = mission.get('nextParkingSpot')
        parking_spots = mission.get('parkingSpotsIds', [])
        
        if current_spot in parking_spots:
            next_index = (parking_spots.index(current_spot) + 1) % len(parking_spots)
            next_spot = parking_spots[next_index]
        else:
            next_spot = parking_spots[0] if parking_spots else None
        
        if next_spot:
            mission_ref = db.reference(f'missions/{mission["id"]}')
            mission_ref.update({"nextParkingSpot": next_spot})
            self.get_logger().info(f'Updated nextParkingSpot to {next_spot} for mission {mission["id"]}')
    
    def process_guider_mission(self, mission):
        parking_floor_id = mission.get('parkingFloorId')
        available_spot = self.find_available_parking_spot(parking_floor_id)
        
        if available_spot:
            self.publish_parking_spot(parking_floor_id, available_spot)
            self.mark_parking_spot_unavailable(parking_floor_id, available_spot)
            self.get_logger().info(f'Assigned parking spot {available_spot} to mission {mission["id"]}')

    def find_available_parking_spot(self, parking_floor_id):
        parking_floor_ref = db.reference(f'parkingFloors/{parking_floor_id}')
        parking_spots = parking_floor_ref.get()
        
        for spot_id, spot_info in parking_spots.items():
            if spot_info.get('status') == 'AVAILABLE':
                return spot_id
        return None

    def publish_parking_spot(self, parking_floor_id, parking_spot_id):
        spot_ref = db.reference(f'parkingFloors/{parking_floor_id}/{parking_spot_id}')
        spot_info = spot_ref.get()

        if spot_info:
            position = spot_info.get('position', {})
            orientation = spot_info.get('orientation', {})

            pose = Pose()
            pose.position.x = position.get('x', 0.0)
            pose.position.y = position.get('y', 0.0)
            pose.position.z = position.get('z', 0.0)
            pose.orientation.x = orientation.get('x', 0.0)
            pose.orientation.y = orientation.get('y', 0.0)
            pose.orientation.z = orientation.get('z', 0.0)
            pose.orientation.w = orientation.get('w', 1.0)

            pose_array = PoseArray()
            pose_array.poses.append(pose)
            
            self.mission_publisher.publish(pose_array)
            self.get_logger().info(f'Published new mission with parking spot {parking_spot_id} to new_mission topic')

    def mark_parking_spot_unavailable(self, parking_floor_id, parking_spot_id):
        spot_ref = db.reference(f'parkingFloors/{parking_floor_id}/{parking_spot_id}')
        spot_ref.update({"status": "UNAVAILABLE"})
        self.get_logger().info(f'Marked parking spot {parking_spot_id} as unavailable')

    def publish_takeoff_signal(self):
        msg = Empty()
        self.publisher.publish(msg)
        self.get_logger().info('Published takeoff signal')
        
    def publish_mission(self):
        # Your code to publish mission
        self.get_logger().info("Database is being updated and mission is published")
        
    def setup_database_listener(self):
        # Assuming 'missions' is the database path you're interested in
        ref = db.reference('drones')
        
        # Listen for any changes in the 'missions' path
        ref.listen(self.on_db_update)
        
    def on_db_update(self, event):
        # Callback function triggered by database updates
        self.publish_mission()

def connect_database(): 
    cred = credentials.Certificate("credentials/project-amado-credentials.json")
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://project-amado-default-rtdb.europe-west1.firebasedatabase.app'
    })

if __name__ == "__main__": 
    connect_database()
    rclpy.init()
    node = FirestoreNode()
    rclpy.spin(node)
    rclpy.shutdown()
