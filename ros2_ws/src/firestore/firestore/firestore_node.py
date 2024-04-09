import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from std_msgs.msg import Empty  # Assuming you're using std_msgs.Empty for your message type

class FirestoreNode(Node):
    
    def __init__(self):
        super().__init__("FirestoreNode")
        self.publisher = self.create_publisher(Empty, 'takeoff', 1)
        self.get_logger().info("Hello from FirestoreNode")
        self.setup_database_listener()
    
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