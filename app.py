# app.py
import threading
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from flask import Flask, render_template, redirect, url_for, Response
from flask_socketio import SocketIO, emit
from process_manager import start_process, kill_process

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secret_key'  # Change this in production!
socketio = SocketIO(app)

# Globals for storing latest frames
bridge = CvBridge()
latest_jpeg_zed = None       # ZED camera
latest_jpeg_stitched = None  # Stitched feed

class MultiImageSubscriber(Node):
    def __init__(self):
        super().__init__('multi_image_subscriber')
        
        # 1) Subscribe to ZED camera
        self.sub_zed = self.create_subscription(
            Image,
            '/zed/image_left',
            self.zed_callback,
            10
        )
        
        # 2) Subscribe to stitched camera
        self.sub_stitched = self.create_subscription(
            Image,
            '/camera/stitched_image',
            self.stitched_callback,
            10
        )

    def zed_callback(self, msg):
        """Callback for ZED camera images."""
        global latest_jpeg_zed
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            success, buffer = cv2.imencode('.jpg', bgr_image)
            if success:
                latest_jpeg_zed = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f"ZED callback: {e}")

    def stitched_callback(self, msg):
        """Callback for the stitched camera images."""
        global latest_jpeg_stitched
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            success, buffer = cv2.imencode('.jpg', bgr_image)
            if success:
                latest_jpeg_stitched = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f"Stitched callback: {e}")

def ros_spin_thread():
    """Spin the ROS2 node in a background thread."""
    rclpy.init()
    node = MultiImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

# Start ROS spin thread
ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
ros_thread.start()

# Define main commands (Arm, Drives, Astrotech)
commands = {
    'Arm': {
        'arm': 'ros2 run cmr_rovernet armnet_node'
    },
    'Drives': {
        'drives': 'ros2 run cmr_controls swerve_controller_node'
    },
    'Astrotech': {
        'start_astrotech_node': 'ros2 run my_astrotech_package astrotech_node',
        'kill_astrotech_node': 'ros2 node kill /astrotech_node'
    }
}

# Define camera commands (ZED, Stitched, End Effector)
camera_commands = {
    'ZED': {
        'run_zed': 'ros2 run cmr_zed zed_publisher_node',
        'kill_zed': 'ros2 node kill /zed_publisher_node'
    },
    'Stitched': {
        'run_stitched': 'ros2 run usb_camera_publisher stitched',
        'kill_stitched': {
            'publish': True,
            'topic': '/node_management/shutdown',
            'msg_type': 'std_msgs/String',
            'fields': {'data': 'stitched_publisher'}
        }
    },
    'EndEffector': {
        'run_ee': 'echo "Running End Effector node..."',
        'kill_ee': 'echo "Killing End Effector node..."'
    }
}


@app.route('/')
def index():
    return redirect(url_for('control_page'))

@app.route('/control')
def control_page():
    return render_template('control.html', commands=commands)

@app.route('/debug')
def debug_page():
    return render_template('debug.html')

@app.route('/camera_feeds')
def camera_feeds_page():
    return render_template('camera_feeds.html', camera_commands=camera_commands)

@app.route('/image_feed')
def image_feed():
    """Return the latest ZED camera frame as JPEG."""
    global latest_jpeg_zed
    if latest_jpeg_zed is not None:
        return Response(latest_jpeg_zed, mimetype='image/jpeg')
    else:
        return "No ZED image received yet", 200

@app.route('/stitched_feed')
def stitched_feed():
    """Return the latest stitched camera frame as JPEG."""
    global latest_jpeg_stitched
    if latest_jpeg_stitched is not None:
        return Response(latest_jpeg_stitched, mimetype='image/jpeg')
    else:
        return "No stitched image received yet", 200

@socketio.on('command')
def handle_command(data):
    command_name = data.get('command_name')
    category = data.get('category')

    cmd = None
    if category in commands and command_name in commands[category]:
        cmd = commands[category][command_name]
    elif category in camera_commands and command_name in camera_commands[category]:
        cmd = camera_commands[category][command_name]

    if cmd:
        result = start_process(command_name, cmd)
        emit('command_response', {
            'pid': result['pid'],
            'command': cmd,
            'stdout': result['stdout'],
            'stderr': result['stderr'],
            'running': result['running']
        })
    else:
        emit('command_response', {
            'error': f'Unknown command/category: {command_name}, {category}'
        })


@socketio.on('kill_command')
def handle_kill_command(data):
    """
    Handles kill commands. Supports both process-based kills and publishing shutdown messages.
    Expects: {command_name: ...}
    """
    command_name = data.get('command_name')

    # Search in camera_commands for a publish-style kill
    for category in camera_commands:
        if command_name in camera_commands[category]:
            cmd = camera_commands[category][command_name]
            if isinstance(cmd, dict) and cmd.get('publish'):
                topic = cmd['topic']
                msg_type = cmd['msg_type']
                fields = cmd['fields']
                fields_str = ', '.join(f"{k}: {v}" for k, v in fields.items())
                ros_cmd = f"ros2 topic pub {topic} {msg_type} '{{{fields_str}}}' -1"
                pid = start_process(f"publish_{topic}", ros_cmd)
                emit('kill_response', {'status': 'Sent shutdown message', 'pid': pid})
                return

    # Fallback: regular kill (e.g. used for 'kill_zed')
    if kill_process(command_name):
        emit('kill_response', {'status': f'Killed process for {command_name}'})
    else:
        emit('kill_response', {'error': f'No running process found for {command_name}'})


@socketio.on('publish_topic')
def handle_publish_topic(data):
    """
    Expects:
      {topic: "/something", msg_type: "std_msgs/String", fields: {...}}
    """
    topic = data.get('topic')
    msg_type = data.get('msg_type')
    fields = data.get('fields', {})
    fields_str = ', '.join(f"{k}: {v}" for k, v in fields.items())
    command = f"ros2 topic pub {topic} {msg_type} '{{{fields_str}}}' -1"
    pid = start_process(f"publish_{topic}", command)
    emit('publish_response', {
        'pid': pid,
        'command': command
    })

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
