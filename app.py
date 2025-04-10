# app.py
from flask import Flask, render_template, redirect, url_for
from flask_socketio import SocketIO, emit
from process_manager import start_process, kill_process
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secret_key'  # Change in production!
socketio = SocketIO(app)

# Predefined commands organized by tab.
# For Arm and Drives we now only use a single key for start.
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

@app.route('/')
def index():
    # Redirect the root URL to the control page.
    return redirect(url_for('control_page'))

@app.route('/control')
def control_page():
    return render_template('control.html', commands=commands)

@app.route('/debug')
def debug_page():
    return render_template('debug.html')

@socketio.on('command')
def handle_command(data):
    """
    Starts a command from the control page.
    Expects: {command_name: ..., category: ...}
    """
    command_name = data.get('command_name')
    category = data.get('category')
    if category in commands and command_name in commands[category]:
        cmd = commands[category][command_name]
        # Start the process and store its PID.
        pid = start_process(command_name, cmd)
        emit('command_response', {'pid': pid, 'command': cmd})
    else:
        emit('command_response', {'error': 'Unknown command'})

@socketio.on('kill_command')
def handle_kill_command(data):
    """
    Kills a running process.
    Expects: {command_name: ...}
    """
    command_name = data.get('command_name')
    if kill_process(command_name):
        emit('kill_response', {'status': f'Killed process for {command_name}'})
    else:
        emit('kill_response', {'error': f'No running process found for {command_name}'})

@socketio.on('publish_topic')
def handle_publish_topic(data):
    """
    Handles publishing a message to a ROS2 topic.
    Expects data:
      { topic: "/topic", msg_type: "std_msgs/String", fields: { field1: value1, ... } }
    """
    topic = data.get('topic')
    msg_type = data.get('msg_type')
    fields = data.get('fields')
    # Convert the fields dictionary to a YAML-like string (without quotes)
    fields_str = ', '.join(f"{k}: {v}" for k, v in fields.items())
    # Construct the ros2 command; using -1 for a one-shot publish.
    command = f"ros2 topic pub {topic} {msg_type} '{{{fields_str}}}' -1"
    pid = start_process(f"publish_{topic}", command)
    emit('publish_response', {'pid': pid, 'command': command})

if __name__ == '__main__':
    # The app will be accessible on your local network.
    socketio.run(app, host='0.0.0.0', port=5000)
