I'll walk you through implementing Option 2 step by step:

## Step 1: Create the Python Launcher File

Create a new file called `robot_launcher.py` in your project directory:

```python
#!/usr/bin/env python3
import subprocess
import time
import signal
import sys
import os
import threading

class RobotLauncher:
    def __init__(self):
        self.processes = []
        self.running = True
        self.threads = []
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        print("\nShutting down robot launcher...")
        self.running = False
        self.cleanup()
    
    def run_command(self, command, name, restart=True):
        """Run a command with optional auto-restart"""
        while self.running:
            try:
                print(f"Starting {name}...")
                process = subprocess.Popen(command)
                process.wait()
                
                if not restart or not self.running:
                    break
                    
                print(f"{name} ended. Restarting in 2 seconds...")
                time.sleep(2)
                
            except Exception as e:
                print(f"Error running {name}: {e}")
                if not self.running:
                    break
                time.sleep(2)
    
    def start_thread(self, command, name, restart=True):
        """Start a command in a separate thread"""
        thread = threading.Thread(
            target=self.run_command,
            args=(command, name, restart),
            daemon=True
        )
        thread.start()
        self.threads.append(thread)
        return thread
    
    def cleanup(self):
        """Clean up all running processes"""
        print("Cleaning up processes...")
        
        # Terminate all subprocesses
        for process in self.processes:
            try:
                process.terminate()
            except:
                pass
        
        # Wait a bit then force kill if needed
        time.sleep(2)
        for process in self.processes:
            try:
                process.kill()
            except:
                pass
    
    def wait_for_rosmaster(self, timeout=10):
        """Wait for ROS master to be available"""
        print("Waiting for ROS master...")
        start_time = time.time()
        while self.running and (time.time() - start_time) < timeout:
            try:
                import rosgraph
                if rosgraph.is_master_online():
                    print("ROS master is online!")
                    return True
            except:
                pass
            time.sleep(1)
        print("ROS master timeout reached")
        return False
    
    def start(self):
        """Start all ROS components"""
        try:
            # Start roscore
            print("Starting roscore...")
            roscore_process = subprocess.Popen(['roscore'])
            self.processes.append(roscore_process)
            
            # Wait for ROS master to be ready
            if not self.wait_for_rosmaster():
                print("Failed to start ROS master")
                return
            
            # Start serial node in a separate thread with auto-restart
            print("Starting serial node...")
            self.start_thread(
                ['rosrun', 'rosserial_python', 'serial_node.py', '_port:=/dev/ttyUSB0', '_baud:=57600'],
                'serial_node',
                restart=True
            )
            
            time.sleep(2)  # Give serial node time to start
            
            # Start keyboard control with auto-restart
            print("Starting keyboard control...")
            self.start_thread(
                ['rosrun', 'robot_control', 'ros_key_control.py'],
                'keyboard_control', 
                restart=True
            )
            
            # Keep main thread alive
            print("All components started. Press Ctrl+C to shutdown completely.")
            while self.running:
                time.sleep(1)
            
        except KeyboardInterrupt:
            print("\nShutdown requested by user...")
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            self.cleanup()
            print("Robot launcher stopped.")

if __name__ == "__main__":
    print("=== Robot Control Launcher ===")
    print("This launcher will auto-restart components if they crash or exit.")
    print("Press Ctrl+C to completely shutdown the system.")
    print("=" * 40)
    
    launcher = RobotLauncher()
    launcher.start()
```

## Step 2: Make the Launcher Executable

Open terminal and run:
```bash
chmod +x robot_launcher.py
```

## Step 3: Test Your Current Keyboard Control

First, let's make sure your original `ros_key_control.py` works. If you want it to be more restart-friendly, you can modify it slightly:

**Current `ros_key_control.py`** (keep this as is for now - it should work):
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pynput import keyboard

def on_press(key):
    try:
        k = key.char.upper()
        if k == 'F':
            pub.publish("F")
            rospy.loginfo("Sent command: F (Forward)")
        elif k == 'R':
            pub.publish("R")
            rospy.loginfo("Sent command: R (Reverse)")
        elif k == 'S':
            pub.publish("S")
            rospy.loginfo("Sent command: S (Stop)")
        elif k == 'Q':
            pub.publish("S")
            rospy.loginfo("Sent command: S (Stop)")
            rospy.signal_shutdown("User pressed Q to quit.")
            return False
    except AttributeError:
        pass

if __name__ == "__main__":
    rospy.init_node('keyboard_motor_control', anonymous=True)
    pub = rospy.Publisher('motor_command', String, queue_size=10)

    print("Press F = Forward, R = Reverse, S = Stop, Q = Quit.")

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
```

## Step 4: Run the Launcher

Now you can start your entire system with one command:

```bash
python3 robot_launcher.py
```

## What to Expect:

1. **The launcher will start roscore**
2. **Wait for ROS master to be ready** 
3. **Start the serial node** (auto-restarts if it crashes)
4. **Start the keyboard control** (auto-restarts when you press 'Q')

## How It Works:

- When you press **'Q'** in the keyboard control, it will exit
- The launcher detects this and restarts the keyboard control after 2 seconds
- You can press **Ctrl+C** in the launcher terminal to completely shutdown everything
- If any component crashes, it automatically restarts

## Troubleshooting:

If you get import errors, make sure you have the required Python packages:

```bash
pip install pynput
```

If the serial node can't find your Arduino, check the port:
```bash
ls /dev/ttyUSB*
```

And update the port in the launcher if needed.

## Testing:

1. Run: `python3 robot_launcher.py`
2. Wait for "All components started" message
3. Press 'F', 'R', 'S' to test motor control
4. Press 'Q' - the keyboard control should restart automatically
5. Press Ctrl+C to completely stop everything

Try this out and let me know if you encounter any issues!
