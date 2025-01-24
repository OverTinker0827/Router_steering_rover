import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
class Step(Node):
    def __init__(self):
        super().__init__("step")
        print("initialized")
        self.pub = self.create_publisher(Int32, "/step", 10)
        self.step_angle = 5
        self.angle = 360/0.9
        self.current_strength = self.signal_strength()
        self.threshold = 2
        self.timer = self.create_timer(20, self.optimizer)
        print("Timer started")

    def optimizer(self):
        
        if self.current_strength == None:
            
            self.current_strength = self.signal_strength()
            print(f"Initial strength: {self.current_strength}")
            return

        print(f"Current strength: {self.current_strength}")
        clockwise_strength = self.stepped_strength(1)
        print(f"Clockwise strength: {clockwise_strength}")
        anticlockwise_strength = self.stepped_strength(-1)
        print(f"Anticlockwise strength: {anticlockwise_strength}")

        if abs(self.current_strength-clockwise_strength) < self.threshold and abs(self.current_strength-anticlockwise_strength) < self.threshold:
            print("Below threshold, no movement needed")
            return

        if clockwise_strength > self.current_strength and anticlockwise_strength < self.current_strength:
            msg = Int32()
            msg.data = int(self.step_angle/360/self.angle)
            self.pub.publish(msg)
            print(f"Moving clockwise: {msg.data}")
            return
        elif clockwise_strength < self.current_strength and anticlockwise_strength > self.current_strength:
            msg = Int32()
            msg.data = -int(self.step_angle/360/self.angle)
            self.pub.publish(msg)
            print(f"Moving anticlockwise: {msg.data}")
            return
        elif clockwise_strength < self.current_strength and anticlockwise_strength < self.current_strength:
            print("Both directions worse, staying put")
            return
        else:
            if clockwise_strength > anticlockwise_strength:
                dir = 1
            elif clockwise_strength < anticlockwise_strength:
                dir = -1
            else:
                dir = 0
            msg = Int32()
            msg.data = dir*int(self.step_angle*self.angle/360)
            self.pub.publish(msg
            print(f"Moving in direction {dir}: {msg.data}")

    def signal_strength(self, interface="wlan0"): 
        try:
            result = subprocess.run(["iwconfig"], capture_output=True, text=True)
            for line in result.stdout.split("\n"):
                if "Link Quality" in line:
                    signal = line.split("Signal level=")[-1].split(" ")[0]
                    return int(signal)
            return None
        except Exception as e:
            print(f"Error: {e}")
            return None


    def stepped_strength(self, dir):
        msg = Int32()
        msg.data = dir*int(self.step_angle/360/self.angle)
        self.pub.publish(msg)
        
        time.sleep(5)
        return self.signal_strength()

def main(args=None):
    rclpy.init(args=args)
    node = Step()
    print("Starting to spin node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()