
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import time
import math;
class Step(Node):
    def __init__(self):
        super().__init__("step")
        self.pub=self.create_publisher(Int16,"/step",10)
        self.constant=2
        self.threshold=5
        self.base_distance=10
        self.sub=self.create_subscription(Odometry,"/odom/filtered",self.turn,10)
        self.cur_router_angle=0
        self.stepper_constant=360/400
        
       
    def quaternion_to_euler_angle(self,w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z


    def turn(self,msg):
        
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        quat=msg.pose.pose.orientation
        _,_,self.z=self.quaternion_to_euler_angle(quat.w,quat.x,quat.y,quat.z)
        
        self.router_turn=math.degrees(math.atan(self.x/(self.y+self.base_distance)))-self.cur_router_angle-(self.z)
        if abs(self.router_turn)>self.threshold:
            if self.cur_router_angle+self.router_turn>300:
                self.router_turn=300-self.router_turn
            self.stepper_turn_angle=self.constant*self.router_turn
            self.steps=self.stepper_constant*self.stepper_turn_angle
            msgs=Int16()
            msgs.data=int(self.steps)
            self.cur_router_angle+=self.router_turn
            
            self.pub.publish(msgs)
            

    

   

def main(args=None):
    rclpy.init(args=args)
    node = Step()
    print("Starting to spin node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()