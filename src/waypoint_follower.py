#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import time

def create_pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = BasicNavigator().get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = yaw
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    
    waypoints = [
        create_pose(-2.0, -0.5),
        create_pose(1.5, 1.5),
        create_pose(1.5, -1.5),
        create_pose(-1.5, -1.5),
    ]
    
    nav.waitUntilNav2Active()
    print("Nav2 ativo. Iniciando waypoint following...")
    
    loop = 0
    while rclpy.ok():
        loop += 1
        print(f"\n=== LOOP {loop} ===")
        
        for i, wp in enumerate(waypoints, 1):
            print(f"Indo para waypoint {i}/4...")
            nav.goToPose(wp)
            
            while not nav.isTaskComplete():
                time.sleep(0.1)
            
            result = nav.getResult()
            if result == 3:
                print(f"✓ Waypoint {i} alcançado")
            else:
                print(f"✗ Falha no waypoint {i}")
                break
        
        time.sleep(2)

if __name__ == '__main__':
    main()
