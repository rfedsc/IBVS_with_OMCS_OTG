#!/usr/bin/env python3
import rospy
from lys_visp_demo.srv import SendCommand

def send_command_client(server_ip,server_port,serial_number,command):
    rospy.wait_for_service('send_command')
    try:
        send_command = rospy.ServiceProxy('send_command',SendCommand)
        response = send_command(server_ip,server_port,serial_number,command)
        print("Prased data:",response.respond)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed:{e}")

if __name__ == '__main__':
    rospy.init_node('socket_client_test')
    send_command_client('10.10.56.214',23333,1,"mot.setGp(0,true)")


