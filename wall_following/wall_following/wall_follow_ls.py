#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
import math as m
from tf_transformations import euler_from_quaternion

import csv
from sklearn.linear_model import LinearRegression
import time
from rclpy.signals import SignalHandlerOptions 

class Robot(Node):
    def __init__(self):
        super().__init__('wall_following_node')
        
        # Parametro konfiguragarrien deklarazioa
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 1.5),  # Kontrol angeluarerako proportzio konstantea
                ('vel_topic', 'cmd_vel'),  # Abiadurak gordetzeko parametroa
                ('pose_topic', 'rosbot_pose'),  # Robotaren posizioa gordetzeko parametroa
                ('output_filename', 'wf_ls')  # CSV irteera-fitxategiaren izenaren oinarria
            ]
        )
        self.kp = self.get_parameter('kp').value
        # pose topic
        self.pose_topic = self.get_parameter('pose_topic').value
        # velocity topic
        self.vel_topic = self.get_parameter('vel_topic').value
        # odometry topic
        self.pose_topic = self.get_parameter('pose_topic').value

        ''' output filename (with no extension). 
        kp value and .csv extension will be added 
        to this name afterwards 
        '''
        # CSV irteera-fitxategiaren konfigurazioa
        f = self.get_parameter('output_filename').value
        fout = f + '_' + str(self.kp) + '.csv'
        file = open(fout, 'w')
        self.csvwriter = csv.writer(file, delimiter=',')
        self.csvwriter.writerow(['kp', 'error', 'v', 'w', 'x', 'y'])  # CSV fitxategiaren goiburua
        
        # Harpidetzak eta argitalpena
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.follow_wall, 1)  # Laser sentsorearen harpidetza
        self.pose_sub = self.create_subscription(Pose2D, self.pose_topic, self.get_position, 1)  # Robotaren posizioaren harpidetza
        self.vel_pub = self.create_publisher(Twist, self.vel_topic, 1)  # Abiadurak argitaratzailea
        
        # Aldagaien hasieraketa
        self.scan_count = 0  # Laser sentsorearen irakurketa kopurua
        self.range_min = 0.0  # Sentsorearen gutxieneko tartea
        self.max_range = 8.0  # Sentsorearen gehienezko tartea
        self.bearings = []  # Laser sentsorearen angeluak
        self.rx = 0  # Robotaren x posizioa
        self.ry = 0  # Robotaren y posizioa
        self.rtheta = 0  # Robotaren orientazioa
        self.uninitialized = True  # Laser irakurketak hasieratzeko aldagaia

    def get_position(self, msg):
        """
        Robotaren posizioa eta orientazioa eguneratzeko funtzioa
        """
        # Gets robot pose (x, y, yaw) from odometry
        #self.get_logger().info("Robot pose: x: %.2f, y: %.2f, theta: %.2f"%(msg.x, msg.y, msg.theta))

        self.rx = msg.x
        self.ry = msg.y
        self.rtheta = msg.theta

    def follow_wall(self, scan):             
        """
        Laser sentsorearen irakurketak prozesatzeko eta pareta jarraitzeko funtzioa
        """
        lscan = scan.ranges
        cmd_vel_msg = Twist()  # Abiadura mezua
        
        # Lehen aldiz bada, laser irakurketak hasieratu
        if self.uninitialized:
            self.uninitialized = False
            self.scan_count = len(scan.ranges)
            self.range_min = scan.range_min
            for i in range(self.scan_count):
                self.bearings.append(scan.angle_min + scan.angle_increment * i)
        
        # Irakurketa garrantzitsuak iragazi
        self.scan = [scan.ranges[i - 800] for i in range(self.scan_count)]

        # Detektatutako puntu laburrak gordetzeko matrizeak sortu
        xpos = np.empty((self.scan_count, 1), float)
        ypos = np.empty((self.scan_count, 1), float)
        j = 0  # Baliozko puntuen zenbatzailea

        # Irakurketa laburrak aukeratu eta haien proiekzioak kalkulatu (x, y koordenatuak)
        for i in range(400, 800):
            if 0 < self.scan[i] < 2.0:  # Barrutiko balioak iragazi
                x = self.scan[i] * m.cos(self.bearings[i])  # x koordenatua
                y = self.scan[i] * m.sin(self.bearings[i])  # y koordenatua
                xpos[j] = x
                ypos[j] = y
                j += 1

        # Erregresio lineala egin puntu nahikoak badaude
        c1 = 0.0
        c0 = 0.0
        theta2 = 0.0
        if j > 5:  # Puntu nahikoak dauden egiaztatu
            xpos.resize(j, 1)
            ypos.resize(j, 1)

            # Detektatutako puntuetarako lerro bat egokitu
            model = LinearRegression()
            model.fit(xpos, ypos)
            c0 = float(model.intercept_)  # Lerroaren ebakidura
            c1 = float(model.coef_)  # Lerroaren malda
    
            # Robotaren eta paretaren arteko angelua kalkulatu
            theta = m.atan2(c1, 1)
            self.get_logger().info("Malda: %.2f Angelua: %.2f (%.2f gradu)" % (c1, theta, m.degrees(theta)))
            
            # Abiadurak doitu: errore angeluarraren araberakoa
            w = self.kp * theta  # Abiadura angeluarra
            v = 0.2  # Abiadura lineal konstantea
            self.get_logger().info("Abiadurak: v = %.2f w = %.2f" % (v, w))

            cmd_vel_msg.linear.x  = v
            cmd_vel_msg.linear.y  = 0.0
            cmd_vel_msg.angular.z = w
        else:
            # Puntu nahikorik ez badago, aurrera lerro zuzenean
            self.get_logger().info("Ez da puntu nahikorik erregresio lineala aplikatzeko")
            cmd_vel_msg.linear.x  = 0.2
            cmd_vel_msg.linear.y  = 0.0
            cmd_vel_msg.angular.z = 0.0

        # Datuak CSV fitxategian gorde
        self.csvwriter.writerow([f"{self.kp:.2f}", f"{c0:.2f}", f"{c1:.2f}", f"{theta2:.2f}", f"{cmd_vel_msg.linear.x:.2f}", f"{cmd_vel_msg.angular.z:.2f}", f"{self.rx:.2f}", f"{self.ry:.2f}"])
        
        # Abiadura komandoak argitaratu
        self.vel_pub.publish(cmd_vel_msg)

def main(args=None):
    """
    Programaren sarrera-puntua
    """
    rclpy.init(args=args)
    wf_node = Robot()
    try:
        rclpy.spin(wf_node)
    except KeyboardInterrupt:
        wf_node.destroy_node() 

if __name__ == '__main__':
    main()  # Funtzio nagusia exekutatu
