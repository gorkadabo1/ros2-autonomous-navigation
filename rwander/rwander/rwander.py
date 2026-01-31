#!/usr/bin/env python

import glob
from ament_index_python import get_package_share_directory
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
import numpy as np

# math importatzen dugu 'm' alias-a emanda aurrerago biraketa kalkulatzeko beharko dugulako
import math as m

class Robot(Node):
    def __init__(self):
        super().__init__('rwander_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('pose_topic', 'rosbot_pose')
        vel_topic_ = self.get_parameter('vel_topic').value
        scan_topic_ = self.get_parameter('scan_topic').value
        pose_topic_ = self.get_parameter('pose_topic').value

        # ROS Subscribers
        self._laser_sub = self.create_subscription(LaserScan, scan_topic_, self.obstacle_detect, 10)
        self._pose_sub = self.create_subscription(Pose2D, pose_topic_, self.pose_callback, 10)
        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, vel_topic_, 10)       
        self._scan_count = 0
        self._max_range = 100.0
        self._uninitialized = 1
        self._bearings = []
        self._scan = []

        
        
    def obstacle_detect(self, scan_msg):
        
        self._scan = scan_msg.ranges

        if self._uninitialized:
            self._uninitialized = 0
            self._scan_count = len(scan_msg.ranges)
            self._max_range = scan_msg.range_max
            # Bearings stores the angle of each range measurement (radians)
            for i in range(0, self._scan_count):
                self._bearings.append(scan_msg.angle_min + scan_msg.angle_increment * i)
            self.get_logger().info("# Scan count %d"%(self._scan_count))  
            self.get_logger().info("# Laser angle min: %.2f"%(np.rad2deg(scan_msg.angle_min)))
            self.get_logger().info("# Laser angle max: %.2f"%(np.rad2deg(scan_msg.angle_max)))
            self.get_logger().info("# Laser angle increment:  %.4f rad (%.2f deg)"%(scan_msg.angle_increment, np.rad2deg(scan_msg.angle_increment)))
            self.get_logger().info("# Time between mesurements [seconds]:  %.2f"%(scan_msg.time_increment))
            self.get_logger().info("# Time between scans [seconds]:  %.2f"%(scan_msg.scan_time))
            self.get_logger().info("# Minimum range value:  %.2f"%(scan_msg.range_min))
            self.get_logger().info("# Maximum range value:  %.2f "%(scan_msg.range_max))
            resolution = (scan_msg.angle_max - scan_msg.angle_min)/len(scan_msg.ranges)
            self.get_logger().info("# Resolution:  %.2f"%(np.rad2deg(resolution))) 

        # Replace infinity values with max_range 
        self._scan = [x if x < self._max_range else self._max_range for x in self._scan] 

        # Reorganize scan indexes to make it easier to work with. 
        # 0 index corresponds to the back side of the robot for both, scan and bearings.
        self._scan = [self._scan[i - 800] for i in range(self._scan_count)]    
    
        # TODO: add your code here
        luzera_max = 0 # segmentu libre luzeenaren luzera gortzeko
        max_hasiera = 0 # segmentu libre luzeenaren hasiera gordetzeko
        max_bukaera = 0 # segmentu libre luzeenaren bukaera gordetzeko
        segmentu_luzera = 0 # segmentu libre bakoitzaren luzera gordetzen joateko iterazio bakoitzean
        segmentu_hasiera = 0 # segmentu libre bakoitzaren hasiera gordetzen joateko
        batazb_distantzia_max = 0 # segmentu luzeearen batazbesteko distantzia gordetzeko. Aldagai hau bi segmentuk luzera berdina dutenean erabiltzen da
                                  # distantzia altuena duena aukeratzeko

	    #robotak eskaneatutako interesatzen zaizkigun balioetan iteratzen duen beigzta
        for i, distance in enumerate(self._scan[400:1200]): #400-etik 1200-era aurreko 180º bakarrik edukitzeko kontuan
            if self._max_range > distance > 2.0: # distantzia 2 metro baino handiagoa bada eta rango maximoa baino txikiagoa, libretzat hartuko dugu
                if segmentu_luzera == 0: #segmentua berria bada, hasieratu
                    segmentu_hasiera = i
                segmentu_luzera += 1

                segment_avg_distance = sum(self._scan[segmentu_hasiera:i + 1]) / segmentu_luzera #iterazio bakoitzean uneko segmentuaren batazbesteko distantzia kalkulatu

                # segmentuaren luzera orain arte luzeena zenarena baino luzeagoa bada, edo berdina bada baina batazbesteko distantiza handiagoa bada, uneko segmentua
                # izango da segmentu luzeena. Kasu honetan, balio maximoakgordetzen dituzten aldagiak eguneratu.
                if segmentu_luzera > luzera_max or (segmentu_luzera == luzera_max and segment_avg_distance > batazb_distantzia_max):
                    luzera_max = segmentu_luzera #
                    max_hasiera = segmentu_hasiera
                    max_bukaera = i
                    batazb_distantzia_max = segment_avg_distance
            else: # distantzia 2 metro edo gutxiagokoa bada (edo maximoa baino handiagoa), okupatutzat hartuko dugu
                segmentu_luzera = 0 # segmentua berriz hasieratu

	    # segmentu luzeenaren luzera 0 baino handiagoa bada (hau da, segmentu librerenbat badago), abiadura eta biraketa kalkulatu
        if luzera_max > 0: 
            helburu_noranzkoa = (max_hasiera + max_bukaera) / 2.0 # Noranzkoari segmentu luzeenaren erdiko balioa ematen diogu
            # Helburuari 400 gehitzen diogu begiztan enumerate() erabiltzen dugunez segmentuen balioak 0-799 tartean daudelako,
            # baina robotaren aurreko 180 graduak 400-etik 1200-era doaztenez, 400 gehitu behar zaio.
            noranzko_berria = helburu_noranzkoa + 400 

	        # Errorea kalkulzaten dugu, zeina 800 - noranzkoa den. Horrela, robotaren aurreko puntua, joan nahi dugun noranzkotik zein urrun dagoen kalkulatzen dugu
            error = 800 - noranzko_berria

            # Biraketa abiadura kalkulatzeko, errorea 0.3-rekin biderkatzen dugu biraketa poliki egin dezan, eta errorea '-1' ekin 
            # biderkatzen dugu, robotak desbideratzearen kontrako noranzkoan biratu behar baitu, mugimendua helbururantz zentratzeko.
            # Ondoren, math.radians() funtzioa erabiltzen dugu biraketa balioa radianetara pasatzeko.
            turn = m.radians(-error * 0.3)

            # Abiadura lineala kalkulatzeko, biraketa balioaren kosenoa kalkulatzen dugu. Horrela, biraketa oso handia bada, kosenoaren
            # abiadura baxua izango da eta ondorioz biraketa polikiago egingo du; aldiz, biraketa txikia bada, kosenaren balioa handiagoa
            # izango da eta biraketa azkarrago egingo du. Ondoren, kosenoarei balio absolutua aplikatzen diogu abiadura beti postiboa izan 
            # dadin (robota ez dadin atzerantz joan). Azkenik, lortutako balioa bider 0.5 egiten dugu, abiadura gehienez 0.5 izan dadin,
            # bestela, biraketa batzuetan azkarregi doa eta oztopoekin talka egiten du.
            speed = abs(m.cos(turn)) * 0.5
            
        # Segmentu librerik ez egotekotan, abiadura 0-ra jartzen dugu robota geltitu dadin eta biraketa 70 gradutako biraketa angelua ematen diogu robotari
        # sekuentzia librerik ez dagoen lekutik noranzkoa aldatu dezan
        else: 
            speed = 0.0
            turn = m.radians(70)
        ## end TODO
        
        cmd_vel_msg_ = Twist()
        cmd_vel_msg_.linear.x  = speed
        cmd_vel_msg_.linear.y  = 0.0
        cmd_vel_msg_.angular.z = turn
        self._cmd_vel_pub.publish( cmd_vel_msg_ ) 
    
    def pose_callback(self, msg):
        self.get_logger().info("Robot pose: x: %.2f, y: %.2f, theta: %.2f"%(msg.x, msg.y, msg.theta))
        

                                            
def main(args=None):
    rclpy.init(args=args)
    rwander_node = Robot()
    try:
        rclpy.spin(rwander_node)
    except KeyboardInterrupt:
        rwander_node.get_logger().info("Node interrupted by keyboard (CTRL+C)")
    finally:  
        rwander_node.destroy_node()  
        rclpy.shutdown()
    
if __name__=="__main__":
    main()
