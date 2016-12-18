"""
  follow the race loop at computed speed (for any track)

  usage:
     race.py <track XML file>
"""

# strategy:
# - set speed high for straight segments and low for sharp turned segments
# - control speed by gas and brake (PID regulator)
# - cut turns by predicted position

# changelog:
# v0.03 - 2016-12-18 - safer exits from turns and better speed regulation
# v0.02 - 2016-12-16 - improved speed in turns but unsafe on exits
# v0.01 - 2016-12-15 - basic universal racing version (for any track)

import math
import socket
import struct
import sys
import time

from track import Track

def segment_turn(segment):
    """Return turn angle in degrees (expected car structure input)"""
    # for Robocar with following definition
    # <section name="Front Axle">
    #   <attnum name="xpos" min="0.1" max="5" val="1.104"/>
    # <section name="Rear Axle">
    #   <attnum name="xpos" min="-5" max="-0.1" val="-1.469"/>
    axis_dist = 1.104 + 1.469

    if segment.arc is None:
        return 0.0
    angle = math.atan2(axis_dist, segment.radius)
    if segment.arc < 0:
        # right left
        angle = -angle
    return math.degrees(angle)


def drive(track):
    print "race.py v0.03 (2016-12-18)"
    #time.sleep(10) # wait for game start
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port = 4001
    soc.bind(('', port))
    soc.settimeout(1.0)
    ctr = 0
    gas = 0.0
    brake = 0.0
    turn = 0.0
    prev_segment = None
    old_segment = None
    predicted_segment = old_segment
    min_speed = 14
    max_speed = 66
    target_speed = max_speed
    turn_speed = min_speed  
    prev_turn_speed = min_speed  
    sum_e = 0
    last_speed = 0
    mem_speed = {}
    start = 1
    while True:
        data = struct.pack('fffiBB', turn, gas, brake, 1, 11, ctr & 0xFF)
        soc.sendto(data, ('127.0.0.1', 3001))
        try:
            status = soc.recv(1024)
            assert len(status) == 794, len(status)
            sim_status = struct.unpack_from('B', status, 792)[0]
            if sim_status == 5: # simulation stopped
                break
            absPosX, absPosY, absPosZ = struct.unpack_from('fff', status, 44)
            angX, angY, angZ = struct.unpack_from('fff', status, 56)
            heading = angZ  # in radiands +/- PI

            absVelX, absVelY = struct.unpack_from('ff', status, 80)
            speed = math.sqrt(absVelX*absVelX + absVelY*absVelY)

            rpm,elevel = struct.unpack_from('ff', status, 16)
            
            prediction_time = speed / 18  # sec
            absPosX1 = absPosX + prediction_time * absVelX
            absPosY1 = absPosY + prediction_time * absVelY
            predicted_segment, rel_pose1 = track.nearest_segment((absPosX1, absPosY1, heading))

            prediction_time = 0.28 + speed / 300  # sec
            absPosX += prediction_time * absVelX
            absPosY += prediction_time * absVelY

            segment, rel_pose = track.nearest_segment((absPosX, absPosY, heading))
            if segment is not None:
                signed_dist, heading_offset = segment.get_offset(rel_pose)

                turn = segment_turn(segment)
                if heading_offset is not None:
                    turn -= math.degrees(heading_offset)

                # set speed for predicted segment (far prediction)                    
                if predicted_segment is not None:
                    if old_segment!=predicted_segment:
                        start = 0
#                         if old_segment is not None:
#                             print old_segment.name,"->",predicted_segment.name,round(speed),predicted_segment.radius,predicted_segment.arc,predicted_segment.length
                        if predicted_segment.arc is None:
                            # straight
                            target_speed = max_speed
                            if predicted_segment.length<70:
                                target_speed = max_speed * 0.85
                        else:
                            # turn
                            target_speed = min_speed + predicted_segment.radius / 2.8
                            if target_speed>32:
                                target_speed = 32 + predicted_segment.radius / 20
                            if target_speed>49:
                                target_speed = 49
                            if predicted_segment.radius>210:
                                target_speed = 44 + predicted_segment.radius / 30
                            if predicted_segment.radius>1000:
                                target_speed = 48 + predicted_segment.radius / 100
                            turn_speed = target_speed
                        old_segment = predicted_segment
                        mem_speed[predicted_segment.name] = target_speed
                    else:
                        target_speed = target_speed  
                else:
                    target_speed = min_speed  

                # set speed for actual segment
                if old_segment is not None and old_segment==segment and segment.radius is None:
                    # straight
                    target_speed = max_speed
                    if segment.name in mem_speed: target_speed = mem_speed[segment.name]
                if segment.radius is not None:
                    # turn
                    target_speed = turn_speed
                    if segment.name in mem_speed: target_speed = mem_speed[segment.name]
                if start==1: target_speed = max_speed
                
                if target_speed>max_speed: target_speed = max_speed 

                # PID speed regulation
                P = 0.05
                I = 0.002
                D = 2.0
                max_sum_e = 1/I
                e = target_speed - speed
                sum_e += e
                if sum_e>max_sum_e: sum_e = max_sum_e
                if sum_e<-max_sum_e: sum_e = -max_sum_e
                gas = P * e + I * sum_e - D * (speed - last_speed)
                last_speed = speed
                brake = 0.0
                if gas>1.0: gas = 1
                if gas<0.0: gas = 0
                if gas<0.3 and speed>min_speed:
                    gas = 0.01
                    if speed>(target_speed + 2):
                        brake = 0.19
                        #brake = 0.2 + speed/300
                if rpm>14900:
                    gas = 0.01

                dead_band = 0.1
                max_dist_turn_deg = 29

                if signed_dist < -dead_band:
                    # turn left
                    turn += min(max_dist_turn_deg, -dead_band - signed_dist)

                elif signed_dist > dead_band:
                    # turn right
                    turn += max(-max_dist_turn_deg, dead_band - signed_dist)

            if prev_segment != segment:
                #print segment.name,round(target_speed),round(speed),round(100*gas)/100,rpm,segment.radius,segment.arc,segment.length
                radius,arc,length = segment.radius,segment.arc,segment.length
                if radius is not None: radius = round(radius)
                if arc is not None: arc = round(arc)
                if length is not None: length = round(length)
                print '{0:10} {1:4} {2:4} {3:4} {4:7} {5:4} {6:4} {6:4}'.format(segment.name,round(target_speed),round(speed),round(100*gas)/100,round(rpm),radius,arc,length)
                prev_segment = segment

        except socket.error:
            print "socket timeout",ctr
            time.sleep(1)
            #pass
        ctr += 1

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    filename = sys.argv[1]
    track = Track.from_xml_file(filename)
    #print_track(track)
    drive(track)

# vim: expandtab sw=4 ts=4
