import math

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

def tune_min_speed(min_speed,filename):
    if "espie" in filename:
        min_speed = 10
    if "karwada" in filename:
        min_speed = 4
    if "migrants" in filename:
        min_speed = 6
    if "ruudskogen" in filename:
        min_speed = 12
    if "aalborg" in filename:
        min_speed = 5
    if "corkscrew" in filename:
        min_speed = 6
    return min_speed

def tune_target_speed(target_speed,turn_radius,predicted_segment,filename):
    if "espie" in filename:
        if "t17" in predicted_segment.name:
            target_speed =  48
    if "forza" in filename:
        if "curve 11" in predicted_segment.name:
            target_speed =  53
        if "curve 13" in predicted_segment.name:
            target_speed =  34
        if "curve 14" in predicted_segment.name:
            target_speed =  38
        if "curve 15" in predicted_segment.name:
            target_speed =  42
        if predicted_segment.radius>210:
            target_speed -=  target_speed / 8
        if predicted_segment.radius>1000:
            target_speed = 48 + turn_radius / 160
    if "migrants" in filename:
        if "S2"==predicted_segment.name:
            target_speed =  38
        if "S9" in predicted_segment.name:
            target_speed =  46
        if "S22"==predicted_segment.name:
            target_speed =  58
    if "ruudskogen" in filename:
        if "curve 1." in predicted_segment.name:
            target_speed =  36
        if "curve 37" in predicted_segment.name:
            target_speed =  30
        if "curve 5." in predicted_segment.name:
            target_speed =  24
        if "curve 18" in predicted_segment.name:
            target_speed =  44
        if "curve 19." in predicted_segment.name:
            target_speed =  38
    if "aalborg" in filename:
        if "40"==predicted_segment.name:
            target_speed =  28
        if "160"==predicted_segment.name:
            target_speed =  29
        if "190"==predicted_segment.name:
            target_speed =  54
        if "200"==predicted_segment.name:
            target_speed =  52
    if "corkscrew" in filename:
        if "s10." in predicted_segment.name:
            target_speed =  46
        if "s11" in predicted_segment.name:
            target_speed =  46
    return target_speed

