import subprocess as sp
import time
import os
import signal
from pathlib import Path

def Branco(SIMTIME):

    print("Now BRANCO")

    sim_name = 'BRANCO'

    uav_total = 1

    SIM_ID = int(time.time())
    sim_dir = Path(f"/mnt/hdd_1_500gb/jbranco/sims/BRANCO{SIM_ID}")
    sim_dir.mkdir()


    bag_fn_single = sim_dir.joinpath(f"Branco{SIM_ID}.bag")

    cmd_roscore = "roscore"
    cmd_launch = "roslaunch offboard_py start_offb_fusion.launch"

    record_topics = '''//rosout
        /rosout_agg
        /uav0/target_position_geolocation
        /uav0/target_position
        /uav1/target_position_geolocation
        /uav1/target_position
        '''

    record_topics = record_topics.splitlines()
    record_topics = [t.strip() for t in record_topics if t.strip() != '']

    cmd_bag = f"rosbag record -O {str(bag_fn_single)} {' '.join(record_topics)}"

    p_roscore = sp.Popen(cmd_roscore.split())

    time.sleep(1)

    p_bag = sp.Popen(cmd_bag.split())

    time.sleep(5)

    p_launch = sp.Popen(cmd_launch.split())


    t_0 = time.time()

    while time.time() - t_0 < SIMTIME :
        continue


    os.kill(p_bag.pid, signal.SIGINT)

    time.sleep(10)

    os.kill(p_launch.pid, signal.SIGINT)

    time.sleep(10)

    os.kill(p_roscore.pid, signal.SIGINT)

def FELIX(SIMTIME):

    print("Now FELIX")

    sim_name = 'FELIX'

    uav_total = 1

    SIM_ID = int(time.time())
    sim_dir = Path(f"/mnt/hdd_1_500gb/jbranco/sims/FELIX{SIM_ID}")
    sim_dir.mkdir()


    bag_fn_single = sim_dir.joinpath(f"FELIX{SIM_ID}.bag")

    cmd_roscore = "roscore"
    cmd_launch = "roslaunch offboard_py start_offb_video_all_compare.launch"

    record_topics = '''/rosout
        /rosout_agg
        /uav0/target_position_geolocation
        /target_position
        '''

    record_topics = record_topics.splitlines()
    record_topics = [t.strip() for t in record_topics if t.strip() != '']

    cmd_bag = f"rosbag record -O {str(bag_fn_single)} {' '.join(record_topics)}"

    p_roscore = sp.Popen(cmd_roscore.split())

    time.sleep(1)

    p_bag = sp.Popen(cmd_bag.split())

    time.sleep(5)

    p_launch = sp.Popen(cmd_launch.split())


    t_0 = time.time()

    while time.time() - t_0 < SIMTIME :
        continue


    os.kill(p_bag.pid, signal.SIGINT)

    time.sleep(10)

    os.kill(p_launch.pid, signal.SIGINT)

    time.sleep(10)

    os.kill(p_roscore.pid, signal.SIGINT)


Branco(400)
FELIX(400)