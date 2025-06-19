# collection of globally used configuration options across all files

import velodyne_decoder as vd
import numpy as np



#################
##### INPUT #####
#################

#=== Input MODE ===
# Select mode we receive the sensor data
# "UDP": stream live from UDP (uses UDP_ options)
# "PCAP": stream from pcap dump file (uses PCAP_ options)
#DATA_RECEIVE_MODE = "PCAP"
DATA_RECEIVE_MODE = "UDP"


#=== UDP Stream config ===
UDP_LISTEN_IP_SENSOR1 = "0.0.0.0" # 0.0.0.0 uses all interfaces
UDP_PORT_SENSOR1 = 5001

UDP_LISTEN_IP_SENSOR2 = "0.0.0.0"
UDP_PORT_SENSOR2 = 5004



#=== PCAP Mode config ===
#PCAP_FILE = "../../data/testdata/2025-04-29_1person-walking_sensor-level.pcap"
#PCAP_FILE = "../../data/testdata/2025-05-06_4ppl-walking_sensor-tilted_VLP-32C.pcap"
#PCAP_FILE = "../../data/testdata/2025-05-20_dual-sensor-test_sensor-tripod.pcap.gz"

#PCAP_FILE_1 = "../../data/testdata/2025-05-20_dual-sensor-test_sensor-lamp.pcap.gz"
#PCAP_FILE_2 = "../../data/testdata/2025-05-20_dual-sensor-test_sensor-tripod.pcap.gz"

#PCAP_FILE_1 = "../../data/testdata/2025-05-28_dual-sensor-test_sensor-tripod.pcap.gz"
#PCAP_FILE_2 = "../../data/testdata/2025-05-28_dual-sensor-test_sensor-lamp.pcap.gz"

PCAP_FILE_1 = "../../data/testdata/2025-06-03_leave-enter-room_sensor-tripod.pcap.gz"
PCAP_FILE_2 = "../../data/testdata/2025-06-03_leave-enter-room_sensor-lamp.pcap.gz"
# manually correct delay between recording start of each file
PCAP_FILE_1_PLAYBACK_DELAY_MS = 0
PCAP_FILE_2_PLAYBACK_DELAY_MS = 900

#PCAP_FILE_1 = "../../data/testdata/2025-06-03_walk-use-chairs_sensor-tripod.pcap.gz"
#PCAP_FILE_2 = "../../data/testdata/2025-06-03_walk-use-chairs_sensor-lamp.pcap.gz"



#Use packet timestamps to playback at original speed if possible
PCAP_FILE_REALTIME_PLAYBACK = True

# When PCAP file is finished (all lines replayed) start over from beginning (adds offset to timestamp as if its new data)
PCAP_LOOP_WHEN_FILE_COMPLETED = True
# Note: its known to break the sync between 2 files (since wireshark recordings usually start and end slightly different...)

# Custom fixed delay between reading frames from file (useful when not playing back in realtime + goal is to speed up / slow down replay)
#PCAP_FILE_PACKET_DELAY = 0.0001
PCAP_FILE_PACKET_DELAY = 0
# Note: when too large delay the decoder outputs full frame too early

# Ignore packets that dont have this target port - set to None to disable this filter
# useful when using unfiltered wireshark dumps, prevents warning spam and decoder confusion
# FIXME: we need to configure 2 ports when using 2 sensors if used?
PCAP_FILE_FILTER_UDP_PORT_SENSOR_1 = None
PCAP_FILE_FILTER_UDP_PORT_SENSOR_2 = None





###############################
#####  Script Behaviour   #####
###############################

# create laz files for each merged frame in output/
FILE_EXPORT_ENABLE = True

# create output/status.json regularly updated with latest values e.g. detected people, framerates, processing duration...
STATUS_FILE_ENABLED = True 
STATUS_FILE_PATH = "output/status.json"
# Ignore some keys to simplyfy the output on demand:
STATUS_FILE_KEYS_NOT_ADDED_TO_FILE = ["LOG_LAST_ERRORS", "TIMING_FRAMERATE_DECODER_1", "TIMING_FRAMERATE_DECODER_2", "TIMING_PROCESSING_DURATION_MS", "LOG_LAST_WARNINGS", "TIMING_MOTION_DETECTION__APPLY_FILTERS", "TIMING_MOTION_DETECTION__CLUSTER_DETECTION", "TIMING_MOTION_DETECTION__VISUALIZER_UPDATE" ]

DASHBOARD_TCP_SERVER_ENABLED = False

# enable motion+people detection and tracking
MOTION_DETECTION_ENABLED = True

# === Visualizer window config ===
# Options:
# - "none"
# - "sensor1"
# - "sensor2"
# - "merged_dual"         → sensor1 + transformed sensor2 in different colors
# - "merged_filtered"     → merged + filtered comparison (visualize crop)
# - "motion_detection"    → run tracking + show detected people
VISUALIZER_WINDOW_1_MODE = "none"
VISUALIZER_WINDOW_2_MODE = "none"
VISUALIZER_WINDOW_3_MODE = "motion_detection"

# logging
LOG_DEBUG_ENABLED   = False
LOG_INFO_ENABLED    = False
LOG_WARN_ENABLED    = True
LOG_ERROR_ENABLED   = True







###############################
#####   POST PROCESSING   #####
###############################

# used in crop filter
CROP_POINTCLOUD_POLYGON = [
    (0.069658042, -0.195796701), # Sensor 1
    (-4.188545, -5.2361961), # Schrank 6
    (2.691041753, -11.679826846), # Tür
    (7.267323630, -6.364732371) # Sensor 2
]
# TODO: add variable to enable polygon cropping


# Transformation (Translation and Rotation) Matrix. calculated in calculate_transformation_maxtrix.py based on 3 Points
# # 2025.05.26: works for `2025-05-20_dual-sensor-test_sensor-xxx.pcap.gz`
# # Determined 3 reference points using recorded data
# # TODO: outsource transformation matrix to CONFIG, or even separate .json file for automatic transfer
# TRANSFORMATION_MATRIX_SENSOR_2 = np.array([
#                 [ 0.62288801, -0.78223369,  0.01099957, -9.16811678],
#                 [ 0.77832635,  0.6210714,   0.09207824, -1.12348435],
#                 [-0.07885822, -0.04879318,  0.99569102, -0.61174572],
#                 [ 0.0,         0.0,         0.0,         1.0]
# ])

# 2025.05.28: works for `data/testdata/2025-05-28_dual-sensor-test_sensor-xxx.pcap.gz` and later
# Determined 3 reference points during live sensor setup
# TODO: outsource transformation matrix to CONFIG, or even separate .json file for automatic transfer
TRANSFORMATION_MATRIX_SENSOR_2 = np.array([
             [ 0.66810762, 0.7309481, -0.13909377, 6.27345587],
             [-0.74305003, 0.66519418,-0.07343942,-6.48224065],
             [ 0.03884396, 0.15241907, 0.98755232, 1.1114492 ],
             [ 0,          0,          0,          1        ],
])





###############################
##### detection algorithm #####
###############################
POINTCLOUD_HISTORY_BUFFER_SIZE = 9  # Rolling buffer of last N frames

COUNT_PEOPLE_ENABLED = True
COUNT_PEOPLE_DRAW_BOXES = True

# what should be drawn as second RED pointcloud: (is also used as input for detecting people)
MODE_SECOND_DATA_SET = "HIGHPASS+DENOISE"
#MODE_SECOND_DATA_SET = "HIGHPASS+CROP+DENOISE"
#MODE_SECOND_DATA_SET = "HIGHPASS"
#MODE_SECOND_DATA_SET = "OLDEST" # to test buffer size


PEOPOLE_TRACKING_INSIDE_ROOM_AREA_POLYGON = [
    # large polygon outsidepointcloud excluding door area (for detecing entered, exited)
    (0.425109267, -10.492938042), # left outside door
    #(1.897194386, -9.769321442), # inner edge room
    (3.835275173, -10.872682571), # right wall room
    (7.989156246, -6.492611885), # top right room edge
    (0.228421226, 0.920938611), # top room edge (sensor1)
    (-6.129019260, -6.429360390) # left room edge
]

# for determining the crop polygon its a good idea to log the current pointcloud edges
CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER = False #deprecated, use cli to start instead





################
##### MISC #####
################
def get_decoder_config():
    config = vd.Config()
    config.model = vd.Model.VLP32C # Sensor model (needed for decoding the data)
    return config
