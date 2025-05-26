import velodyne_decoder as vd
# collection of globally used configuration options across all files



#################
##### INPUT #####
#################

#=== UDP Stream ===
UDP_IP = "0.0.0.0"
UDP_PORT = 5001

#=== Simulate / From PCAP File ===
USE_PCAP_FILE_INSTEAD_OF_UDP_STREAM = True

#PCAP_FILE = "../../data/testdata/2025-04-29_1person-walking_sensor-level.pcap"
#PCAP_FILE = "../../data/testdata/2025-05-06_4ppl-walking_sensor-tilted_VLP-32C.pcap"
#PCAP_FILE = "../../data/testdata/2025-05-20_dual-sensor-test_sensor-tripod.pcap.gz"
PCAP_FILE_1 = "../../data/testdata/2025-05-20_dual-sensor-test_sensor-lamp.pcap.gz"
PCAP_FILE_2 = "../../data/testdata/2025-05-20_dual-sensor-test_sensor-tripod.pcap.gz"

#Use packet timestamps to playback at original speed if possible
PCAP_FILE_REALTIME_PLAYBACK = True

# Custom fixed delay between reading frames from file (useful when not playing back in realtime + goal is to speed up / slow down replay)
#PCAP_FILE_PACKET_DELAY = 0.0001
PCAP_FILE_PACKET_DELAY = 0 
# Note: when too large delay the decoder outputs full frame too early

# Ignore packets that dont have this target port - set to None to disable this filter
# useful when using unfiltered wireshark dumps, prevents warning spam and decoder confusion
PCAP_FILE_FILTER_UDP_PORT = None




###############################
##### DETECTION ALGORITHM #####
###############################
POINTCLOUD_HISTORY_BUFFER_SIZE = 10  # Rolling buffer of last N frames

COUNT_PEOPLE_ENABLED = True
COUNT_PEOPLE_DRAW_BOXES = True

# what should be drawn as second RED pointcloud: (is also used as input for detecting people)
MODE_SECOND_DATA_SET = "HIGHPASS+DENOISE"
#MODE_SECOND_DATA_SET = "HIGHPASS+CROP+DENOISE"
#MODE_SECOND_DATA_SET = "HIGHPASS"
#MODE_SECOND_DATA_SET = "OLDEST" # to test buffer size

# used in crop filter
CROP_POINTCLOUD_POLYGON = [
    (0.5, 0),  # bottom left
    (-5.2, -4.7),   # bottom right
    (3.3, -12),  # top right
    (7.6, -6.6)    # top left
]
# for determining the crop polygon its a good idea to log the current pointcloud edges
CROP_POINTCLOUD_STOP_SCRIPT_OPEN_POINT_PICKER = False


################
##### MISC #####
################
def get_decoder_config():
    config = vd.Config()
    config.model = vd.Model.VLP32C # Sensor model (needed for decoding the data)
    return config
