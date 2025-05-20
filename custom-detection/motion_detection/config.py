import velodyne_decoder as vd
# collection of globally used configuration options across all files



#################
##### INPUT #####
#################
UDP_IP = "0.0.0.0"
UDP_PORT = 5001

USE_PCAP = True
#PCAP_FILE = "../../data/testdata/2025.05.16_wireshark-dump_sensor-on-desk.pcap"
PCAP_FILE = "../../data/testdata/2025-05-06_4ppl-walking_sensor-tilted_VLP-32C.pcap"
FRAME_DELAY = 0.002  # Delay between simulated UDP packets (speed up / slow down replay)



###############################
##### DETECTION ALGORITHM #####
###############################
POINTCLOUD_HISTORY_BUFFER_SIZE = 10  # Rolling buffer of last N frames

COUNT_PEOPLE_ENABLED = True
COUNT_PEOPLE_DRAW_BOXES = True

# what should be drawn as second RED pointcloud: (is also used as input for detecting people)
MODE_SECOND_DATA_SET = "HIGHPASS+DENOISE"
#MODE_SECOND_DATA_SET = "HIGHPASS"
#MODE_SECOND_DATA_SET = "OLDEST" # to test buffer size



################
##### MISC #####
################
def get_decoder_config():
    config = vd.Config()
    config.model = vd.Model.VLP32C # Sensor model (needed for decoding the data)
    return config
