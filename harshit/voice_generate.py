#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import pyttsx3
import threading

class DroneSpeaker:
    def __init__(self):
        rospy.init_node('smart_mission_speaker', anonymous=True)

        # Subscribers
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_cb)
        rospy.Subscriber("/qr/status", String, self.qr_cb)

        # Text-to-speech setup
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 170)
        self.engine.setProperty('volume', 1.0)

        # State trackers
        self.last_mode = ""
        self.armed = False
        self.altitude = 0.0
        self.qr_found = False
        self.last_alert = ""

        self.speak("Drone voice assistant online and ready.")

    def speak(self, text):
        """Speaks text asynchronously and logs it."""
        rospy.loginfo(f"[Voice] {text}")
        thread = threading.Thread(target=self._speak_thread, args=(text,))
        thread.start()

    def _speak_thread(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def state_cb(self, msg):
        # Arming / disarming
        if msg.armed and not self.armed:
            self.speak("Drone armed successfully.")
        elif not msg.armed and self.armed:
            self.speak("Drone disarmed.")
        self.armed = msg.armed

        # Mode change
        if msg.mode != self.last_mode:
            self.speak(f"Mode changed to {msg.mode}.")
            self.last_mode = msg.mode

    def pose_cb(self, msg):
        z = msg.pose.position.z
        if z > 9.5 and self.last_alert != "cruise":
            self.speak("Reached cruising altitude.")
            self.last_alert = "cruise"

        if self.last_alert == "landing" and z < 0.3:
            self.speak("Drone has landed successfully.")
            self.last_alert = "landed"

    def battery_cb(self, msg):
        percent = msg.percentage * 100
        if percent < 20 and self.last_alert != "low_battery":
            self.speak("Warning, battery level below twenty percent.")
            self.last_alert = "low_battery"

    def qr_cb(self, msg):
        if msg.data.lower() == "detected" and not self.qr_found:
            self.qr_found = True
            self.last_alert = "landing"
            self.speak("Target QR detected. Aligning for descent.")
    
    def servo(self,msg):
        if 

if __name__ == "__main__":
    try:
        DroneSpeaker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



#process to run it:-


# ~/catkin_ws/src/smart_mission_speaker/scripts/smart_mission_speaker.py


#installing dependencies 
'''sudo apt-get install espeak
pip3 install pyttsx3
'''

#creating package
'''cd ~/catkin_ws/src
catkin_create_pkg smart_mission_speaker rospy std_msgs geometry_msgs sensor_msgs mavros_msgs
mkdir -p smart_mission_speaker/scripts
# Paste the code above in the script folder
chmod +x smart_mission_speaker/scripts/smart_mission_speaker.py
'''

# build & source 

'''cd ~/catkin_ws
catkin_make
source devel/setup.bash
'''

# Run MAVROS
'''roslaunch mavros apm.launch fcu_url:=udp://:14540@'''
# Run the voice assistant
'''rosrun smart_mission_speaker smart_mission_speaker.py'''
