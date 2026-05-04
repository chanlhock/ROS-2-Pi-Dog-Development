#!/usr/bin/env python3
##########################################################################
# Python program for autonomous Sunfounder Pi Dog with Raspberry Pi 5...
#
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 14/04/2026     Bernard Chan    Initial release
#
# autonomous_dog.py is licensed under the GNU General Public License v3.0
# Permissions of this strong copyleft license are conditioned on making
# available complete source code of licensed works and modifications,
# which include larger works using a licensed work, under the same
# license. Copyright and license notices must be preserved. Contributors
# provide an express grant of patent rights.
##########################################################################

from pidog import Pidog
import os
import pidog
from vilib import Vilib
from pidog.tts import Piper
from pidog.stt import Vosk
from collections import deque
import time
import random
import cv2
import numpy as np
import pygame
import queue
import threading
from pidog.preset_actions import *
#import preset_actions
import pyaudio
import json
from vosk import Model, KaldiRecognizer
from scipy.signal import butter, lfilter
import vosk
import logging

###### Constants of autonomous pi dog ########
SOUNDS_PATH = "/home/chanlhock/pidog/sounds/"
OBSTACLE_DISTANCE_CM = 30    # Stop and avoid if object is closer than this
FORWARD_SPEED = 98           # Walk speed, slow him down when debugging!!
TURN_SPEED = 98
BACKWARD_TIME = 1.0          # Time in seconds to move backward
TURN_TIME = 0.6              # Time in seconds to turn
FORWARD_INTERVAL = 0.2       # How often to check while moving forward
SHUTDOWN = False

###### image capture autonomous pi dog ########
SPIRAL = 1
BACKANDFORTH = 2
STUCK = 3
state = SPIRAL
StartTurn = 80
foundObstacle = 40
StuckDist = 10
lastPhoto = ""
currentPhoto = ""
MSE_THRESHOLD = 20

# Configure with more options
logging.basicConfig(
    filename='app.log',
    filemode='w',  # 'a' for append (default), 'w' for overwrite
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

#########################################################################
# Initialize pygame mixer for playing mp3 sounds
#########################################################################
pygame.mixer.init()
    
def load_sound(filename):
    return pygame.mixer.Sound(SOUNDS_PATH + filename)
    

########################################################################## Sounds dictionary for emotions 
##########################################################################
SOUNDS = {
    "happy": load_sound("single_bark_1.mp3"),
    "curious": load_sound("woohoo.mp3"),
    "startled": load_sound("growl_1.mp3"),
    "bored": load_sound("snoring.mp3"),
    "lonely": load_sound("howling.mp3"),
}

def play_sound(sound):
    if sound:
        sound.play()

#########################################################################

#########################################################################
def butter_lowpass_filter(data, cutoff, fs, order=5):
    """Filter high-frequency noise (e.g., servo whine) above cutoff"""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return lfilter(b, a, data)

# Disable Vosk logging
vosk.SetLogLevel(-1)  # Negative value disables most logging

# Or set to 0 for warnings only, 1 for info, etc.
# vosk.SetLogLevel(0)

def voice_recognition_worker():
    # Initialize Vosk
    model = Model("/home/chanlhock/pidog/woofer/vosk-model-small-en-us-0.15")
    recognizer = KaldiRecognizer(model, 16000)  # 16kHz sample rate
    
    # Audio capture parameters
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK = 4000
    CUTOFF_FREQ = 4000   # Hz - preserves speech clarity
        # Filter Parameters: The 4000 Hz cutoff preserves most
        # speech frequencies while removing high-frequency noise.
        # You can adjust this based on your noise profile:
        # 3000-3500 Hz: More aggressive noise reduction
        # 4000-4500 Hz: Better speech preservation
        # Order 5: Provides good rolloff without excessive phase distortion
    
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

    print("Listening... (Press Ctrl+C to stop)")
    logging.info("Voice recognition worker started, listening for commands...")

    # Processing loop

    print("Voice recognition started")
    logging.info("Voice recognition started")
    print("Valid commands: sit down, stand up, walk/walking, stretch, ")
    print("push up, hand shake, scratch, high five, stop, resume")

    try:
        while True:
            # Read from microphone
            raw_data = stream.read(CHUNK, exception_on_overflow=False)
        
            # Convert to numpy array for filtering
            audio_array = np.frombuffer(raw_data, dtype=np.int16).astype(np.float32)
        
            # Apply low-pass filter
            filtered_audio = butter_lowpass_filter(audio_array, CUTOFF_FREQ, RATE)
        
            # Convert back to int16 bytes
            filtered_data = filtered_audio.astype(np.int16).tobytes()
        
            # Send to Vosk
            if recognizer.AcceptWaveform(filtered_data):
                result = json.loads(recognizer.Result())
                if result["text"]:
                    print(f"Recognized: {result['text']}")
                    logging.info("Recognized voice command: %s", result["text"])
                    # Add your PiDog action code here
                    print("Voice input recognized:", result["text"])
                    logging.info("Voice input recognized: %s", result["text"])  
                    voice_command_queue.put(result["text"])
    except KeyboardInterrupt:
        print("\nStopping audio stream...")
        logging.info("Stopping audio stream...")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

#########################################################################
# image capture autonomous pi dog 
#########################################################################
def compareImages():
    if lastPhoto == "":
        return 0
    img1 = cv2.imread(lastPhoto)
    img2 = cv2.imread(currentPhoto)
    if img1 is None or img2 is None:
        return(MSE_THRESHOLD + 1) 
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    h, w = img1.shape
    try:
        diff = cv2.subtract(img1, img2)
    except:
        return(0)
    err = np.sum(diff**2)
    mse = err/(float(h*w))
    print("comp mse = ", mse)
    logging.info("Image comparison MSE: %f", mse)
    return mse    

def take_photo():
    global lastPhoto, currentPhoto
    _time = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
    name = 'photo_%s'%_time
    username = os.getlogin()
    path = f"/home/{username}/Pictures/"
    print('Taking photo: %s', name)
    logging.info('Taking photo: %s', name)
    print('Photo path: %s', path)
    logging.info('Photo path: %s', path)

    status = Vilib.take_photo(name, path)
    if status:
        print('photo save as %s%s.jpg'%(path,name))
        logging.info('Photo saved: %s%s.jpg', path, name)
    else:
        print("Photo save failed")
        logging.error("Failed to save photo: %s%s.jpg", path, name)
    if lastPhoto != "":
        try:
            os.remove(lastPhoto)
            time.sleep(1)
        except Exception as e:
            print("Photo not remove...", e)
            logging.error("Error occurred while removing photo: %s", e)
    lastPhoto = currentPhoto
    currentPhoto = path + name + ".jpg"

def executeSpiral(dog):
    global state
    print("Dog turn right...")
    dog.do_action('turn_right', step_count=5, speed=98)      
    dog.wait_all_done()
    distance = round(dog.read_distance(), 2)
    print("spiral distance: ",distance)
    if distance <= foundObstacle and distance != -1:
        state = BACKANDFORTH

def executeUnskick(dog):
    global state    
    print("unskick backing up")
    dog.speak("single_bark_1")
    dog.do_action('backward', step_count=5, speed=98)    
    dog.wait_all_done()
    time.sleep(1.2)
    state = SPIRAL                    

def executeBackandForth(dog):
    global state    
    distance = round(dog.read_distance(), 2)
    print("back and forth distance: ",distance)
    if distance >= StartTurn or distance == -1:       
        dog.do_action('trot', step_count=2, speed=98) # 5
        dog.wait_all_done()
    elif distance < StuckDist:
        state = STUCK
    else:
        dog.do_action('turn_right', step_count=5, speed=98)
        dog.wait_all_done()
    time.sleep(0.5)                

#########################################################################
# Just a mini function to min typing for voice command handler
#########################################################################
def indiv_commands(operation):
    OKToWalk.clear()
    dog.legs_stop()
    time.sleep(1)
    print("Command: ", operation)
    logging.info("Command: %s", operation)
    if operation == "hand_shake":
        hand_shake(dog)
    elif operation == "scratch":
        scratch(dog)
        head_angs = [ [0, 0, 0], [0, 0, 0] ]
        dog.head_move_raw(head_angs, immediately=False, speed=80)
        time.sleep(2)
    elif operation == "high_five":
        high_five(dog)
    else:
        dog.do_action(operation)
    dog.wait_legs_done()
    time.sleep(5)
    OKToWalk.set()

#########################################################################

def ShowSomePersonality(RanAction):
  if   RanAction==1 : pidog.preset_actions.scratch(dog)
  #elif RanAction==2 : preset_actions.hand_shake(dog)
  elif RanAction==3 : pidog.preset_actions.high_five(dog)
  elif RanAction==4 : pidog.preset_actions.pant(dog)
  elif RanAction==5 : pidog.preset_actions.body_twisting(dog)
  elif RanAction==6 : pidog.preset_actions.bark_action(dog)
  elif RanAction==7 : pidog.preset_actions.shake_head(dog)
  elif RanAction==8 : pidog.preset_actions.shake_head_smooth(dog)
  #elif RanAction==1 : bark(dog)
  #elif RanAction==1 : push_up(dog)
  elif RanAction==9 : pidog.preset_actions.howling(dog)
  elif RanAction==10: pidog.preset_actions.attack_posture(dog)
  elif RanAction==11: pidog.preset_actions.lick_hand(dog)
  elif RanAction==12: pidog.preset_actions.waiting(dog,0)#no def pitch for some reason
  elif RanAction==13: pidog.preset_actions.feet_shake(dog)
  elif RanAction==14: pidog.preset_actions.sit_2_stand(dog)
  elif RanAction==15: pidog.preset_actions.relax_neck(dog)
  elif RanAction==16: pidog.preset_actions.nod(dog)
  elif RanAction==17: pidog.preset_actions.think(dog)
  elif RanAction==18: pidog.preset_actions.recall(dog)
  elif RanAction==19: pidog.preset_actions.head_down_left(dog)
  elif RanAction==20: pidog.preset_actions.head_down_right(dog)
  elif RanAction==21: pidog.preset_actions.fluster(dog)
  elif RanAction==22: pidog.preset_actions.alert(dog)
  elif RanAction==23: pidog.preset_actions.surprise(dog)
  elif RanAction==24: pidog.preset_actions.stretch(dog)
  #Several likelihoods for turning, as it helps reduce the long straight walk
  #until it sees a wall
  elif RanAction==27: dog.do_action("turn_left", speed=TURN_SPEED)
  elif RanAction==28: dog.do_action("turn_left", speed=TURN_SPEED)
  elif RanAction==29: dog.do_action("turn_right", speed=TURN_SPEED)
  elif RanAction==30: dog.do_action("turn_right", speed=TURN_SPEED)

    
  print("Action is",RanAction)
    
  #put head back after any actions such that ultrasonic is pointing straight ahead
  head_angs = [ [0, 0, 0], [0, 0, 0] ]
  dog.head_move_raw(head_angs, immediately=False, speed=80)
  dog.wait_all_done()

#########################################################################
# Servo noise makes it difficult for pidog to hear clearly
#########################################################################
def voice_command_handler():
    while True:
        command = voice_command_queue.get()
        cmd_lower = command.lower()
        if "sit" in cmd_lower or "six" in cmd_lower or "sick" in cmd_lower or "fit" in cmd_lower or "sit down" in cmd_lower:
            indiv_commands('sit')
        elif "stand" in cmd_lower or "stan" in cmd_lower or "stand up" in cmd_lower:
            indiv_commands('stand')
        elif "walk" in cmd_lower or "walking" in cmd_lower:
            indiv_commands('forward')
        elif "stretch" in cmd_lower:
            indiv_commands('stretch')
        elif "push up" in cmd_lower or "push-up" in cmd_lower:
            indiv_commands('push_up')
        elif "hand" in cmd_lower or "shake" in cmd_lower or "handshake" in cmd_lower:
            indiv_commands('hand_shake')
        elif "scratch" in cmd_lower:
            indiv_commands('scratch')
        elif "high" in cmd_lower or "five" in cmd_lower or "high five" in cmd_lower:
            indiv_commands('high_five')
        elif "stop" in cmd_lower:
            OKToWalk.clear()
            dog.legs_stop()
            time.sleep(1)
            print("Command: Emergency stop wandering")
            logging.info("Command: Emergency stop wandering")
        elif "resume" in cmd_lower:
            OKToWalk.set()
            time.sleep(1)
            print("Command: Resuming wandering")
            logging.info("Command: Resuming wandering")
        elif "shutdown" in cmd_lower or "shut down" in cmd_lower:
            OKToWalk.clear()
            dog.legs_stop()
            time.sleep(1)
            print("Command: Shutting down Pi Dog...")
            logging.info("Command: Shutting down Pi Dog...")
            dog.do_action('sit', speed=50)
            dog.wait_all_done()
            time.sleep(.5)  
            SHUTDOWN = True
        else:
            print("Unknown command:", command)
            logging.info("Unknown command:", command)
            print("Valid commands: sit down, stand up, walk/walking, stretch, ")
            print("push up, hand shake, scratch, high five, stop, resume")


#########################################################################
# The default low heirarchy behaviour in my subsumption model
#########################################################################
class PiDogWanderWrapper:
    def __init__(self):
        self.dog = dog
        self.running = True
        self.turn_history = deque(maxlen=5)
        self.emotions = ["happy", "curious", "startled", "bored", "lonely"]
        self.last_emotion_time = time.time()
        self.emotion_interval = 30  # seconds between emotion triggers
        
    def get_distance(self):
        dist = self.dog.read_distance()
        if dist is None or dist < 0 or dist > 150:
            return None
        return dist
    
    def scan_direction(self, head_yaw):
        self.dog.head_move([(head_yaw, 0, 0)], immediately=True, speed=50)
        time.sleep(0.8)
        distance = self.get_distance()
        if distance is None:
            distance = 999.0
        print(f"Scanned {head_yaw:+}Â°: {distance:.1f} cm")
        logging.info(f"Scanned {head_yaw:+}Â°: {distance:.1f} cm")
        return distance
    
    def turn_smart(self):
        left_distance = self.scan_direction(-30)
        right_distance = self.scan_direction(30)
    
        left_count = self.turn_history.count("left")
        right_count = self.turn_history.count("right")
    
        print(f"[Smart Turn] Left: {left_distance:.1f} cm, Right: {right_distance:.1f} cm")
        logging.info(f"[Smart Turn] Left: {left_distance:.1f} cm, Right: {right_distance:.1f} cm")
        print(f"[Turn History] Left: {left_count}, Right: {right_count}")
        logging.info(f"[Turn History] Left: {left_count}, Right: {right_count}")
    
        preferred = "left" if left_distance > right_distance else "right"
    
        if self.turn_history.count(preferred) >= 3:
            preferred = "left" if preferred == "right" else "right"
            print("[Smart Turn] Switching direction to avoid repeating turns.")
            logging.info("[Smart Turn] Switching direction to avoid repeating turns.")
    
        if preferred == "left":
            print("Smart turning left")
            logging.info("Smart turning left")
            self.dog.do_action("turn_left", speed=TURN_SPEED)
            self.turn_history.append("left")
        else:
            print("Smart turning right")
            logging.info("Smart turning right")
            self.dog.do_action("turn_right", speed=TURN_SPEED)
            self.turn_history.append("right")
    
        dog.wait_legs_done()
        time.sleep(TURN_TIME)
        self.stop()
    
    def stand_up(self):
        self.dog.do_action("stand", speed=70)
        self.dog.wait_all_done()
    
    def move_forward(self):
        self.dog.do_action("forward", speed=FORWARD_SPEED)
    
    def stop(self):
        self.dog.body_stop()
        self.dog.wait_all_done()
                 
    def backup(self, duration=BACKWARD_TIME):
        self.dog.do_action("backward", speed=FORWARD_SPEED)
        time.sleep(duration)
        self.dog.wait_legs_done()
    
    def play_emotion(self, emotion):
        sound = SOUNDS.get(emotion)
        if sound:
            print(f"Emotion triggered: {emotion}")
            logging.info(f"Emotion triggered: {emotion}")
            play_sound(sound)
            # Simple servo reactions for emotion (optional)
            if emotion == "happy":
                self.dog.head_move([(0, 0, 10)], immediately=True, speed=100)
            elif emotion == "curious":
                self.dog.head_move([(-20, 0, 10)], immediately=True, speed=80)
                time.sleep(0.5)
                self.dog.head_move([(20, 0, 10)], immediately=True, speed=80)
                time.sleep(0.5)
                self.dog.head_move([(0, 0, 10)], immediately=True, speed=80)
            elif emotion == "startled":
                self.dog.do_action("shake_head")
            elif emotion == "bored":
                # maybe a slow nod ?
                pass
            elif emotion == "lonely":
                # maybe a slow wag tail or look around?
                pass
    
    def wander(self):
        print("Starting autonomous wandering mode...")
        logging.info("Starting autonomous wandering mode...")
        self.stand_up()
        while self.running:
            #Suspend walk behaviour while other higher priorities such as voice are executing
            OKToWalk.wait()
            distance = self.get_distance()
            if distance is not None and distance < OBSTACLE_DISTANCE_CM:
                print("Obstacle detected! Avoiding...")
                logging.info("Obstacle detected! Avoiding...")
                if DEBUG == False:
                    self.stop()
                    self.backup()
                    self.turn_smart()
                self.play_emotion("startled")
            else:
                if DEBUG == False:
                    self.move_forward()
                    self.dog.wait_legs_done()
            ShowSomePersonality(random.randrange(1,100))
    
            # Occasionally trigger random emotions during idle wandering
            now = time.time()
            if now - self.last_emotion_time > self.emotion_interval:
                emotion = random.choice(self.emotions)
                self.play_emotion(emotion)
                self.last_emotion_time = now
    
            time.sleep(FORWARD_INTERVAL)

    def shutdown(self):
        print("Shutting down Pi Dog...")
        logging.info("Shutting down Pi Dog...")
        self.stop()
        self.dog.do_action("sit", speed=70)
        head_angs = [ [0, 0, 0], [0, 0, 0] ]
        dog.head_move_raw(head_angs, immediately=False, speed=80)
        time.sleep(1)
        self.dog.close()

    def wander_image(self):
        global state
        state_desc = ['SPIRAL', 'BACKANDFORTH', 'STUCK']
        while True:
            OKToWalk.wait()
            print("Starting autonomous image capture wandering mode...")
            print("Take photo...")
            print(state_desc[state-1])
            take_photo()
            if state == SPIRAL:               
                executeSpiral(dog)
            elif state == BACKANDFORTH:
                executeBackandForth(dog)
            elif state == STUCK:
                executeUnskick(dog)
            if compareImages() < MSE_THRESHOLD:
                 state = STUCK
        
###############################################

def safe_shutdown(exit_code=0): #(dog, exit_code=0):
    #print("Stopping PiDog motion...")
    #try:
    #    dog.body_stop()       # stop all servos
    #    dog.wait_legs_done()  # wait for actions to finish
    # except Exception:
    #    pass

    print("Exiting Python process... Until next time,bye!")
    logging.info("Exiting Python process... Until next time,bye!")
    os._exit(exit_code)

######## Global initialization ##########
dog = Pidog()
OKToWalk = threading.Event()
OKToWalk.set()
voice_command_queue = queue.Queue()
# Choose the language that Woofer speaks (English = True, Chinese = False)
ENGLISH = True
# Set Debug ON to bypass certain code block
DEBUG = True
# Pi Dog's name
NAME = "Woofer" # Name of the dog during my childhood
GREETING_EN = f"Hi, I am {NAME}. Your obedient Artificial Intelligence Pi Dog!"
GREETING_CN = f"嗨,您好，我名叫 {NAME}. 你忠心的人工智能机器狗儿!"


def main():
    SHUTDOWN = False
    time.sleep(2)
    print("Starting Woofer autonomous Pi Dog...")
    logging.info('Starting Woofer autonomous Pi Dog...')
    wanderer = PiDogWanderWrapper()
    
    dog.head_move([(0, 0, 10)], immediately=True, speed=50)
    
    #dog = Pidog()
    dog.do_action('sit', speed=50)
    dog.wait_all_done()
    time.sleep(.5)        
    dog.rgb_strip.set_mode(style="boom", color="#a10a0a", bps=2.5, brightness=0.5)

    tts = Piper()
    if ENGLISH == True:
        #tts.set_model("en_US-amy-low")
        tts.set_model("en_US-ryan-low")
        print(GREETING_EN)
        logging.info(GREETING_EN)
        tts.say(GREETING_EN)
    else:
        tts.set_model("zh_CN-huayan-x_low")
        print(GREETING_CN)
        logging.info(GREETING_CN)
        tts.say(GREETING_CN)
        
    time.sleep(1)
    #dog.do_action('stand', speed=80)
    #dog.wait_all_done()
    #time.sleep(.5)        
    dog.rgb_strip.set_mode('breath', 'white', bps=0.5)

    # You can use the following two functions to load the model and the corresponding label
    Vilib.object_detect_set_model(path='/opt/vilib/detect.tflite')
    Vilib.object_detect_set_labels(path='/opt/vilib/coco_labels.txt')

    # Start camera streaming
    try:
        # Vilib.camera_start(vflip=False,hflip=False) # size=(640, 480)
        Vilib.camera_start(vflip=False, hflip=False, size=(1280, 720))
        Vilib.show_fps()
        Vilib.display(local=False,web=True)
        time.sleep(1)  # give camera time to warm up
        Vilib.face_detect_switch(True)
        #Vilib.hands_detect_switch(True)
        #Vilib.color_detect(color="red")  # red, green, blue, yellow , orange, purple
        #Vilib.object_detect_switch(True)
        print("Camera started...")
        logging.info("Camera started...")

    except Exception as e:
        print("Camera start error:", e)
        logging.error("Camera start error:", e)
    
    # Start threads
    threading.Thread(target=voice_recognition_worker, daemon=True).start()
    threading.Thread(target=voice_command_handler, daemon=True).start()
    threading.Thread(target=wanderer.wander, daemon=True).start()
    
    print("PiDog is ready and listening...")
    logging.info("PiDog is ready and listening...")

    # Keep main thread alive
    try:
        while True:
            # Check how many faces are detected
            face_count = Vilib.detect_obj_parameter['human_n']
            if face_count > 0:
                # Retrieve coordinates of the first detected face
                x = Vilib.detect_obj_parameter['human_x']
                y = Vilib.detect_obj_parameter['human_y']
                print(f"Detected {face_count} face(s) at X:{x}, Y:{y}")
                
            time.sleep(0.5)
           # print(Vilib.detect_obj_parameter['hands_joints']) # Print finger joint coordinates
            time.sleep(0.5)
           # print(Vilib.object_detection_list_parameter)
            #print() # new line
            #bat_level = dog.get_battery_voltage() # get Pi Dog battery voltage
            # print("Battery voltage level: ", bat_level)
            if SHUTDOWN:
                break
        
    except KeyboardInterrupt:
        print("")
        Vilib.camera_close()
        print("Closing Vilib camera")
        logging.info("Closing Vilib camera")
        wanderer.shutdown()
        print("Closing pidog")
        logging.info("Closing pidog")
        safe_shutdown(0) #my_dog, 0)
    
if __name__ == "__main__":
    main()
    
        
        
