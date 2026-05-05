#!/usr/bin/env python3
"""
ROS 2 TTS (Text-to-Speech) Node for Pi Dog
Generates voice output using Piper TTS or eSpeak
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy

# Standard ROS 2 imports instead of custom ones
from std_msgs.msg import String
from std_srvs.srv import Trigger

import time
import threading
import os
import subprocess
import random


class PiDogTTSNode(Node):
    def __init__(self):
        super().__init__('pidog_tts_speaks_node')
        
        # Parameters
        self.declare_parameter('tts_model', 'en_US-ryan-low')
        self.declare_parameter('voice_volume', 0.8)
        self.declare_parameter('speak_rate', 1.0)
        self.declare_parameter('use_piper', True)  # Use Piper TTS
        self.declare_parameter('use_espeak', True)  # Fallback to espeak
        
        self.tts_model = self.get_parameter('tts_model').value
        self.voice_volume = self.get_parameter('voice_volume').value
        self.speak_rate = self.get_parameter('speak_rate').value
        self.use_piper = self.get_parameter('use_piper').value
        self.use_espeak = self.get_parameter('use_espeak').value
        
        # Queue for speech
        self.speech_queue = []
        self.is_speaking = False
        
        # ROS 2 Service (using standard Trigger for simple speak)
        self.speak_srv = self.create_service(
            Trigger,
            'speak',
            self.speak_callback
        )
        
        # For the speak_text subscriber:
        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Topic for text to speak (with optional emotion flag)
        # Format: "text" or "text:emotion"
        self.speak_sub = self.create_subscription(
            String,
            'speak_text',
            #self.speak_callback,
            self.speak_text_callback,  # Change from 'speak_callback' to 'speak_text_callback'
            qos_profile_best_effort  # Use BEST_EFFORT instead of RELIABLE
        )
        
        # Initialize TTS engine
        self.init_tts_engine()
        
        # Start speech processing thread
        self.speech_thread = threading.Thread(target=self.process_speech_queue, daemon=True)
        self.speech_thread.start()
        
        self.get_logger().info(f"PiDog TTS Node Ready (piper={self.use_piper}, espeak={self.use_espeak})")
        self.get_logger().info("Use: ros2 service call /speak std_srvs/srv/Trigger '{data: \"Hello\"}'")
        self.get_logger().info("Or publish to /speak_text topic")
    
    def init_tts_engine(self):
        """Initialize TTS engine"""
        self.piper_process = None
        self.play_process = None
        
        if self.use_piper:
            try:
                # Check if piper is available
                result = subprocess.run(['which', 'piper'], capture_output=True, text=True)
                if result.returncode == 0:
                    self.get_logger().info("Piper TTS found")
                    
                    # Check if model exists
                    model_path = f"/usr/share/piper/models/{self.tts_model}.onnx"
                    if not os.path.exists(model_path):
                        self.get_logger().warning(f"Model not found: {model_path}")
                        self.get_logger().info("Using default model or falling back to espeak")
                        self.use_piper = False
                        self.use_espeak = True
                    else:
                        # Start piper in a subprocess
                        self.piper_process = subprocess.Popen(
                            ['piper', '--model', model_path, '--output-raw'],
                            stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE
                        )
                        
                        # Play audio using aplay
                        self.play_process = subprocess.Popen(
                            ['aplay', '-r', '22050', '-f', 'S16_LE', '-t', 'raw'],
                            stdin=self.piper_process.stdout,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL
                        )
                        self.get_logger().info("Piper TTS initialized successfully")
                else:
                    self.get_logger().warning("Piper not found, falling back to espeak")
                    self.use_piper = False
                    self.use_espeak = True
            except Exception as e:
                self.get_logger().error(f"Failed to initialize Piper: {e}")
                self.use_piper = False
                self.use_espeak = True
        
        if self.use_espeak:
            try:
                result = subprocess.run(['which', 'espeak'], capture_output=True, text=True)
                if result.returncode == 0:
                    self.get_logger().info("eSpeak found and ready")
                else:
                    self.get_logger().warning("eSpeak not found. Please install: sudo apt install espeak")
                    self.use_espeak = False
            except Exception as e:
                self.get_logger().error(f"eSpeak initialization error: {e}")
                self.use_espeak = False
        
        if not self.use_piper and not self.use_espeak:
            self.get_logger().warning("No TTS engine available. Speech will be logged only.")
    
    def speak_piper(self, text):
        """Speak text using Piper TTS"""
        if self.piper_process is None or self.piper_process.poll() is not None:
            return False
        
        try:
            # Send text to piper
            self.piper_process.stdin.write((text + '\n').encode('utf-8'))
            self.piper_process.stdin.flush()
            return True
        except Exception as e:
            self.get_logger().error(f"Piper speak error: {e}")
            return False
    
    def speak_espeak(self, text):
        """Speak text using eSpeak"""
        try:
            # Adjust volume and speed
            volume_amp = int(self.voice_volume * 200)  # espeak volume 0-200
            speed = int(self.speak_rate * 175)  # espeak speed 80-450
            
            # Run espeak (non-blocking to allow queue processing)
            subprocess.Popen([
                'espeak',
                '-a', str(volume_amp),
                '-s', str(speed),
                text
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception as e:
            self.get_logger().error(f"eSpeak error: {e}")
            return False
    
    #def speak_text_callback(self, msg):
    #    """Callback for simple text topic"""
    #    data = msg.data
    #    use_emotion = False
        
    #    # Check if emotion flag is included (format: "text:emotion")
    #    if ':' in data:
    #        parts = data.split(':', 1)
    #        text = parts[0]
    #        emotion_flag = parts[1].lower()
    #        use_emotion = emotion_flag in ['true', 'yes', '1', 'emotion']
    #    else:
    #        text = data
        
    #    self.add_to_speech_queue(text, use_emotion)
    
    def speak_text_callback(self, msg):
        """Handle text to speak from topic"""
        self.get_logger().info(f"Speak topic received: {msg.data}")
        # Add to queue or speak directly
        if hasattr(self, 'speech_queue'):
            self.speech_queue.append(msg.data)
        else:
            self.speak_text(msg.data)


    #def speak_callback(self, request, response):
    #    """Service callback for speech commands"""
    #    # The Trigger service doesn't carry text, so we need a different approach
    #    # For now, return error indicating to use topic
    #    response.success = False
    #    response.message = "Please use /speak_text topic with String message containing text to speak"
    #    self.get_logger().warning("Trigger service called. Use /speak_text topic instead.")
    #   return response
    
    #def speak_callback(self, request, response):  # Correct for ROS2 service
    #    self.get_logger().info(f"Speak service called: {request.data}")
    #    # Add speech to queue
    #    self.add_to_speech_queue(request.data)
    #    response.success = True
    #    response.message = "Speech added to queue"
    #    return response

    def speak_callback(self, request, response):
        """Service callback for speaking"""
        self.get_logger().info(f"Speak service called with: {request.data}")
    
        # Add to queue
        if hasattr(self, 'speech_queue'):
            self.speech_queue.append(request.data)
            response.success = True
            response.message = "Speech added to queue"
        else:
            # Direct speak
            self.speak_text(request.data)
            response.success = True
            response.message = "Speech completed"
    
        return response

    def add_to_speech_queue(self, text, use_emotion=False):
        """Add text to speech queue"""
        if not text or len(text.strip()) == 0:
            return
        
        # Add emotional fluff if requested
        if use_emotion:
            emotions = {
                "happy": ["Yay!", "Woof woof!", "I'm so happy!", "Great!"],
                "curious": ["Hmm?", "What's that?", "Interesting!", "Oh?"],
                "startled": ["Oh!", "Wow!", "That surprised me!", "Whoa!"],
                "sad": ["Aww...", "Oh dear...", "That's too bad..."]
            }
            
            # Simple emotion detection based on keywords
            text_lower = text.lower()
            if any(word in text_lower for word in ["happy", "love", "great", "good"]):
                prefix = random.choice(emotions['happy'])
                text = f"{prefix} {text}"
            elif any(word in text_lower for word in ["what", "where", "who", "why", "how", "hmm", "curious"]):
                prefix = random.choice(emotions['curious'])
                text = f"{prefix} {text}"
            elif any(word in text_lower for word in ["scared", "startled", "surprise", "oh", "wow"]):
                prefix = random.choice(emotions['startled'])
                text = f"{prefix} {text}"
            elif any(word in text_lower for word in ["sad", "sorry", "unfortunate"]):
                prefix = random.choice(emotions['sad'])
                text = f"{prefix} {text}"
        
        self.speech_queue.append(text)
        self.get_logger().info(f"Added to speech queue: {text[:50]}...")
    
    def process_speech_queue(self):
        """Process speech queue sequentially"""
        while rclpy.ok():
            if self.speech_queue and not self.is_speaking:
                text = self.speech_queue.pop(0)
                self.is_speaking = True
                
                try:
                    self.get_logger().info(f"🔊 Speaking: {text}")
                    
                    if self.use_piper:
                        success = self.speak_piper(text)
                    elif self.use_espeak:
                        success = self.speak_espeak(text)
                    else:
                        # Fallback - just log text
                        self.get_logger().info(f"[SPEECH OUTPUT] {text}")
                        success = True
                    
                    if not success:
                        self.get_logger().warning(f"Failed to speak: {text}")
                    
                    # Brief pause between speech
                    time.sleep(0.3)
                    
                except Exception as e:
                    self.get_logger().error(f"Speech error: {e}")
                finally:
                    self.is_speaking = False
            
            time.sleep(0.1)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down TTS node...")
        
        # Clear queue
        self.speech_queue.clear()
        
        # Kill subprocesses
        if self.piper_process:
            try:
                self.piper_process.stdin.close()
                self.piper_process.terminate()
                self.piper_process.wait(timeout=2)
            except Exception as e:
                self.get_logger().debug(f"Error terminating piper: {e}")
                try:
                    self.piper_process.kill()
                except:
                    pass
        
        if self.play_process:
            try:
                self.play_process.terminate()
            except:
                pass
        
        self.get_logger().info("TTS node shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = PiDogTTSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TTS node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():  # Check if already shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
