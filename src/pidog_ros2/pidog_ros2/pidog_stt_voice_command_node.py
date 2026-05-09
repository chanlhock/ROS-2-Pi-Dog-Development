#!/usr/bin/env python3
"""
ROS 2 STT (Speech-to-Text) Voice Command Node for Pi Dog
Publishes recognized commands to voice_command topic
Now with TTS speaking state awareness - ignores self-speech
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool  # ADDED: Import Bool for speaking state

import json
import time
import threading
import os


class PiDogSTTVoiceCommandNode(Node):
    def __init__(self):
        super().__init__('pidog_stt_voice_command_node')
        
        self.get_logger().info("STT node starting...")
        
        # Parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('model_path', '/pidog/woofer/vosk-model-small-en-us-0.15')
        self.declare_parameter('debug', False)
        self.declare_parameter('post_speech_delay', 0.5)  # ADDED: Delay after speech ends
        
        self.enabled = self.get_parameter('enabled').value
        self.model_path = self.get_parameter('model_path').value
        self.debug = self.get_parameter('debug').value
        self.post_speech_delay = self.get_parameter('post_speech_delay').value  # ADDED
        
        # ADDED: Track if TTS is currently speaking
        self.tts_is_speaking = False
        self.last_speech_end_time = 0
        
        # Publisher for recognized voice commands
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.voice_pub = self.create_publisher(String, 'voice_command', qos_profile)
        
        # ADDED: Subscribe to TTS speaking state
        self.tts_state_sub = self.create_subscription(
            Bool,
            'tts_speaking_state',
            self.tts_state_callback,
            10
        )
        
        # Initialize Vosk
        self.recognizer = None
        self.stream = None
        self.pyaudio = None
        self.listening = False
        
        if self.enabled:
            self.init_vosk()
        
        if self.recognizer:
            self.get_logger().info("Voice commands will be published to /pidog/voice_command")
            self.get_logger().info(f"Will ignore speech while TTS is speaking + {self.post_speech_delay}s delay")
            self.get_logger().info("Try saying: sit, stand, walk, stop, turn left, turn right")
        else:
            self.get_logger().warning("STT not available - voice commands disabled")
    
    # ADDED: Callback for TTS speaking state
    def tts_state_callback(self, msg):
        """Handle TTS speaking state changes."""
        was_speaking = self.tts_is_speaking
        self.tts_is_speaking = msg.data
        
        if was_speaking and not self.tts_is_speaking:
            # Speech just ended, record the time
            self.last_speech_end_time = time.time()
            if self.debug:
                self.get_logger().debug("TTS finished speaking, starting cooldown period")
        elif not was_speaking and self.tts_is_speaking:
            # Speech started
            if self.debug:
                self.get_logger().debug("TTS started speaking, ignoring voice input")
    
    # ADDED: Check if we should process audio based on TTS state
    def should_process_audio(self):
        """Determine if we should process audio (not speaking, or in cooldown)."""
        if self.tts_is_speaking:
            return False
        
        # Check cooldown period after speech ends
        time_since_speech_ended = time.time() - self.last_speech_end_time
        if time_since_speech_ended < self.post_speech_delay:
            return False
        
        return True
    
    def init_vosk(self):
        """Initialize Vosk speech recognition."""
        try:
            
            import vosk
            import pyaudio

            # Check if model exists
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"Vosk model not found at {self.model_path}")
                self.get_logger().info("Please download from: https://alphacephei.com/vosk/models")
                return
            
            # Suppress Vosk logging
            vosk.SetLogLevel(-1) # Negative value disables most logging
            
            # Load model
            self.get_logger().info(f"Loading Vosk model from {self.model_path}...")
            model = vosk.Model(self.model_path)
            self.recognizer = vosk.KaldiRecognizer(model, 16000)
            self.recognizer.SetWords(False)
            
            # Initialize PyAudio
            self.pyaudio = pyaudio.PyAudio()
            
            # Find input device
            device_index = None
            for i in range(self.pyaudio.get_device_count()):
                dev_info = self.pyaudio.get_device_info_by_index(i)
                if dev_info['maxInputChannels'] > 0:
                    if 'default' in dev_info['name'].lower() or 'usb' in dev_info['name'].lower():
                        device_index = i
                        break
            
            if device_index is None:
                device_index = self.pyaudio.get_default_input_device_info()['index']
            
            device_name = self.pyaudio.get_device_info_by_index(device_index)['name']
            self.get_logger().info(f"Using audio device: {device_name}")
            
            # Open stream with smaller frames for better real-time response
            self.stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=4000
            )
            
            # Start listening thread
            self.listening = True
            self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
            self.listen_thread.start()
            
            self.get_logger().info("Vosk initialized successfully - listening for commands")
            
        except ImportError as e:
            self.get_logger().error(f"Required library not available: {e}")
            self.get_logger().error("Please install: pip3 install vosk pyaudio")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Vosk: {e}")
    
    def listen_loop(self):
        """Main listening loop."""
        self.get_logger().info("Listening for voice commands...")
        
        # ADDED: Buffer to accumulate audio during silence periods
        audio_buffer = []
        silence_threshold = 0.5  # seconds of silence before processing
        last_audio_time = time.time()
        
        while rclpy.ok() and self.listening and self.recognizer and self.stream:
            try:
                data = self.stream.read(4000, exception_on_overflow=False)
                
                # MODIFIED: Only process audio if TTS is not speaking
                if self.should_process_audio():
                    # Add to buffer for processing
                    audio_buffer.append(data)
                    
                    if self.recognizer.AcceptWaveform(data):
                        result = json.loads(self.recognizer.Result())
                        text = result.get("text", "").strip()
                        
                        if text:
                            self.get_logger().info(f"🎤 Recognized: '{text}'")
                            
                            # Publish to voice_command topic
                            msg = String()
                            msg.data = text
                            self.voice_pub.publish(msg)
                            
                            # Reset buffer after detection
                            audio_buffer = []
                    else:
                        # Partial result - could be used for real-time feedback
                        partial = json.loads(self.recognizer.PartialResult())
                        partial_text = partial.get("partial", "").strip()
                        if partial_text and self.debug:
                            self.get_logger().debug(f"Partial: {partial_text}")
                else:
                    # MODIFIED: Clear audio buffer when ignoring speech
                    if audio_buffer:
                        audio_buffer = []
                        # Reset recognizer to clear any partial results
                        if self.recognizer:
                            self.recognizer.Reset()
                    
                    # Small sleep to prevent busy-waiting while ignoring
                    time.sleep(0.05)
                
                # Update last audio time if we got data
                if data:
                    last_audio_time = time.time()
                
            except Exception as e:
                if self.debug:
                    self.get_logger().debug(f"Listen error: {e}")
                time.sleep(0.1)
    
    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down STT node...")
        self.listening = False
        
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass
        
        if self.pyaudio:
            try:
                self.pyaudio.terminate()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PiDogSTTVoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("STT node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
