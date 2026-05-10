#!/usr/bin/env python3
"""
ROS 2 STT (Speech-to-Text) Voice Command Node for Pi Dog
Publishes recognized commands to voice_command topic
With TTS speaking state awareness and audio filtering for servo noise reduction
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool

import json
import time
import threading
import os
import numpy as np
from scipy.signal import butter, lfilter
from signal import signal, SIGPIPE, SIG_DFL

# Ignore SIGPIPE (prevents BrokenPipeError)
signal(SIGPIPE, SIG_DFL)

class PiDogSTTVoiceCommandNode(Node):
    def __init__(self):
        # Suppress ALSA warnings before any PyAudio initialization
        os.environ['ALSALOG_LEVEL'] = '0'
        
        super().__init__('pidog_stt_voice_command_node')
        
        self.get_logger().info("STT node starting...")
        
        # Parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('model_path', '/pidog/woofer/vosk-model-small-en-us-0.15')
        self.declare_parameter('debug', False)
        self.declare_parameter('post_speech_delay', 0.5)
        self.declare_parameter('cutoff_freq', 4000)  # Hz - filter cutoff frequency
        self.declare_parameter('sample_rate', 16000)  # Hz
        self.declare_parameter('chunk_size', 4000)  # Frames per buffer
        
        self.enabled = self.get_parameter('enabled').value
        self.model_path = self.get_parameter('model_path').value
        self.debug = self.get_parameter('debug').value
        self.post_speech_delay = self.get_parameter('post_speech_delay').value
        self.cutoff_freq = self.get_parameter('cutoff_freq').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        
        # Track TTS speaking state
        self.tts_is_speaking = False
        self.last_speech_end_time = 0
        
        # Publisher for recognized voice commands
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.voice_pub = self.create_publisher(String, 'voice_command', qos_profile)
        
        # Subscribe to TTS speaking state
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
        
        # Filter parameters (from autonomous_dog1_ubuntu.py)
        self.order = 5  # Filter order
        
        if self.enabled:
            self.init_vosk()
        
        if self.recognizer:
            self.get_logger().info("Voice commands will be published to /pidog/voice_command")
            self.get_logger().info(f"Will ignore speech while TTS is speaking + {self.post_speech_delay}s delay")
            self.get_logger().info(f"Audio filter: low-pass at {self.cutoff_freq} Hz (reduces servo noise)")
            self.get_logger().info("Try saying: sit, stand, walk, stop, turn left, turn right")
        else:
            self.get_logger().warning("STT not available - voice commands disabled")
    
    def butter_lowpass_filter(self, data):
        """
        Filter high-frequency noise (e.g., servo whine) above cutoff frequency.
        Adapted from autonomous_dog1_ubuntu.py
        """
        nyq = 0.5 * self.sample_rate
        normal_cutoff = self.cutoff_freq / nyq
        b, a = butter(self.order, normal_cutoff, btype='low', analog=False)
        return lfilter(b, a, data)
    
    def tts_state_callback(self, msg):
        """Handle TTS speaking state changes."""
        was_speaking = self.tts_is_speaking
        self.tts_is_speaking = msg.data
        
        if was_speaking and not self.tts_is_speaking:
            self.last_speech_end_time = time.time()
            if self.debug:
                self.get_logger().debug("TTS finished speaking, starting cooldown period")
        elif not was_speaking and self.tts_is_speaking:
            if self.debug:
                self.get_logger().debug("TTS started speaking, ignoring voice input")
    
    def should_process_audio(self):
        """Determine if we should process audio (not speaking, or in cooldown)."""
        if self.tts_is_speaking:
            return False
        
        time_since_speech_ended = time.time() - self.last_speech_end_time
        if time_since_speech_ended < self.post_speech_delay:
            return False
        
        return True
    
    def init_vosk(self):
        """Initialize Vosk speech recognition with audio filtering."""
        try:
            import vosk
            import pyaudio

            # Check if model exists
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"Vosk model not found at {self.model_path}")
                self.get_logger().info("Please download from: https://alphacephei.com/vosk/models")
                return
            
            # Suppress Vosk logging
            vosk.SetLogLevel(-1)
            
            # Load model
            self.get_logger().info(f"Loading Vosk model from {self.model_path}...")
            model = vosk.Model(self.model_path)
            self.recognizer = vosk.KaldiRecognizer(model, self.sample_rate)
            self.recognizer.SetWords(False)
            
            # Initialize PyAudio
            self.pyaudio = pyaudio.PyAudio()
            
            # Find input device (same as original)
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
            
            # Open stream with same parameters as autonomous_dog1_ubuntu.py
            self.stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk_size
            )
            
            # Start listening thread
            self.listening = True
            self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
            self.listen_thread.start()
            
            self.get_logger().info("Vosk initialized successfully - listening for commands")
            
        except ImportError as e:
            self.get_logger().error(f"Required library not available: {e}")
            self.get_logger().error("Please install: pip3 install vosk pyaudio scipy numpy")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Vosk: {e}")
    
    def listen_loop(self):
        """
        Main listening loop with audio filtering.
        Adapted from voice_recognition_worker() in autonomous_dog1_ubuntu.py
        """
        self.get_logger().info("Listening for voice commands with audio filtering...")
        
        audio_buffer = []
        
        while rclpy.ok() and self.listening and self.recognizer and self.stream:
            try:
                # Read raw audio data from microphone (same as autonomous_dog)
                raw_data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                
                # MODIFIED: Only process audio if TTS is not speaking
                if self.should_process_audio():
                    # Convert to numpy array for filtering (same as autonomous_dog)
                    audio_array = np.frombuffer(raw_data, dtype=np.int16).astype(np.float32)
                    
                    # Apply low-pass filter to reduce servo/motor noise
                    filtered_audio = self.butter_lowpass_filter(audio_array)
                    
                    # Convert back to int16 bytes
                    filtered_data = filtered_audio.astype(np.int16).tobytes()
                    
                    # Add to buffer for processing
                    audio_buffer.append(filtered_data)
                    
                    # Send filtered audio to Vosk
                    if self.recognizer.AcceptWaveform(filtered_data):
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
                        # Optional: log partial results in debug mode
                        if self.debug:
                            partial = json.loads(self.recognizer.PartialResult())
                            partial_text = partial.get("partial", "").strip()
                            if partial_text:
                                self.get_logger().debug(f"Partial: {partial_text}")
                else:
                    # Clear audio buffer when ignoring speech
                    if audio_buffer:
                        audio_buffer = []
                        if self.recognizer:
                            self.recognizer.Reset()
                    
                    # Small sleep to prevent busy-waiting
                    time.sleep(0.05)
                
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
    
    # Setup signal handlers for graceful shutdown
    def signal_handler(sig, frame):
        node.get_logger().info(f"Received signal {sig}, shutting down...")
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        import sys
        sys.exit(0)
    
    import signal as sig_module
    sig_module.signal(sig_module.SIGINT, signal_handler)
    sig_module.signal(sig_module.SIGTERM, signal_handler)
    
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
