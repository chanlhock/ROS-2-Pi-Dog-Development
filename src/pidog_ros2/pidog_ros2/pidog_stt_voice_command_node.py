#!/usr/bin/env python3
"""
ROS 2 STT (Speech-to-Text) Voice Command Node for Pi Dog
Listens for voice commands and converts to text using Vosk
Uses no custom imports - only standard ROS 2 and Python libraries
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String

import json
import time
import threading
import queue
import math


class PiDogSTTNode(Node):
    def __init__(self):
        super().__init__('pidog_stt_voice_command_node')
        
        # Parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('model_path', '/pidog/woofer/vosk-model-small-en-us-0.15')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 4000)
        self.declare_parameter('silence_timeout', 2.0)  # Seconds of silence to finalize
        self.declare_parameter('speech_timeout', 10.0)  # Max seconds of speech
        self.declare_parameter('debug', False)
        
        self.enabled = self.get_parameter('enabled').value
        self.model_path = self.get_parameter('model_path').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.silence_timeout = self.get_parameter('silence_timeout').value
        self.speech_timeout = self.get_parameter('speech_timeout').value
        self.debug = self.get_parameter('debug').value
        
        # Voice command recognition
        self.recognizer = None
        self.audio_stream = None
        self.pyaudio = None
        self.is_listening = False
        self.recording = False
        self.recorded_audio = []
        self.last_audio_time = 0
        self.speech_detected = False
        
        # ROS 2 Publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.command_pub = self.create_publisher(
            String,
            'voice_command',
            qos_profile
        )
        
        # Initialize Vosk
        self.init_vosk()
        
        # Start listening thread
        if self.enabled and self.recognizer:
            self.listening_thread = threading.Thread(target=self.listen_loop, daemon=True)
            self.listening_thread.start()
        
        # Start command processing thread
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_command_queue, daemon=True)
        self.command_thread.start()
        
        self.get_logger().info(f"PiDog STT Node Ready (enabled={self.enabled})")
        self.get_logger().info(f"Model path: {self.model_path}")
        
        # Announce readiness
        self.get_logger().info("Voice commands: sit, stand, walk, stretch, push up, hand shake, scratch, high five, stop, resume, shutdown")
    
    def init_vosk(self):
        """Initialize Vosk speech recognition"""
        try:
            from vosk import Model, KaldiRecognizer
            import pyaudio
            
            # Check if model exists
            import os
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"Vosk model not found at {self.model_path}")
                self.get_logger().info("Please download model from: https://alphacephei.com/vosk/models")
                self.enabled = False
                return
            
            # Initialize Vosk
            self.get_logger().info(f"Loading Vosk model from {self.model_path}...")
            self.vosk_model = Model(self.model_path)
            self.recognizer = KaldiRecognizer(self.vosk_model, self.sample_rate)
            
            # Initialize PyAudio
            self.pyaudio = pyaudio.PyAudio()
            
            # List available audio devices
            if self.debug:
                self.get_logger().info("Available audio input devices:")
                for i in range(self.pyaudio.get_device_count()):
                    dev_info = self.pyaudio.get_device_info_by_index(i)
                    if dev_info['maxInputChannels'] > 0:
                        self.get_logger().info(f"  Device {i}: {dev_info['name']}")
            
            self.chunk_size = 4096  # 128ms at 16kHz, reduced for lower memory usage
            self.RATE = 16000
            # Open audio stream
            self.audio_stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=1,
                #rate=self.sample_rate,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            
            self.is_listening = True
            self.get_logger().info("Vosk initialized successfully")
            self.get_logger().info("Listening for voice commands...")
            
        except ImportError as e:
            self.get_logger().error(f"Required library not available: {e}")
            self.get_logger().error("Please install: pip3 install vosk pyaudio")
            self.enabled = False
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Vosk: {e}")
            self.enabled = False
    
    def calculate_audio_energy(self, audio_data):
        """Calculate audio energy without numpy"""
        # Convert bytes to int16 values and calculate RMS
        # Simple energy calculation without numpy
        try:
            # Process in chunks to avoid memory issues
            total = 0
            count = 0
            # Read every 4th byte for efficiency (still gives good estimate)
            for i in range(0, len(audio_data), 4):
                if i + 1 < len(audio_data):
                    # Convert two bytes to int16
                    value = int.from_bytes(audio_data[i:i+2], byteorder='little', signed=True)
                    total += abs(value)
                    count += 1
            
            if count > 0:
                return total / count
            return 0
        except Exception:
            return 0
    
    def is_speech_present(self, audio_data):
        """Detect if speech is present in audio chunk using energy threshold"""
        # Simple energy-based VAD (Voice Activity Detection)
        energy = self.calculate_audio_energy(audio_data)
        
        # Threshold for speech detection (adjust as needed)
        speech_threshold = 400
        
        if self.debug:
            self.get_logger().debug(f"Audio energy: {energy:.0f}")
        
        return energy > speech_threshold
    
    def listen_loop(self):
        """Main listening loop"""
        self.get_logger().info("Starting voice command listening loop...")
        
        silent_chunks = 0
        speech_chunks = 0
        max_silent_chunks = int(self.silence_timeout * self.sample_rate / self.chunk_size)
        max_speech_chunks = int(self.speech_timeout * self.sample_rate / self.chunk_size)
        
        # For debug reporting
        last_debug_time = time.time()
        
        while rclpy.ok() and self.is_listening:
            if not self.enabled:
                time.sleep(0.1)
                continue
            
            try:
                # Read audio chunk
                data = self.audio_stream.read(self.chunk_size, exception_on_overflow=False)
                
                # Check for speech
                if self.is_speech_present(data):
                    if not self.recording:
                        # Start recording
                        self.recording = True
                        self.recorded_audio = [data]
                        self.last_audio_time = time.time()
                        speech_chunks = 1
                        silent_chunks = 0
                        if self.debug:
                            self.get_logger().debug("Speech detected, recording started")
                    else:
                        # Continue recording
                        self.recorded_audio.append(data)
                        speech_chunks += 1
                        self.last_audio_time = time.time()
                        
                        # Check for timeout
                        if speech_chunks > max_speech_chunks:
                            self.get_logger().debug("Speech timeout reached")
                            self.process_recorded_audio()
                            self.recording = False
                            self.recorded_audio = []
                            speech_chunks = 0
                else:
                    if self.recording:
                        # Still recording but no speech currently
                        self.recorded_audio.append(data)
                        silent_chunks += 1
                        
                        # Check for silence timeout
                        if silent_chunks > max_silent_chunks:
                            if self.debug:
                                self.get_logger().debug("Silence timeout reached")
                            self.process_recorded_audio()
                            self.recording = False
                            self.recorded_audio = []
                            speech_chunks = 0
                            silent_chunks = 0
                
                # Debug: Report that we're still listening periodically
                if self.debug and time.time() - last_debug_time > 30:
                    last_debug_time = time.time()
                    self.get_logger().debug("Still listening for voice commands...")
                    
            except Exception as e:
                self.get_logger().error(f"Error in listen loop: {e}")
                time.sleep(0.1)
    
    def process_recorded_audio(self):
        """Process recorded audio and perform recognition"""
        if not self.recorded_audio:
            return
        
        audio_length = len(self.recorded_audio) * self.chunk_size / self.sample_rate
        self.get_logger().debug(f"Processing {len(self.recorded_audio)} chunks ({audio_length:.1f} seconds)")
        
        try:
            # Combine audio chunks
            full_audio = b''.join(self.recorded_audio)
            
            # Process with Vosk
            if self.recognizer.AcceptWaveform(full_audio):
                result = json.loads(self.recognizer.Result())
                text = result.get("text", "").strip()
                
                if text:
                    self.get_logger().info(f"🎤 Recognized: '{text}'")
                    
                    # Publish command
                    cmd_msg = String()
                    cmd_msg.data = text
                    self.command_pub.publish(cmd_msg)
                    
                    # Also add to command queue for logging
                    self.command_queue.put(text)
                else:
                    self.get_logger().debug("No text recognized")
            else:
                # Partial result
                partial = json.loads(self.recognizer.PartialResult())
                partial_text = partial.get("partial", "")
                if partial_text and self.debug:
                    self.get_logger().debug(f"Partial: {partial_text}")
                    
        except Exception as e:
            self.get_logger().error(f"Recognition error: {e}")
    
    def process_command_queue(self):
        """Process commands from queue for logging/debugging"""
        while rclpy.ok():
            try:
                command = self.command_queue.get(timeout=1)
                if self.debug:
                    self.get_logger().debug(f"Command queued: {command}")
            except queue.Empty:
                pass
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down STT node...")
        self.is_listening = False
        self.enabled = False
        
        if self.audio_stream:
            try:
                self.audio_stream.stop_stream()
                self.audio_stream.close()
                self.get_logger().info("Audio stream closed")
            except Exception as e:
                self.get_logger().debug(f"Error closing audio stream: {e}")
        
        if self.pyaudio:
            try:
                self.pyaudio.terminate()
                self.get_logger().info("PyAudio terminated")
            except Exception as e:
                self.get_logger().debug(f"Error terminating PyAudio: {e}")
        
        self.get_logger().info("STT node shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = PiDogSTTNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("STT node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():  # Check if already shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
