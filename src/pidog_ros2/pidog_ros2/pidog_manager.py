#!/usr/bin/env python3
"""
PiDog Hardware Manager - Simple singleton for main node only

IMPORTANT: Only ros2_autonomous_pidog.py should use this module.
Other nodes should NOT import this module at all.
"""

import sys
import os
import threading
import time
from typing import Optional


class PiDogManager:
    """
    Singleton manager for PiDog hardware.
    
    ONLY the main node creates and uses this.
    Other nodes should NOT import this module.
    """
    
    _instance: Optional['PiDogManager'] = None
    _lock = threading.Lock()
    _dog = None
    _initialized: bool = False
    _init_error: Optional[str] = None
    
    def __new__(cls) -> 'PiDogManager':
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(PiDogManager, cls).__new__(cls)
            return cls._instance
    
    def initialize(self, disable_sensors: bool = True) -> bool:
        """Initialize PiDog hardware (ONLY called by main node)."""
        if self._initialized:
            return True
        
        print("[PiDogManager] Initializing PiDog hardware...")
        
        try:
            # Add PiDog library path
            possible_paths = [
                '/usr/local/lib/python3.10/dist-packages',
                '/usr/local/lib/python3.9/dist-packages',
                '/usr/lib/python3/dist-packages',
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    sys.path.append(path)
                    break
            
            from pidog import Pidog
            
            # Create PiDog instance (ONLY ONE in the entire system)
            self._dog = Pidog()
            
            # Disable automatic sensor threads to prevent conflicts
            if disable_sensors:
                if hasattr(self._dog, 'sensory_thread_running'):
                    self._dog.sensory_thread_running = False
                if hasattr(self._dog, 'sensory_process_stop'):
                    self._dog.sensory_process_stop = True
                if hasattr(self._dog, 'sensory_thread'):
                    try:
                        self._dog.sensory_thread.join(timeout=0.5)
                    except:
                        pass
            
            self._initialized = True
            print("[PiDogManager] PiDog hardware initialized successfully")
            return True
            
        except ImportError as e:
            self._init_error = f"Failed to import PiDog: {e}"
            print(f"[PiDogManager] ERROR: {self._init_error}")
            return False
        except Exception as e:
            self._init_error = f"Failed to initialize PiDog: {e}"
            print(f"[PiDogManager] ERROR: {self._init_error}")
            return False
    
    def get_pidog(self):
        """Get PiDog instance (ONLY for main node)."""
        return self._dog if self._initialized else None
    
    def is_available(self) -> bool:
        """Check if hardware is available."""
        return self._initialized and self._dog is not None
    
    def get_error(self) -> Optional[str]:
        """Get initialization error."""
        return self._init_error
    
    def shutdown(self):
        """Shutdown PiDog hardware (ONLY called by main node)."""
        if self._dog is not None:
            try:
                self._dog.do_action('sit', speed=50)
                self._dog.wait_all_done()
                time.sleep(0.5)
                self._dog.close()
                print("[PiDogManager] PiDog hardware closed")
            except Exception as e:
                print(f"[PiDogManager] Error closing PiDog: {e}")
            finally:
                self._dog = None
        self._initialized = False


# Global instance
_pidog_manager = None

def get_pidog_manager():
    """Get the singleton PiDogManager instance (ONLY for main node)."""
    global _pidog_manager
    if _pidog_manager is None:
        _pidog_manager = PiDogManager()
    return _pidog_manager


# These functions are for backward compatibility 
def is_hardware_available():
    """Check if hardware is available."""
    manager = get_pidog_manager()
    return manager.is_available()


def attach_to_pidog_hardware():
    """Legacy function - returns hardware status."""
    manager = get_pidog_manager()
    return manager.is_available()
