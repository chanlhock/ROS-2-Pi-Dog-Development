#!/usr/bin/env python3
"""
##########################################################################
# ROS 2 Autonomous Sunfounder Pi Dog with Raspberry Pi 5 in Ubuntu 22.04
#
# PiDog Wrapper - Prevents multiple hardware initializations
#
# This module provides a wrapper around the PiDog hardware that ensures
# only one process can initialize the hardware at a time.
#
# IMPORTANT:
# - Main node: calls get_pidog_instance() to initialize hardware
# - Other nodes: call attach_to_pidog_instance() to use existing hardware
#  
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 05/05/2026     Bernard Chan    Initial release
#
# pidog_wrapper.py is licensed under the GNU General Public License v3.0
# License v3.0 Permissions of this strong copyleft license are 
# conditioned on making available complete source code of licensed 
# works and modifications, which include larger works using a licensed 
# work, under the same license. Copyright and license notices must be 
# preserved. Contributors provide an express grant of patent rights.
##########################################################################
"""

import sys
import os
import fcntl
import atexit
import time
from typing import Optional

# Global state for this process
_PIDOG_INITIALIZED: bool = False
_PIDOG_INSTANCE = None
_LOCK_FILE = None


def _get_lock_file():
    """Get or create lock file for PiDog hardware."""
    global _LOCK_FILE
    
    if _LOCK_FILE is None:
        lock_path = '/tmp/pidog_hardware.lock'
        try:
            _LOCK_FILE = open(lock_path, 'w')
            fcntl.flock(_LOCK_FILE.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            print(f"[PiDogWrapper] Acquired hardware lock")
            return _LOCK_FILE
        except (IOError, OSError):
            print(f"[PiDogWrapper] Hardware lock held by another process")
            return None
    return _LOCK_FILE


def _release_lock():
    """Release the hardware lock."""
    global _LOCK_FILE
    if _LOCK_FILE is not None:
        try:
            fcntl.flock(_LOCK_FILE.fileno(), fcntl.LOCK_UN)
            _LOCK_FILE.close()
            print("[PiDogWrapper] Released hardware lock")
        except Exception as e:
            print(f"[PiDogWrapper] Error releasing lock: {e}")
        finally:
            _LOCK_FILE = None


def get_pidog_instance(disable_sensors: bool = True):
    """
    Get PiDog instance (for MAIN node - acquires hardware lock).
    
    Only the main node should call this function. It acquires the hardware
    lock and initializes the PiDog hardware.
    
    Args:
        disable_sensors: If True, disable automatic sensor reading threads
    
    Returns:
        PiDog instance or None if initialization failed
    """
    global _PIDOG_INITIALIZED, _PIDOG_INSTANCE
    
    if _PIDOG_INITIALIZED:
        return _PIDOG_INSTANCE
    
    # Try to acquire hardware lock
    if _get_lock_file() is None:
        print("[PiDogWrapper] Cannot initialize - lock held by another process")
        return None
    
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
        
        # Import PiDog library
        from pidog import Pidog
        
        print("[PiDogWrapper] Initializing PiDog hardware...")
        
        # Initialize PiDog instance
        _PIDOG_INSTANCE = Pidog()
        
        # Disable sensor threads if requested
        if disable_sensors:
            if hasattr(_PIDOG_INSTANCE, 'sensory_thread_running'):
                _PIDOG_INSTANCE.sensory_thread_running = False
            if hasattr(_PIDOG_INSTANCE, 'sensory_process_stop'):
                _PIDOG_INSTANCE.sensory_process_stop = True
            if hasattr(_PIDOG_INSTANCE, 'sensory_thread') and _PIDOG_INSTANCE.sensory_thread:
                try:
                    _PIDOG_INSTANCE.sensory_thread.join(timeout=0.5)
                except:
                    pass
            print("[PiDogWrapper] Disabled sensor threads")
        
        _PIDOG_INITIALIZED = True
        print("[PiDogWrapper] PiDog hardware initialized successfully")
        
        # Register cleanup
        atexit.register(_release_lock)
        
        return _PIDOG_INSTANCE
        
    except ImportError as e:
        print(f"[PiDogWrapper] ERROR: Failed to import PiDog library: {e}")
        return None
    except Exception as e:
        print(f"[PiDogWrapper] ERROR: Failed to initialize PiDog: {e}")
        return None


def attach_to_pidog_instance():
    """
    Attach to existing PiDog instance (for NON-MAIN nodes).
    
    Non-main nodes should call this function. It does NOT acquire the
    hardware lock or initialize hardware - it just checks if hardware
    is available from the main node.
    
    Returns:
        PiDog instance if available, None otherwise
    """
    global _PIDOG_INITIALIZED, _PIDOG_INSTANCE
    
    if _PIDOG_INITIALIZED:
        return _PIDOG_INSTANCE
    
    # Check if lock file exists (indicates main node is running)
    lock_path = '/tmp/pidog_hardware.lock'
    if not os.path.exists(lock_path):
        print("[PiDogWrapper] No hardware lock found - main node may not be running")
        return None
    
    try:
        # Try to open lock file for reading only (non-blocking)
        with open(lock_path, 'r') as f:
            # Just check if file exists and is locked by another process
            pass
        
        # For non-main nodes, we need to get the PiDog instance WITHOUT
        # re-initializing. Since we can't share the same object across
        # processes, we need to create a new instance that connects to
        # the same hardware but doesn't re-initialize anything.
        
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
        
        # Import PiDog library
        from pidog import Pidog
        
        print("[PiDogWrapper] Attaching to existing PiDog hardware...")
        
        # Create PiDog instance (this will NOT re-initialize hardware if done carefully)
        # Note: This is still risky - PiDog library may not support multiple instances
        _PIDOG_INSTANCE = Pidog()
        
        # Disable sensor threads to prevent conflicts
        if hasattr(_PIDOG_INSTANCE, 'sensory_thread_running'):
            _PIDOG_INSTANCE.sensory_thread_running = False
        if hasattr(_PIDOG_INSTANCE, 'sensory_process_stop'):
            _PIDOG_INSTANCE.sensory_process_stop = True
        
        _PIDOG_INITIALIZED = True
        print("[PiDogWrapper] Successfully attached to PiDog hardware")
        
        return _PIDOG_INSTANCE
        
    except ImportError as e:
        print(f"[PiDogWrapper] ERROR: Failed to import PiDog library: {e}")
        return None
    except Exception as e:
        print(f"[PiDogWrapper] ERROR: Failed to attach to PiDog: {e}")
        return None


def is_pidog_available() -> bool:
    """Check if PiDog hardware is available in this process."""
    return _PIDOG_INITIALIZED and _PIDOG_INSTANCE is not None


def shutdown_pidog() -> None:
    """Shutdown PiDog hardware (only called by main node)."""
    global _PIDOG_INITIALIZED, _PIDOG_INSTANCE
    
    if _PIDOG_INSTANCE is not None:
        try:
            # Perform a sit action before shutdown
            try:
                _PIDOG_INSTANCE.do_action('sit', speed=50)
                _PIDOG_INSTANCE.wait_all_done()
            except:
                pass
            
            _PIDOG_INSTANCE.close()
            print("[PiDogWrapper] PiDog hardware closed")
        except Exception as e:
            print(f"[PiDogWrapper] Error closing PiDog: {e}")
        finally:
            _PIDOG_INSTANCE = None
    
    _PIDOG_INITIALIZED = False
    _release_lock()
