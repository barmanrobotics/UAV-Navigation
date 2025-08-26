"""
UAV Navigation Classes Package

This package contains well-structured classes for UAV navigation:
- Drone: Comprehensive drone control class
- Tower: Multi-drone coordination and control class

Usage:
    from classes import Drone, Tower
    
    # Create and use drone
    drone = Drone()
    drone.start()
    
    # Create and use tower
    tower = Tower()
    tower.start()
"""

from .drone import Drone
from .tower import Tower

__version__ = "1.0.0"
__author__ = "Generated from drone_pi and tower_pi folders"

__all__ = ['Drone', 'Tower']



