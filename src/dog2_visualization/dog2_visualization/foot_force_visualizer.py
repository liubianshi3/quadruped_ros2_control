#!/usr/bin/env python3
"""
Foot Force Visualizer for Dog2

Visualizes foot forces as arrows in RViz2 with:
- Arrow length proportional to force magnitude
- Color coding based on force magnitude (green/yellow/red)
- Text labels showing force values
- Hiding/fading for non-contact feet
"""

import math
from typing import List, Tuple
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class FootForceVisualizer:
    """Visualizes foot forces as colored arrows with labels"""
    
    # Force thresholds for color coding (Newtons)
    FORCE_LOW_THRESHOLD = 30.0    # Green below this
    FORCE_MID_THRESHOLD = 60.0    # Yellow between low and mid
    # Red above mid threshold
    
    # Visualization parameters
    ARROW_SCALE_FACTOR = 0.01     # Scale force to arrow length (m/N)
    ARROW_SHAFT_DIAMETER = 0.02   # m
    ARROW_HEAD_DIAMETER = 0.04    # m
    TEXT_SIZE = 0.05              # m
    NON_CONTACT_ALPHA = 0.3       # Opacity for non-contact feet
    
    def __init__(self, frame_id: str = 'base_link'):
        """
        Initialize the foot force visualizer
        
        Args:
            frame_id: Reference frame for markers
        """
        self.frame_id = frame_id
        
        # Foot positions in base_link frame, ordered like WBC/MPC: LF, LH, RH, RF.
        self.foot_positions = [
            Point(x=-0.2, y=-0.15, z=-0.3),  # LF
            Point(x=-0.2, y=0.15, z=-0.3),   # LH
            Point(x=0.2, y=0.15, z=-0.3),    # RH
            Point(x=0.2, y=-0.15, z=-0.3),   # RF
        ]
        
        self.foot_names = ['LF', 'LH', 'RH', 'RF']
    
    def create_markers(
        self,
        foot_forces: List[float],
        contact_states: List[bool] = None
    ) -> MarkerArray:
        """
        Create marker array for foot forces
        
        Args:
            foot_forces: List of 12 floats [fx0, fy0, fz0, fx1, fy1, fz1, ...]
            contact_states: List of 4 bools indicating contact for each foot
        
        Returns:
            MarkerArray containing arrow and text markers
        """
        if len(foot_forces) != 12:
            raise ValueError(f"Expected 12 foot forces, got {len(foot_forces)}")
        
        if contact_states is None:
            contact_states = [True] * 4
        
        marker_array = MarkerArray()
        
        for leg_id in range(4):
            # Extract force vector for this leg
            fx = foot_forces[leg_id * 3 + 0]
            fy = foot_forces[leg_id * 3 + 1]
            fz = foot_forces[leg_id * 3 + 2]
            
            force_magnitude = math.sqrt(fx**2 + fy**2 + fz**2)
            in_contact = contact_states[leg_id]
            
            # Create arrow marker
            arrow_marker = self._create_arrow_marker(
                leg_id, fx, fy, fz, force_magnitude, in_contact
            )
            marker_array.markers.append(arrow_marker)
            
            # Create text label marker
            text_marker = self._create_text_marker(
                leg_id, force_magnitude, in_contact
            )
            marker_array.markers.append(text_marker)
        
        return marker_array
    
    def _create_arrow_marker(
        self,
        leg_id: int,
        fx: float,
        fy: float,
        fz: float,
        magnitude: float,
        in_contact: bool
    ) -> Marker:
        """Create arrow marker for foot force"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "foot_forces"
        marker.id = leg_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Start point (foot position)
        start = self.foot_positions[leg_id]
        
        # End point (start + force vector scaled)
        end = Point()
        if magnitude > 0.1:  # Avoid division by zero
            scale = self.ARROW_SCALE_FACTOR
            end.x = start.x + fx * scale
            end.y = start.y + fy * scale
            end.z = start.z + fz * scale
        else:
            end.x = start.x
            end.y = start.y
            end.z = start.z + 0.01  # Small arrow for zero force
        
        marker.points = [start, end]
        
        # Arrow dimensions
        marker.scale.x = self.ARROW_SHAFT_DIAMETER
        marker.scale.y = self.ARROW_HEAD_DIAMETER
        marker.scale.z = 0.0
        
        # Color based on force magnitude
        marker.color = self._get_force_color(magnitude, in_contact)
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(0.2 * 1e9)  # 0.2 seconds
        
        return marker
    
    def _create_text_marker(
        self,
        leg_id: int,
        force_magnitude: float,
        in_contact: bool
    ) -> Marker:
        """Create text label marker for force value"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "foot_force_labels"
        marker.id = leg_id + 100  # Offset to avoid ID collision
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position slightly above foot
        pos = self.foot_positions[leg_id]
        marker.pose.position.x = pos.x
        marker.pose.position.y = pos.y
        marker.pose.position.z = pos.z + 0.1
        marker.pose.orientation.w = 1.0
        
        # Text content
        marker.text = f"{self.foot_names[leg_id]}: {force_magnitude:.1f}N"
        
        # Text size
        marker.scale.z = self.TEXT_SIZE
        
        # Color (same as arrow)
        marker.color = self._get_force_color(force_magnitude, in_contact)
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(0.2 * 1e9)
        
        return marker
    
    def _get_force_color(self, magnitude: float, in_contact: bool) -> ColorRGBA:
        """
        Get color based on force magnitude
        
        Args:
            magnitude: Force magnitude in Newtons
            in_contact: Whether foot is in contact
        
        Returns:
            ColorRGBA with appropriate color and alpha
        """
        color = ColorRGBA()
        
        # Determine base color based on magnitude
        if magnitude < self.FORCE_LOW_THRESHOLD:
            # Green for low forces
            color.r = 0.0
            color.g = 1.0
            color.b = 0.0
        elif magnitude < self.FORCE_MID_THRESHOLD:
            # Yellow for medium forces
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        else:
            # Red for high forces
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
        
        # Set alpha based on contact state
        if in_contact:
            color.a = 1.0
        else:
            color.a = self.NON_CONTACT_ALPHA
        
        return color
    
    def update_foot_positions(self, positions: List[Point]):
        """
        Update foot positions (e.g., from forward kinematics)
        
        Args:
            positions: List of 4 Point objects for foot positions
        """
        if len(positions) != 4:
            raise ValueError(f"Expected 4 foot positions, got {len(positions)}")
        self.foot_positions = positions
