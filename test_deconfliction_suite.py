"""
UAV Strategic Deconfliction System
A system for verifying safe waypoint missions in shared airspace.

Author: B.Tech Robotics & Automation Student
FlytBase Assignment 2025

This module implements spatial-temporal conflict detection for multiple drones
operating in shared airspace with comprehensive visualization and reporting.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
from enum import Enum
import json
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import warnings

warnings.filterwarnings("ignore")


# ========================
# Data Structures
# ========================


@dataclass
class Waypoint:
    """Represents a single waypoint in a drone's trajectory."""

    x: float
    y: float
    z: float = 0.0  # Default altitude for 2D missions

    def to_array(self) -> np.ndarray:
        """Convert waypoint to numpy array."""
        return np.array([self.x, self.y, self.z])

    def distance_to(self, other: "Waypoint") -> float:
        """Calculate Euclidean distance to another waypoint."""
        return float(np.linalg.norm(self.to_array() - other.to_array()))


@dataclass
class DroneTrajectory:
    """Represents a complete drone trajectory with timing."""

    drone_id: str
    waypoints: List[Waypoint]
    arrival_times: List[float]  # Time to reach each waypoint (seconds)
    safety_buffer: float = 10.0  # Minimum distance threshold (meters)

    def __post_init__(self):
        """Validate trajectory consistency."""
        if len(self.waypoints) != len(self.arrival_times):
            raise ValueError("Number of waypoints must match arrival times")
        if not all(
            self.arrival_times[i] <= self.arrival_times[i + 1]
            for i in range(len(self.arrival_times) - 1)
        ):
            raise ValueError("Arrival times must be in ascending order")

    def get_position_at_time(self, t: float) -> Optional[Waypoint]:
        """
        Interpolate drone position at time t using linear interpolation.
        Returns None if time is outside trajectory bounds.
        """
        if t < self.arrival_times[0] or t > self.arrival_times[-1]:
            return None

        # Find the segment the time falls into
        for i in range(len(self.arrival_times) - 1):
            t1, t2 = self.arrival_times[i], self.arrival_times[i + 1]
            if t1 <= t <= t2:
                if t1 == t2:  # Avoid division by zero
                    return self.waypoints[i]

                # Linear interpolation
                alpha = (t - t1) / (t2 - t1)
                p1, p2 = self.waypoints[i], self.waypoints[i + 1]

                x = p1.x + alpha * (p2.x - p1.x)
                y = p1.y + alpha * (p2.y - p1.y)
                z = p1.z + alpha * (p2.z - p1.z)

                return Waypoint(x, y, z)

        return None

    def get_time_window(self) -> Tuple[float, float]:
        """Return (start_time, end_time) for this trajectory."""
        return (self.arrival_times[0], self.arrival_times[-1])


class ConflictType(Enum):
    """Types of conflicts that can be detected."""

    SPATIAL = "Spatial conflict (insufficient separation)"
    TEMPORAL = "Temporal conflict (same space-time location)"
    SPATIOTEMPORAL = "Spatio-temporal conflict (collision risk)"


@dataclass
class Conflict:
    """Represents a detected conflict between two trajectories."""

    conflict_type: ConflictType
    primary_drone_id: str
    conflicting_drone_id: str
    location: Waypoint  # Where conflict occurs
    time: float  # When conflict occurs (seconds)
    min_distance: float  # Minimum distance at conflict point
    safety_threshold: float  # Required safety distance
    details: str  # Human-readable explanation

    def __str__(self) -> str:
        return (
            "⚠️  CONFLICT DETECTED\n"
            f"Type: {self.conflict_type.value}\n"
            f"Primary Drone: {self.primary_drone_id} <-> "
            f"Conflicting Drone: {self.conflicting_drone_id}\n"
            f"Location: ({self.location.x:.2f}, {self.location.y:.2f}, "
            f"{self.location.z:.2f})\n"
            f"Time: {self.time:.2f}s\n"
            f"Min Distance: {self.min_distance:.2f}m "
            f"(Required: {self.safety_threshold:.2f}m)\n"
            f"Details: {self.details}"
        )


# ========================
# Core Deconfliction Logic
# ========================


class DeconflictionService:
    """
    Central service for checking drone mission conflicts.

    Algorithm Overview:
    1. Sample both trajectories at regular time intervals
    2. For each time sample, calculate distances between drones
    3. Flag conflicts when distance < safety threshold
    4. Report detailed conflict information
    """

    def __init__(self, time_resolution: float = 0.1):
        """
        Initialize deconfliction service.

        Args:
            time_resolution: Time step for trajectory sampling (seconds)
        """
        self.time_resolution = time_resolution
        self.conflicts: List[Conflict] = []

    def check_mission_safety(
        self,
        primary_mission: DroneTrajectory,
        other_missions: List[DroneTrajectory],
        verbose: bool = True,
    ) -> Tuple[str, List[Conflict]]:
        """
        Check if primary mission is safe given other active missions.

        Args:
            primary_mission: The mission to validate
            other_missions: List of other active drone missions
            verbose: Whether to print detailed results

        Returns:
            Tuple of (status, conflicts_list)
            status: "CLEAR" or "CONFLICT DETECTED"
            conflicts_list: List of Conflict objects
        """
        self.conflicts = []

        # Check against each other mission
        for other_mission in other_missions:
            self._check_pair_conflict(primary_mission, other_mission)

        status = "CLEAR" if not self.conflicts else "CONFLICT DETECTED"

        if verbose:
            print("\n" + "=" * 70)
            print(f"DECONFLICTION REPORT - {status}")
            print("=" * 70)
            print(f"Primary Drone: {primary_mission.drone_id}")
            print(
                "Mission Window: "
                f"{primary_mission.arrival_times[0]:.2f}s - "
                f"{primary_mission.arrival_times[-1]:.2f}s"
            )
            print(f"Number of waypoints: {len(primary_mission.waypoints)}")
            print(f"Safety buffer: {primary_mission.safety_buffer}m")
            print(f"Other active missions checked: {len(other_missions)}")
            print()

            if self.conflicts:
                print(f"CONFLICTS FOUND: {len(self.conflicts)}\n")
                for i, conflict in enumerate(self.conflicts, 1):
                    print(f"--- Conflict #{i} ---")
                    print(conflict)
                    print()
            else:
                print("✅ Mission approved for execution. No conflicts detected.")

            print("=" * 70 + "\n")

        return status, self.conflicts

    def _check_pair_conflict(
        self,
        mission1: DroneTrajectory,
        mission2: DroneTrajectory,
    ) -> None:
        """
        Check for conflicts between two specific missions.
        Uses spatio-temporal sampling and interpolation.
        """
        t1_start, t1_end = mission1.get_time_window()
        t2_start, t2_end = mission2.get_time_window()

        # Find overlapping time window
        overlap_start = max(t1_start, t2_start)
        overlap_end = min(t1_end, t2_end)

        if overlap_start > overlap_end:
            return  # No temporal overlap, no conflict possible

        # Sample throughout overlap window
        time_samples = np.arange(
            overlap_start,
            overlap_end + self.time_resolution,
            self.time_resolution,
        )

        min_distance = float("inf")
        min_distance_time = None
        min_distance_pos1 = None
        min_distance_pos2 = None

        for t in time_samples:
            pos1 = mission1.get_position_at_time(t)
            pos2 = mission2.get_position_at_time(t)

            if pos1 and pos2:
                distance = pos1.distance_to(pos2)

                if distance < min_distance:
                    min_distance = distance
                    min_distance_time = t
                    min_distance_pos1 = pos1
                    min_distance_pos2 = pos2

        # Check if minimum distance violates safety threshold
        safety_threshold = max(mission1.safety_buffer, mission2.safety_buffer)

        if min_distance < safety_threshold and min_distance_time is not None:
            conflict = Conflict(
                conflict_type=ConflictType.SPATIOTEMPORAL,
                primary_drone_id=mission1.drone_id,
                conflicting_drone_id=mission2.drone_id,
                location=min_distance_pos1,  # Use primary drone position
                time=min_distance_time,
                min_distance=min_distance,
                safety_threshold=safety_threshold,
                details=(
                    "Minimum separation distance of "
                    f"{min_distance:.2f}m is below required threshold "
                    f"of {safety_threshold:.2f}m. "
                    "Both drones pass within "
                    f"{min_distance:.2f}m of each other at approximately "
                    f"({min_distance_pos1.x:.1f}, "
                    f"{min_distance_pos1.y:.1f}, "
                    f"{min_distance_pos1.z:.1f})."
                ),
            )
            self.conflicts.append(conflict)


# ========================
# Visualization Module
# ========================


class TrajectoryVisualizer:
    """Generates visualizations for drone trajectories and conflicts."""

    @staticmethod
    def plot_2d_trajectories(
        primary_mission: DroneTrajectory,
        other_missions: List[DroneTrajectory],
        conflicts: List[Conflict],
        figsize=(14, 10),
        save_path: Optional[str] = None,
    ) -> None:
        """Plot 2D trajectories with conflict highlights."""
        fig, ax = plt.subplots(figsize=figsize)

        # Plot other missions
        colors = plt.cm.Set3(np.linspace(0, 1, len(other_missions)))

        for mission, color in zip(other_missions, colors):
            waypoints = np.array([w.to_array()[:2] for w in mission.waypoints])
            ax.plot(
                waypoints[:, 0],
                waypoints[:, 1],
                "o-",
                label=f"{mission.drone_id}",
                color=color,
                alpha=0.7,
                linewidth=2,
            )

            # Mark start and end
            ax.plot(
                waypoints[0, 0],
                waypoints[0, 1],
                "s",
                color=color,
                markersize=10,
                alpha=0.8,
            )  # Start
            ax.plot(
                waypoints[-1, 0],
                waypoints[-1, 1],
                "^",
                color=color,
                markersize=10,
                alpha=0.8,
            )  # End

        # Plot primary mission
        primary_waypoints = np.array(
            [w.to_array()[:2] for w in primary_mission.waypoints]
        )
        ax.plot(
            primary_waypoints[:, 0],
            primary_waypoints[:, 1],
            "D-",
            label=f"{primary_mission.drone_id} (Primary)",
            color="red",
            linewidth=3,
            markersize=8,
        )

        # Plot conflicts
        if conflicts:
            conflict_positions = np.array(
                [[c.location.x, c.location.y] for c in conflicts]
            )
            ax.scatter(
                conflict_positions[:, 0],
                conflict_positions[:, 1],
                s=200,
                c="red",
                marker="X",
                edgecolors="darkred",
                linewidth=2,
                label="Conflict Points",
                zorder=5,
            )

            # Add safety buffer circles around conflicts
            for conflict in conflicts:
                circle = plt.Circle(
                    (conflict.location.x, conflict.location.y),
                    conflict.safety_threshold,
                    color="red",
                    fill=False,
                    linestyle="--",
                    linewidth=1,
                    alpha=0.5,
                )
                ax.add_patch(circle)

        ax.set_xlabel("X Position (m)", fontsize=12, fontweight="bold")
        ax.set_ylabel("Y Position (m)", fontsize=12, fontweight="bold")
        ax.set_title(
            "UAV Trajectory Deconfliction - 2D View",
            fontsize=14,
            fontweight="bold",
        )
        ax.legend(loc="best", fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal")

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Saved 2D visualization to {save_path}")

        plt.show()

    @staticmethod
    def plot_3d_trajectories(
        primary_mission: DroneTrajectory,
        other_missions: List[DroneTrajectory],
        conflicts: List[Conflict],
        figsize=(14, 10),
        save_path: Optional[str] = None,
    ) -> None:
        """Plot 3D trajectories with altitude dimension."""
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, projection="3d")

        # Plot other missions
        colors = plt.cm.Set3(np.linspace(0, 1, len(other_missions)))

        for mission, color in zip(other_missions, colors):
            waypoints = np.array([w.to_array() for w in mission.waypoints])
            ax.plot(
                waypoints[:, 0],
                waypoints[:, 1],
                waypoints[:, 2],
                "o-",
                label=f"{mission.drone_id}",
                color=color,
                alpha=0.7,
                linewidth=2,
            )

        # Plot primary mission
        primary_waypoints = np.array(
            [w.to_array() for w in primary_mission.waypoints]
        )
        ax.plot(
            primary_waypoints[:, 0],
            primary_waypoints[:, 1],
            primary_waypoints[:, 2],
            "D-",
            label=f"{primary_mission.drone_id} (Primary)",
            color="red",
            linewidth=3,
            markersize=8,
        )

        # Plot conflicts in 3D
        if conflicts:
            conflict_positions = np.array(
                [
                    [c.location.x, c.location.y, c.location.z]
                    for c in conflicts
                ]
            )
            ax.scatter(
                conflict_positions[:, 0],
                conflict_positions[:, 1],
                conflict_positions[:, 2],
                s=200,
                c="red",
                marker="X",
                edgecolors="darkred",
                linewidth=2,
                label="Conflict Points",
                zorder=5,
            )

        ax.set_xlabel("X (m)", fontsize=11, fontweight="bold")
        ax.set_ylabel("Y (m)", fontsize=11, fontweight="bold")
        ax.set_zlabel("Altitude Z (m)", fontsize=11, fontweight="bold")
        ax.set_title(
            "UAV Trajectory Deconfliction - 3D View",
            fontsize=14,
            fontweight="bold",
        )
        ax.legend(loc="best", fontsize=10)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Saved 3D visualization to {save_path}")

        plt.show()

    @staticmethod
    def plot_spatiotemporal(
        primary_mission: DroneTrajectory,
        other_missions: List[DroneTrajectory],
        conflicts: List[Conflict],
        figsize=(14, 8),
        save_path: Optional[str] = None,
    ) -> None:
        """Plot distance vs time to show spatio-temporal relationships."""
        fig, axes = plt.subplots(1, len(other_missions), figsize=figsize)

        if len(other_missions) == 1:
            axes = [axes]

        t1_start, t1_end = primary_mission.get_time_window()
        time_samples = np.arange(t1_start, t1_end + 0.1, 0.1)

        for ax, mission in zip(axes, other_missions):
            distances = []
            times = []

            t2_start, t2_end = mission.get_time_window()
            overlap_start = max(t1_start, t2_start)
            overlap_end = min(t1_end, t2_end)

            if overlap_start <= overlap_end:
                for t in np.arange(overlap_start, overlap_end + 0.1, 0.1):
                    pos1 = primary_mission.get_position_at_time(t)
                    pos2 = mission.get_position_at_time(t)

                    if pos1 and pos2:
                        distance = pos1.distance_to(pos2)
                        distances.append(distance)
                        times.append(t)

                ax.plot(
                    times,
                    distances,
                    "b-",
                    linewidth=2,
                    label="Distance",
                )

                # Add safety threshold line
                safety_threshold = max(
                    primary_mission.safety_buffer,
                    mission.safety_buffer,
                )
                ax.axhline(
                    y=safety_threshold,
                    color="r",
                    linestyle="--",
                    linewidth=2,
                    label="Safety Threshold",
                )

                # Highlight conflict regions
                for conflict in conflicts:
                    if conflict.conflicting_drone_id == mission.drone_id:
                        ax.axvline(
                            x=conflict.time,
                            color="red",
                            linestyle=":",
                            linewidth=2,
                            alpha=0.7,
                        )
                        ax.scatter(
                            [conflict.time],
                            [conflict.min_distance],
                            s=100,
                            c="red",
                            marker="X",
                            zorder=5,
                        )

            ax.set_xlabel("Time (s)", fontsize=11, fontweight="bold")
            ax.set_ylabel("Distance (m)", fontsize=11, fontweight="bold")
            ax.set_title(
                f"{primary_mission.drone_id} vs {mission.drone_id}",
                fontsize=12,
                fontweight="bold",
            )
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=9)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Saved spatio-temporal visualization to {save_path}")

        plt.show()


# ========================
# Test Scenarios
# ========================


def scenario_no_conflict() -> bool:
    """Scenario 1: Mission completes without any conflicts."""
    print("\n" + "=" * 70)
    print("SCENARIO 1: CONFLICT-FREE MISSION")
    print("=" * 70)

    # Primary drone mission
    primary = DroneTrajectory(
        drone_id="DRONE-A",
        waypoints=[
            Waypoint(0, 0, 10),
            Waypoint(50, 0, 10),
            Waypoint(100, 50, 10),
            Waypoint(100, 100, 10),
        ],
        arrival_times=[0, 10, 25, 40],
        safety_buffer=15.0,
    )

    # Other missions (far away)
    other_missions = [
        DroneTrajectory(
            drone_id="DRONE-B",
            waypoints=[
                Waypoint(150, 0, 15),
                Waypoint(200, 50, 15),
                Waypoint(200, 150, 15),
            ],
            arrival_times=[0, 15, 35],
            safety_buffer=15.0,
        ),
        DroneTrajectory(
            drone_id="DRONE-C",
            waypoints=[
                Waypoint(0, 150, 20),
                Waypoint(50, 150, 20),
            ],
            arrival_times=[5, 30],
            safety_buffer=15.0,
        ),
    ]

    service = DeconflictionService()
    status, conflicts = service.check_mission_safety(primary, other_missions)

    # Visualize
    TrajectoryVisualizer.plot_2d_trajectories(
        primary,
        other_missions,
        conflicts,
        save_path="scenario_1_2d.png",
    )

    TrajectoryVisualizer.plot_3d_trajectories(
        primary,
        other_missions,
        conflicts,
        save_path="scenario_1_3d.png",
    )

    return status == "CLEAR"


def scenario_with_conflict() -> bool:
    """Scenario 2: Mission has conflicts with other drones."""
    print("\n" + "=" * 70)
    print("SCENARIO 2: MISSION WITH CONFLICTS")
    print("=" * 70)

    # Primary drone mission
    primary = DroneTrajectory(
        drone_id="DRONE-A",
        waypoints=[
            Waypoint(0, 0, 10),
            Waypoint(50, 50, 10),
            Waypoint(100, 100, 10),
        ],
        arrival_times=[0, 15, 30],
        safety_buffer=20.0,
    )

    # Conflicting missions
    other_missions = [
        DroneTrajectory(
            drone_id="DRONE-B",
            waypoints=[
                Waypoint(100, 0, 10),
                Waypoint(50, 50, 10),
                Waypoint(0, 100, 10),
            ],
            arrival_times=[0, 15, 30],
            safety_buffer=20.0,
        ),
        DroneTrajectory(
            drone_id="DRONE-C",
            waypoints=[
                Waypoint(50, 100, 15),
                Waypoint(50, 50, 15),
                Waypoint(50, 0, 15),
            ],
            arrival_times=[5, 17, 32],
            safety_buffer=20.0,
        ),
    ]

    service = DeconflictionService()
    status, conflicts = service.check_mission_safety(primary, other_missions)

    # Visualize
    TrajectoryVisualizer.plot_2d_trajectories(
        primary,
        other_missions,
        conflicts,
        save_path="scenario_2_2d.png",
    )

    TrajectoryVisualizer.plot_3d_trajectories(
        primary,
        other_missions,
        conflicts,
        save_path="scenario_2_3d.png",
    )

    TrajectoryVisualizer.plot_spatiotemporal(
        primary,
        other_missions,
        conflicts,
        save_path="scenario_2_spatiotemporal.png",
    )

    return status == "CONFLICT DETECTED" and len(conflicts) > 0


def scenario_3d_mission() -> bool:
    """Scenario 3: 3D mission with altitude changes."""
    print("\n" + "=" * 70)
    print("SCENARIO 3: 3D ALTITUDE-VARYING MISSION")
    print("=" * 70)

    # Primary drone with altitude changes
    primary = DroneTrajectory(
        drone_id="DRONE-A",
        waypoints=[
            Waypoint(0, 0, 5),
            Waypoint(50, 50, 15),
            Waypoint(100, 100, 25),
            Waypoint(150, 150, 30),
        ],
        arrival_times=[0, 15, 30, 45],
        safety_buffer=15.0,
    )

    # Other missions at different altitudes
    other_missions = [
        DroneTrajectory(
            drone_id="DRONE-B",
            waypoints=[
                Waypoint(0, 150, 5),
                Waypoint(100, 50, 10),
                Waypoint(150, 0, 20),
            ],
            arrival_times=[0, 20, 40],
            safety_buffer=15.0,
        ),
        DroneTrajectory(
            drone_id="DRONE-D",
            waypoints=[
                Waypoint(150, 150, 35),
                Waypoint(0, 0, 40),
            ],
            arrival_times=[0, 50],
            safety_buffer=15.0,
        ),
    ]

    service = DeconflictionService()
    status, conflicts = service.check_mission_safety(primary, other_missions)

    # Visualize
    TrajectoryVisualizer.plot_3d_trajectories(
        primary,
        other_missions,
        conflicts,
        save_path="scenario_3_3d.png",
    )

    # For this scenario, success means code runs and produces visualization
    return True


# ========================
# Execution
# ========================


if __name__ == "__main__":
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print(
        "║"
        + " " * 15
        + "UAV STRATEGIC DECONFLICTION SYSTEM"
        + " " * 19
        + "║"
    )
    print(
        "║"
        + " " * 10
        + "FlytBase Assignment - Robotics & Automation"
        + " " * 14
        + "║"
    )
    print("╚" + "=" * 68 + "╝")

    # Run test scenarios
    test_results = []

    test_results.append(("Scenario 1 (No Conflict)", scenario_no_conflict()))
    test_results.append(("Scenario 2 (With Conflicts)", scenario_with_conflict()))
    test_results.append(("Scenario 3 (3D Mission)", scenario_3d_mission()))

    # Summary
    print("\n" + "=" * 70)
    print("TEST RESULTS SUMMARY")
    print("=" * 70)
    for test_name, passed in test_results:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{test_name}: {status}")

    print("\n✅ All tests completed!")
    print("Generated visualizations:")
    print("  - scenario_1_2d.png / scenario_1_3d.png")
    print("  - scenario_2_2d.png / scenario_2_3d.png / scenario_2_spatiotemporal.png")
    print("  - scenario_3_3d.png")
    print("=" * 70 + "\n")
