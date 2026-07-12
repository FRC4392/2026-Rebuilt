# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a WPILib-based FRC robot project for team 4392 (Deceivers), 2026 season. It uses Java with Gradle, AdvantageKit for logging/replay, PathPlanner for autonomous paths, and Phoenix 6 / REV for motor controllers.

## Commands

```bash
# Build
./gradlew build

# Run simulation (opens WPILib sim GUI)
./gradlew simulateJava

# Deploy to robot (must be connected to robot's network)
./gradlew deploy

# Run tests
./gradlew test

# Replay AdvantageKit log
./gradlew replayWatch

# Format code (also runs automatically before compile)
./gradlew spotlessApply
```

Spotless runs automatically on every `compileJava` — code is auto-formatted with Google Java Format, so don't manually reformat.

## Architecture

### IO Layer Pattern (AdvantageKit)

Every subsystem follows a strict three-file pattern:
- `SubsystemIO.java` — interface defining hardware inputs/outputs (annotated with `@AutoLog`)
- `SubsystemIOReal.java` — real hardware implementation (Phoenix 6 / REV motors)
- `SubsystemIOSim.java` — simulation implementation

The subsystem class (`Subsystem.java`) only talks to the IO interface, never directly to hardware. This enables AdvantageKit log replay (`Mode.REPLAY`) where the IO layer replays recorded inputs without touching hardware.

### Robot Modes

`RobotConstants.currentMode` controls behavior:
- `REAL` — competition robot
- `COMMISIONING` — real hardware + extra diagnostics (currently the default for real)
- `SIM` — simulation with simulated IO
- `REPLAY` — AdvantageKit replay with stub IO (no hardware)

Switch `simMode` / `realMode` in [RobotConstants.java](src/main/java/frc/robot/RobotConstants.java) to change modes.

### Subsystems

| Subsystem | Description |
|-----------|-------------|
| `Swerve` | 4-module swerve drive using Phoenix 6 (Kraken motors), Pigeon2 gyro, odometry thread |
| `Shooter` | Flywheel shooter with turret and hood; `ShotCalculator` does distance-based interpolation |
| `Hopper` | Ball hopper with turret absolute encoder (passed into Shooter) |
| `Indexer` | Ball indexer between hopper and shooter |
| `Intake` | Roller intake with telescoping extension |
| `Vision` | Multi-Limelight vision (4 cameras) for pose estimation; PhotonVision sim support |
| `Leds` | LED strip control |
| `Climber` | Currently disabled/commented out |

### Global State

`DeceiverRobotState` is a singleton that tracks robot mode timing, intake/feeder status, robot pose, and chassis speeds. Subsystems read from and write to it to share state without direct coupling.

### Auto Modes

PathPlanner is used for autonomous trajectories. `EventTrigger`s in `RobotContainer.configureAutoModes()` bind named path events to commands. Auto paths are stored in `src/main/deploy/pathplanner/`. The navgrid is at `src/main/deploy/pathplanner/navgrid.json`.

### Location-Based Triggers

`RobotContainer.configureBindings()` defines field `Bounds` regions that automatically switch shooter behavior (hub shot vs. left/right pass) based on robot position. `AllianceFlipUtil` mirrors coordinates for red alliance.

### Key Utilities

- `LoggedTunableNumber` — dashboard-tunable number that logs changes (use for PID gains, setpoints)
- `PhoenixUtil` / `SparkUtil` — retry helpers for CAN motor configuration
- `LocalADStarAK` — AdvantageKit-compatible PathPlanner pathfinder wrapper

### Vendor Dependencies

Phoenix 6 (CTRE), REVLib, PathPlannerLib, ChoreoLib, PhotonVision, AdvantageKit — see `vendordeps/`.
