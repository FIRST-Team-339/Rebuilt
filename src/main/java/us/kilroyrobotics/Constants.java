// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.simulation.SimCameraProperties;
import us.kilroyrobotics.util.LoggedTunableNumber;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode kSimMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Whether or not we are tuning the robot */
  public static final boolean kTuning = true;

  public static final class DriveConstants {
    public static final LinearVelocity kMaxDriveSpeed = MetersPerSecond.of(3);
  }

  public static final class VisionConstants {
    // AprilTag layout
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Camera names, must match names configured on coprocessor
    public static final String camera0Name = "FL-LL2";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static final Transform3d robotToCamera0 =
        new Transform3d(
            0.2745,
            0.19575,
            0.2135,
            new Rotation3d(Degrees.of(0.0), Degrees.of(-34.0), Degrees.of(-12.5)));
    public static final SimCameraProperties camera0SimProperties =
        SimCameraProperties.LL2_640_480();

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
        };

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class IntakeConstants {
    public static final class ActuatorConstants {
      /** The CAN ID of the actuator motor */
      public static final int kMotorCanId = 41;

      /** The setpoint for the actuator when fully extended */
      public static final LoggedTunableNumber kExtendedRads =
          new LoggedTunableNumber("Intake/Actuator/ExtendedRads", Units.degreesToRadians(94));

      /** The gear ratio of the actuator motor */
      public static final int kGearing = 5;

      public static final double kP = 0.05;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class RollerConstants {
      /** The CAN ID of the roller motor */
      public static final int kMotorCanId = 42;

      /** The set percent of the motor when intaking fuel */
      public static final LoggedTunableNumber kIntakePercent =
          new LoggedTunableNumber("Intake/Roller/IntakePercent", 0.4);

      /** The set percent of the motor when agitating/launching fuel */
      public static final LoggedTunableNumber kAgitatePercent =
          new LoggedTunableNumber("Intake/Roller/AgitatePercent", 0.1);

      /** The set percent of the motor when outtaking fuel */
      public static final LoggedTunableNumber kOuttakePercent =
          new LoggedTunableNumber("Intake/Roller/OuttakePercent", -0.4);
    }
  }

  public static final class ElevatorConstants {
    /** The CAN ID of the leader motor */
    public static final int kLeaderMotorCanId = 50;

    /** The CAN ID of the follower motor */
    public static final int kFollowerMotorCanId = 51;

    /** The gear ratio of the elevator (9:1) */
    public static final double kGearing = 9.0;

    /** The drum radius for the elevator in meters (example: 1 inch = 0.0254 meters) */
    public static final double kDrumRadiusMeters = 0.0254;

    /** The carriage mass in kg (example: 5 kg) */
    public static final double kCarriageMassKg = 5.0;

    /** The minimum height of the elevator in meters */
    public static final double kMinHeightMeters = 0.0;

    /** The maximum height of the elevator in meters (example: 1 meter) */
    public static final double kMaxHeightMeters = 1.0;

    /**
     * Position conversion factor: motor rotations to meters
     * Formula: (drum circumference) / (gear ratio)
     * = (2 * PI * radius) / gearing
     */
    public static final double kPositionConversionFactor = 
        (2.0 * Math.PI * kDrumRadiusMeters) / kGearing;

    /**
     * Velocity conversion factor: motor RPM to meters per second
     * = position conversion factor / 60
     */
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    /** PID constants - tunable for optimal performance */
    public static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("Elevator/kP", 5.0);
    public static final LoggedTunableNumber kI = 
        new LoggedTunableNumber("Elevator/kI", 0.0);
    public static final LoggedTunableNumber kD = 
        new LoggedTunableNumber("Elevator/kD", 0.1);

    /** Common elevator positions */
    public static final LoggedTunableNumber kStowedPositionMeters =
        new LoggedTunableNumber("Elevator/StowedPosition", 0.0);
    public static final LoggedTunableNumber kMidPositionMeters =
        new LoggedTunableNumber("Elevator/MidPosition", 0.5);
    public static final LoggedTunableNumber kHighPositionMeters =
        new LoggedTunableNumber("Elevator/HighPosition", 1.0);
  }
}
