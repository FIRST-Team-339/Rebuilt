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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

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

  public static final class FieldConstants {
    public static final Pose2d hubPose =
        new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);
  }

  public static final class DriveConstants {
    public static final LinearVelocity kMaxDriveSpeed = MetersPerSecond.of(3);

    public static final double kDeadband = 0.1;

    public static final double kAngleKP = 5.0;
    public static final double kAngleKD = 0.4;
    public static final double kAngleMaxVelocity = 8.0;
    public static final double kAngleMaxAcceleration = 20.0;
    public static final double kFFStartDelay = 2.0; // Secs
    public static final double kFFRampRate = 0.1; // Volts/Sec
    public static final double kWheelRadiusMaxVelocity = 0.25; // Rad/Sec
    public static final double kWheelRadiusRampRate = 0.05; // Rad/Sec^2
  }

  public static final class LauncherConstants {
    public static final class FlywheelConstants {
      public static final int kMotorId = 41;
      public static final int kFollowerMotorId = 42;

      public static final double kP = 0.1;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final Translation3d kTranslation = new Translation3d(0.2657602, 0.0, 0.3731006);
    }
  }
}
