package us.kilroyrobotics.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum Zone {
  ALLIANCE_ZONE(
      Translation2d.kZero, new Translation2d(Meters.of(4.0), Meters.of(8.0)), Zone.ZoneType.NORMAL),
  ALLIANCE_LEFT_TRENCH(
      new Translation2d(Meters.of(4.0), Meters.of(6.7)),
      new Translation2d(Meters.of(5.15), Meters.of(8.0)),
      Zone.ZoneType.TRENCH),
  ALLIANCE_LEFT_BUMP(
      new Translation2d(Meters.of(4.0), Meters.of(4.0)),
      new Translation2d(Meters.of(5.15), Meters.of(6.7)),
      Zone.ZoneType.BUMP),
  ALLIANCE_RIGHT_BUMP(
      new Translation2d(Meters.of(4.0), Meters.of(1.3)),
      new Translation2d(Meters.of(5.15), Meters.of(4.0)),
      Zone.ZoneType.BUMP),
  ALLIANCE_RIGHT_TRENCH(
      new Translation2d(Meters.of(4.0), Meters.of(0.0)),
      new Translation2d(Meters.of(5.15), Meters.of(1.3)),
      Zone.ZoneType.TRENCH),
  NEUTRAL_ZONE(
      new Translation2d(Meters.of(5.15), Meters.of(0.0)),
      new Translation2d(Meters.of(12.4), Meters.of(8.0)),
      Zone.ZoneType.NORMAL),
  OPPOSING_ALLIANCE_ZONE(
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_ZONE.getCornerA()),
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_ZONE.getCornerB()),
      Zone.ZoneType.NORMAL),
  OPPOSING_ALLIANCE_LEFT_TRENCH(
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_LEFT_TRENCH.getCornerA()),
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_LEFT_TRENCH.getCornerB()),
      Zone.ZoneType.TRENCH),
  OPPOSING_ALLIANCE_LEFT_BUMP(
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_LEFT_BUMP.getCornerA()),
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_LEFT_BUMP.getCornerB()),
      Zone.ZoneType.BUMP),
  OPPOSING_ALLIANCE_RIGHT_BUMP(
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_RIGHT_BUMP.getCornerA()),
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_RIGHT_BUMP.getCornerB()),
      Zone.ZoneType.BUMP),
  OPPOSING_ALLIANCE_RIGHT_TRENCH(
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_RIGHT_TRENCH.getCornerA()),
      FlippingUtil.flipFieldPosition(Zone.ALLIANCE_RIGHT_TRENCH.getCornerB()),
      Zone.ZoneType.TRENCH);

  public static enum ZoneType {
    NORMAL,
    TRENCH,
    BUMP
  }

  private final Translation2d cornerA;
  private final Translation2d cornerB;

  private final Rectangle2d zone;
  private final ZoneType type;

  private Zone(Translation2d cornerA, Translation2d cornerB, ZoneType type) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      cornerA = FlippingUtil.flipFieldPosition(cornerA);
      cornerB = FlippingUtil.flipFieldPosition(cornerB);
    }

    this.cornerA = cornerA;
    this.cornerB = cornerB;

    this.zone = new Rectangle2d(cornerA, cornerB);
    this.type = type;
  }

  public static Zone getZoneFromPose(Pose2d pose) {
    for (Zone zone : values()) {
      if (zone.inZone(pose)) return zone;
    }

    return null;
  }

  public Translation2d getCornerA() {
    return cornerA;
  }

  public Translation2d getCornerB() {
    return cornerB;
  }

  public boolean inZone(Pose2d pose) {
    return zone.contains(pose.getTranslation());
  }

  public Zone.ZoneType getType() {
    return type;
  }
}
