package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

public enum Region {
  BLUE_ALLIANCE_ZONE(
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(FieldConstants.LinesVertical.allianceZone, FieldConstants.fieldWidth))),
  RED_ALLIANCE_ZONE(
      new Rectangle2d(
          new Translation2d(
              FieldConstants.fieldLength - FieldConstants.LinesVertical.allianceZone, 0),
          new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth))),
  UPPER_NEUTRAL_ZONE(
      new Rectangle2d(
          new Translation2d(
              FieldConstants.LinesVertical.allianceZone, FieldConstants.fieldWidth / 2),
          new Translation2d(
              FieldConstants.fieldLength - FieldConstants.LinesVertical.allianceZone,
              FieldConstants.fieldWidth))),
  LOWER_NEUTRAL_ZONE(
      new Rectangle2d(
          new Translation2d(FieldConstants.LinesVertical.allianceZone, 0),
          new Translation2d(
              FieldConstants.fieldLength - FieldConstants.LinesVertical.allianceZone,
              FieldConstants.fieldWidth / 2)));

  private final Rectangle2d region;

  private Region(Rectangle2d region) {
    this.region = region;
  }

  public boolean contains(Pose2d point) {
    return region.contains(point.getTranslation());
  }

  public static Optional<Region> getRegion(Pose2d point) {
    for (Region r : Region.values()) {
      if (r.contains(point)) {
        return Optional.of(r);
      }
    }
    return Optional.empty();
  }
}
