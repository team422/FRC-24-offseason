package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.shooter.Shooter.ShooterPosition;
import org.littletonrobotics.junction.Logger;

/** Uses a lookup table to calculate velocity for the Shooter */
public class ShooterMath {
  private InterpolatingDoubleTreeMap m_speakerTopMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap m_speakerBottomMap = new InterpolatingDoubleTreeMap();
  private final Translation2d kSpeaker = FieldConstants.kCenterSpeakerOpening.toTranslation2d();

  private InterpolatingDoubleTreeMap m_feedingTopMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap m_feedingBottomMap = new InterpolatingDoubleTreeMap();
  private final Translation2d kCorner = FieldConstants.kFeederAim;

  public ShooterMath() {
    // Data for top flywheel speaker
    m_speakerTopMap.put(0.0, 5.0);
    m_speakerTopMap.put(1.0, 10.0);
    m_speakerTopMap.put(2.0, 15.0);
    m_speakerTopMap.put(3.0, 40.0);

    // Data for bottom flywheel speaker
    m_speakerBottomMap.put(0.0, 5.0);
    m_speakerBottomMap.put(1.0, 10.0);
    m_speakerBottomMap.put(2.0, 15.0);
    m_speakerBottomMap.put(3.0, 30.0);

    // distance from measured 0 when tuning to the feeding spot
    double feedingDistance = Units.feetToMeters(27);
    // Data for top flywheel feeding
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(-72), 53.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(-48), 59.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(-24), 45.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(0), 53.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(24), 60.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(48), 64.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(72), 65.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(96), 70.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(120), 72.0);
    m_feedingTopMap.put(feedingDistance + Units.inchesToMeters(144), 78.0);

    // Data for bottom flywheel feeding
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(-72), 12.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(-48), 16.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(-24), 38.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(0), 35.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(24), 30.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(48), 31.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(72), 29.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(96), 28.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(120), 29.0);
    m_feedingBottomMap.put(feedingDistance + Units.inchesToMeters(144), 38.0);
  }

  public double getSpeakerDistance(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d speakerFlipped = AllianceFlipUtil.apply(kSpeaker);
    Logger.recordOutput("ShooterMath/SpeakerTarget", speakerFlipped);
    return speakerFlipped.getDistance(robotTranslation);
  }

  public double getCornerDistance(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d cornerFlipped = AllianceFlipUtil.apply(kCorner);
    Logger.recordOutput("ShooterMath/FeederTarget", cornerFlipped);
    return cornerFlipped.getDistance(robotTranslation);
  }

  public ShooterPosition calculateSpeakerShooter(Pose2d robotPose) {
    double distance = getSpeakerDistance(robotPose);

    // Pull from lookup table
    ShooterPosition res =
        new ShooterPosition(m_speakerTopMap.get(distance), m_speakerBottomMap.get(distance));

    return res;
  }

  public Rotation2d calculateSpeakerHeading(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d speakerFlipped = AllianceFlipUtil.apply(kSpeaker);

    // Use atan2 to calculate the yaw of the robot
    // Basically just finds the distance between the points
    // and calculates the angle from the origin to the new point
    // which is the same as the angle we want to be at
    Rotation2d robotYaw =
        Rotation2d.fromRadians(
            Math.atan2(
                speakerFlipped.getY() - robotTranslation.getY(),
                speakerFlipped.getX() - robotTranslation.getX()));
    return robotYaw;
  }

  public ShooterPosition calculateFeedingShooter(Pose2d robotPose) {
    double distance = getCornerDistance(robotPose);

    // Pull from lookup table
    ShooterPosition res =
        new ShooterPosition(m_feedingTopMap.get(distance), m_feedingBottomMap.get(distance));

    return res;
  }

  public Rotation2d calculateFeedingHeading(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d cornerFlipped = AllianceFlipUtil.apply(kCorner);

    // Use atan2 to calculate the yaw of the robot
    // Basically just finds the distance between the points
    // and calculates the angle from the origin to the new point
    // which is the same as the angle we want to be at
    Rotation2d robotYaw =
        Rotation2d.fromRadians(
            Math.atan2(
                cornerFlipped.getY() - robotTranslation.getY(),
                cornerFlipped.getX() - robotTranslation.getX()));
    return robotYaw;
  }
}
