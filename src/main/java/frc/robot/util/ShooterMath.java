package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.shooter.Shooter.ShooterPosition;

/** Uses a lookup table to calculate velocity for the Shooter */
public class ShooterMath {
  private InterpolatingDoubleTreeMap m_topMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap m_bottomMap = new InterpolatingDoubleTreeMap();

  private final Translation2d kSpeaker;

  public ShooterMath() {
    // Data for top flywheel
    m_topMap.put(0.0, 5.0);
    m_topMap.put(1.0, 10.0);
    m_topMap.put(2.0, 15.0);
    m_topMap.put(3.0, 20.0);

    // Data for bottom flywheel
    m_bottomMap.put(0.0, 5.0);
    m_bottomMap.put(1.0, 10.0);
    m_bottomMap.put(2.0, 15.0);
    m_bottomMap.put(3.0, 20.0);

    kSpeaker = FieldConstants.kCenterSpeakerOpening.toTranslation2d();
  }

  public ShooterPosition calculateShooter(Pose2d robotPose) {
    // Calculate distance
    Translation2d robotTranslation = robotPose.getTranslation();
    double distance = kSpeaker.getDistance(robotTranslation);

    // Pull from lookup table
    ShooterPosition res = new ShooterPosition(m_topMap.get(distance), m_bottomMap.get(distance));

    return res;
  }

  public Rotation2d calculateHeading(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();

    // Use atan2 to calculate the yaw of the robot
    // Basically just finds the distance between the points
    // and calculates the angle from the origin to the new point
    // which is the same as the angle we want to be at
    Rotation2d robotYaw =
        Rotation2d.fromRadians(
            Math.atan2(
                kSpeaker.getY() - robotTranslation.getY(),
                kSpeaker.getX() - robotTranslation.getX()));
    return robotYaw;
  }
}
