package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;

public class ReefSelector {

  private int m_height; // 1 through 4
  private int m_position; // 1 through 12

  public ReefSelector() {
    m_height = 1;
    m_position = 1;
  }

  public Pose3d getPose() {
    // do fancy maths and logic to determine offset from center to selected branch
    Translation2d xyOffset =
        FieldConstants.kBlueReef.plus(
            new Translation2d(
                    FieldConstants.kBranchOffsetXY.getX(),
                    FieldConstants.kBranchOffsetXY.getY() * Math.pow(-1, m_position % 2))
                .rotateBy(
                    new Rotation2d(
                        Units.degreesToRadians(60 * Math.floor((m_position - 0.5) / 2)))));

    // do similar logic for rotation heading:
    Rotation3d wristHeading =
        FieldConstants.kLowWristAngle.rotateBy(
            new Rotation3d(0, 0, Units.degreesToRadians(60 * Math.floor((m_position - 0.5) / 2))));
    if (m_height == 4) {
      wristHeading =
          FieldConstants.kL4WristAngle.rotateBy(
              new Rotation3d(
                  0, 0, Units.degreesToRadians(60 * Math.floor((m_position - 0.5) / 2))));
    }

    // and finally, logic for the height:
    double height = 0;
    if (m_height == 1) {
      height = FieldConstants.kL1Height;
    } else if (m_height == 2) {
      height = FieldConstants.kL2Height;
    } else if (m_height == 3) {
      height = FieldConstants.kL3Height;
    } else if (m_height == 4) {
      height = FieldConstants.kL4Height;
    }

    // create and return final Pose3d
    return new Pose3d(xyOffset.getX(), xyOffset.getY(), height, wristHeading);
  }

  public int getHeight() {
    return m_height;
  }

  public int getPosition() {
    return m_position;
  }

  public void incrementHeight() {
    m_height++;
    if (m_height > 4) {
      m_height = 1;
    }
  }

  public void decrementHeight() {
    m_height--;
    if (m_height < 1) {
      m_height = 4;
    }
  }

  public void incrementPosition() {
    m_position++;
    if (m_position > 12) {
      m_position = 1;
    }
  }

  public void decrementPosition() {
    m_position--;
    if (m_position < 1) {
      m_position = 12;
    }
  }

  public void setPosition(int position, int height) {
    m_position = position;
    m_height = height;
  }
}
