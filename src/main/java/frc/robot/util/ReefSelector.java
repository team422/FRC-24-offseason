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

    // do fancy maths and logic to determine offset from center of reef to selected branch

    // Main logic for selecting branch position relative to reef center:
    // 1. Start with face closer to DS, right is positon 1, CW is positive increments
    // 2. For each branch, pick the correct X inset based on height (L4 is slightly closer)
    // 2. For each pair, pick the correct branch based on parity of m_position
    // 3. Rotate to correct branch pair based on m_position. 1,2 -> 0, 3,4 -> 60, 5,6 -> 120, etc.
    //    This is the job of the janky floor division

    // From here, we take the reef Position and add the branch offset, then make this relative first
    // to field center, flipping based on alliance color, then mapping to origin

    Translation2d xyPosRelToCenter =
        FieldConstants.kReefDisplacementToCenter
            .plus(
                new Translation2d(
                        FieldConstants.kBranchOffsetXY.getX()
                            - ((m_height == 4)
                                ? FieldConstants.kBranchInsetL4
                                : 0), // pull setpoint closer to reef for L4
                        FieldConstants.kBranchOffsetXY.getY()
                            * (m_position % 2 == 0 ? 1 : -1)) // flip y pose for odd positions
                    .rotateBy(
                        new Rotation2d(
                            Units.degreesToRadians(
                                60
                                    * Math.floor(
                                        (m_position - 0.5) / 2))))) // rotate to correct branch pair
            .rotateBy(
                Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? 0 : 180)); // flip if on blue
    Translation2d xyPos =
        FieldConstants.kFieldCenter.plus(xyPosRelToCenter); // translate to origin-relative

    // Pick the correct height based on m_height
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

    // Calculate wrist angles based on height and position
    double wristRollRad = 0; // will be 0 for all reef positions
    double wristPitchRad =
        m_height != 4
            ? FieldConstants.kLowWristPitch
            : FieldConstants.kL4WristPitch; // adjust angle if on L4
    double wristYawRad =
        Units.degreesToRadians(
            60 * Math.floor((m_position - 0.5) / 2) // rotate to correct heading based on m_position
                + (AllianceFlipUtil.shouldFlip() ? 0 : 180) // flip based on alliance
            );

    // Create and return final Pose3d
    return new Pose3d(
        xyPos.getX(),
        xyPos.getY(),
        height,
        new Rotation3d(wristRollRad, wristPitchRad, wristYawRad));
  }

  // Getters and setters
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

  public void setPoint(int position, int height) {
    m_position = position;
    m_height = height;
  }
}
