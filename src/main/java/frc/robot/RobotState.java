package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.subsystems.drive.Drive;

@SuppressWarnings("unused")
public class RobotState {
  private static RobotState m_instance;

  // Subsystems
  private Drive m_drive;
  private AprilTagVision m_aprilTagVision;

  private RobotState(Drive drive, AprilTagVision aprilTagVision) {
    m_drive = drive;
    m_aprilTagVision = aprilTagVision;
  }

  public static void start(Drive drive, AprilTagVision aprilTagVision) {
    if (m_instance != null) {
      throw new IllegalStateException("RobotState has already been initialized");
    }
    m_instance = new RobotState(drive, aprilTagVision);
  }

  public static RobotState getInstance() {
    if (m_instance == null) {
      throw new IllegalStateException("RobotState must be initialized before it can be retrieved");
    }
    return m_instance;
  }

  public void updateRobotState() {}

  public Pose2d getEstimatedPose() {
    return m_drive.getPose();
  }

  public void addVisionObservation(VisionObservation observation) {
    m_drive.addVisionObservation(observation);
  }
}
