package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;

@SuppressWarnings("unused")
public class RobotState {
  private static RobotState m_instance;

  // Subsystems
  private Drive m_drive;
  private Intake m_intake;
  private Kicker m_kicker;
  private Shooter m_shooter;
  private AprilTagVision m_aprilTagVision;

  private RobotState(
      Drive drive, Intake intake, Kicker kicker, Shooter shooter, AprilTagVision aprilTagVision) {
    m_drive = drive;
    m_intake = intake;
    m_kicker = kicker;
    m_shooter = shooter;
    m_aprilTagVision = aprilTagVision;
  }

  public static void start(
      Drive drive, Intake intake, Kicker kicker, Shooter shooter, AprilTagVision aprilTagVision) {
    m_instance = new RobotState(drive, intake, kicker, shooter, aprilTagVision);
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
