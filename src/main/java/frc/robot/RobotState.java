package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.Kicker.KickerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterPosition;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.ShooterMath;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unused")
public class RobotState {
  private static RobotState m_instance;

  // Subsystems
  private Drive m_drive;
  private Intake m_intake;
  private Kicker m_kicker;
  private Shooter m_shooter;
  private AprilTagVision m_aprilTagVision;

  private ShooterMath m_shooterMath;

  public enum RobotAction {
    kTeleopDefault,
    kIntake,
    kRevAndAlign,
    kEjecting
  }

  private SubsystemProfiles m_profiles;

  private RobotState(
      Drive drive, Intake intake, Kicker kicker, Shooter shooter, AprilTagVision aprilTagVision) {
    m_drive = drive;
    m_intake = intake;
    m_kicker = kicker;
    m_shooter = shooter;
    m_aprilTagVision = aprilTagVision;

    m_shooterMath = new ShooterMath();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kIntake, () -> {});
    periodicHash.put(RobotAction.kEjecting, () -> {});
    periodicHash.put(RobotAction.kRevAndAlign, this::revAndAlignPeriodic);
    m_profiles = new SubsystemProfiles(RobotAction.class, periodicHash, RobotAction.kTeleopDefault);
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

  public void updateRobotState() {
    m_profiles.getPeriodicFunction().run();

    Logger.recordOutput("RobotState/CurrentAction", (RobotAction) m_profiles.getCurrentProfile());
  }

  public void revAndAlignPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateShooter(currPose);
    Rotation2d heading = m_shooterMath.calculateHeading(currPose);

    m_shooter.setDesiredVelocity(position);
    m_drive.setDesiredHeading(heading);
  }

  public void updateRobotAction(RobotAction newAction) {
    switch (newAction) {
      case kTeleopDefault:
        m_intake.updateState(IntakeState.kIdle);
        m_kicker.updateState(KickerState.kIdle);
        m_shooter.updateState(ShooterState.kIdle);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;

      case kIntake:
        m_intake.updateState(IntakeState.kIntaking);
        m_kicker.updateState(KickerState.kIdle);
        m_shooter.updateState(ShooterState.kIdle);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;

      case kRevAndAlign:
        m_intake.updateState(IntakeState.kIdle);
        m_kicker.updateState(KickerState.kIdle);
        m_shooter.updateState(ShooterState.kRevving);
        m_drive.updateProfile(DriveProfiles.kAutoAlign);

        break;

      case kEjecting:
        m_intake.updateState(IntakeState.kOuttaking);
        m_kicker.updateState(KickerState.kEjecting);
        m_shooter.updateState(ShooterState.kEjecting);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;
    }

    m_profiles.setCurrentProfile(newAction);
  }

  public Pose2d getEstimatedPose() {
    return m_drive.getPose();
  }

  public void addVisionObservation(VisionObservation observation) {
    m_drive.addVisionObservation(observation);
  }
}
