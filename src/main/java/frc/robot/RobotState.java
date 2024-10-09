package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.KickerConstants;
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
import frc.robot.util.AllianceFlipUtil;
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
    kRevNoAlign,
    kVomitting,
    kFeeding,
    kAmpLineup,

    kAutoDefault,
    kAutoShoot,
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
    periodicHash.put(RobotAction.kVomitting, () -> {});
    periodicHash.put(RobotAction.kRevAndAlign, this::revAndAlignPeriodic);
    periodicHash.put(RobotAction.kRevNoAlign, this::revNoAlignPeriodic);
    periodicHash.put(RobotAction.kFeeding, this::feedingPeriodic);
    periodicHash.put(RobotAction.kAmpLineup, this::ampLineupPeriodic);

    periodicHash.put(RobotAction.kAutoDefault, () -> {});
    periodicHash.put(RobotAction.kAutoShoot, this::autoShootPeriodic);
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
    Logger.recordOutput(
        "Alliance",
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get().toString()
            : "Unknown");
  }

  public void revAndAlignPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateSpeakerShooter(currPose);
    Rotation2d heading = m_shooterMath.calculateSpeakerHeading(currPose);

    m_shooter.setDesiredVelocity(position);
    m_drive.setDesiredHeading(heading);
  }

  public void revNoAlignPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateSpeakerShooter(currPose);

    m_shooter.setDesiredVelocity(position);
  }

  public void autoShootPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateSpeakerShooter(currPose);
    Rotation2d heading = m_shooterMath.calculateSpeakerHeading(currPose);

    m_shooter.setDesiredVelocity(position);
    m_drive.setDesiredHeading(heading);

    Rotation2d actualHeading = m_drive.getRotation();
    ShooterPosition actualPosition = m_shooter.getVelocity();
    double topVelocity = actualPosition.topVelocityRPS();
    double bottomVelocity = actualPosition.bottomVelocityRPS();

    double headingError = heading.minus(actualHeading).getDegrees();
    double topVelocityError = position.topVelocityRPS() - topVelocity;
    double bottomVelocityError = position.bottomVelocityRPS() - bottomVelocity;

    boolean headingWithinTolerance = Math.abs(headingError) < 1.0;
    boolean topVelocityWithinTolerance = Math.abs(topVelocityError) < 1.0;
    boolean bottomVelocityWithinTolerance = Math.abs(bottomVelocityError) < 1.0;

    Logger.recordOutput("ReadyToShoot/Distance", m_shooterMath.getSpeakerDistance(currPose));
    Logger.recordOutput("ReadyToShoot/HeadingWithinTolerance", headingWithinTolerance);
    Logger.recordOutput("ReadyToShoot/TopVelocityWithinTolerance", topVelocityWithinTolerance);
    Logger.recordOutput(
        "ReadyToShoot/BottomVelocityWithinTolerance", bottomVelocityWithinTolerance);

    if (headingWithinTolerance && topVelocityWithinTolerance && bottomVelocityWithinTolerance) {
      m_kicker.updateState(KickerState.kShooting);

      // wait for shooting timer to expire before resetting
      // if button is released it'll still reset so timer is just a backup
      Commands.waitSeconds(KickerConstants.kShootingTimeout.get())
          .andThen(
              Commands.runOnce(
                  () -> {
                    setDefaultAction();
                  }))
          .schedule();
    }
  }

  public void feedingPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateFeedingShooter(currPose);
    Rotation2d heading = m_shooterMath.calculateFeedingHeading(currPose);

    m_shooter.setDesiredVelocity(position);
    m_drive.setDesiredHeading(heading);
  }

  public void ampLineupPeriodic() {}

  public void updateRobotAction(RobotAction newAction) {
    switch (newAction) {
      case kTeleopDefault:
      case kAutoDefault:
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kIdle);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;

      case kIntake:
        m_intake.updateState(IntakeState.kIntaking);
        m_kicker.updateState(KickerState.kIntaking);
        m_shooter.updateState(ShooterState.kIdle);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;

      case kRevAndAlign:
      case kAutoShoot:
      case kFeeding:
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kRevving);
        m_drive.updateProfile(DriveProfiles.kAutoAlign);

        break;

      case kRevNoAlign:
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kRevving);

        break;

      case kVomitting:
        m_intake.updateState(IntakeState.kVomitting);
        m_kicker.updateState(KickerState.kVomitting);
        m_shooter.updateState(ShooterState.kEjecting);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;

      case kAmpLineup:
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kAmp);
        m_drive.updateProfile(DriveProfiles.kAmpLineup);
        m_drive.setDesiredHeading(AllianceFlipUtil.apply(Rotation2d.fromDegrees(90)));

        break;
    }

    m_profiles.setCurrentProfile(newAction);
  }

  public void setDefaultAction() {
    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      updateRobotAction(RobotAction.kAutoDefault);
    } else {
      updateRobotAction(RobotAction.kTeleopDefault);
    }
  }

  public Pose2d getEstimatedPose() {
    return m_drive.getPose();
  }

  public void addVisionObservation(VisionObservation observation) {
    m_drive.addVisionObservation(observation);
  }

  public RobotAction getRobotAction() {
    return (RobotAction) m_profiles.getCurrentProfile();
  }

  public void setDrive(DriveProfiles profile) {
    m_drive.updateProfile(profile);
  }

  public void setIntake(IntakeState state) {
    m_intake.updateState(state);
  }

  public void setKicker(KickerState state) {
    m_kicker.updateState(state);
  }

  public void setShooter(ShooterState state) {
    m_shooter.updateState(state);
  }

  public void onEnabled() {
    m_shooter.resetPID();
  }
}
