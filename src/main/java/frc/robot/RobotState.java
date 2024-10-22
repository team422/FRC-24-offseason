package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
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
  private Indexer m_indexer;
  private Shooter m_shooter;
  private AprilTagVision m_aprilTagVision;

  private ShooterMath m_shooterMath;

  public enum RobotAction {
    kTeleopDefault,
    kIntake,
    kRevAndAlign,
    kRevNoAlign,
    kSubwooferShot,
    kVomitting,
    kFeeding,
    kMidlineFeeding,
    kSetpointFeeding,
    kAmpLineup,

    kAutoDefault,
    kAutoShoot,
  }

  private SubsystemProfiles m_profiles;

  private RobotState(
      Drive drive, Intake intake, Indexer indexer, Shooter shooter, AprilTagVision aprilTagVision) {
    m_drive = drive;
    m_intake = intake;
    m_indexer = indexer;
    m_shooter = shooter;
    m_aprilTagVision = aprilTagVision;

    m_shooterMath = new ShooterMath();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kIntake, () -> {});
    periodicHash.put(RobotAction.kVomitting, () -> {});
    periodicHash.put(RobotAction.kRevAndAlign, this::revAndAlignPeriodic);
    periodicHash.put(RobotAction.kRevNoAlign, this::revNoAlignPeriodic);
    periodicHash.put(RobotAction.kSubwooferShot, this::subwooferShotPeriodic);
    periodicHash.put(RobotAction.kFeeding, this::feedingPeriodic);
    periodicHash.put(RobotAction.kMidlineFeeding, this::midlineFeedingPeriodic);
    periodicHash.put(RobotAction.kSetpointFeeding, this::setpointFeedingPeriodic);
    periodicHash.put(RobotAction.kAmpLineup, this::ampLineupPeriodic);

    periodicHash.put(RobotAction.kAutoDefault, () -> {});
    periodicHash.put(RobotAction.kAutoShoot, this::autoShootPeriodic);
    m_profiles = new SubsystemProfiles(RobotAction.class, periodicHash, RobotAction.kTeleopDefault);
  }

  public static void start(
      Drive drive, Intake intake, Indexer indexer, Shooter shooter, AprilTagVision aprilTagVision) {
    m_instance = new RobotState(drive, intake, indexer, shooter, aprilTagVision);
  }

  public static RobotState getInstance() {
    if (m_instance == null) {
      throw new IllegalStateException("RobotState must be initialized before it can be retrieved");
    }
    return m_instance;
  }

  public void updateRobotState() {
    double start = Timer.getFPGATimestamp();

    m_profiles.getPeriodicFunction().run();

    Logger.recordOutput("RobotState/CurrentAction", (RobotAction) m_profiles.getCurrentProfile());
    Logger.recordOutput(
        "Alliance",
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get().toString()
            : "Unknown");

    Logger.recordOutput("PeriodicTime/RobotState", Timer.getFPGATimestamp() - start);
  }

  public void revAndAlignPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateSpeakerShooter(currPose);
    Rotation2d heading = m_shooterMath.calculateSpeakerHeading(currPose);

    m_shooter.setDesiredVelocity(position);
    m_drive.setDesiredHeading(heading);

    Logger.recordOutput("ShootDistance", m_shooterMath.getSpeakerDistance(currPose));
  }

  public void revNoAlignPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateSpeakerShooter(currPose);

    m_shooter.setDesiredVelocity(position);

    Logger.recordOutput("ShootDistance", m_shooterMath.getSpeakerDistance(currPose));
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

    Logger.recordOutput("ShootDistance", m_shooterMath.getSpeakerDistance(currPose));
    Logger.recordOutput("ReadyToShoot/HeadingWithinTolerance", headingWithinTolerance);
    Logger.recordOutput("ReadyToShoot/TopVelocityWithinTolerance", topVelocityWithinTolerance);
    Logger.recordOutput(
        "ReadyToShoot/BottomVelocityWithinTolerance", bottomVelocityWithinTolerance);

    if (headingWithinTolerance && topVelocityWithinTolerance && bottomVelocityWithinTolerance) {
      m_indexer.updateState(IndexerState.kShooting);

      // wait for shooting timer to expire before resetting
      // if button is released it'll still reset so timer is just a backup
      Commands.waitSeconds(IndexerConstants.kShootingTimeout.get())
          .andThen(
              Commands.runOnce(
                  () -> {
                    setDefaultAction();
                  }))
          .schedule();
    }
  }

  public void subwooferShotPeriodic() {
    m_shooter.setDesiredVelocity(
        ShooterConstants.kSubwooferTopVelocity.get(),
        ShooterConstants.kSubwooferBottomVelocity.get());
  }

  public void feedingPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateFeedingShooter(currPose);
    Rotation2d heading = m_shooterMath.calculateFeedingHeading(currPose);

    m_shooter.setDesiredVelocity(position);
    m_drive.setDesiredHeading(heading);
  }

  public void midlineFeedingPeriodic() {
    Pose2d currPose = getEstimatedPose();
    ShooterPosition position = m_shooterMath.calculateSourceFeedingShooter(currPose);
    Rotation2d heading = m_shooterMath.calculateSourceFeedingHeading(currPose);

    m_shooter.setDesiredVelocity(position);
    m_drive.setDesiredHeading(heading);
  }

  public void setpointFeedingPeriodic() {
    Pose2d currPose = new Pose2d(FieldConstants.kFeedingSetpoint, new Rotation2d());
    ShooterPosition position = m_shooterMath.calculateFeedingShooter(currPose);

    m_shooter.setDesiredVelocity(position);
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
        m_indexer.updateState(IndexerState.kIntaking);
        m_shooter.updateState(ShooterState.kIdle);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;

      case kRevAndAlign:
      case kAutoShoot:
      case kFeeding:
      case kMidlineFeeding:
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kRevving);
        m_drive.updateProfile(DriveProfiles.kAutoAlign);

        break;

      case kSubwooferShot:
      case kRevNoAlign:
      case kSetpointFeeding:
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kRevving);

        break;

      case kVomitting:
        m_intake.updateState(IntakeState.kVomitting);
        m_indexer.updateState(IndexerState.kVomitting);
        m_shooter.updateState(ShooterState.kEjecting);
        m_drive.updateProfile(DriveProfiles.kDefault);

        break;

      case kAmpLineup:
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kAmp);
        m_drive.updateProfile(DriveProfiles.kDefault);
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

  public void setIndexer(IndexerState state) {
    m_indexer.updateState(state);
  }

  public void setShooter(ShooterState state) {
    m_shooter.updateState(state);
  }

  public void onEnabled() {
    m_shooter.resetPID();
  }
}
