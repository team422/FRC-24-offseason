// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock m_odometryLock = new ReentrantLock();
  private final GyroIO m_gyroIO;
  public final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine m_sysId;

  public enum DriveProfiles {
    kDefault,
    kAutoAlign,
    kAmpLineup
  }

  private SubsystemProfiles m_profiles;

  private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

  private Rotation2d m_desiredHeading = new Rotation2d();
  private PIDController m_headingController = new PIDController(0.1, 0, 0.09);

  private Rotation2d m_rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] m_lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics, m_rawGyroRotation, m_lastModulePositions, new Pose2d());

  private int m_withinToleranceFrames = 0;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    m_gyroIO = gyroIO;
    m_modules[0] = new Module(flModuleIO, 0);
    m_modules[1] = new Module(frModuleIO, 1);
    m_modules[2] = new Module(blModuleIO, 2);
    m_modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            DriveConstants.kMaxLinearSpeed,
            DriveConstants.kDriveBaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    m_modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(DriveProfiles.kDefault, this::defaultPeriodic);
    periodicHash.put(DriveProfiles.kAutoAlign, this::autoAlignPeriodic);
    periodicHash.put(DriveProfiles.kAmpLineup, this::ampLineupPeriodic);

    m_profiles = new SubsystemProfiles(DriveProfiles.class, periodicHash, DriveProfiles.kDefault);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    m_odometryLock.lock(); // Prevents odometry updates while reading data
    m_gyroIO.updateInputs(m_gyroInputs);
    for (var module : m_modules) {
      module.updateInputs();
    }
    m_odometryLock.unlock();

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Drive/Gyro", m_gyroInputs);
    for (var module : m_modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : m_modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        m_modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = m_modules[moduleIndex].getM_odometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        m_lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (m_gyroInputs.connected) {
        // Use the real gyro angle
        m_rawGyroRotation = m_gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
        m_rawGyroRotation = m_rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      m_poseEstimator.updateWithTime(sampleTimestamps[i], m_rawGyroRotation, modulePositions);
    }

    Logger.recordOutput("Drive/Profile", (DriveProfiles) m_profiles.getCurrentProfile());
  }

  public void defaultPeriodic() {
    runVelocity(m_desiredChassisSpeeds);

    Logger.recordOutput("Drive/DesiredHeading", m_desiredHeading.getDegrees());
    Logger.recordOutput("Drive/DesiredSpeeds", m_desiredChassisSpeeds);
  }

  public void autoAlignPeriodic() {
    m_desiredChassisSpeeds = calculateAutoAlignSpeeds();

    defaultPeriodic();
  }

  public void ampLineupPeriodic() {
    m_desiredChassisSpeeds = calculateAutoAlignSpeeds();
    if (Math.abs(m_headingController.getPositionError()) < Units.degreesToRadians(5)) {
      m_withinToleranceFrames++;
      if (m_withinToleranceFrames > 10) {
        // if we reach the setpoint switch back to default
        updateProfile(DriveProfiles.kDefault);
        m_desiredChassisSpeeds.omegaRadiansPerSecond = 0;
      }
    } else {
      m_withinToleranceFrames = 0;
    }
    defaultPeriodic();
  }

  public ChassisSpeeds calculateAutoAlignSpeeds() {
    if (m_desiredHeading != null) {
      m_desiredChassisSpeeds.omegaRadiansPerSecond =
          m_headingController.calculate(
              getPose().getRotation().getRadians(), m_desiredHeading.getRadians());
    }

    return m_desiredChassisSpeeds;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxLinearSpeed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = m_modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
    m_desiredChassisSpeeds = speeds;
  }

  public ChassisSpeeds getDesiredChassisSpeeds() {
    return m_desiredChassisSpeeds;
  }

  public void setDesiredHeading(Rotation2d heading) {
    m_desiredHeading = heading;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.kModuleTranslations[i].getAngle();
    }
    DriveConstants.kDriveKinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param observation The VisionObservation object containing the vision data.
   */
  public void addVisionObservation(VisionObservation observation) {
    addVisionMeasurement(observation.visionPose(), observation.timestamp());
  }

  public void updateProfile(DriveProfiles newProfile) {
    m_profiles.setCurrentProfile(newProfile);
  }
}
