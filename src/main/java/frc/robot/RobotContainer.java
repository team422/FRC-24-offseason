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

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterMathConstants;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.OperatorControlsXbox;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.indexer.IndexerIONeo;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIONeo;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.FlywheelIONeo;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PathPlannerUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive m_drive;
  private Intake m_intake;
  private Indexer m_indexer;
  private Shooter m_shooter;
  private AprilTagVision m_aprilTagVision;

  // Controller
  private DriverControls m_driverControls;
  private OperatorControls m_operatorControls;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  private RobotState m_robotState;

  private AutoFactory m_autoFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureButtonBindings();
  }

  /** Configure the subsystems. */
  private void configureSubsystems() {
    m_aprilTagVision =
        new AprilTagVision(
            new AprilTagVisionIONorthstar("northstar_0", ""),
            new AprilTagVisionIONorthstar("northstar_1", ""),
            new AprilTagVisionIONorthstar("northstar_2", ""),
            new AprilTagVisionIONorthstar("northstar_3", ""));

    if (RobotBase.isReal()) {
      m_drive =
          new Drive(
              new GyroIOPigeon2(true),
              new ModuleIOTalonFX(0),
              new ModuleIOTalonFX(1),
              new ModuleIOTalonFX(2),
              new ModuleIOTalonFX(3));

      m_intake = new Intake(new IntakeIONeo(Ports.kIntakeNeo));

      m_indexer =
          new Indexer(
              new IndexerIONeo(
                  Ports.kIndexerNeo, Ports.kPhotoElectricOne, Ports.kPhotoElectricTwo));

      m_shooter =
          new Shooter(
              new FlywheelIONeo(Ports.kTopFlywheel, Ports.kBottomFlywheel),
              ShooterConstants.kTopController,
              ShooterConstants.kBottomController,
              new SimpleMotorFeedforward(
                  ShooterConstants.kTopKS.get(), ShooterConstants.kTopKV.get()),
              new SimpleMotorFeedforward(
                  ShooterConstants.kBottomKS.get(), ShooterConstants.kBottomKV.get()));
    } else {
      m_drive =
          new Drive(
              new GyroIOPigeon2(true),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());

      m_intake = new Intake(new IntakeIOSim());

      m_indexer = new Indexer(new IndexerIOSim());

      m_shooter =
          new Shooter(
              new FlywheelIOSim(),
              ShooterConstants.kTopController,
              ShooterConstants.kBottomController,
              new SimpleMotorFeedforward(
                  ShooterConstants.kTopKS.get(), ShooterConstants.kTopKV.get()),
              new SimpleMotorFeedforward(
                  ShooterConstants.kBottomKS.get(), ShooterConstants.kBottomKV.get()));
    }

    RobotState.start(m_drive, m_intake, m_indexer, m_shooter, m_aprilTagVision);
    m_robotState = RobotState.getInstance();
  }

  /** Configure the commands. */
  private void configureCommands() {
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_autoFactory = new AutoFactory(m_drive);

    // Configure autos here
    m_autoChooser.addOption("Do Nothing", Commands.none());
    List<String> paths = PathPlannerUtil.getExistingPaths();
    m_autoChooser.addDefaultOption("4 piece alt", m_autoFactory.getAutoCommand("4 piece alt"));
    for (String path : paths) {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    }
  }

  /** Configure the controllers. */
  private void configureControllers() {
    m_driverControls = new DriverControlsPS5(0);
    // m_driverControls = new DriverControlsXbox(0);
    m_operatorControls = new OperatorControlsXbox(5);
  }

  /** Configure the button bindings. */
  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            m_driverControls::getForward,
            m_driverControls::getStrafe,
            m_driverControls::getTurn));

    m_driverControls
        .resetOdometry()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drive.setPose(
                            new Pose2d(
                                m_drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.fromDegrees(180)))),
                    m_drive)
                .ignoringDisable(true));

    m_driverControls
        .runIntake()
        .or(m_operatorControls.runIntake())
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!m_indexer.hasGamePiece()) {
                    m_robotState.updateRobotAction(RobotAction.kIntake);
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Stow/Intake button released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();

                  // advance to indexing if still intaking
                  // otherwise it's moved on alr so no need to change state
                  if (m_indexer.getState() == IndexerState.kIntaking) {
                    m_indexer.updateState(IndexerState.kIndexing);
                  }
                }));

    m_driverControls
        .runIndexer()
        .or(m_operatorControls.runIndexer())
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_indexer.updateState(IndexerState.kShooting);
                }));

    m_driverControls
        .ejectGamePiece()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kVomitting);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Stow/Eject button released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();
                  m_indexer.updateState(IndexerState.kIdle);
                }));

    m_driverControls
        .revShooter()
        .or(m_operatorControls.revAndAlign())
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kRevAndAlign);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Stow/Rev released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();
                }));

    m_driverControls
        .hockeyPuck()
        .or(m_operatorControls.hockeyPuck())
        .onTrue(
            Commands.runOnce(
                () -> {
                  // check if robot is close enough to source to switch to midline feeding
                  Translation2d robotTranslation = m_robotState.getEstimatedPose().getTranslation();
                  Translation2d sourceTranslation = AllianceFlipUtil.apply(FieldConstants.kSource);
                  double sourceDistance = robotTranslation.getDistance(sourceTranslation);
                  if (sourceDistance < ShooterMathConstants.kMidpointTolerance.get()) {
                    m_robotState.updateRobotAction(RobotAction.kMidlineFeeding);
                  } else {
                    m_robotState.updateRobotAction(RobotAction.kFeeding);
                  }
                  Logger.recordOutput("Feeding/Distance", sourceDistance);
                  Logger.recordOutput("Feeding/Target", sourceTranslation);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Stow/Hockey puck released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();
                }));

    m_driverControls
        .amp()
        .or(m_operatorControls.amp())
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kAmpLineup);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Stow/Amp released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();
                }));

    m_driverControls
        .cancelAmpLineup()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (m_robotState.getRobotAction() == RobotAction.kAmpLineup) {
                    m_drive.updateProfile(DriveProfiles.kDefault);
                  }
                }));

    m_driverControls
        .subwooferShot()
        .or(m_operatorControls.subwooferShot())
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kSubwooferShot);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Stow/Subwoofer shot released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();
                }));

    // manual override to set to idle in case of emergency
    m_operatorControls
        .setIdle()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_intake.updateState(IntakeState.kIdle);
                  m_indexer.updateState(IndexerState.kIdle);
                  m_shooter.updateState(ShooterState.kIdle);
                  m_drive.updateProfile(DriveProfiles.kDefault);
                }));

    m_driverControls
        .midlineHockeyPuck()
        .or(m_operatorControls.midlineHockeyPuck())
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kMidlineFeeding);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput(
                      "Stow/Midline hockey puck released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();
                }));

    m_driverControls
        .setpointHockeyPuck()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kSetpointFeeding);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput(
                      "Stow/Setpoint hockey puck released", Timer.getFPGATimestamp());
                  m_robotState.setDefaultAction();
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
