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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXbox;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.OperatorControlsXbox;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIONeo;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.Kicker.KickerState;
import frc.robot.subsystems.kicker.KickerIONeo;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.shooter.FlywheelIONeo;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.PathPlannerUtil;
import java.util.List;
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
  private Kicker m_kicker;
  private Shooter m_shooter;
  private AprilTagVision m_aprilTagVision;

  // Controller
  private DriverControls m_driverControls;
  private OperatorControls m_operatorControls;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  private RobotState m_robotState;

  private AutoFactory m_autoFactory;

  private boolean m_ampToggle; // toggle for amp controls

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

      m_kicker =
          new Kicker(new KickerIONeo(Ports.kKickerNeo, Ports.kBeamBreakOne, Ports.kBeamBreakTwo));

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

      m_kicker = new Kicker(new KickerIOSim());

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

    RobotState.start(m_drive, m_intake, m_kicker, m_shooter, m_aprilTagVision);
    m_robotState = RobotState.getInstance();
  }

  /** Configure the commands. */
  private void configureCommands() {
    m_autoChooser = new LoggedDashboardChooser<>("AutoChooser");
    m_autoFactory = new AutoFactory(m_drive);

    // Configure autos here
    m_autoChooser.addOption("Do Nothing", Commands.none());
    List<String> paths = PathPlannerUtil.getExistingPaths();
    System.out.println("Paths: " + paths);
    for (String path : paths) {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    }
  }

  /** Configure the controllers. */
  private void configureControllers() {
    // m_driverControls = new DriverControlsPS5(0);
    m_driverControls = new DriverControlsXbox(0);
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
                            new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
                    m_drive)
                .ignoringDisable(true));

    m_driverControls
        .runIntake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kIntake);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                }));

    m_driverControls
        .runKicker()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_kicker.updateState(KickerState.kShooting);

                  // if in amp cancel after done shooting
                  if (m_robotState.getRobotAction() == RobotAction.kAmpLineup) {
                    Commands.waitSeconds(KickerConstants.kShootingTimeout.get())
                        .andThen(
                            Commands.runOnce(
                                () -> {
                                  m_robotState.setDefaultAction();
                                  m_drive.updateProfile(DriveProfiles.kDefault);
                                  m_ampToggle = false;
                                }))
                        .schedule();
                  }
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
                  m_robotState.setDefaultAction();
                  m_kicker.updateState(KickerState.kIdle);
                }));

    m_driverControls
        .revShooter()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kRevAndAlign);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                }));

    m_driverControls
        .hockeyPuck()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kFeeding);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                }));

    m_driverControls
        .amp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_ampToggle = !m_ampToggle;
                  if (m_ampToggle) {
                    m_robotState.updateRobotAction(RobotAction.kAmpLineup);
                  } else {
                    m_robotState.setDefaultAction();
                  }
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

    m_operatorControls
        .runIntake()
        .whileTrue(
            Commands.run(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kIntake);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                }));

    m_operatorControls
        .runKicker()
        .whileTrue(Commands.run(() -> m_kicker.updateState(KickerState.kShooting)));

    m_operatorControls
        .ejectGamePiece()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kVomitting);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                  m_kicker.updateState(KickerState.kIdle);
                }));

    m_operatorControls
        .revAndAlign()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kRevAndAlign);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                }));

    m_operatorControls
        .revAndShoot()
        .onTrue(Commands.runOnce(() -> m_robotState.updateRobotAction(RobotAction.kAutoShoot)));

    m_operatorControls
        .revNoAlign()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotState.updateRobotAction(RobotAction.kRevNoAlign);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                }));

    m_operatorControls
        .revAndShoot()
        .onTrue(Commands.runOnce(() -> m_robotState.updateRobotAction(RobotAction.kAutoShoot)))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_robotState.setDefaultAction();
                }));

    // manual override to set to idle in case of emergency
    m_operatorControls
        .setIdle()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_intake.updateState(IntakeState.kIdle);
                  m_kicker.updateState(KickerState.kIdle);
                  m_shooter.updateState(ShooterState.kIdle);
                  m_drive.updateProfile(DriveProfiles.kDefault);
                }));

    m_operatorControls
        .amp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_ampToggle = !m_ampToggle;
                  if (m_ampToggle) {
                    m_robotState.updateRobotAction(RobotAction.kAmpLineup);
                  } else {
                    m_robotState.setDefaultAction();
                  }
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

  public void updateRobotState() {
    m_robotState.updateRobotState();
  }
}
