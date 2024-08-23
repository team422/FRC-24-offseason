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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.commands.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIONeo;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.Kicker.KickerState;
import frc.robot.subsystems.kicker.KickerIONeo;
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

  // Controller
  private DriverControls m_driverControls;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  private RobotState m_robotState;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureButtonBindings();
  }

  /** Configure the subsystems. */
  private void configureSubsystems() {
    if (RobotBase.isReal()) {
      m_drive =
          new Drive(
              new GyroIOPigeon2(true),
              new ModuleIOTalonFX(0),
              new ModuleIOTalonFX(1),
              new ModuleIOTalonFX(2),
              new ModuleIOTalonFX(3));

      m_intake = new Intake(new IntakeIONeo(Ports.kIntakeNeo));

      m_kicker = new Kicker(new KickerIONeo(Ports.kKickerNeo));
    } else {
      m_drive =
          new Drive(
              new GyroIOPigeon2(true),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
    }

    RobotState.start(m_drive, m_intake, m_kicker);
    m_robotState = RobotState.getInstance();
  }

  /** Configure the commands. */
  private void configureCommands() {
    m_autoChooser = new LoggedDashboardChooser<>("AutoChooser");

    // Configure autos here
  }

  /** Configure the controllers. */
  private void configureControllers() {
    m_driverControls = new DriverControlsPS5(0);
    // m_driverControls = new DriverControlsXbox(0);
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
                  m_intake.updateState(IntakeState.kIntaking);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_intake.updateState(IntakeState.kIdle);
                }));

    m_driverControls
        .runKicker()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_kicker.updateState(KickerState.kShooting);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_kicker.updateState(KickerState.kIdle);
                }));

    m_driverControls
        .ejectGamePiece()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_kicker.updateState(KickerState.kEjecting);
                  m_intake.updateState(IntakeState.kOuttaking);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_kicker.updateState(KickerState.kIdle);
                  m_intake.updateState(IntakeState.kIdle);
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
