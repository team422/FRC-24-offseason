package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXbox implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public double getForward() {
    return m_controller.getLeftY();
  }

  @Override
  public double getStrafe() {
    return m_controller.getLeftX();
  }

  @Override
  public double getTurn() {
    return m_controller.getRightX();
  }

  @Override
  public Trigger resetOdometry() {
    return m_controller.povUp();
  }

  @Override
  public Trigger runIntake() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger runKicker() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger ejectGamePiece() {
    return m_controller.b();
  }

  @Override
  public Trigger revShooter() {
    return m_controller.leftTrigger();
  }
}
