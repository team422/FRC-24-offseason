package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControlsXbox implements OperatorControls {
  private CommandXboxController m_controller;

  public OperatorControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public Trigger runIntake() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger ejectGamePiece() {
    return m_controller.b();
  }

  @Override
  public Trigger revAndAlign() {
    return m_controller.leftTrigger();
  }

  @Override
  public Trigger revNoAlign() {
    return m_controller.y();
  }

  @Override
  public Trigger revAndShoot() {
    return m_controller.x();
  }

  @Override
  public Trigger runKicker() {
    return m_controller.a();
  }

  @Override
  public Trigger setIdle() {
    return m_controller.start();
  }

  @Override
  public Trigger amp() {
    return m_controller.leftBumper();
  }
}
