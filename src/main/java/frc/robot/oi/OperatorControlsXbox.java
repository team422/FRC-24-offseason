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
    return m_controller.rightBumper();
  }

  @Override
  public Trigger revAndAlign() {
    return m_controller.leftTrigger();
  }

  @Override
  public Trigger runIndexer() {
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

  @Override
  public Trigger hockeyPuck() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger midlineHockeyPuck() {
    return m_controller.y();
  }

  @Override
  public Trigger subwooferShot() {
    return m_controller.x();
  }

  @Override
  public Trigger incrementPosition() {
    return m_controller.y();
  }

  @Override
  public Trigger decrementPosition() {
    return m_controller.a();
  }

  @Override
  public Trigger incrementHeight() {
    return m_controller.b();
  }

  @Override
  public Trigger decrementHeight() {
    return m_controller.x();
  }
}
