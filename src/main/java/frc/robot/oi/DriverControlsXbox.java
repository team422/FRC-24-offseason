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
    return -m_controller.getRightX();
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
  public Trigger runIndexer() {
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

  @Override
  public Trigger hockeyPuck() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger amp() {
    return m_controller.y();
  }

  @Override
  public Trigger cancelAmpLineup() {
    return new Trigger(() -> Math.abs(m_controller.getRightX()) > 0.3);
  }

  @Override
  public Trigger subwooferShot() {
    return m_controller.a();
  }

  @Override
  public Trigger midlineHockeyPuck() {
    return m_controller.x();
  }

  @Override
  public Trigger setpointHockeyPuck() {
    return m_controller.povLeft();
  }
}
