package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsPS5 implements DriverControls {
  private CommandPS5Controller m_controller;

  public DriverControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
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
    return m_controller.R2();
  }

  @Override
  public Trigger runKicker() {
    return m_controller.R1();
  }

  @Override
  public Trigger ejectGamePiece() {
    return m_controller.cross();
  }

  @Override
  public Trigger revShooter() {
    return m_controller.L2();
  }
}
