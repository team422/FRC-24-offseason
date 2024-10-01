package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public Trigger runIntake();

  public Trigger ejectGamePiece();

  public Trigger revAndAlign();

  public Trigger revNoAlign();

  public Trigger revAndShoot();

  public Trigger runKicker();

  public Trigger setIdle();

  public Trigger amp();
}
