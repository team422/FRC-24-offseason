package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public Trigger runIntake();

  public Trigger revAndAlign();

  public Trigger runIndexer();

  public Trigger setIdle();

  public Trigger amp();

  public Trigger hockeyPuck();

  public Trigger midlineHockeyPuck();
}
