package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getForward();

  public double getStrafe();

  public double getTurn();

  public Trigger resetOdometry();

  public Trigger runIntake();

  public Trigger runIndexer();

  public Trigger ejectGamePiece();

  public Trigger revShooter();

  // this is the feeding command
  // named it hockey puck to let the legacy live on
  public Trigger hockeyPuck();

  public Trigger amp();

  public Trigger cancelAmpLineup();

  public Trigger subwooferShot();
}
