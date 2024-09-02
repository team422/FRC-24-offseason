package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelInputs {
    public double topVelocityRPM;
    public double topVoltage;
    public double topCurrentAmps;
    public double bottomVelocityRPM;
    public double bottomVoltage;
    public double bottomCurrentAmps;
  }

  public void setVoltage(double topVoltage, double bottomVoltage);

  public void updateInputs(FlywheelInputs inputs);
}
