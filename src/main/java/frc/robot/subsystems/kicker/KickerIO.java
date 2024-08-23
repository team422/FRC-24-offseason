package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerInputs {
    public double angularVelocityRPM;
    public double voltage;
    public double current;
  }

  public void updateInputs(KickerInputs inputs);

  public void setVoltage(double voltage);
}
