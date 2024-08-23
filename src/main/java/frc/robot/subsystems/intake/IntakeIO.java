package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double angularVelocityRPM;
    public double voltage;
    public double current;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double voltage);
}
