package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerInputs {
    public double angularVelocityRPM;
    public double voltage;
    public double current;
    public boolean photoElectricOne;
    public boolean photoElectricTwo;
  }

  public void updateInputs(IndexerInputs inputs);

  public void setVoltage(double voltage);

  public boolean hasGamePiece();
}
