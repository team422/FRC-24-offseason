package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerInputs {
    public double angularVelocityRPS;
    public double desiredVelocityRPS;
    public double voltage;
    public double current;
    public boolean photoElectricOne;
    public boolean photoElectricTwo;
    public boolean hasGamePiece;
  }

  public void updateInputs(IndexerInputs inputs);

  public void setDesiredVelocity(double velocityRPS);

  public void setNeutral();

  public boolean hasGamePiece();
}
