package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIONeo implements IndexerIO {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private DigitalInput m_photoElectricOne;
  private DigitalInput m_photoElectricTwo;

  public IndexerIONeo(int motorPort, int photoElectricOnePort, int photoElectricTwoPort) {
    m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kCoast);
    m_encoder = m_motor.getEncoder();

    m_photoElectricOne = new DigitalInput(photoElectricOnePort);
    m_photoElectricTwo = new DigitalInput(photoElectricTwoPort);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.angularVelocityRPM = m_encoder.getVelocity();

    inputs.voltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.current = m_motor.getOutputCurrent();

    inputs.photoElectricOne = photoElectricOneDetected();
    inputs.photoElectricTwo = photoElectricTwoDetected();
    inputs.hasGamePiece = hasGamePiece();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public boolean hasGamePiece() {
    return photoElectricOneDetected() || photoElectricTwoDetected();
  }

  /** Returns true if the photoelectric sensor detects a piece */
  private boolean photoElectricOneDetected() {
    return !m_photoElectricOne.get();
  }

  /** Returns true if the photoelectric sensor detects a piece */
  private boolean photoElectricTwoDetected() {
    return !m_photoElectricTwo.get();
  }
}
