package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.LoggedTunableNumber;

public class IndexerIONeo implements IndexerIO {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
  private double m_desiredVelocity;
  private DigitalInput m_photoElectricOne;
  private DigitalInput m_photoElectricTwo;

  public IndexerIONeo(int motorPort, int photoElectricOnePort, int photoElectricTwoPort) {
    m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kCoast);
    m_encoder = m_motor.getEncoder();
    m_encoder.setVelocityConversionFactor(1 / 60);

    m_controller = m_motor.getPIDController();
    m_controller.setFeedbackDevice(m_encoder);
    m_controller.setP(IndexerConstants.kIndexerP.get());
    m_controller.setI(IndexerConstants.kIndexerI.get());
    m_controller.setD(IndexerConstants.kIndexerD.get());

    m_photoElectricOne = new DigitalInput(photoElectricOnePort);
    m_photoElectricTwo = new DigitalInput(photoElectricTwoPort);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_controller.setP(IndexerConstants.kIndexerP.get());
          m_controller.setI(IndexerConstants.kIndexerI.get());
          m_controller.setD(IndexerConstants.kIndexerD.get());
        },
        IndexerConstants.kIndexerP,
        IndexerConstants.kIndexerI,
        IndexerConstants.kIndexerD);

    inputs.angularVelocityRPS = m_encoder.getVelocity();
    inputs.desiredVelocityRPS = m_desiredVelocity;

    inputs.voltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.current = m_motor.getOutputCurrent();

    inputs.photoElectricOne = photoElectricOneDetected();
    inputs.photoElectricTwo = photoElectricTwoDetected();
    inputs.hasGamePiece = hasGamePiece();
  }

  @Override
  public void setDesiredVelocity(double velocityRPS) {
    m_controller.setReference(velocityRPS, ControlType.kVelocity);
    m_desiredVelocity = velocityRPS;
  }

  @Override
  public void setNeutral() {
    m_controller.setReference(0, ControlType.kDutyCycle);
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
