package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIONeo implements IntakeIO {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;

  public IntakeIONeo(int port) {
    m_motor = new CANSparkMax(port, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.angularVelocityRPM = m_encoder.getVelocity();
    inputs.voltage = m_motor.getBusVoltage() * m_motor.getAppliedOutput();
    inputs.current = m_motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
