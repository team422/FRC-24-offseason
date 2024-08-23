package frc.robot.subsystems.kicker;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class KickerIONeo implements KickerIO {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;

  public KickerIONeo(int port) {
    m_motor = new CANSparkMax(port, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
  }

  @Override
  public void updateInputs(KickerInputs inputs) {
    inputs.angularVelocityRPM = m_encoder.getVelocity();
    inputs.voltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.current = m_motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
