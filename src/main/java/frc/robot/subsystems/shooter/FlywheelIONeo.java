package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class FlywheelIONeo implements FlywheelIO {
  private CANSparkMax m_topMotor;
  private CANSparkMax m_bottomMotor;
  private RelativeEncoder m_topEncoder;
  private RelativeEncoder m_bottomEncoder;

  public FlywheelIONeo(int topPort, int bottomPort) {
    m_topMotor = new CANSparkMax(topPort, MotorType.kBrushless);
    m_topEncoder = m_topMotor.getEncoder();
    m_bottomMotor = new CANSparkMax(bottomPort, MotorType.kBrushless);
    m_bottomEncoder = m_bottomMotor.getEncoder();
  }

  @Override
  public void setVoltage(double topVoltage, double bottomVoltage) {
    m_topMotor.setVoltage(topVoltage);
    m_bottomMotor.setVoltage(bottomVoltage);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.topVelocityRPS = m_topEncoder.getVelocity() / 60;
    inputs.topVoltage = m_topMotor.getBusVoltage() * m_topMotor.getAppliedOutput();
    inputs.topCurrentAmps = m_topMotor.getOutputCurrent();
    inputs.bottomVelocityRPS = m_bottomEncoder.getVelocity() / 60;
    inputs.bottomVoltage = m_bottomMotor.getBusVoltage() * m_bottomMotor.getAppliedOutput();
    inputs.bottomCurrentAmps = m_bottomMotor.getOutputCurrent();
  }
}
