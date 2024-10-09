package frc.robot.subsystems.kicker;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class KickerIONeo implements KickerIO {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private DigitalInput m_beamBreakOne;
  private DigitalInput m_beamBreakTwo;

  public KickerIONeo(int motorPort, int beamBreakOnePort, int beamBreakTwoPort) {
    m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kCoast);
    m_encoder = m_motor.getEncoder();
    m_beamBreakOne = new DigitalInput(beamBreakOnePort);
    m_beamBreakTwo = new DigitalInput(beamBreakTwoPort);
  }

  @Override
  public void updateInputs(KickerInputs inputs) {
    inputs.angularVelocityRPM = m_encoder.getVelocity();
    inputs.voltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.current = m_motor.getOutputCurrent();
    inputs.beamBreakOne = beamBreakOneBroken();
    inputs.beamBreakTwo = beamBreakTwoBroken();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public boolean hasGamePiece() {
    return beamBreakOneBroken() || beamBreakTwoBroken();
  }

  private boolean beamBreakOneBroken() {
    return !m_beamBreakOne.get();
  }

  private boolean beamBreakTwoBroken() {
    return !m_beamBreakTwo.get();
  }
}
