package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim m_sim;
  private double m_voltage;

  public IntakeIOSim() {
    m_sim =
        new DCMotorSim(
            IntakeConstants.kSimGearbox, IntakeConstants.kSimGearing, IntakeConstants.kSimMOI);

    m_voltage = 0.0;
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.angularVelocityRPM = m_sim.getAngularVelocityRPM();
    inputs.voltage = m_voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }
}
