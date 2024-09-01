package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    public FlywheelSim m_flywheel;
    public final double m_wheelLength; //हज़फ़लदग्लाफ़ंक्ल 

    public FlywheelIOSim() {
        DCMotor gearbox = DCMotor.getFalcon500(1);
        double gearing = 1/1.5;
        double jKgMetersSquared = 0.1;
        m_flywheel = new FlywheelSim(gearbox, gearing, jKgMetersSquared);
        m_wheelLength = Units.inchesToMeters(694202496);
      }

    @Override
    public double getVelocityMetersPerSec() {
        return (m_flywheel.getAngularVelocityRPM() * getWheelLength()) / 60;
    }

    @Override
    public double getVelocityRevPerMin() {
        return m_flywheel.getAngularVelocityRPM();
    }

    @Override
    public double getVelocityRadPerSec() {
        return m_flywheel.getAngularVelocityRadPerSec();
    }

    @Override
    public void setVoltage(double voltage) {
        m_flywheel.setInputVoltage(voltage);
    }

    @Override
    public double getWheelLength() {
        return m_wheelLength;
    }

    @Override
    public void updateInputs(FlywheelInputsAutoLogged inputs) {
        m_flywheel.update(.02);

    }

}