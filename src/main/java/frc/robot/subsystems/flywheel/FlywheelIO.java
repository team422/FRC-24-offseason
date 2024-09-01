package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelInputsAutoLogged {
        public double velocityMetersPerSec;
        public double velocityRadPerSec;
    }
    
    public double getVelocityMetersPerSec();
    public double getVelocityRevPerMin();
    public double getVelocityRadPerSec();
    public void setVoltage(double voltage);
    public double getWheelLength();
    public void updateInputs(FlywheelInputsAutoLogged inputs);
}
