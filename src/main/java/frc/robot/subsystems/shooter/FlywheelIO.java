package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelInputsAutoLogged {
        public double topVelocityRadPerSec;
        public double topVoltage;
        public double topCurrentAmps;
        public double bottomVelocityRadPerSec;
        public double bottomVoltage;
        public double bottomCurrentAmps;
    }
    
    public void setVoltage(double topVoltage, double bottomVoltage);
    public void updateInputs(FlywheelInputsAutoLogged inputs);
    public void setVelocity(double desiredTopVelocity, double desiredBottomVelocity);
}
