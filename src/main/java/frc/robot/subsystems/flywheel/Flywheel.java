package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Flywheel extends SubsystemBase{
    private FlywheelIO m_io;
    private FlywheelInputsAutoLogged m_inputs;
    private SubsystemProfiles m_profiles;

    public enum ShooterState {
        kIdle,
        kRevving
    }

    public Flywheel(FlywheelIO io) {
        m_io = io;
        m_profiles = new SubsystemProfiles(ShooterState.class, new HashMap<>(), ShooterState.kIdle);
    }

    @Override
    public void periodic(){
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Flywheel", (LoggableInputs) m_inputs);
        Logger.recordOutput("Flywheel/State", (ShooterState) m_profiles.getCurrentProfile());
    }   

    public void updateState(ShooterState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(ShooterConstants.kIdleVoltage.get()); // get hardcoded nerd
        break;
      case kRevving:
        m_io.setVoltage(ShooterConstants.kShootingVoltage.get());
        break;
    }

    m_profiles.setCurrentProfile(state);
}
}
