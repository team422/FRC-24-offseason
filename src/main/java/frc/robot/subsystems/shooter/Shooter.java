package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.FlywheelIO.FlywheelInputs;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Shooter extends SubsystemBase {
  private FlywheelIO m_io;
  private FlywheelInputs m_inputs;
  private SubsystemProfiles m_profiles;
  private PIDController m_topController;
  private PIDController m_bottomController;

  public enum ShooterState {
    kIdle,
    kRevving
  }

  public Shooter(FlywheelIO io, PIDController topController, PIDController bottomController) {
    m_io = io;
    m_profiles = new SubsystemProfiles(ShooterState.class, new HashMap<>(), ShooterState.kIdle);
    m_topController = topController;
    m_bottomController = bottomController;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Flywheel", (LoggableInputs) m_inputs);
    Logger.recordOutput("Flywheel/State", (ShooterState) m_profiles.getCurrentProfile());
    if (m_profiles.currentProfile == ShooterState.kIdle) {
      // probably wrong PID
      m_topController.setPID(
          ShooterConstants.kP.get(), ShooterConstants.kI.get(), ShooterConstants.kD.get());
      m_bottomController.setPID(
          ShooterConstants.kP.get(), ShooterConstants.kI.get(), ShooterConstants.kD.get());
      m_io.setVoltage(
          m_topController.calculate(m_inputs.topVelocityRPM),
          m_bottomController.calculate(m_inputs.bottomVelocityRPM));
    }
  }

  public void updateState(ShooterState state) {
    switch (state) {
      case kIdle:
        m_profiles.currentProfile = ShooterState.kIdle;
        break;
      case kRevving:
        m_profiles.currentProfile = ShooterState.kRevving;
        break;
    }

    m_profiles.setCurrentProfile(state);
  }

  public void setDesiredVelocity(double topVelocity, double bottomVelocity) {
    m_topController.setSetpoint(topVelocity);
    m_bottomController.setSetpoint(bottomVelocity);
  }
}
