package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private FlywheelIO m_io;
  public final FlywheelInputsAutoLogged m_inputs;
  private SubsystemProfiles m_profiles;
  private PIDController m_topController;
  private PIDController m_bottomController;

  public enum ShooterState {
    kIdle,
    kRevving,
    kEjecting
  }

  /** Class to represent the position of the Shooter */
  public record ShooterPosition(double topVelocityRPS, double bottomVelocityRPS) {}

  public Shooter(FlywheelIO io, PIDController topController, PIDController bottomController) {
    m_io = io;
    m_topController = topController;
    m_bottomController = bottomController;

    m_inputs = new FlywheelInputsAutoLogged();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ShooterState.kIdle, this::idlePeriodic);
    periodicHash.put(ShooterState.kRevving, this::revvingPeriodic);
    m_profiles = new SubsystemProfiles(ShooterState.class, periodicHash, ShooterState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Shooter", m_inputs);
    Logger.recordOutput("Shooter/State", (ShooterState) m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(ShooterConstants.kIdleVoltage.get(), ShooterConstants.kIdleVoltage.get());

    Logger.recordOutput("Shooter/TopSetVoltage", ShooterConstants.kIdleVoltage.get());
    Logger.recordOutput("Shooter/BottomSetVoltage", ShooterConstants.kIdleVoltage.get());
    Logger.recordOutput("Shooter/CurrentTopVelocity", m_inputs.topVelocityRPS);
    Logger.recordOutput("Shooter/CurrentBottomVelocity", m_inputs.bottomVelocityRPS);
    Logger.recordOutput("Shooter/DesiredTopVelocity", m_topController.getSetpoint());
    Logger.recordOutput("Shooter/DesiredBottomVelocity", m_bottomController.getSetpoint());
  }

  public void ejectingPeriodic() {
    m_io.setVoltage(
        ShooterConstants.kEjectingVoltage.get(), ShooterConstants.kEjectingVoltage.get());

    Logger.recordOutput("Shooter/TopSetVoltage", ShooterConstants.kEjectingVoltage.get());
    Logger.recordOutput("Shooter/BottomSetVoltage", ShooterConstants.kEjectingVoltage.get());
    Logger.recordOutput("Shooter/CurrentTopVelocity", m_inputs.topVelocityRPS);
    Logger.recordOutput("Shooter/CurrentBottomVelocity", m_inputs.bottomVelocityRPS);
    Logger.recordOutput("Shooter/DesiredTopVelocity", m_topController.getSetpoint());
    Logger.recordOutput("Shooter/DesiredBottomVelocity", m_bottomController.getSetpoint());
  }

  public void revvingPeriodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_topController.setPID(
              ShooterConstants.kP.get(), ShooterConstants.kI.get(), ShooterConstants.kD.get());
          m_bottomController.setPID(
              ShooterConstants.kP.get(), ShooterConstants.kI.get(), ShooterConstants.kD.get());
        },
        ShooterConstants.kP,
        ShooterConstants.kI,
        ShooterConstants.kD);

    double topPidVoltage = m_topController.calculate(m_inputs.topVelocityRPS);
    double bottomPidVoltage = m_bottomController.calculate(m_inputs.bottomVelocityRPS);

    m_io.setVoltage(topPidVoltage, bottomPidVoltage);

    Logger.recordOutput("Shooter/TopSetVoltage", topPidVoltage);
    Logger.recordOutput("Shooter/BottomSetVoltage", bottomPidVoltage);
    Logger.recordOutput("Shooter/CurrentTopVelocity", m_inputs.topVelocityRPS);
    Logger.recordOutput("Shooter/CurrentBottomVelocity", m_inputs.bottomVelocityRPS);
    Logger.recordOutput("Shooter/DesiredTopVelocity", m_topController.getSetpoint());
    Logger.recordOutput("Shooter/DesiredBottomVelocity", m_bottomController.getSetpoint());
  }

  public void updateState(ShooterState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(0.0, 0.0);
        break;
      case kRevving:
        break;
      case kEjecting:
        break;
    }
    m_profiles.setCurrentProfile(state);
  }

  public void setDesiredVelocity(double topVelocityRPS, double bottomVelocityRPS) {
    m_topController.setSetpoint(topVelocityRPS);
    m_bottomController.setSetpoint(bottomVelocityRPS);
  }

  public void setDesiredVelocity(ShooterPosition position) {
    m_topController.setSetpoint(position.topVelocityRPS());
    m_bottomController.setSetpoint(position.bottomVelocityRPS());
  }
}
