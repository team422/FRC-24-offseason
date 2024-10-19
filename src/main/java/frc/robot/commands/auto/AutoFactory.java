package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;

public class AutoFactory extends Command {
  public static final PIDConstants kLinearPID = new PIDConstants(2.5, 0.0, 0.0);
  public static final PIDConstants kAngularPID = new PIDConstants(1.2, 0.0, 0.0);

  private final Drive m_drive;

  public AutoFactory(Drive drive) {
    m_drive = drive;

    NamedCommands.registerCommand(
        "Idle",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().updateRobotAction(RobotAction.kAutoDefault);
            }));

    NamedCommands.registerCommand(
        "Intake",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().updateRobotAction(RobotAction.kIntake);
            }));

    NamedCommands.registerCommand(
        "RevAndAlign",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().updateRobotAction(RobotAction.kRevAndAlign);
            }));

    NamedCommands.registerCommand("AutoShoot", new AutoShoot());

    AutoBuilder.configureHolonomic(
        m_drive::getPose, // Robot pose supplier
        (Pose2d pose) -> {},
        m_drive::getDesiredChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_drive::setDesiredChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            kLinearPID, // Translation PID constants
            kAngularPID, // Rotation PID constants
            5.6, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(
                true, true, 1.7,
                1) // default path replanning config. See the API for the options here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_drive // The subsystem that will be used to follow the path
        );

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public Command getAutoCommand(String name) {
    Command autoCommand = AutoBuilder.buildAuto(name);
    return autoCommand.andThen(Commands.runOnce(() -> m_drive.stopWithX()));
  }
}
