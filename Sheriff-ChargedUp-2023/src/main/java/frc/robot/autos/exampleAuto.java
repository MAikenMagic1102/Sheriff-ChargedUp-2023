package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.ArmToNode;
import frc.robot.commands.ArmToSetpoint;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve, Arm a_Arm, Intake i_Intake){
        PathPlannerTrajectory test = PathPlanner.loadPath("1102Test", 2.0, 2.0);
        PathPlannerTrajectory test2 = PathPlanner.loadPath("1102TestReturn", 2.0, 2.0);
            addRequirements(s_Swerve, a_Arm, i_Intake);
            addCommands(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new ArmToSetpoint(a_Arm, Constants.Arm.FLOORLOAD),
                        new InstantCommand(() -> i_Intake.intakeIn()),
                        s_Swerve.followTrajectoryCommand(test, true),
                        s_Swerve.followTrajectoryCommand(test2, false).alongWith(new ArmToSetpoint(a_Arm, Constants.Arm.STOW)),
                        new ArmToNode(a_Arm, 3),
                        new InstantCommand(() -> i_Intake.intakeOut())
                    ))
    
            );
    }
}