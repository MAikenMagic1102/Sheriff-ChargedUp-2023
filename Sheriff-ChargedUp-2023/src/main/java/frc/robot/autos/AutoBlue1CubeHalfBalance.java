package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.ArmToNode;
import frc.robot.commands.ArmToSetpoint;
import frc.robot.commands.Score;
import frc.robot.subsystems.CubeKicker;
import frc.robot.subsystems.DigitalServo;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoBlue1CubeHalfBalance extends SequentialCommandGroup {
    public AutoBlue1CubeHalfBalance(Swerve s_Swerve, Arm a_Arm, Intake i_Intake, CubeKicker lilKick){
        PathPlannerTrajectory test = PathPlanner.loadPath("1102TestBlue", 5.0, 3.0);
        PathPlannerTrajectory testAq = PathPlanner.loadPath("1102TestAquireGamepiece", 2.0, 1.5);
        PathPlannerTrajectory test2 = PathPlanner.loadPath("1102TestReturnBalance", 1.5, 1.5);
        
            addRequirements(s_Swerve, a_Arm, i_Intake);
            addCommands(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> lilKick.fire()),
                        new WaitCommand(0.3),
                        s_Swerve.followTrajectoryCommand(test, true).alongWith(new ArmToSetpoint(a_Arm, Constants.Arm.FLOORLOAD).alongWith(new InstantCommand(() -> i_Intake.intakeIn()).alongWith(new InstantCommand(() -> lilKick.home())))),
                        s_Swerve.followTrajectoryCommand(testAq, false),
                        s_Swerve.followTrajectoryCommand(test2, false).alongWith(new ArmToSetpoint(a_Arm, Constants.Arm.STOW)),
                        new RepeatCommand(new InstantCommand(() -> s_Swerve.AutoBalance()))                             
                    ))
    
            );
    }
}