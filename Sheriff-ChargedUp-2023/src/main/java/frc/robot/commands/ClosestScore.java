// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.GamePiece;
import frc.lib.util.GamePiece.GamePieceType;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClosestScore extends SequentialCommandGroup {
  /** Creates a new PIDdriveToClosestGrid. */
  Pose2d target;
  public ClosestScore(Swerve swerve, Arm arm) {
    target = swerve.getNearestGridPose();

    addRequirements(swerve);
    addCommands(
      new PIDdriveToNearestGrid(swerve)
    );
  }
}
