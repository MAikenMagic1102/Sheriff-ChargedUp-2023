// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.Vision;

public class PIDdriveToPoint extends CommandBase {
  private final Swerve swerve;
  private final Vision vision;
  private final Pose2d target;

  private final PIDController xController = new PIDController(1.3, 0, 0);
  private final PIDController yController = new PIDController(1.3, 0, 0);
  private final PIDController thetaController = new PIDController(3, 0, 0);
  /** Creates a new PIDdriveToPoint. */
  public PIDdriveToPoint(Swerve swerve, Vision vision, Pose2d target) {
    this.swerve = swerve;
    this.vision = vision;
    this.target = target;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(Rotation2d.fromDegrees(0.5).getRadians());
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
    thetaController.setSetpoint(target.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xController.calculate(vision.getCurrentPose().getX()),
        yController.calculate(vision.getCurrentPose().getY()),
        thetaController.calculate(vision.getCurrentPose().getRotation().minus(target.getRotation()).getRadians(),0),
        vision.getCurrentPose().getRotation()
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
