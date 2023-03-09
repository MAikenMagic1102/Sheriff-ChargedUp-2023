// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.GamePiece;
import frc.lib.util.Node;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmSetpoint;

public class Score extends CommandBase {
  /** Creates a new ArmToNode. */
  private int level = 0;
  private Arm arm;
  private Intake intake;

  private double lowerArmCurrentSetpoint = 0.0;
  private double upperArmCurrentSetpoint = 0.0;

  private boolean isFirstRun = true;
  private boolean isFinished = false;
  private boolean scored = false;

  public Score(Arm arm, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    addRequirements(arm, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFirstRun = true;
    isFinished = false;
    scored = false;
    this.upperArmCurrentSetpoint = arm.getUpperArmPosition();
    this.lowerArmCurrentSetpoint = arm.getLowerArmPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Is First run?: " + isFirstRun);
    if(isFirstRun){
      arm.setLowerArmSetPoint(lowerArmCurrentSetpoint - 0.3);
      //System.out.println("Upper Arm is set to : " + Constants.Arm.RETRACT);
      isFirstRun = false;
      //System.out.println("IsFirstRun: " + isFirstRun);
    }


    if(Math.abs((lowerArmCurrentSetpoint - 0.3) - arm.getLowerArmPosition() ) < 0.2){
      intake.intakeOut();
      scored = true;
      //System.out.println("Lower Arm is set to : " + lowerArmSetpoint);
    }


    if(scored){
      Timer.delay(0.3);
      isFinished = true;
    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
