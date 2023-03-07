// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.GamePiece;
import frc.lib.util.Node;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmSetpoint;

public class ArmToNode extends CommandBase {
  /** Creates a new ArmToNode. */
  private int level = 0;
  private Arm m_Arm;

  private double lowerArmSetpoint = 0.0;
  private double upperArmSetpoint = 0.0;

  private boolean isFirstRun = true;
  private boolean isFinished = false;

  public ArmToNode(Arm arm, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    this.level = level;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var setpoint = Constants.Arm.ArmMap.get(new Node(level, GamePiece.getGamePiece().ordinal()));
    lowerArmSetpoint = setpoint.lowerArmSetpoint;
    //System.out.println("Lower Arm setpoint is : " + lowerArmSetpoint);
    upperArmSetpoint = setpoint.upperArmSetpoint;
    //System.out.println("Upper Arm setpoint is : " + upperArmSetpoint);
    isFirstRun = true;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isFirstRun){
      m_Arm.setUpperArmSetPoint(Constants.Arm.RETRACT);
      //System.out.println("Upper Arm is set to : " + Constants.Arm.RETRACT);
      isFirstRun = false;
      //System.out.println("IsFirstRun: " + isFirstRun);
    }


    if(Math.abs(Constants.Arm.RETRACT - m_Arm.getUpperArmPosition()) < 0.3){
      m_Arm.setLowerArmSetPoint(lowerArmSetpoint);
      //System.out.println("Lower Arm is set to : " + lowerArmSetpoint);
    }

    if(Math.abs(lowerArmSetpoint - m_Arm.getLowerArmPosition()) < 0.3){
      m_Arm.setUpperArmSetPoint(upperArmSetpoint);
      //System.out.println("Upper Arm is set to : " + upperArmSetpoint);
    }

    if((Math.abs(upperArmSetpoint - m_Arm.getUpperArmPosition()) < 0.3) && (Math.abs(lowerArmSetpoint - m_Arm.getLowerArmPosition()) < 0.3)){
      //System.out.println("Complete Exiting Command");
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
