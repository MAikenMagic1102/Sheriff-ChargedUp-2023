// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax leftIntake = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);
  private CANSparkMax rightIntake = new CANSparkMax(Constants.Intake.rightIntakeID, MotorType.kBrushless);

  private SparkMaxPIDController PIDIntake1;
  private SparkMaxPIDController PIDIntake2;

  private RelativeEncoder intake1Enc;
  private RelativeEncoder intake2Enc;

  private boolean resetHoldPOS = false;
  private boolean hasGamepiece = false;

  double leftPOS = 0;
  double rightPOS = 0;

  boolean keepHold;


  public Intake() {
    intake1Enc = leftIntake.getEncoder();
    intake2Enc = rightIntake.getEncoder();

    PIDIntake1 = leftIntake.getPIDController();
    PIDIntake2 = rightIntake.getPIDController();

    leftIntake.setSmartCurrentLimit(Constants.Intake.smartCurrentLimit);
    rightIntake.setSmartCurrentLimit(Constants.Intake.smartCurrentLimit);

    leftIntake.setInverted(true);
    rightIntake.setInverted(false);

    leftIntake.setIdleMode(IdleMode.kBrake);
    rightIntake.setIdleMode(IdleMode.kBrake);

    PIDIntake1.setP(Constants.Intake.positionkP);
    PIDIntake2.setP(Constants.Intake.positionkP);
  }

  public void setholdPosition(){
    if(keepHold){
      PIDIntake1.setReference(0.6, ControlType.kDutyCycle);
      PIDIntake2.setReference(0.6, ControlType.kDutyCycle);
    }else{
      PIDIntake1.setReference(0.0, ControlType.kDutyCycle);
      PIDIntake2.setReference(0.0, ControlType.kDutyCycle);
    }
  }

  public void intakeIn(){
    PIDIntake1.setReference(0.6, ControlType.kDutyCycle);
    PIDIntake2.setReference(0.6, ControlType.kDutyCycle);
    keepHold = true;
  }

  public void intakeOut(){
    PIDIntake1.setReference(-0.1, ControlType.kDutyCycle);
    PIDIntake2.setReference(-0.1, ControlType.kDutyCycle);
    keepHold = false;
  }

  public boolean getHasGamepiece(){
    if((leftIntake.getAppliedOutput() > 0.1 && rightIntake.getAppliedOutput() > 0.1) && (intake1Enc.getVelocity() < 1 && intake2Enc.getVelocity() < 1)){
      hasGamepiece = true;
    }
    if((leftIntake.getAppliedOutput() < 0 && rightIntake.getAppliedOutput() < 0)){
      hasGamepiece = false;
    }
    
    return hasGamepiece;
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake1 Target", leftPOS);
    SmartDashboard.putNumber("Intake2 Target", rightPOS);
    SmartDashboard.putNumber("Intake1 Error", intake1Enc.getPosition() - leftPOS);
    SmartDashboard.putNumber("Intake2 Error", intake2Enc.getPosition() - rightPOS);
    SmartDashboard.putNumber("Intake1 Pos", intake1Enc.getPosition());
    SmartDashboard.putNumber("Intake2 Pos", intake2Enc.getPosition());
    SmartDashboard.putNumber("Intake1 Velocity", intake1Enc.getVelocity());
    SmartDashboard.putNumber("Intake2 Velocity", intake2Enc.getVelocity());
    SmartDashboard.putBoolean("Has Gamepiece", getHasGamepiece());
  }
}
