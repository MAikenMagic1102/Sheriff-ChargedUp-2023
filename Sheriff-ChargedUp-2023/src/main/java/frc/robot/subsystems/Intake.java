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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax leftIntake = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax rightIntake = new CANSparkMax(11, MotorType.kBrushless);

  private SparkMaxPIDController PIDIntake1;
  private SparkMaxPIDController PIDIntake2;

  private RelativeEncoder intake1Enc;
  private RelativeEncoder intake2Enc;

  private boolean resetHoldPOS = false;

  public Intake() {
    intake1Enc = leftIntake.getEncoder();
    intake2Enc = rightIntake.getEncoder();

    PIDIntake1 = leftIntake.getPIDController();
    PIDIntake2 = rightIntake.getPIDController();

    leftIntake.setSmartCurrentLimit(30);
    rightIntake.setSmartCurrentLimit(30);
    leftIntake.setIdleMode(IdleMode.kBrake);
    rightIntake.setIdleMode(IdleMode.kBrake);

    PIDIntake1.setP(0.05);
    PIDIntake2.setP(0.05);
  }

  public void setholdPosition(){
    double leftPOS = 0;
    double rightPOS = 0;
      if(resetHoldPOS){
        leftPOS = intake1Enc.getPosition();
        rightPOS = intake2Enc.getPosition();
        resetHoldPOS = false;
      }
      PIDIntake1.setReference(leftPOS, ControlType.kPosition);
      PIDIntake2.setReference(rightPOS, ControlType.kPosition);
  }

  public void setOpenLoop(double output){
    PIDIntake1.setReference(output, ControlType.kDutyCycle);
    PIDIntake2.setReference(output, ControlType.kDutyCycle);
    resetHoldPOS = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
