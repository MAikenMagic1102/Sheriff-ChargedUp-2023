// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.GamePiece;
import frc.lib.util.GamePiece.GamePieceType;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax leftIntake = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);
  private CANSparkMax rightIntake = new CANSparkMax(Constants.Intake.rightIntakeID, MotorType.kBrushless);
  private CANSparkMax hRollerIntake = new CANSparkMax(Constants.Intake.hRollerID, MotorType.kBrushless);

  private final CANdle m_candle = new CANdle(Constants.CANdleID);
  private final int LedCount = 208;

  private SparkMaxPIDController PIDIntake1;
  private SparkMaxPIDController PIDIntake2;

  private RelativeEncoder intake1Enc;
  private RelativeEncoder intake2Enc;

  private boolean resetHoldPOS = false;
  private boolean hasGamepiece = false;

  private double leftPOS = 0;
  private double rightPOS = 0;

  public boolean keepHold;

  private SparkMaxLimitSwitch leftLimit;
  private SparkMaxLimitSwitch rightLimit;


  public Intake() {
    intake1Enc = leftIntake.getEncoder();
    intake2Enc = rightIntake.getEncoder();

    PIDIntake1 = leftIntake.getPIDController();
    PIDIntake2 = rightIntake.getPIDController();

    leftLimit = leftIntake.getForwardLimitSwitch(Type.kNormallyClosed);
    rightLimit = rightIntake.getForwardLimitSwitch(Type.kNormallyClosed);

    leftLimit.enableLimitSwitch(false);
    rightLimit.enableLimitSwitch(false);

    leftIntake.setSmartCurrentLimit(Constants.Intake.smartCurrentLimit);
    rightIntake.setSmartCurrentLimit(Constants.Intake.smartCurrentLimit);
    hRollerIntake.setSmartCurrentLimit(15);

    leftIntake.setInverted(true);
    rightIntake.setInverted(false);
    hRollerIntake.setInverted(true);

    leftIntake.setIdleMode(IdleMode.kBrake);
    rightIntake.setIdleMode(IdleMode.kBrake);
    hRollerIntake.setIdleMode(IdleMode.kBrake);

    PIDIntake1.setP(Constants.Intake.positionkP);
    PIDIntake2.setP(Constants.Intake.positionkP);

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.On;
  }

  public void setholdPosition(){
    if(keepHold){
        if(!leftLimit.isPressed() && GamePiece.getGamePiece() == GamePieceType.Cube){
          PIDIntake1.setReference(0.05, ControlType.kDutyCycle);
          PIDIntake2.setReference(0.05, ControlType.kDutyCycle);
        }else{
          PIDIntake1.setReference(0.6, ControlType.kDutyCycle);
          PIDIntake2.setReference(0.6, ControlType.kDutyCycle);
        }

        if(!leftLimit.isPressed() || GamePiece.getGamePiece() == GamePieceType.Cone){
          hRollerIntake.set(0.0);
        }else{
            hRollerIntake.set(0.6);
        }
    }else{
      PIDIntake1.setReference(0.0, ControlType.kDutyCycle);
      PIDIntake2.setReference(0.0, ControlType.kDutyCycle);
      hRollerIntake.set(0.0);
    }
  }

  public void intakeIn(){
    PIDIntake1.setReference(0.6, ControlType.kDutyCycle);
    PIDIntake2.setReference(0.6, ControlType.kDutyCycle);
    hRollerIntake.set(0.6);
    keepHold = true;
  }

  public void intakeOut(){
    PIDIntake1.setReference(-0.2, ControlType.kDutyCycle);
    PIDIntake2.setReference(-0.2, ControlType.kDutyCycle);
    hRollerIntake.set(-0.2);
    keepHold = false;
  }

  public void intakeOutFast(){
    PIDIntake1.setReference(-0.6, ControlType.kDutyCycle);
    PIDIntake2.setReference(-0.6, ControlType.kDutyCycle);
    hRollerIntake.set(-0.6);
    keepHold = false;
  }

  public boolean getHasGamepiece(){
    if(keepHold && (intake1Enc.getVelocity() < 2000 || intake2Enc.getVelocity() < 2000 || !leftLimit.isPressed())){
      hasGamepiece = true;
    }else{
      hasGamepiece = false;
    }
    
    return hasGamepiece;
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Intake1 Target", leftPOS);
    // SmartDashboard.putNumber("Intake2 Target", rightPOS);
    // SmartDashboard.putNumber("Intake1 Error", intake1Enc.getPosition() - leftPOS);
    // SmartDashboard.putNumber("Intake2 Error", intake2Enc.getPosition() - rightPOS);
    // SmartDashboard.putNumber("Intake1 Pos", intake1Enc.getPosition());
    // SmartDashboard.putNumber("Intake2 Pos", intake2Enc.getPosition());
    SmartDashboard.putNumber("Intake1 Velocity", intake1Enc.getVelocity());
    SmartDashboard.putNumber("Intake2 Velocity", intake2Enc.getVelocity());
    if(this.getHasGamepiece()){
      m_candle.setLEDs(0, 255, 0);
    }else{
      if(GamePiece.getGamePiece() == GamePieceType.Cone){
        m_candle.setLEDs(255, 255, 0);
      }else{
        if(GamePiece.getGamePiece() == GamePieceType.Cube){
          m_candle.setLEDs(128, 0, 128);
        }
      }
    }

    SmartDashboard.putBoolean("LeftIntake Limit", leftLimit.isPressed());
    SmartDashboard.putBoolean("RightIntake Limit", rightLimit.isPressed());
    SmartDashboard.putBoolean("Has Gamepiece", getHasGamepiece());
  }
}
