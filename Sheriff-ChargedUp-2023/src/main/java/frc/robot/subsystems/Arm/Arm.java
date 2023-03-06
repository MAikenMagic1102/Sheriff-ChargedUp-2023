// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;

import frc.lib.util.GamePiece;
import frc.lib.util.GamePiece.GamePieceType;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX lowerArm = new TalonFX(Constants.Arm.lowerArmID, Constants.Arm.CANbus);
  private final TalonFX upperArm = new TalonFX(Constants.Arm.upperArmID, Constants.Arm.CANbus);
  private DutyCycleOut lowerOutput = new DutyCycleOut(0);
  private DutyCycleOut upperOutput = new DutyCycleOut(0);

  private TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration upperConfig = new TalonFXConfiguration();

  private boolean FOCenabled = true;
  private boolean BrakeMode = true;
  private boolean isClosedLoop = false;

  private boolean upperArmZeroed = false;

  private boolean resetHoldUpper = false;
  private boolean resetHoldLower = false;
  private double lowerArmHold = 0.0;
  private double upperArmHold = 0.0;

  public Arm() {
    lowerConfig.Feedback.SensorToMechanismRatio = Constants.Arm.lowerArmRatio;
    lowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    lowerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    lowerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 3;
    lowerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    lowerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.01;

    //From Lower to Higher
    lowerConfig.Slot0.kP = 80; // An error of 1 rotation = 80 amps
    lowerConfig.Slot0.kI = 30;
    lowerConfig.Slot0.kD = 25; // A change of 1 rotation per second results in 2 amps output

    // Peak output of 130 amps
    lowerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    lowerConfig.TorqueCurrent.PeakReverseTorqueCurrent = 130;


    upperConfig.Feedback.SensorToMechanismRatio = Constants.Arm.upperArmRatio;
    upperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    upperConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    upperConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.8;

    //From Lower to Higher
    upperConfig.Slot0.kP = 60; // An error of 1 rotation = 80 amps
    upperConfig.Slot0.kI = 10;
    upperConfig.Slot0.kD = 7; // A change of 1 rotation per second results in 2 amps output


    // Peak output of 130 amps
    upperConfig.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    upperConfig.TorqueCurrent.PeakReverseTorqueCurrent = 130;

    var defaultConfig = new TalonFXConfiguration();
    TalonFXConfigurator lowerCfg = lowerArm.getConfigurator();
    TalonFXConfigurator upperCfg = upperArm.getConfigurator();

    lowerCfg.apply(defaultConfig);
    upperCfg.apply(defaultConfig);
    Timer.delay(0.5);
    
    upperCfg.apply(upperConfig);
    lowerCfg.apply(lowerConfig);

    lowerArm.setRotorPosition(0);
    upperArm.setRotorPosition(0);
  }

  public void initArm(){
    if(!upperArmZeroed && (upperArm.getStatorCurrent().getValue() < 40.0)){
      this.setUpperArmOpenLoop(-0.2);
    }else{
      this.setUpperArmOpenLoop(0.0);
      upperArm.setRotorPosition(0);
      upperArmZeroed = true;
    }
  }

  public void setLowerArmOpenLoop(double demand){
    lowerOutput = new DutyCycleOut(demand, FOCenabled, BrakeMode);
    lowerArm.setControl(lowerOutput);
  }

  public void setLowerArmSetPoint(double setPoint){
    isClosedLoop = true;


      PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(setPoint, 4, 0, BrakeMode);
      lowerArm.setControl(m_torquePosition);

    // if(lowerArmHold > setPoint){
    //   PositionTorqueCurrentFOC m_torquePosition1 = new PositionTorqueCurrentFOC(setPoint, 0, 1, BrakeMode);
    //   lowerArm.setControl(m_torquePosition1);
    // }
    lowerArmHold = setPoint;
  } 

  public void setUpperArmSetPoint(double setPoint){
    isClosedLoop = true;


      PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(setPoint, 0, 0, BrakeMode);
      upperArm.setControl(m_torquePosition);

    // if(lowerArmHold > setPoint){
    //   PositionTorqueCurrentFOC m_torquePosition1 = new PositionTorqueCurrentFOC(setPoint, 0, 1, BrakeMode);
    //   upperArm.setControl(m_torquePosition1);
    // }
    upperArmHold = setPoint;
  } 

  public void setUpperArmOpenLoop(double demand){
    upperOutput = new DutyCycleOut(demand, FOCenabled, BrakeMode);
    upperArm.setControl(upperOutput);
  }

  public void setArmsOpenLoop(double upperDemand, double lowerDemand){
    if(!upperArmZeroed){
      this.initArm();
    }else{
      if(Math.abs(upperDemand) > Constants.stickDeadband){
        if(this.getUpperArmPosition() < 0.03 && upperDemand < 0){
          this.setUpperArmOpenLoop(0.0);
        }else{
          this.setUpperArmOpenLoop(upperDemand);
          resetHoldUpper = true;
        }
      }else{
        if(resetHoldUpper){
          upperArmHold = this.getUpperArmPosition();
          resetHoldUpper = false;
        }

        if(!resetHoldUpper){
          this.setUpperArmSetPoint(upperArmHold);
        }else{
          this.setUpperArmOpenLoop(0.0);
        }
      }


      if(Math.abs(lowerDemand) > Constants.stickDeadband){
        this.setLowerArmOpenLoop(lowerDemand);
        resetHoldLower = true;
      }else{
        if(resetHoldLower){
          lowerArmHold = this.getLowerArmPosition();
          resetHoldLower = false;
        }

        if(!resetHoldLower){
          this.setLowerArmSetPoint(lowerArmHold);
        }else{
          this.setLowerArmOpenLoop(0.0);
        }
      }
    }
  }

  public double getLowerArmPosition(){
    return lowerArm.getPosition().getValue();
  }

  public double getUpperArmPosition(){
    return upperArm.getPosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Arm Init", upperArmZeroed);
    SmartDashboard.putNumber("Upper Arm Holdpoint", upperArmHold);
    SmartDashboard.putNumber("Lower Arm Holdpoint", lowerArmHold);

    SmartDashboard.putNumber("Lower Arm Sensor POS", lowerArm.getPosition().getValue());
    SmartDashboard.putNumber("Upper Arm Sensor POS", upperArm.getPosition().getValue());

    SmartDashboard.putNumber("Lower Arm Current Draw", lowerArm.getTorqueCurrent().getValue());
    SmartDashboard.putNumber("Upper Arm Current Draw", upperArm.getTorqueCurrent().getValue());
    SmartDashboard.putNumber("Upper Arm Stator Current", upperArm.getStatorCurrent().getValue());

    SmartDashboard.putBoolean("Cone", GamePiece.getGamePiece() == GamePieceType.Cone);
    SmartDashboard.putBoolean("Cube", GamePiece.getGamePiece() == GamePieceType.Cube);
    if(GamePiece.getGamePiece() != null){
      SmartDashboard.putNumber("Gamepiece Ordinal", GamePiece.getGamePiece().ordinal());
    }
    if(isClosedLoop){
      SmartDashboard.putNumber("Lower Arm Error", lowerArm.getClosedLoopError().getValue());
    }
  }
}
