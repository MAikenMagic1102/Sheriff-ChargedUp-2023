// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX lowerArm = new TalonFX(Constants.Arm.lowerArmID, Constants.Arm.CANbus);
  private final TalonFX upperArm = new TalonFX(Constants.Arm.upperArmID, Constants.Arm.CANbus);
  private DutyCycleOut lowerOutput = new DutyCycleOut(0);
  private DutyCycleOut upperOutput = new DutyCycleOut(0);

  TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
  TalonFXConfiguration upperConfig = new TalonFXConfiguration();

  private boolean FOCenabled = true;
  private boolean BrakeMode = true;

  public Arm() {
    lowerConfig.Feedback.SensorToMechanismRatio = Constants.Arm.lowerArmRatio;
    lowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    lowerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    lowerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 3;
    lowerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    lowerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.01;

    upperConfig.Feedback.SensorToMechanismRatio = Constants.Arm.upperArmRatio;
    upperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    upperConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    upperConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.8;
    upperConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    upperConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.01;

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

  public void lowerArmOpenLoop(double demand){
    lowerOutput = new DutyCycleOut(demand * 0.4, FOCenabled, BrakeMode);
    lowerArm.setControl(lowerOutput);
  }

  public void upperArmOpenLoop(double demand){
    upperOutput = new DutyCycleOut(demand * 0.2, FOCenabled, BrakeMode);
    upperArm.setControl(upperOutput);
  }

  public void armsOpenLoop(double upperDemand, double lowerDemand){
    this.lowerArmOpenLoop(lowerDemand);
    this.upperArmOpenLoop(upperDemand);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lower Arm Sensor POS", lowerArm.getPosition().getValue());
    SmartDashboard.putNumber("Upper Arm Sensor POS", upperArm.getPosition().getValue());

    SmartDashboard.putNumber("Lower Arm Current Draw", lowerArm.getTorqueCurrent().getValue());
    SmartDashboard.putNumber("Upper Arm Current Draw", upperArm.getTorqueCurrent().getValue());
  }
}
