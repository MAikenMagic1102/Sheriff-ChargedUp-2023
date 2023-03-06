// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

/** Add your docs here. */
public class ArmSetpoint {
    public double lowerArmSetpoint;
    public double upperArmSetpoint;
    public ArmSetpoint(double lowerArmSetpoint, double upperArmSetpoint){
        this.lowerArmSetpoint = lowerArmSetpoint;
        this.upperArmSetpoint = upperArmSetpoint;
    }
}
