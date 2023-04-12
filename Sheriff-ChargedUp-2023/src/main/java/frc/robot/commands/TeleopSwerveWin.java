package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.LimelightHelpers;


public class TeleopSwerveWin extends CommandBase {    
    private Swerve m_Swerve;    
    private DoubleSupplier xSup;
    private DoubleSupplier ySup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier m_halfSpeed;
    private BooleanSupplier m_quarterSpeed;
    private BooleanSupplier m_90, m_180, m_270, m_0;
    private BooleanSupplier m_limelightmode;

    private double rotationVal, xVal, yVal;
    double m_angle = 0d;
    private PIDController m_thetaController;
    private SendableChooser<Double> m_speedChooser;
  
    /**
     * 
     * @param s_Swerve
     * @param xSup
     * @param ySup
     * @param rotationSup
     * @param halfSpeed
     * @param quarterSpeed
     */
    public TeleopSwerveWin(Swerve swerve, DoubleSupplier xSup, DoubleSupplier ySup, 
                        DoubleSupplier rotationSup, 
                        BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed, 
                        BooleanSupplier zero, BooleanSupplier ninety, BooleanSupplier oneEighty, BooleanSupplier twoSeventy){
        m_Swerve = swerve;
        this.ySup = ySup;
        this.xSup = xSup;
        this.rotationSup = rotationSup;
        m_halfSpeed = halfSpeed;
        m_quarterSpeed = quarterSpeed;
        m_0 = zero;
        m_180 = oneEighty;
        m_90 = ninety;
        m_270 = twoSeventy;
        addRequirements(m_Swerve);
        m_speedChooser = new SendableChooser<Double>();
        m_speedChooser.addOption("100%", 1.0);
        m_speedChooser.addOption("90%", 0.9);
        m_speedChooser.setDefaultOption("85%", 0.85);
        m_speedChooser.addOption("80%", 0.8);
        m_speedChooser.addOption("70%", 0.7);
        m_speedChooser.addOption("60%", 0.6);
        SmartDashboard.putData("Speed Percent", m_speedChooser);
    }

    @Override 
    public void initialize(){
        m_thetaController = new PIDController(Constants.Swerve.GAINS_ANGLE_SNAP.kP, Constants.Swerve.GAINS_ANGLE_SNAP.kI, Constants.Swerve.GAINS_ANGLE_SNAP.kD);
        
        m_thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        boolean rotateWithButton = m_0.getAsBoolean() || m_90.getAsBoolean() || m_180.getAsBoolean() || m_270.getAsBoolean();
        xVal = MathUtil.applyDeadband(xSup.getAsDouble() * m_speedChooser.getSelected() , Constants.stickDeadband);
        yVal = MathUtil.applyDeadband(ySup.getAsDouble() * m_speedChooser.getSelected(), Constants.stickDeadband);
        // SmartDashboard.putBoolean("rotate with button", rotateWithButton);
        
        if(rotateWithButton){
            if(m_0.getAsBoolean()){
                m_thetaController.setSetpoint(0.0);
            }
            else if(m_90.getAsBoolean()){
                m_thetaController.setSetpoint(-90.0);
            }
            else if(m_180.getAsBoolean()){
                m_thetaController.setSetpoint(180.0);
            }
            else if(m_270.getAsBoolean()){
                m_thetaController.setSetpoint(90.0);
            }
            rotationVal = m_thetaController.calculate((MathUtil.inputModulus(m_Swerve.getPose().getRotation().getDegrees(), -180, 180)), m_thetaController.getSetpoint());
            rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxAngularVelocity * 0.075, Constants.Swerve.maxAngularVelocity * 0.075);
            // SmartDashboard.putNumber("RotationVal", rotationVal);
            // SmartDashboard.putNumber("Theta Controller setpoint", m_thetaController.getSetpoint());
        }
        else if (!rotateWithButton){
            rotationVal = (MathUtil.applyDeadband(rotationSup.getAsDouble() * m_speedChooser.getSelected(), Constants.stickDeadband))*0.75;
        }

        if(m_quarterSpeed.getAsBoolean()){
            // xVal = xVal*0.25;
            // yVal =yVal*0.25;
            // if(!rotateWithButton){
            //     rotationVal = rotationVal *0.25;
            // }
            xVal = xVal*0.5;
            yVal =yVal*0.5;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.5;
            }
        }
        else if(m_halfSpeed.getAsBoolean()){
            xVal = xVal*0.5;
            yVal =yVal*0.5;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.5;
            }
        }
        else{
            xVal = xVal*1.0;
            yVal =yVal*1.0;
            if(!rotateWithButton){
                rotationVal = rotationVal *1.0;
            } 
        }

        m_Swerve.drive(
            new Translation2d(xVal, yVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity * 0.9, 
            true,
            false);
    }
}