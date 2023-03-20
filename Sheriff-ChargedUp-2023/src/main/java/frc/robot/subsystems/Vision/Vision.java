// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Swerve;


public class Vision extends SubsystemBase {
  private final Swerve swerve;

  //increase values to trust the drivetrain less (x,y,theta)
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(0.1));

  private static final Vector<N3> visionStdDevs = VecBuilder.fill(0.9,0.9, Units.degreesToRadians(0.9));
  //Values tried for X/Y 0.1, 1, 3, 5, 10
  //Values tried for Theta 0.1, 500 
  //(I think we want to ignore the camera pose's angles since they will always be wrong compared to gyro)

  private final SwerveDrivePoseEstimator poseEst;

  private final Field2d field2d;
  private final Field2d leftCam;
  private final Field2d rightCam;

  private int leftTags = 0;
  private int rightTags = 0;

  /** Creates a new Vision. */
  public Vision(Swerve swerve) {
    this.swerve = swerve;
    field2d = new Field2d();
    leftCam = new Field2d();
    rightCam = new Field2d();

    poseEst = new SwerveDrivePoseEstimator(
      Constants.Swerve.swerveKinematics, 
      swerve.getYaw(), 
      swerve.getModulePositions(), 
      new Pose2d(),
      stateStdDevs,
      visionStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(DriverStation.getAlliance()==Alliance.Blue){
        Pose2d blueLeftBotPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Limelight.left);
        Pose2d blueRightBotPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Limelight.right);

        Pose2d blueLeftBotNoRot = new Pose2d(new Translation2d(blueLeftBotPose.getX(), blueLeftBotPose.getY()),
        swerve.getYaw());

        Pose2d blueRightBotNoRot = new Pose2d(new Translation2d(blueRightBotPose.getX(), blueRightBotPose.getY()),
        swerve.getYaw());

        if(isInMap(blueLeftBotNoRot) && isValidPose(blueLeftBotNoRot)){
          leftCam.setRobotPose(blueLeftBotNoRot);
          poseEst.addVisionMeasurement(blueLeftBotNoRot, LimelightHelpers.getLatency_Pipeline(Constants.Limelight.left));
          SmartDashboard.putNumber("Left Latancy", LimelightHelpers.getLatency_Pipeline(Constants.Limelight.left));
        }
        
        if(isInMap(blueRightBotNoRot) && isValidPose(blueRightBotNoRot)){
          rightCam.setRobotPose(blueRightBotNoRot);
          poseEst.addVisionMeasurement(blueRightBotNoRot,LimelightHelpers.getLatency_Pipeline(Constants.Limelight.right));
          SmartDashboard.putNumber("Right Latency", LimelightHelpers.getLatency_Pipeline(Constants.Limelight.right));
        }
    }

    // if(DriverStation.getAlliance()==Alliance.Red){
    //     var redLeftBotPose = LimelightHelpers.getBotPose2d_wpiRed(Constants.Limelight.left);
    //     var redRightBotPose = LimelightHelpers.getBotPose2d_wpiRed(Constants.Limelight.right);

    //     leftCam.setRobotPose(redLeftBotPose);
    //     rightCam.setRobotPose(redRightBotPose);


    //     poseEst.addVisionMeasurement(redLeftBotPose, LimelightHelpers.getLatency_Capture(Constants.Limelight.left));
    //     //swerve.resetOdometry(redLeftBotPose);

    //     poseEst.addVisionMeasurement(redRightBotPose, LimelightHelpers.getLatency_Capture(Constants.Limelight.right));
    //     //swerve.resetOdometry(redRightBotPose);
    // }

    poseEst.update(swerve.getYaw(), swerve.getModulePositions());

    field2d.setRobotPose(getCurrentPose());

    SmartDashboard.putData("Field Pose Est", field2d);
    SmartDashboard.putData("Left CAM Field", leftCam);
    SmartDashboard.putData("Right Cam Field", rightCam);
  }

  public Pose2d getCurrentPose(){
    return poseEst.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose){
    poseEst.resetPosition(swerve.getYaw(), swerve.getModulePositions(), newPose);
  }

  public void resetFieldPosition(){
    this.setCurrentPose(new Pose2d());
  }

  //From Spectrum
  public boolean isInMap(Pose2d botPose){
    return ((botPose.getX() > 1.39 && botPose.getX() < 5.01)
    && (botPose.getY() > 0.1 && botPose.getY() < 5.49));
  }

  //From Spectrum
  public boolean isValidPose(Pose2d botPose){
    Pose2d odomPose = swerve.getPose();
    if(odomPose.getX() <= 0.3 && odomPose.getY() <= 0.3 && odomPose.getRotation().getDegrees() <= 1){
      return false;
    }

    return (Math.abs(botPose.getX() - odomPose.getX()) <= 1) && 
    (Math.abs(botPose.getY() - odomPose.getY()) <= 1);
  }
  
  //Sets the pose origin in auton. This works.
  public void resetPathPose(PathPlannerTrajectory path1){
    PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(path1, DriverStation.getAlliance());
    setCurrentPose(transformed.getInitialHolonomicPose());
  }

}
