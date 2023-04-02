// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Swerve;


public class Vision {
  private final Swerve swerve;

  //increase values to trust the drivetrain less (x,y,theta)
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(0.1));

  private static final Vector<N3> visionStdDevs = VecBuilder.fill(0.9,0.9, 0.9);

  private final SwerveDrivePoseEstimator poseEst;

  private final Field2d field2d;
  private final Field2d leftCam;
  private final Field2d rightCam;

  private final Lock lock = new ReentrantLock();
  

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

      vision_thread();
  }

  public void vision_thread(){
    try{
      new Thread(() -> {
        while(true){
          periodic();
          try {
            Thread.sleep(20);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
        }
      }).start();
    }catch(Exception e){}
  }

  public void periodic() {
    // This method will be called once per scheduler run

    if(DriverStation.getAlliance()==Alliance.Blue){
        var blueLeftResult = LimelightHelpers.getLatestResults(Constants.Limelight.left).targetingResults;
        var blueRightResult = LimelightHelpers.getLatestResults(Constants.Limelight.right).targetingResults;

        Pose2d blueLeftBotPose = blueLeftResult.getBotPose2d_wpiBlue();
        Pose2d blueRightBotPose = blueRightResult.getBotPose2d_wpiBlue();

        double leftTimestamp = Timer.getFPGATimestamp() - (blueLeftResult.latency_capture/1000.0) - (blueLeftResult.latency_pipeline/1000.0);
        double rightTimestamp = Timer.getFPGATimestamp() - (blueRightResult.latency_capture/1000.0) - (blueRightResult.latency_pipeline/1000.0);

        if(blueLeftResult.targets_Fiducials.length > 0){
          leftCam.setRobotPose(blueLeftBotPose.relativeTo(Constants.BlueOrigin));
          poseEst.addVisionMeasurement(blueLeftBotPose, leftTimestamp);
          SmartDashboard.putNumber("Left Timestamp", leftTimestamp);
        }
        
        if(blueRightResult.targets_Fiducials.length > 0){
          rightCam.setRobotPose(blueRightBotPose.relativeTo(Constants.BlueOrigin));
          poseEst.addVisionMeasurement(blueRightBotPose, rightTimestamp);
          SmartDashboard.putNumber("Right Timestamp", rightTimestamp);
        }

        field2d.setRobotPose(getCurrentPose().relativeTo(Constants.BlueOrigin));
    }

    if(DriverStation.getAlliance()==Alliance.Red){
        var redLeftResult = LimelightHelpers.getLatestResults(Constants.Limelight.left).targetingResults;
        var redRightResult = LimelightHelpers.getLatestResults(Constants.Limelight.right).targetingResults;

        Pose2d redLeftBotPose = redLeftResult.getBotPose2d_wpiRed();
        Pose2d redRightBotPose = redRightResult.getBotPose2d_wpiRed();

        double leftTimestamp = Timer.getFPGATimestamp() - (redLeftResult.latency_capture/1000.0) - (redLeftResult.latency_pipeline/1000.0);
        double rightTimestamp = Timer.getFPGATimestamp() - (redRightResult.latency_capture/1000.0) - (redRightResult.latency_pipeline/1000.0);

        if(redLeftResult.targets_Fiducials.length > 0){
          leftCam.setRobotPose(redLeftBotPose.relativeTo(Constants.RedOrigin));
          poseEst.addVisionMeasurement(redLeftBotPose, leftTimestamp);
          SmartDashboard.putNumber("Left Timestamp", leftTimestamp);
        }
        
        if(redRightResult.targets_Fiducials.length > 0){
          rightCam.setRobotPose(redRightBotPose.relativeTo(Constants.RedOrigin));
          poseEst.addVisionMeasurement(redRightBotPose, rightTimestamp);
          SmartDashboard.putNumber("Right Timestamp", rightTimestamp);
        }

        field2d.setRobotPose(getCurrentPose().relativeTo(Constants.RedOrigin));
    }

    poseEst.update(swerve.getYaw(), swerve.getModulePositions());

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

  public Pose2d getRightCamPose(){
    return LimelightHelpers.getBotPose2d_wpiRed(Constants.Limelight.right);
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
