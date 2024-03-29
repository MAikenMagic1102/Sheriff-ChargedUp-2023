package frc.robot.subsystems.Drive;

import frc.lib.util.GamePiece;
import frc.lib.util.GamePiece.GamePieceType;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.sql.Driver;

import javax.swing.GroupLayout.Alignment;

import org.opencv.core.Mat;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
        private SwerveDriveOdometry swerveOdometry;
        private SwerveModule[] mSwerveMods;
        private Pigeon2 m_gyro;

        private Field2d m_field;
        private PIDController m_balancePID = new PIDController(Constants.Swerve.GAINS_BALANCE.kP, Constants.Swerve.GAINS_BALANCE.kI, Constants.Swerve.GAINS_BALANCE.kD);
        private SwerveAutoBuilder m_autoBuilder;

        private Vision eyes;

        public Swerve() {
            m_gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.CANbus);
            m_gyro.configFactoryDefault();
            zeroGyro();
            m_field = new Field2d();
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
            };
    
            /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
             * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
             */
            Timer.delay(1.0);
            resetModulesToAbsolute();
    
            swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
            eyes = new Vision(this);
        }
    
        @Override
        public void periodic(){
            // if(DriverStation.isDisabled()){
            //     resetModulesToAbsolute();
            // }
            // for(SwerveModule mod : mSwerveMods){
            //     mod.putToTempDashboard();
            // } 
    
            m_balancePID.setTolerance(Constants.Swerve.BALANCE_TOLLERANCE);
            //double pidOutput = m_balancePID.calculate(getRoll(), 0);
            // SmartDashboard.putNumber("Balance PID", pidOutput);
            // SmartDashboard.putNumber("Robot Pitch", getPitch());
            // SmartDashboard.putNumber("Robot Roll", getRoll());
    
            SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());
            SmartDashboard.putNumber("Swerve Estimated Pos", eyes.getCurrentPose().getY());
    
            swerveOdometry.update(getYaw(), getModulePositions()); 
    
            //m_field.setRobotPose(swerveOdometry.getPoseMeters());
            Pose2d fieldRalativePos = getPose();
            
            if(DriverStation.getAlliance() == Alliance.Red)
                m_field.setRobotPose(fieldRalativePos.relativeTo(Constants.RedOrigin));

            if(DriverStation.getAlliance() == Alliance.Blue)
                m_field.setRobotPose(fieldRalativePos.relativeTo(Constants.BlueOrigin));
    
            SmartDashboard.putData("Field Swerve Odom", m_field);
            if(Constants.tuningMode){
                SmartDashboard.putNumber("gyro Yaw", getYaw().getDegrees());
    
                for(SwerveModule mod : mSwerveMods){
                    SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                } 
            }
        }

        public void drive(ChassisSpeeds speeds){
            SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
            setModuleStates(swerveModuleStates);
        }
    
        // public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        //     SwerveModuleState[] swerveModuleStates =
        //     Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        //             fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        //                                 translation.getX(), 
        //                                 translation.getY(), 
        //                                 rotation, 
        //                                 swerveOdometry.getPoseMeters().getRotation()
        //                             )
        //                             : new ChassisSpeeds(
        //                                 translation.getX(), 
        //                                 translation.getY(), 
        //                                 rotation)
        //                             );
        //     SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    
        //     for(SwerveModule mod : mSwerveMods){
        //         mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        //     }
    
        //     // SmartDashboard.putNumber("X Translation", translation.getX());
        //     // SmartDashboard.putNumber("Y Translation", translation.getY());
        //     // SmartDashboard.putNumber("Rotation Value", rotation);
        // }    

            /**
     * Moves the swerve drive train, creates twist2d that accounts for the fact that positions are
     * not updated continuously, (updated every .02 seconds.)
     *
     * @param translation The 2d translation in the X-Y plane
     * @param rotation The amount of rotation in the Z axis
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param isOpenLoop Open or closed loop system
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        double dt = TimedRobot.kDefaultPeriod;

        ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, swerveOdometry.getPoseMeters().getRotation())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * dt, chassisSpeeds.vyMetersPerSecond * dt,
            Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * dt));
        Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
        ChassisSpeeds updated = new ChassisSpeeds(twist_vel.dx / dt, twist_vel.dy / dt, twist_vel.dtheta / dt);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(updated);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
        public void stopDrive(){
            drive(new Translation2d(0, 0), 0, false, true);
        }
    
    
        /* Used by SwerveControllerCommand in Auto */
        public void setModuleStates(SwerveModuleState[] desiredStates) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            }
        }    
    
        public Pose2d getPose() {
            
            return swerveOdometry.getPoseMeters();
        }

        public Pose2d getVisionPose(){
            return eyes.getCurrentPose();
        }
    
        public void resetOdometry(Pose2d pose) {
            swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        }
    
        public SwerveModuleState[] getModuleStates(){
            SwerveModuleState[] states = new SwerveModuleState[4];
            for(SwerveModule mod : mSwerveMods){
                states[mod.moduleNumber] = mod.getState();
            }
            return states;
        }
    
        public SwerveModulePosition[] getModulePositions(){
            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            for(SwerveModule mod : mSwerveMods){
                positions[mod.moduleNumber] = mod.getPosition();
            }
            return positions;
        }
    
        public void zeroGyro(){
            m_gyro.setYaw(0);
            // swerveOdometry.resetPosition(getYaw(), getModulePositions(), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        }
    
        public void setYaw(double yaw){
            m_gyro.setYaw(yaw);
        }
    
        public Rotation2d getYaw() {
            return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) : Rotation2d.fromDegrees(m_gyro.getYaw());
        }
    
        public double getPitch(){
            return m_gyro.getPitch();
        }
    
        public double getRoll(){
            return m_gyro.getRoll();
        }
    
       
        public void resetModulesToAbsolute(){
            for(SwerveModule mod : mSwerveMods){
                mod.resetToAbsolute();
            }
        }

        public void setSwervetoRightCAM(){
            swerveOdometry.resetPosition(getYaw(), getModulePositions(), eyes.getRightCamPose());
        }

        public Pose2d getNearestSubstation() {
            double currentY = this.getVisionPose().getY();
            double closestY = 0;
            if(DriverStation.getAlliance() == Alliance.Blue) {
                closestY = Constants.Swerve.BLUEsubstationcoord[0];
                int i = 1;
                    if(Math.abs(Constants.Swerve.BLUEsubstationcoord[i] - currentY) < Math.abs(closestY - currentY)){
                        System.out.println("Closest Y Coord: " + closestY);
                        closestY = Constants.Swerve.BLUEsubstationcoord[i];
                    }
                
                return new Pose2d(new Translation2d(15.4, closestY), Rotation2d.fromDegrees(0));
            }
            else {
                closestY = Constants.Swerve.REDsubstationcoord[0];
                int i = 1;
                    if(Math.abs(Constants.Swerve.REDsubstationcoord[i] - currentY) < Math.abs(closestY - currentY)){
                        System.out.println("Closest Y Coord: " + closestY);
                        closestY = Constants.Swerve.REDsubstationcoord[i];
                    }
                return new Pose2d(new Translation2d(15.4, closestY), Rotation2d.fromDegrees(0));
            }
        }

        public Pose2d getNearestGridPose(){
            //Check the cameras to see where we are
            double currentY = this.getVisionPose().getY();
            double closestY = 0;

            if(GamePiece.getGamePiece() == GamePieceType.Cube){
                //set the default starting position for Y
                if(DriverStation.getAlliance() == Alliance.Blue){
                     closestY = Constants.Swerve.BLUEcubeYcoord[0];
                    for(int i = 1; i < Constants.Swerve.BLUEcubeYcoord.length; i++){
                        if(Math.abs(Constants.Swerve.BLUEcubeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                            System.out.println("Closest Y Coord: " + closestY);
                            closestY = Constants.Swerve.BLUEcubeYcoord[i];
                        }
                    }
                    return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
                }
                
                //Here we will write Red
                if(DriverStation.getAlliance() == Alliance.Red){
                    closestY = Constants.Swerve.REDcubeYcoord[0];
                    for(int i = 1; i< Constants.Swerve.REDcubeYcoord.length; i++){
                        if(Math.abs(Constants.Swerve.REDcubeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                            closestY = Constants.Swerve.REDcubeYcoord[i];
                        }
                    }
                    return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
                }           
            }

            if(GamePiece.getGamePiece() == GamePieceType.Cone){
                //set the default starting position for Y
                if(DriverStation.getAlliance() == Alliance.Blue){
                    closestY = Constants.Swerve.BLUEconeYcoord[0];
                   for(int i = 1; i < Constants.Swerve.BLUEconeYcoord.length; i++){
                       if(Math.abs(Constants.Swerve.BLUEconeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                           System.out.println("Closest Y Coord: " + closestY);
                           closestY = Constants.Swerve.BLUEconeYcoord[i];
                       }
                   }
                   return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
               }
               
               //Here we will write Red
               if(DriverStation.getAlliance() == Alliance.Red){
                   closestY = Constants.Swerve.REDconeYcoord[0];
                   for(int i = 1; i< Constants.Swerve.REDconeYcoord.length; i++){
                       if(Math.abs(Constants.Swerve.REDconeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                           closestY = Constants.Swerve.REDconeYcoord[i];
                       }
                   }
                   return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
               }     
            }

            //Defaults to the middle-ish of the grid but this shouldn't happen
            return new Pose2d(new Translation2d(2.3, 3.0), Rotation2d.fromDegrees(180));

        }

        public Pose2d getSubstationPose(){
            if(DriverStation.getAlliance() == Alliance.Blue){
                return Constants.Swerve.BLUEsubStation;
            }   
            return Constants.Swerve.REDsubStation;
        }
    
        public void AutoBalance(){
            m_balancePID.setTolerance(Constants.Swerve.BALANCE_TOLLERANCE);
            double pidOutput;
            pidOutput = MathUtil.clamp(m_balancePID.calculate(getRoll(), 0), -1.0, 1.0);
            if(Constants.tuningMode){
                SmartDashboard.putNumber("Balance PID", pidOutput);
            }
            drive(new Translation2d(-pidOutput, 0), 0.0, false, true);
        }
    
        public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory path1, boolean isFirstPath) {
            PIDController thetaController = new PIDController(3.5, 0, 0);
            PIDController xController = new PIDController(1.3, 0, 0);
            PIDController yController = new PIDController(1.3, 0, 0);
    
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
            return new SequentialCommandGroup(
                 new InstantCommand(() -> {
                   // Reset odometry for the first path you run during auto
                   if(isFirstPath){
                        PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(path1, DriverStation.getAlliance());
                        resetOdometry(transformed.getInitialHolonomicPose());
                        eyes.resetPathPose(path1);
                   }
                 }),
                 new PPSwerveControllerCommand(
                     path1, 
                     this::getPose, // Pose supplier
                     Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                     xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                     yController, // Y controller (usually the same values as X controller)
                     thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                     this::setModuleStates,  // Module states consumer
                     true, //Automatic mirroring
                     this // Requires this drive subsystem
                 ) 
                 .andThen(() -> stopDrive())
             );
         }

    }
