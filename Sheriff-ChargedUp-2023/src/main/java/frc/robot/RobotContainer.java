package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.util.GamePiece;
import frc.lib.util.Node;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int lowerArmAxis = XboxController.Axis.kLeftY.value;
    private final int upperArmAxis = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driver.getHID(), XboxController.Button.kLeftBumper.value);

    /* Operator Buttons */
 
    public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    public static final int BlockButton = XboxController.Button.kStart.value;
    public static final int MaxBrightnessAngle = 90;
    public static final int MidBrightnessAngle = 180;
    public static final int ZeroBrightnessAngle = 270;
    // public static final int VbatButton = XboxController.Button.kA.value;
    // public static final int V5Button = XboxController.Button.kB.value;
    // public static final int CurrentButton = XboxController.Button.kX.value;
    // public static final int TemperatureButton = XboxController.Button.kY.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    //private final CANdleSystem m_candleSubsystem = new CANdleSystem(driver.getHID());
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final CubeKicker cubekicker = new CubeKicker();

    private SendableChooser<Command> m_autoChooser = new SendableChooser<>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        intake.keepHold = false;

        m_autoChooser.addOption("BLUE 1 Cube AqCube Balance", new AutoBlue1CubeHalfBalance(s_Swerve, arm, intake, cubekicker));
        m_autoChooser.addOption("BLUE 2 Cube Balance" , new AutoBlue2CubesBalance(s_Swerve, arm, intake, cubekicker));
        m_autoChooser.addOption("1 Cube", new CubeBalance(s_Swerve, arm, intake, cubekicker));
        m_autoChooser.addOption("RED 2 Cube Balance", new AutoRed2CubesBalance(s_Swerve, arm, intake, cubekicker));
        m_autoChooser.addOption("RED 1 Cube AqCube Balance", new AutoRed1CubeHalfBalance(s_Swerve, arm, intake, cubekicker));
        m_autoChooser.addOption("RED 3 Cube", new AutoRed3Cube(s_Swerve, arm, intake, cubekicker));
        m_autoChooser.addOption("No Auto", null);
        SmartDashboard.putData("Auto", m_autoChooser);

        s_Swerve.setDefaultCommand(
            new TeleopSwerveWin(
                s_Swerve, 
                () -> -driver.getLeftY(), //translation
                () -> -driver.getLeftX(), //strafe
                () -> -driver.getRightX(), //rotation
                driver.leftTrigger(), //robot centric 
                driver.leftBumper(), //50% speed
                driver.rightBumper(), //25% speed
                driver.y(), //face away
                driver.b(), // face right
                driver.a(), //face towards
                driver.x(), //face left
                driver.rightTrigger()
            )
        );

        //Left thumbstick moves upper arm and right thumbstick moves lower arm
        arm.setDefaultCommand(new RunCommand(() -> arm.setArmsOpenLoop(operator.getLeftY(), operator.getRightY()), arm));

        intake.setDefaultCommand(new RunCommand(() -> intake.setholdPosition(), intake));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        
        driver.povUp().onTrue((new InstantCommand(() -> s_Swerve.zeroGyro())));
        driver.start().whileTrue(Commands.run(s_Swerve::AutoBalance, s_Swerve).andThen(s_Swerve::stopDrive, s_Swerve));

        // driver.povLeft().onTrue(new InstantCommand(() -> servo.set(0)));
        // driver.povDown().onTrue(new InstantCommand(() -> servo.set(0.05)));
        // driver.povRight().onTrue(new InstantCommand(() -> servo.set(0.3)));
        //operator.a().whileTrue((new RepeatCommand(new InstantCommand(() -> arm.setLowerArmSetPoint(2.0)))));

        //Start Button Pressed; Change Gamepiece
        operator.start().onTrue(new InstantCommand(() -> GamePiece.toggleGamePiece()));
        //Up arrow pressed; arm goes to substation height and turns the intake on
        operator.povUp().onTrue(new ArmToNode(arm, 4).andThen(new InstantCommand(() -> intake.intakeIn())));
        //Down button pressed; Sets arm to stow position
        operator.povDown().onTrue(new ArmToSetpoint(arm, Constants.Arm.STOW));
        //A button pressed; Sets arm to level 1
        operator.a().onTrue(new ArmToNode(arm, 1));
        //B button pressed; sets arm to level 2
        operator.b().onTrue(new ArmToNode(arm, 2));
        //Y button pressed; sets arm to level 3
        operator.y().onTrue(new ArmToNode(arm, 3));
        //Right trigger pressed; sets arm to floor level and turns on intake
        operator.rightTrigger().onTrue(new ArmToSetpoint(arm, Constants.Arm.FLOORLOAD).andThen(new InstantCommand(() -> intake.intakeIn())));
        //Left trigger pressed; sets intake to score then sets arm to stow position
        operator.leftTrigger().onTrue(new Score(arm, intake).andThen(new ArmToSetpoint(arm, Constants.Arm.STOW)));
        //Right bumber pressed; intake in
        operator.rightBumper().whileTrue(new RepeatCommand(new InstantCommand(() -> intake.intakeIn())));
        //Left bumper pressed; intake out
        operator.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(() -> intake.intakeOut())));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoChooser.getSelected();
    }
}
