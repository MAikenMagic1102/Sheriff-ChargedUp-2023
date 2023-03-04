package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    // public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    // public static final int BlockButton = XboxController.Button.kStart.value;
    // public static final int MaxBrightnessAngle = 90;
    // public static final int MidBrightnessAngle = 180;
    // public static final int ZeroBrightnessAngle = 270;
    // public static final int VbatButton = XboxController.Button.kA.value;
    // public static final int V5Button = XboxController.Button.kB.value;
    // public static final int CurrentButton = XboxController.Button.kX.value;
    // public static final int TemperatureButton = XboxController.Button.kY.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final CANdleSystem m_candleSubsystem = new CANdleSystem(driver);
    private final Arm arm = new Arm();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        arm.setDefaultCommand(new RunCommand(() -> arm.armsOpenLoop(operator.getRawAxis(0), operator.getRawAxis(1)), arm));

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // new JoystickButton(driver, BlockButton).whenPressed(m_candleSubsystem::setColors, m_candleSubsystem);
        // new JoystickButton(driver, IncrementAnimButton).whenPressed(m_candleSubsystem::incrementAnimation, m_candleSubsystem);
        // new JoystickButton(driver, DecrementAnimButton).whenPressed(m_candleSubsystem::decrementAnimation, m_candleSubsystem);
    
        // new POVButton(driver, MaxBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 1.0));
        // new POVButton(driver, MidBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0.3));
        // new POVButton(driver, ZeroBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0));
        // new JoystickButton(driver, VbatButton).whenPressed(new CANdlePrintCommands.PrintVBat(m_candleSubsystem));
        // new JoystickButton(driver, V5Button).whenPressed(new CANdlePrintCommands.Print5V(m_candleSubsystem));
        // new JoystickButton(driver, CurrentButton).whenPressed(new CANdlePrintCommands.PrintCurrent(m_candleSubsystem));
        // new JoystickButton(driver, TemperatureButton).whenPressed(new CANdlePrintCommands.PrintTemperature(m_candleSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}