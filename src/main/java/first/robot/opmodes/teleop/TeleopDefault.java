package first.robot.opmodes.teleop;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;
import first.robot.commands.ExampleCommand;

@Teleop // labeled as teleop, will show up under "Teleoperated" in Driver station
public class TeleopDefault implements OpMode {
    private final Robot robot;

    // Controllers
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    // Triggers
    Trigger exampleTrigger;

    Trigger turretDown;
    Trigger turretUp;

    /* 1e. Trigger bindings
    Create a new Trigger for your shooter's setVoltage method.
    Then, bind it to a controller button.
     */


    /**
     * Put all the triggers in one place to make them easier to find/change
     */
    private void configureBindings() {
        // Bind triggers to controller input
        exampleTrigger = driverController.y();

        turretDown = driverController.dpadLeft();
        turretUp = driverController.dpadRight();
    }

    // Default commands and commands bound to triggers can go in here
    private void configureCommands() {
        // Default commands
        robot.drive.setDefaultCommand(
            robot.drive.joystickDrive(
                robot.drive, 
                () -> -driverController.getLeftY(), 
                () -> -driverController.getLeftX(), 
                () -> -driverController.getRightX()));

        // Commands bound to triggers
        exampleTrigger.onTrue(new ExampleCommand());

        turretDown.whileTrue(robot.turret.decrementGoal());
        turretUp.whileTrue(robot.turret.incrementGoal());
    }

    // Constructor, stuff that needs to run when the opmode is selected goes in here
    public TeleopDefault(Robot r) {
        robot = r;

        driverController = robot.driverController;
        operatorController = robot.operatorController;

        configureBindings();
        configureCommands();
    }
}
