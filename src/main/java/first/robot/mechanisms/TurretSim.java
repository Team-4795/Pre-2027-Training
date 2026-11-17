package first.robot.mechanisms;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;

public class TurretSim implements Mechanism {
    private static TurretSim instance;

    // Simulated turret motor (yes i know its not actually single jointed arm)
    private DCMotorSim motor = new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            DCMotor.getKrakenX44(1), 0.005, 40), 
            DCMotor.getKrakenX44(1));

    private ProfiledPIDController controller = new ProfiledPIDController(
        2.0, 0, 0, 
        new TrapezoidProfile.Constraints(12.5, 47.0));
    private SimpleMotorFeedforward ffController = new SimpleMotorFeedforward(0, 0.57);

    // bunch of variables for logging
    private double voltage;
    private double velocity;
    private double acceleration;
    private double current;
    private double position;

    private double pidVolts;
    private double ffVolts;

    private double goalRads;

    private double nextGoal = 0.0;

    // this is the type of stuff we will be doing during the season
    // no public constructor, everything is filtered through this method
    // it will make sense with advantagekit's IO layer, see drive for reference
    public static TurretSim initialize() {
        if (instance == null) instance = new TurretSim();
        return instance;
    }

    public static TurretSim getInstance() {
        return instance;
    }

    private TurretSim() {
        instance = this;
        controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets the voltage of the motor once.
     * @param volts the desired voltage to set
     */
    public Command setVoltage(double volts) {
        // One-shot command, requires no while loop because it's just setting something once
        return run(coroutine -> {
            motor.setInputVoltage(volts);
        }).named("Turret/setVoltage");
    }

    /**
     * Sets the goal position of the turret.
     * @param volts the desired goal, in rotations
     */
    public Command setGoal(double goalRots) {
        // Another one-shot command, so no loop
        return run(coroutine -> {
            goalRads = Units.rotationsToRadians(goalRots);
            controller.setGoal(goalRads);
        }).named("Turret/setGoal");
    }

    /**
     * Increase turret goal by a small amount.
     * @return command that slowly increases the turret goal
     */
    public Command incrementGoal() {
        return run(coroutine -> {
            // use a loop for commands that have to run constantly
            while (true) {
                nextGoal += 0.01;
                controller.setGoal(nextGoal);
                coroutine.yield(); // VERY important
            }
        }).named("Turret/incrementGoal");
    }

    /**
     * Decrease turret goal by a small amount.
     * @return command that slowly decreases the turret goal
     */
    public Command decrementGoal() {
        // alternatively, for simple commands that have to run continuously,
        // use runRepeatedly to avoid the loop
        return runRepeatedly(() -> {
            nextGoal -= 0.01;
            controller.setGoal(nextGoal);
        }).named("Turret/decrementGoal");
    }

    public void periodic() {
        // Update PID calculations
        pidVolts = controller.calculate(motor.getAngularPosition());
        ffVolts = ffController.calculate(motor.getAngularVelocity(), controller.getSetpoint().velocity);

        // Feed the motor voltage from PID+FF calculations
        motor.setInputVoltage(pidVolts + ffVolts);

        // Update the motor sim
        motor.update(0.02);

        // Update values for logging
        position = Units.radiansToRotations(motor.getAngularPosition());
        voltage = motor.getInputVoltage();
        velocity = Units.radiansToRotations(motor.getAngularVelocity());
        acceleration = Units.radiansToRotations(motor.getAngularAcceleration());
        current = motor.getCurrentDraw();

        // Log updated values
        Logger.recordOutput("Turret/Position", position);
        Logger.recordOutput("Turret/Goal", Units.radiansToRotations(goalRads));
        Logger.recordOutput("Turret/Voltage", voltage);
        Logger.recordOutput("Turret/Velocity", velocity);
        Logger.recordOutput("Turret/Acceleration", acceleration);
        Logger.recordOutput("Turret/Current", current);
        Logger.recordOutput("Turret/Setpoint/Position", Units.radiansToRotations(controller.getSetpoint().position));
        Logger.recordOutput("Turret/Setpoint/Velocity", Units.radiansToRotations(controller.getSetpoint().velocity));
        Logger.recordOutput("Turret/PID Volts", pidVolts);
        Logger.recordOutput("Turret/FF Volts", ffVolts);
    }
}
