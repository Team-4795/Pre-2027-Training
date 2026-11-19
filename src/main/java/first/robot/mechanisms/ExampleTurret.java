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

public class ExampleTurret implements Mechanism {
    // Simulated turret motor (yes i know its not actually single jointed arm)
    private DCMotorSim turretSim = new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            DCMotor.getKrakenX44(1), 0.005, 40), 
            DCMotor.getKrakenX44(1));

    // PID and feedforward controllers
    private ProfiledPIDController controller = new ProfiledPIDController(
        2.0, 0, 0.2, 
        new TrapezoidProfile.Constraints(12.5, 47.0)); // Maximum amount that the setpoint can change
    private SimpleMotorFeedforward ffController = new SimpleMotorFeedforward(0, 0.6);

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

    public ExampleTurret() {}

    /**
     * Sets the voltage of the motor once.
     * @param volts the desired voltage to set
     */
    public Command setVoltage(double volts) {
        // One-shot command, requires no while loop because it's just setting something once
        return run(coroutine -> {
            turretSim.setInputVoltage(volts);
        }).named("Turret/setVoltage");
    }

    /**
     * Sets the goal position of the turret.
     * @param goalRots the desired goal, in rotations
     */
    public Command setGoal(double goalRots) {
        // Another one-shot command, so no loop
        return run(coroutine -> {
            nextGoal = goalRots;
            goalRads = Units.rotationsToRadians(goalRots);
            controller.setGoal(goalRads);
        }).named("Turret/setGoal");
    }

    /**
     * Sets the goal position of the turret and waits until the turret reaches the goal position before finishing.
     * @param goalRots the desired goal, in rotations
     */
    public Command setGoalAndWait(double goalRots) {
        return run(coroutine -> {
            while (Math.abs(goalRots - Units.radiansToRotations(turretSim.getAngularPosition())) > 0.01) {
                coroutine.await(setGoal(goalRots));
                coroutine.yield();
            }
        }).named("Turret/setGoalAndWait");
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
                coroutine.await(setGoal(nextGoal));
                coroutine.yield(); // VERY important
            }
        }).named("Turret/incrementGoal");
    }

    /**
     * Decrease turret goal by a small amount.
     * @return command that slowly decreases the turret goal
     */
    public Command decrementGoal() {
        return run(coroutine -> {
            while (true) {
                nextGoal -= 0.01;
                coroutine.await(setGoal(nextGoal));
                coroutine.yield();
            }
        }).named("Turret/decrementGoal");
    }

    public void periodic() {
        // Update PID calculations
        pidVolts = controller.calculate(turretSim.getAngularPosition());

        // Update FF calculations
        ffVolts = ffController.calculate(turretSim.getAngularVelocity(), controller.getSetpoint().velocity);

        // Feed the motor voltage from PID + FF calculations
        turretSim.setInputVoltage(pidVolts + ffVolts);

        // Update the motor sim
        turretSim.update(0.02);

        // Update values for logging
        position = Units.radiansToRotations(turretSim.getAngularPosition());
        voltage = turretSim.getInputVoltage();
        velocity = Units.radiansToRotations(turretSim.getAngularVelocity());
        acceleration = Units.radiansToRotations(turretSim.getAngularAcceleration());
        current = turretSim.getCurrentDraw();

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
