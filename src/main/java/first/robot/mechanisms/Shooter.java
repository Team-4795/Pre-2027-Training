package first.robot.mechanisms;

import static org.wpilib.units.Units.Volts;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.units.measure.Voltage;

public class Shooter implements Mechanism {
    private double voltage;
    private double velocity;

    private FlywheelSim shooterSim = 
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(2), 0.002, 0.7), 
            DCMotor.getKrakenX60(2));

    public Shooter() {}

    public Command setVoltage(double volts) {
        return run(coroutine -> {
                shooterSim.setInputVoltage(volts);
        }).named("shooter");
        /* 1a. Finish this command!
        Write a command to set the voltage of the simulated shooter.
         */
    }


    public void periodic() {
        shooterSim.update(0.02);

        /* 1b. Logging
        Update the shooter's velocity and voltage,
        then log them so that you can see it moving in Advantagescope
        Hint: Use Logger (the Littleton Robotics one).
         */
        Shooter.recordOutput("Shooter/Velocity", velocity);
        Shooter.recordOutput("Shooter/Volts", voltage);
        
    }
    
}