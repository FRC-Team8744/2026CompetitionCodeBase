package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A drive modifier that applies teleoperated control inputs to the robot's drive system.
 * It processes joystick inputs for translation and rotation, applying a deadband to filter out small inputs.
 */
public class Teleop extends DriveModifier {
    private final double deadband;
    private final Supplier<Double> x;
    private final Supplier<Double> y;
    private final Supplier<Double> rot;

    /** 
     * Constructs a new Teleop drive modifier.
     * 
     * @param daedband The deadband threshold to apply to joystick inputs.
     * @param x A supplier that provides the x-axis translation input.
     * @param y A supplier that provides the y-axis translation input.
     * @param rot A supplier that provides the rotational input.
     */
    public Teleop(double daedband, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
        this.deadband = daedband;
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    @Override
    public boolean shouldRun(DriveContext context) {
        return true;
    }

    // Applies the deadband to the given input value.
    private double applyDeadband(Supplier<Double> value) {        
        return MathUtil.applyDeadband(value.get(), deadband, 1.0);
    }

    @Override
    protected void doExecute(DriveContext context) {
        double x = applyDeadband(this.x);
        double y = applyDeadband(this.y);
        double rot = applyDeadband(this.rot);

        context.currentSpeed = SpeedBuilder.from(x, y, rot, true);
    }
}
