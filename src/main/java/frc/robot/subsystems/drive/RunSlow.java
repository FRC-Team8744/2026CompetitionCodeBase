package frc.robot.subsystems.drive;

/**
 * A drive modifier that reduces the speed of the robot when driving slowly.
 */
public class RunSlow extends DriveModifier {
    private final Speed factor;

    public RunSlow(Speed factor) {
        super();
        this.factor = factor;
    }

    @Override
    public boolean shouldRun(DriveContext context) {
        return context.isDrivingSlow;
    }

    @Override
    protected void doExecute(DriveContext context) {
        context.currentSpeed = context.currentSpeed.times(factor);
    }
    
}
