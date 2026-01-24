package frc.robot.subsystems.drive;

import frc.robot.subsystems.DriveSubsystem;

/**
 * Processes a series of {@link DriveModifier} instances to modify the speed
 * of a {@link DriveSubsystem}. This class applies each modifier in sequence,
 * allowing for complex drive behavior modifications.
 */
public final class SpeedProcessor {
    private final DriveContext context;
    private final DriveModifier[] modifiers;

    /**
     * Constructs a new {@link SpeedProcessor} with the specified drive modifiers.
     *
     * @param context The drive context.
     * @param modifiers The array of {@link DriveModifier} instances to apply.
     */
    public SpeedProcessor(DriveContext context, DriveModifier... modifiers) {
        this.context = context;
        this.modifiers = modifiers;
    }

    /**
     * Gets the drive context.
     *
     * @return The {@link DriveContext} instance.
     */
    public DriveContext getContext() {
        return context;
    }

    /**
     * Processes the default initial speed (0, 0, 0) through all applicable drive modifiers.
     */
    public  void process() {
        process(new Speed(0, 0, 0, true));
    }

    /**
     * Processes the given initial speed through all applicable drive modifiers.
     *
     * @param context The {@link DriveContext} containing the drive subsystem and current speed.
     */
    public void process(Speed initialSpeed) {
        context.currentSpeed = SpeedBuilder.from(initialSpeed);

        for (DriveModifier modifier : modifiers) {
            if (modifier.shouldRun(context)) {
                modifier.execute(context);
            }
        }

        Speed speed = context.currentSpeed.build();

        if (speed.isFieldRelative()) {
            context.drivable.drive(speed);
        } else {
            context.drivable.driveRobotRelative(speed);
        }
    }
}