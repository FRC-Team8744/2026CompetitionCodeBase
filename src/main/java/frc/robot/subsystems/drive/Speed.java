package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Represents the speed of a robot in a 2D plane, including its translational
 * and rotational velocities. This class extends {@link ChassisSpeeds} and adds
 * the concept of field-relative motion.
 */
public class Speed extends ChassisSpeeds {
    private final boolean fieldRelative;

    /**
     * Constructs a new {@link Speed} object with the specified velocities and
     * field-relative flag.
     *
     * @param x The velocity in the x-direction (meters per second).
     * @param y The velocity in the y-direction (meters per second).
     * @param rot The rotational velocity (radians per second).
     * @param fieldRelative {@code true} if the speed is field-relative, {@code false} otherwise.
     */
    public Speed(double x, double y, double rot, boolean fieldRelative) {
        super(x, y, rot);
        this.fieldRelative = fieldRelative;
    }

    /**
     * Constructs a new {@link Speed} object with the specified velocities.
     * The speed is assumed to be field-relative by default.
     *
     * @param x The velocity in the x-direction (meters per second).
     * @param y The velocity in the y-direction (meters per second).
     * @param rot The rotational velocity (radians per second).
     */
    public Speed(double x, double y, double rot) {
        this(x, y, rot, true);
    }

    /**
     * Constructs a new {@link Speed} object from an existing {@link ChassisSpeeds} object.
     * The speed is assumed to be field-relative by default.
     *
     * @param speeds The {@link ChassisSpeeds} object to copy.
     */
    public Speed(ChassisSpeeds speeds) {
        this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
    }

    /**
     * Constructs a new {@link Speed} object from an existing {@link ChassisSpeeds} object
     * with the specified field-relative flag.
     *
     * @param speeds The {@link ChassisSpeeds} object to copy.
     * @param fieldRelative {@code true} if the speed is field-relative, {@code false} otherwise.
     */
    public Speed(ChassisSpeeds speeds, boolean fieldRelative) {
        this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    /**
     * Gets the velocity in the x-direction.
     *
     * @return The velocity in the x-direction (meters per second).
     */
    public double getX() {
        return this.vxMetersPerSecond;
    }

    /**
     * Gets the velocity in the y-direction.
     *
     * @return The velocity in the y-direction (meters per second).
     */
    public double getY() {
        return this.vyMetersPerSecond;
    }

    /**
     * Gets the rotational velocity.
     *
     * @return The rotational velocity (radians per second).
     */
    public double getRot() {
        return this.omegaRadiansPerSecond;
    }

    /**
     * Checks if the speed is field-relative.
     *
     * @return {@code true} if the speed is field-relative, {@code false} otherwise.
     */
    public boolean isFieldRelative() {
        return fieldRelative;
    }

    /**
     * Adds the velocities of this {@link Speed} object to the velocities of the given {@link ChassisSpeeds} object.
     * The field-relative flag of this object is preserved.
     *
     * @param other The {@link ChassisSpeeds} object to add.
     * @return A new {@link Speed} object with the combined velocities.
     */
    @Override
    public final Speed plus(ChassisSpeeds other) {
        return new Speed(super.plus(other));
    }

    @Override
    public final Speed minus(ChassisSpeeds other) {
        return new Speed(super.minus(other));
    }

    @Override
    public final Speed unaryMinus() {
        return new Speed(super.unaryMinus());
    }

    @Override
    public final Speed times(double scaler) {
        return new Speed(super.times(scaler));
    }

    @Override
    public final Speed div(double saler) {
        return new Speed(super.div(saler));
    }
}