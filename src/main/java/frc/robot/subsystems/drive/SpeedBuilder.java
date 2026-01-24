package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A builder class for creating and manipulating {@link Speed} objects.
 */
public class SpeedBuilder {
    private double x = 0;
    private double y = 0;
    private double rot = 0;
    private boolean fieldRelative = true;

    /**
     * Default constructor initializing all speeds to zero and field-relative to true.
     */
    private SpeedBuilder() {
        // Default constructor
    }

    /**
     * Constructs a {@link SpeedBuilder} with the specified parameters.
     *
     * @param x The velocity in the x-direction (meters per second).
     * @param y The velocity in the y-direction (meters per second).
     * @param rot The rotational velocity (radians per second).
     * @param fieldRelative {@code true} if the speed is field-relative, {@code false} otherwise.
     */
    public static SpeedBuilder from(double x, double y, double rot, boolean fieldRelative) {
        SpeedBuilder builder = new SpeedBuilder();
        builder.x = x;
        builder.y = y;
        builder.rot = rot;
        builder.fieldRelative = fieldRelative;
        return builder;
    }

    /**
     * Constructs a {@link SpeedBuilder} from an existing {@link Speed} object.
     *
     * @param speed The {@link Speed} object to copy.
     */
    public static SpeedBuilder from(Speed speed) {
        return from(speed.getX(), speed.getY(), speed.getRot(), speed.isFieldRelative());
    }

    /**
     * Constructs a {@link SpeedBuilder} from an existing {@link ChassisSpeeds} object
     * with the specified field-relative flag.
     *
     * @param speeds The {@link ChassisSpeeds} object to copy.
     * @param fieldRelative {@code true} if the speed is field-relative, {@code false} otherwise.
     */
    public static SpeedBuilder from(ChassisSpeeds speeds, boolean fieldRelative) {
        return from(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    // Setter methods for builder pattern
    public SpeedBuilder setX(double x) {
        this.x = x;
        return this;
    }

    // Setter methods for builder pattern
    public SpeedBuilder setY(double y) {
        this.y = y;
        return this;
    }

    // Setter methods for builder pattern
    public SpeedBuilder setRot(double rot) {
        this.rot = rot;
        return this;
    }

    // Setter methods for builder pattern
    public SpeedBuilder setFieldRelative(boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
        return this;
    }

    /**
     * Adds the velocities of this {@link Speed} object to the velocities of the given {@link ChassisSpeeds} object.
     * The field-relative flag of this object is preserved.
     * 
     * @param other The {@link ChassisSpeeds} object to add.
     * @return A new {@link Speed} object with the combined velocities.
     */
    public final SpeedBuilder plus(ChassisSpeeds other) {
        return plus(new Speed(other), this.fieldRelative);
    }

    /**
     * Adds the velocities of this {@link Speed} object to the velocities of the given {@link Speed} object.
     * The field-relative flag of this object is preserved.
     *
     * @param other The {@link Speed} object to add.
     * @return A new {@link Speed} object with the combined velocities.
     */
    public final SpeedBuilder plus(Speed other) {
        return plus(other, this.fieldRelative);
    }

    /**
     * Adds the velocities of this {@link Speed} object to the velocities of the given {@link Speed} object.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param other The {@link Speed} object to add.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the combined velocities and the specified field-relative flag.
     */
    public final SpeedBuilder plus(Speed other, boolean fieldRelative) {
        this.x += other.getX();
        this.y += other.getY();
        this.rot += other.getRot();
        this.fieldRelative = fieldRelative;
        
        return this;
    }
    /**
     * Adds the given velocities to this {@link Speed} object.
     *
     * @param x The velocity to add in the x-direction (meters per second).
     * @param y The velocity to add in the y-direction (meters per second).
     * @param rot The rotational velocity to add (radians per second).
     * @return A new {@link Speed} object with the combined velocities.
     */
    public final SpeedBuilder plus(double x, double y, double rot) {
        this.x += x;
        this.y += y;
        this.rot += rot;

        return this;
    }

    /**
     * Adds the given velocities to this {@link Speed} object.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param x The velocity to add in the x-direction (meters per second).
     * @param y The velocity to add in the y-direction (meters per second).
     * @param rot The rotational velocity to add (radians per second).
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the combined velocities and the specified field-relative flag.
     */
    public final SpeedBuilder plus(double x, double y, double rot,  boolean fieldRelative) {
        this.x += x;
        this.y += y;
        this.rot += rot;
        this.fieldRelative = fieldRelative;

        return this;
    }
    
    /**
     * Subtracts the velocities of the given {@link ChassisSpeeds} object from this {@link Speed} object.
     * The field-relative flag of this object is preserved.
     * 
     * @param other The {@link ChassisSpeeds} object to subtract.
     * @return A new {@link Speed} object with the resulting velocities.
     */
    public final SpeedBuilder minus(ChassisSpeeds other) {
        return minus(new Speed(other), this.fieldRelative);
    }

    /**
     * Subtracts the velocities of the given {@link Speed} object from this {@link Speed} object.
     * The field-relative flag of this object is preserved.
     *
     * @param other The {@link Speed} object to subtract.
     * @return A new {@link Speed} object with the resulting velocities.
     */
    public final SpeedBuilder minus(Speed other) {
        return minus(other, this.fieldRelative);
    }

    /**
     * Subtracts the velocities of the given {@link Speed} object from this {@link Speed} object.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param other The {@link Speed} object to subtract.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the resulting velocities and
     */
    public final SpeedBuilder minus(Speed other, boolean fieldRelative) {
        this.x -= other.getX();
        this.y -= other.getY();
        this.rot -= other.getRot();
        this.fieldRelative = fieldRelative;

        return this;
    }

    /**
     * Subtracts the given velocities from this {@link Speed} object.
     *
     * @param x The velocity to subtract in the x-direction (meters per second).
     * @param y The velocity to subtract in the y-direction (meters per second).
     * @param rot The rotational velocity to subtract (radians per second).
     * @return A new {@link Speed} object with the resulting velocities.
     */
    public final SpeedBuilder minus(double x, double y, double rot) {
        this.x -= x;
        this.y -= y;
        this.rot -= rot;

        return this;
    }

    /**
     * Subtracts the given velocities from this {@link Speed} object.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param x The velocity to subtract in the x-direction (meters per second).
     * @param y The velocity to subtract in the y-direction (meters per second).
     * @param rot The rotational velocity to subtract (radians per second).
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the resulting velocities and the specified field-relative flag.
     */
    public final SpeedBuilder minus(double x, double y, double rot, boolean fieldRelative) {
        this.x -= x;
        this.y -= y;
        this.rot -= rot;
        this.fieldRelative = fieldRelative;

        return this;
    }

    /**
     * Negates the velocities of this {@link Speed} object.
     * The field-relative flag of this object is preserved.
     *
     * @return A new {@link Speed} object with the negated velocities.
     */
    public final SpeedBuilder unaryMinus() {
        return times(-1.0, this.fieldRelative);
    }

    /**
     * Scales the velocities of this {@link Speed} object by the given scaler.
     * The field-relative flag of this object is preserved.
     *
     * @param scaler The scaler to multiply the velocities by.
     * @return A new {@link Speed} object with the scaled velocities.
     */
    public final SpeedBuilder times(double scaler) {
        return times(scaler, this.fieldRelative);
    }

    /**
     * Scales the velocities of this {@link Speed} object by the given scaler.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param scaler The scaler to multiply the velocities by.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the scaled velocities and the specified field-relative flag.
     */
    public final SpeedBuilder times(Speed other) {
        return times(other, this.fieldRelative);
    }

    /**
     * Scales the velocities of this {@link Speed} object by the given scaler.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param other The {@link Speed} object to multiply the velocities by.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the scaled velocities and the specified field-relative flag.
     */
    public final SpeedBuilder times(Speed other, boolean fieldRelative) {
        return times(other.getX(), other.getY(), other.getRot(), fieldRelative);
    }

    /**
     * Scales the velocities of this {@link Speed} object by the given scalers.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param scaler The scaler to multiply the velocities by.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the scaled velocities and the specified field-relative flag.
     */
    public final SpeedBuilder times(double scaler, boolean fieldRelative) {
        return times(scaler, scaler, scaler, fieldRelative);
    }

    /**
     * Scales the velocities of this {@link Speed} object by the given scalers.
     *
     * @param xScaler The scaler to multiply the x-velocity by.
     * @param yScaler The scaler to multiply the y-velocity by.
     * @param rotScaler The scaler to multiply the rotational velocity by.
     * @return A new {@link Speed} object with the scaled velocities.
     */
    public final SpeedBuilder times(double xScaler, double yScaler, double rotScaler) {
        return times(xScaler, yScaler, rotScaler, this.fieldRelative);
    }

    /**
     * Scales the velocities of this {@link Speed} object by the given scalers.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param xScaler The scaler to multiply the x-velocity by.
     * @param yScaler The scaler to multiply the y-velocity by.
     * @param rotScaler The scaler to multiply the rotational velocity by.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the scaled velocities and the specified field-relative flag.
     */
    public final SpeedBuilder times(double xScaler, double yScaler, double rotScaler, boolean fieldRelative) {
        this.x *= xScaler;
        this.y *= yScaler;
        this.rot *= rotScaler;
        this.fieldRelative = fieldRelative;

        return this;
    }

    /**
     * Divides the velocities of this {@link Speed} object by the given scaler.
     * The field-relative flag of this object is preserved.
     *
     * @param saler The scaler to divide the velocities by.
     * @return A new {@link Speed} object with the scaled velocities.
     */
    public final SpeedBuilder div(double saler) {
        return div(saler, this.fieldRelative);
    }

    /**
     * Divides the velocities of this {@link Speed} object by the given scaler.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param saler The scaler to divide the velocities by.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the scaled velocities and the specified field-relative flag.
     */
    public final SpeedBuilder div(double saler, boolean fieldRelative) {
        return div(saler, saler, saler, fieldRelative);
    }

    /**
     * Divides the velocities of this {@link Speed} object by the given {@link Speed} object's velocities.
     * The field-relative flag of this object is preserved.
     *
     * @param other The {@link Speed} object to divide the velocities by.
     * @return A new {@link Speed} object with the scaled velocities.
     */
    public final SpeedBuilder div(Speed other) {
        return div(other, this.fieldRelative);
    }

    /**
     * Divides the velocities of this {@link Speed} object by the given {@link Speed} object's velocities.
     * Allows specifying whether the resulting {@link Speed} object should be field-relative.
     *
     * @param other The {@link Speed} object to divide the velocities by.
     * @param fieldRelative {@code true} if the resulting speed should be field-relative, {@code false} otherwise.
     * @return A new {@link Speed} object with the scaled velocities and the specified field-relative flag.
     */
    public final SpeedBuilder div(Speed other, boolean fieldRelative) {
        return div(other.getX(), other.getY(), other.getRot(), fieldRelative);
    }

    /**
     * Divides the velocities of this {@link Speed} object by the given scalers.
     *
     * @param xSaler The scaler to divide the x-velocity by.
     * @param ySaler The scaler to divide the y-velocity by.
     * @param rotSaler The scaler to divide the rotational velocity by.
     * @return A new {@link Speed} object with the scaled velocities.
     */
    public final SpeedBuilder div(double xSaler, double ySaler, double rotSaler, boolean fieldRelative) {
        this.x /= xSaler;
        this.y /= ySaler;
        this.rot /= rotSaler;
        this.fieldRelative = fieldRelative;

        return this;
    }

    /**
     * Builds the {@link Speed} object with the current parameters.
     *
     * @return A new {@link Speed} object.
     */
    public Speed build() {
        return new Speed(x, y, rot, fieldRelative);
    }
}