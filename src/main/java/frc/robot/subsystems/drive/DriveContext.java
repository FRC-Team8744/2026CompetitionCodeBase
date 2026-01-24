package frc.robot.subsystems.drive;

import frc.robot.RotationEnum;

public class DriveContext {
    public final Drivable drivable;
    public SpeedBuilder currentSpeed = SpeedBuilder.from(0, 0, 0, true);
    public boolean isDrivingSlow = false;
    public boolean isAutoXSpeed = false;
    public RotationEnum isAutoRotate = RotationEnum.NONE;
    public double autoXSpeed = 0;

    public DriveContext(Drivable drivable) {
        this.drivable = drivable;
    }
}
