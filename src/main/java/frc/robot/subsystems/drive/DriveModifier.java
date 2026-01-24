// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.subsystems.DriveSubsystem;

/**
 * An abstract class that modifies the behavior of the drive subsystem.
 * This class provides a framework for implementing specific drive modifications
 * by defining the lifecycle of the modification (initialize, execute, cleanup).
 */
public abstract class DriveModifier {
    private boolean firstRun = true;

    /**
     * Called once when the modifier is first executed.
     * This method can be overridden to perform any initialization logic.
     *
     * @param context The {@link DriveContext} instance being modified.
     */
    protected void initialize(DriveContext context){}

    /**
     * Determines whether the modifier should run.
     * This method must be implemented by subclasses to define the condition
     * under which the modifier should execute.
     *
     * @param context The {@link DriveContext} instance being modified.
     * @return {@code true} if the modifier should run, {@code false} otherwise.
     */
    protected abstract boolean shouldRun(DriveContext context);

    /**
     * Executes the modification logic.
     * This method must be implemented by subclasses to define how the drive
     * behavior is modified.
     *
     * @param context The {@link DriveContext} instance being modified.
     * @return The modified {@link SpeedBuilder} object.
     */
    protected abstract void doExecute(DriveContext context);

    /**
     * Called once when the modifier is no longer active.
     * This method can be overridden to perform any cleanup logic.
     *
     * @param context The {@link DriveContext} instance being modified.
     */
    protected void cleanUp(DriveContext context){}

    /**
     * Executes the drive modification logic.
     * This method manages the lifecycle of the modifier, including initialization,
     * execution, and cleanup. It ensures that the modifier is only initialized
     * once and cleaned up when it is no longer active.
     *
     * @param context The {@link DriveContext} instance being modified.
     * @return The modified {@link SpeedBuilder} object.
     */
    public final void execute(DriveContext context) {
        if(shouldRun(context)) {
            if (firstRun) {
                initialize(context);
                firstRun = false;
            }
            
            doExecute(context);
        } else {
            if (!firstRun) {
                cleanUp(context);
                firstRun = true;
            }
        }
    } 
}