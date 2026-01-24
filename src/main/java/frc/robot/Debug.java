package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Debug utility class to conditionally run debug code and log data to SmartDashboard.
 */
public class Debug {
    private static boolean ENABLE = false;

    /**
     * Sets the debug level.
     * 
     * @param level The debug level to set.
     */
    public static final void on() {
        ENABLE = true;
    }

    /**
     * Disables debug mode.
     */
    public static final void off() {
        ENABLE = false;
    }

    /**
     * Runs the given task if debug mode is enabled.
     * 
     * @param task The task to run.
     */
    public static final void run(Runnable task) {
        if (ENABLE) {
            task.run();
        }
    }

    /**
     * Logs multiple Dashboard data entries to SmartDashboard if debug mode is enabled.
     * 
     * @param data The Dashboard data entries to log.
     */
    public static final void dashboard(Dashboard...data) {
        if (ENABLE) {
            for (Dashboard d : data) {
                processDashboard(d.name, d.values);
            }
        }
    }

    /**
     * Logs data to SmartDashboard if debug mode is enabled.
     * 
     * @param name  The name of the data.
     * @param value The value(s) to log.
     */
    public static final void dashboard(String name, Object...value) {
        if (ENABLE) {
            processDashboard(name, value);
        }
    }

    /**
     * Processes the data and logs it to SmartDashboard.
     * 
     * @param name  The name of the data.
     * @param value The value to log.
     */
    private static final void processDashboard(String name, Object value) {
        if (value instanceof Object[]) {
            dashboardArray(name, (Object[]) value);
        } else {
            dashboardSingle(name, value);
        }
    }

    /**
     * Logs a single value to SmartDashboard.
     * 
     * @param name  The name of the data.
     * @param value The value to log.
     */
    private static final void dashboardSingle(String name, Object value) {
        if (value instanceof Double) {
            SmartDashboard.putNumber(name, (Double) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(name, (Boolean) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(name, (String) value);
        } else if (value instanceof Sendable) {
            SmartDashboard.putData(name, (Sendable) value);
        } else {
            SmartDashboard.putString(name, value.toString());
        }
    }

    /**
     * Logs an array of values to SmartDashboard.
     * 
     * @param name  The name of the data.
     * @param value The array of values to log.
     */
    private static final void dashboardArray(String name, Object[] value) {
        if (value instanceof Double[]) {
            SmartDashboard.putNumberArray(name, (Double[]) value);
        } else if (value instanceof Boolean[]) {
            SmartDashboard.putBooleanArray(name, (Boolean[]) value);
        } else if (value instanceof String[]) {
            SmartDashboard.putStringArray(name, (String[]) value);
        }
    }

    /**
     * Dashboard data entry class.
     */
    public static class Dashboard {
        private String name;
        private Object[] values;

        // Private constructor to enforce the use of the static factory method.
        private Dashboard(String name, Object[] values) {
            this.name = name;
            this.values = values;
        }

        /**
         * Static factory method to create a Dashboard data entry.
         * 
         * @param name   The name of the data.
         * @param values The value(s) to log.
         * @return A new Dashboard data entry.
         */
        public static final Dashboard of(String name, Object... values) {
            return new Dashboard(name, values);
        }
    }
}
