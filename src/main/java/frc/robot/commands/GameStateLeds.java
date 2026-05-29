package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Color;
import frc.robot.subsystems.LedSubsystem;

/**
 * Displays the amount of time remaining in the current shooting phase on an LED string.
 */
public class GameStateLeds extends Command {
    private Color m_activeHubColor;
    private Color m_inactiveHubColor;
    private Color m_defaultColor;

    private LedSubsystem m_ledSubsystem;

    public GameStateLeds(LedSubsystem ledSubsystem, Color activeHubColor, Color inactiveHubColor, Color defaultColor) {
        this.m_activeHubColor = activeHubColor;
        this.m_inactiveHubColor = inactiveHubColor;
        this.m_defaultColor = defaultColor;

        this.m_ledSubsystem = ledSubsystem;
        this.m_ledSubsystem.setSolidColor(this.m_defaultColor);

        this.addRequirements(m_ledSubsystem);
    }

    public void execute() {
        double remainingTime = DriverStation.getMatchTime();
        String autoWinner = DriverStation.getGameSpecificMessage();

        if (DriverStation.getAlliance().isEmpty()) {
            return;
        }

        String ourAlliance;
        
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            ourAlliance = "R";
        } else {
            ourAlliance = "B";
        }

        if (remainingTime == -1) {
            m_ledSubsystem.setSolidColor(m_defaultColor);
            return;
        }

        // 1. Establish core state
        boolean isAutoWinner = ourAlliance.equals(autoWinner);

        // 2. Set default parameters for the LED animation
        double baseTime = 0;
        Color activeColor = m_activeHubColor;
        Color fallbackColor = m_defaultColor;

        // 3. Process time windows purely for configuration data
        if (remainingTime <= 140 && remainingTime > 130) {
            baseTime = isAutoWinner ? 130 : 105;
        } 
        else if (remainingTime <= 130 && remainingTime > 105) {
            baseTime = 105;
            activeColor = !isAutoWinner ? m_activeHubColor : m_inactiveHubColor;
        } 
        else if (remainingTime <= 105 && remainingTime > 80) {
            baseTime = 80;
            activeColor = isAutoWinner ? m_activeHubColor : m_inactiveHubColor;
        } 
        else if (remainingTime <= 80 && remainingTime > 55) {
            baseTime = 55;
            activeColor = !isAutoWinner ? m_activeHubColor : m_inactiveHubColor;
        } 
        else if (remainingTime <= 55 && remainingTime > 30) {
            // Unique case: different base times per turn configuration
            boolean isOurTurn = isAutoWinner;
            baseTime = isOurTurn ? 0 : 30;
            activeColor = isOurTurn ? m_activeHubColor : m_inactiveHubColor;
        } 
        else if (remainingTime <= 30 && remainingTime > 0) {
            baseTime = 0; // remainingTime > i + 1 simplifies to remainingTime - (-1) > i
        }

        // 4. A single, unified lambda to update the LEDs
        final double finalBaseTime = baseTime;
        final Color finalActiveColor = activeColor;
        final Color finalFallbackColor = fallbackColor;

        m_ledSubsystem.setColors((i, l) -> 
            (remainingTime - finalBaseTime > i) ? finalActiveColor : finalFallbackColor
        );
    }
}