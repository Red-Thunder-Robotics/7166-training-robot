package frc.robot;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Controls {
    public static final CommandXboxController driveController = new CommandXboxController(DRIVER_CONTROLLER);
    public static final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER);

    public static final Trigger resetGyroButton = driveController.start();

    public static final Trigger deployIntakeButton = operatorController.rightBumper();
    public static final Trigger retractIntakeButton = operatorController.leftBumper();

    public static final Trigger hubButton = operatorController.rightTrigger();
    public static final Trigger allianceFeedButton = operatorController.x();

    public static final Trigger climbSafetyButton = driveController.b();
    public static final Trigger climbForward = driveController.povUp(); // bottom right paddle
    public static final Trigger climbBackward = driveController.povLeft(); // bottom left paddle

}
