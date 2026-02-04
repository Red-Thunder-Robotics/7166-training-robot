package frc.robot;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Controls {
    public static final CommandXboxController controller = new CommandXboxController(CONTROLLER);

    public static final Trigger resetGyroButton = controller.start();

    public static final Trigger deployIntakeButton = controller.rightBumper();
    public static final Trigger retractIntakeButton = controller.leftBumper();

    public static final Trigger shootButton = controller.rightTrigger();
    public static final Trigger allianceFeedButton = controller.x();

    public static final Trigger climbSafetyButton = controller.b();
    public static final Trigger climbForward = controller.povUp(); // bottom right paddle
    public static final Trigger climbBackward = controller.povLeft(); // bottom left paddle

}
