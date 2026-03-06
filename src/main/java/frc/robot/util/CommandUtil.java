package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

public final class CommandUtil {
    public static Command cmdName(Command command, String name) {
        command.setName(name);
        return command;
    }
}
