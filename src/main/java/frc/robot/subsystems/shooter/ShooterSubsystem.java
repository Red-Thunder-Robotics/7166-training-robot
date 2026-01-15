package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine.ShooterState;

public final class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem instance = null;

    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

    public ShooterSubsystem(ShooterIO io) {
        instance = this;

        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Shooter", m_inputs);
    }

    public void stateUpdate(ShooterState shooterState) {
        switch (shooterState) {
            case Idle:
                m_io.flywheelDutyCycle(flywheelOutput);
                m_io.kickerStop();
                break;
            case Shooting:
                m_io.flywheelDutyCycle(flywheelOutput);
                m_io.kickerDutyCycle(kickerOutput);
                break;
        }
    }
}
