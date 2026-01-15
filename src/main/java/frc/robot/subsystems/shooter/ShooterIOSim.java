package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import edu.wpi.first.math.controller.PIDController;

public final class ShooterIOSim implements ShooterIO {
    private double m_flywheelOutput = 0d;

    private double m_hoodTargetPosition = hoodPositionHome;
    private double m_hoodPosition = m_hoodTargetPosition;

    private final PIDController m_hoodPID = new PIDController(0.5d, 0d, 0d);

    private double m_kickerOutput = 0d;

    public ShooterIOSim() {

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelMotorDutyCycle = m_flywheelOutput;
        inputs.kickerMotorDutyCycle = m_kickerOutput;

        final double hoodOutput = m_hoodPID.calculate(m_hoodPosition);
        m_hoodPosition += hoodOutput;

        final double hoodTargetPosition = m_hoodTargetPosition;
        inputs.hoodTargetPositionRotations = hoodTargetPosition;
        inputs.hoodTargetPositionDegrees = mechanismPositionToAngle(hoodTargetPosition).in(Degrees);

        final double hoodPosition = m_hoodPosition;
        inputs.hoodPositionRotations = hoodPosition;
        inputs.hoodPositionDegrees = mechanismPositionToAngle(hoodPosition).in(Degrees);

        inputs.hoodMotorDutyCycle = hoodOutput;
    }

    @Override
    public void idle() {
        m_flywheelOutput = 0d;
        m_kickerOutput = 0d;
    }

    @Override
    public void flywheelDutyCycle(double output) {
        m_flywheelOutput = output;
    }
    @Override
    public void flywheelStop() {
        m_flywheelOutput = 0d;
    }

    @Override
    public void setHoodPosition(double position) {
        m_hoodTargetPosition = position;
        m_hoodPID.setSetpoint(position);
    }

    @Override
    public void kickerDutyCycle(double output) {
        m_kickerOutput = output;
    }
    @Override
    public void kickerStop() {
        m_kickerOutput = 0d;
    }
}
