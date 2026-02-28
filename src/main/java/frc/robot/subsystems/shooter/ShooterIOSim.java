package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShooterIOSim implements ShooterIO {
    private boolean m_flywheelStopped = false;
    private double m_flywheelOutput = 0d;
    private double m_flywheelTargetVelocityRPS = 0d;
    private double m_flywheelVelocityRPS = 0d;

    private double m_kickerTargetVelocityRPS = 0d;
    private double m_kickerVelocityRPS = 0d;

    private boolean m_hoodStopped = false;
    private double m_hoodTargetPosition = hoodPositionHome;
    private double m_hoodPosition = m_hoodTargetPosition;

    private final PIDController m_flywheelVelocityPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_kickerVelocityPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_hoodPID = new PIDController(0.5d, 0d, 0d);

    public ShooterIOSim() {

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelMotorLeftDutyCycle = m_flywheelOutput;

        if (!m_flywheelStopped) {
            final double flywheelVelocityOutput = m_flywheelVelocityPID.calculate(m_flywheelVelocityRPS);
            m_flywheelVelocityRPS += flywheelVelocityOutput;
        }

        inputs.flywheelTargetVelocityRPS = m_flywheelTargetVelocityRPS;
        inputs.flywheelMotorLeftVelocityRPS = m_flywheelVelocityRPS;

        if (!m_hoodStopped) {
            final double hoodOutput = m_hoodPID.calculate(m_hoodPosition);
            m_hoodPosition += hoodOutput;

            inputs.hoodMotorDutyCycle = hoodOutput;
        }

        inputs.kickerTargetVelocityRPS = m_kickerTargetVelocityRPS;
        m_kickerVelocityRPS += m_kickerVelocityPID.calculate(m_kickerVelocityRPS);
        inputs.kickerMotorVelocityRPS = m_kickerVelocityRPS;

        final double hoodTargetPosition = m_hoodTargetPosition;
        inputs.hoodTargetPositionRotations = hoodTargetPosition;
        inputs.hoodTargetPositionDegrees = mechanismPositionToAngle(hoodTargetPosition).in(Degrees);

        final double hoodPosition = m_hoodPosition;
        inputs.hoodPositionRotations = hoodPosition;
        inputs.hoodPositionDegrees = mechanismPositionToAngle(hoodPosition).in(Degrees);
    }

    @Override
    public void idle() {
        m_hoodStopped = true;
        flywheelStop();
        kickerStop();
    }

    @Override
    public void flywheelDutyCycle(double output) {
        m_flywheelStopped = false;
        m_flywheelOutput = output;
    }
    @Override
    public void flywheelVelocity(AngularVelocity velocity) {
        m_flywheelStopped = false;
        m_flywheelTargetVelocityRPS = velocity.in(RotationsPerSecond);
        m_flywheelVelocityPID.setSetpoint(m_flywheelTargetVelocityRPS);
    }
    @Override
    public void flywheelStop() {
        m_flywheelStopped = true;
        m_flywheelOutput = 0d;
        m_flywheelTargetVelocityRPS = 0d;
        m_flywheelVelocityRPS = 0d;
    }

    @Override
    public void setHoodPosition(double position) {
        m_hoodStopped = false;
        m_hoodTargetPosition = position;
        m_hoodPID.setSetpoint(m_hoodTargetPosition);
    }
    @Override
    public void setHoodPosition(Angle angle) {
        setHoodPosition(angleToMechanismPosition(angle));
    }

    @Override
    public void kickerVelocity(AngularVelocity velocity) {
        m_kickerTargetVelocityRPS = velocity.in(RotationsPerSecond);
        m_kickerVelocityPID.setSetpoint(m_kickerTargetVelocityRPS);
    }
    @Override
    public void kickerStop() {
        m_kickerVelocityPID.setSetpoint(0d);
    }
}
