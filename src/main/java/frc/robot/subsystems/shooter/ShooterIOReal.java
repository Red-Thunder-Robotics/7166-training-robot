package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.turret.TurretConstants.maxVelocity;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class ShooterIOReal implements ShooterIO {
    private final TalonFX m_flywheelMotor;
    private final TalonFX m_hoodMotor;
    private final TalonFX m_kickerMotor;
    private final boolean m_hasKicker;
    
    private double m_hoodTargetPosition = hoodPositionHome;

    private final StatusSignal<AngularVelocity> m_flywheelVelocitySignal;
    private final StatusSignal<Current> m_flywheelCurrentSignal;

    private final StatusSignal<Angle> m_hoodPositionSignal;
    private final StatusSignal<AngularVelocity> m_hoodVelocitySignal;
    private final StatusSignal<Current> m_hoodCurrentSignal;

    private final StatusSignal<AngularVelocity> m_kickerVelocitySignal;
    private final StatusSignal<Current> m_kickerCurrentSignal;

    private final DutyCycleOut m_flywheelDutyCycleRequest = new DutyCycleOut(0d);
    private final MotionMagicVelocityVoltage m_flywheelVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);
    private final MotionMagicVoltage m_hoodPositionRequest = new MotionMagicVoltage(m_hoodTargetPosition);
    private final DutyCycleOut m_kickerDutyCycleRequest = new DutyCycleOut(0d);

    public ShooterIOReal(int flywheelMotorId, int hoodMotorId, int kickerMotorId) {
        m_flywheelMotor = new TalonFX(flywheelMotorId, Constants.CANBUS);
        m_hoodMotor = new TalonFX(hoodMotorId, Constants.CANBUS);
        m_hasKicker = kickerMotorId != -2;
        if (m_hasKicker)
            m_kickerMotor = new TalonFX(kickerMotorId, Constants.CANBUS);
        else
            m_kickerMotor = null;

        m_flywheelVelocitySignal = m_flywheelMotor.getVelocity();
        m_flywheelCurrentSignal = m_flywheelMotor.getSupplyCurrent();

        m_hoodPositionSignal = m_hoodMotor.getPosition();
        m_hoodVelocitySignal = m_hoodMotor.getVelocity();
        m_hoodCurrentSignal = m_hoodMotor.getSupplyCurrent();

        if (m_hasKicker) {
            m_kickerVelocitySignal = m_kickerMotor.getVelocity();
            m_kickerCurrentSignal = m_kickerMotor.getSupplyCurrent();
        } else {
            m_kickerVelocitySignal = null;
            m_kickerCurrentSignal = null;
        }

        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = flywheelNeutralMode;
        flywheelConfig.MotorOutput.Inverted = flywheelInverted;

        // flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // flywheelConfig.CurrentLimits.SupplyCurrentLimit = flywheelCurrentLimit;

        flywheelConfig.Slot0.kP = flywheelPidP;

        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = hoodNeutralMode;
        hoodConfig.MotorOutput.Inverted = hoodInverted;

        hoodConfig.Feedback.SensorToMechanismRatio = hoodMotorReduction;

        hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        hoodConfig.Slot0.kP = hoodPidP;

        hoodConfig.MotionMagic.MotionMagicAcceleration = hoodTargetAcceleration;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;

        PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotor.getConfigurator().apply(flywheelConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_hoodMotor.getConfigurator().apply(hoodConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_flywheelVelocitySignal, m_flywheelCurrentSignal, m_hoodPositionSignal, m_hoodVelocitySignal, m_hoodCurrentSignal);

        if (m_hasKicker) {
            var kickerConfig = new TalonFXConfiguration();
            kickerConfig.MotorOutput.NeutralMode = kickerNeutralMode;
            kickerConfig.MotorOutput.Inverted = kickerInverted;

            // kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            // kickerConfig.CurrentLimits.SupplyCurrentLimit = kickerCurrentLimit;

            PhoenixUtil.tryUntilOk(5, () -> m_kickerMotor.getConfigurator().apply(kickerConfig));
            BaseStatusSignal.setUpdateFrequencyForAll(50d, m_kickerVelocitySignal, m_kickerCurrentSignal);
        }

        m_hoodMotor.setPosition(m_hoodTargetPosition);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_flywheelMotor.getAppliedControl());
        inputs.flywheelMotorVelocityRPS = m_flywheelVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorCurrentAmps = m_flywheelCurrentSignal.getValueAsDouble();

        inputs.flywheelTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_flywheelMotor.getAppliedControl());

        final double hoodTargetPosition = m_hoodTargetPosition;
        inputs.hoodTargetPositionRotations = hoodTargetPosition;
        inputs.hoodTargetPositionDegrees = mechanismPositionToAngle(hoodTargetPosition).in(Degrees);

        final double hoodPositionRotations = m_hoodPositionSignal.getValueAsDouble();
        inputs.hoodPositionRotations = hoodPositionRotations;
        inputs.hoodPositionDegrees = mechanismPositionToAngle(hoodPositionRotations).in(Degrees);

        inputs.hoodMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_hoodMotor.getAppliedControl());
        inputs.hoodMotorVelocityRPS = m_hoodVelocitySignal.getValueAsDouble();
        inputs.hoodMotorCurrentAmps = m_hoodCurrentSignal.getValueAsDouble();

        if (m_hasKicker) {
            inputs.kickerMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_kickerMotor.getAppliedControl());
            inputs.kickerMotorVelocityRPS = m_kickerVelocitySignal.getValueAsDouble();
            inputs.kickerMotorCurrentAmps = m_kickerCurrentSignal.getValueAsDouble();
        }
    }

    @Override
    public void idle() {
        m_flywheelMotor.disable();
        if (m_hasKicker)
            m_kickerMotor.disable();
    }

    @Override
    public void flywheelDutyCycle(double output) {
        m_flywheelMotor.setControl(m_flywheelDutyCycleRequest.withOutput(output));
    }
    @Override
    public void flywheelVelocity(AngularVelocity velocity) {
        m_flywheelMotor.setControl(m_flywheelVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void flywheelStop() {
        m_flywheelMotor.disable();
    }

    @Override
    public void setHoodPosition(Angle angle) {
        m_hoodTargetPosition = angleToMechanismPosition(angle);
        m_hoodMotor.setControl(m_hoodPositionRequest.withPosition(m_hoodTargetPosition));
    }

    @Override
    public void kickerDutyCycle(double output) {
        if (!m_hasKicker)
            return;
        m_kickerMotor.setControl(m_kickerDutyCycleRequest.withOutput(output));
    }
    @Override
    public void kickerStop() {
        if (!m_hasKicker)
            return;
        m_kickerMotor.disable();
    }
}
