package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.turret.TurretConstants.maxVelocity;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class ShooterIOReal implements ShooterIO {
    private final TalonFX m_flywheelMotor = new TalonFX(flywheelMotorId, Constants.CANBUS);
    private final TalonFX m_hoodMotor = new TalonFX(hoodMotorId, Constants.CANBUS);
    private final TalonFX m_kickerMotor = new TalonFX(kickerMotorId, Constants.CANBUS);

    private double m_hoodTargetPosition = hoodPositionHome;

    private final StatusSignal<AngularVelocity> m_flywheelVelocitySignal = m_flywheelMotor.getVelocity();
    private final StatusSignal<Current> m_flywheelCurrentSignal = m_flywheelMotor.getSupplyCurrent();

    private final StatusSignal<Angle> m_hoodPositionSignal = m_hoodMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_hoodVelocitySignal = m_hoodMotor.getVelocity();
    private final StatusSignal<Current> m_hoodCurrentSignal = m_hoodMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_kickerVelocitySignal = m_kickerMotor.getVelocity();
    private final StatusSignal<Current> m_kickerCurrentSignal = m_kickerMotor.getSupplyCurrent();

    private final DutyCycleOut m_flywheelDutyCycleRequest = new DutyCycleOut(0d);
    private final MotionMagicVoltage m_hoodPositionRequest = new MotionMagicVoltage(m_hoodTargetPosition);
    private final DutyCycleOut m_kickerDutyCycleRequest = new DutyCycleOut(0d);

    public ShooterIOReal() {
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = flywheelNeutralMode;
        flywheelConfig.MotorOutput.Inverted = flywheelInverted;

        // flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // flywheelConfig.CurrentLimits.SupplyCurrentLimit = flywheelCurrentLimit;

        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = hoodNeutralMode;
        hoodConfig.MotorOutput.Inverted = hoodInverted;

        hoodConfig.Feedback.SensorToMechanismRatio = hoodMotorReduction;

        hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        hoodConfig.Slot0.kP = hoodPidP;

        hoodConfig.MotionMagic.MotionMagicAcceleration = hoodTargetAcceleration;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;

        var kickerConfig = new TalonFXConfiguration();
        kickerConfig.MotorOutput.NeutralMode = kickerNeutralMode;
        kickerConfig.MotorOutput.Inverted = kickerInverted;

        // kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // kickerConfig.CurrentLimits.SupplyCurrentLimit = kickerCurrentLimit;

        PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotor.getConfigurator().apply(flywheelConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_hoodMotor.getConfigurator().apply(hoodConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_kickerMotor.getConfigurator().apply(kickerConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_flywheelVelocitySignal, m_flywheelCurrentSignal, m_hoodPositionSignal, m_hoodVelocitySignal, m_hoodCurrentSignal, m_kickerVelocitySignal, m_kickerCurrentSignal);

        m_hoodMotor.setPosition(m_hoodTargetPosition);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_flywheelMotor.getAppliedControl());
        inputs.flywheelMotorVelocityRPS = m_flywheelVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorCurrentAmps = m_flywheelCurrentSignal.getValueAsDouble();

        final double hoodTargetPosition = m_hoodTargetPosition;
        inputs.hoodTargetPositionRotations = hoodTargetPosition;
        inputs.hoodTargetPositionDegrees = mechanismPositionToAngle(hoodTargetPosition).in(Degrees);

        final double hoodPositionRotations = m_hoodPositionSignal.getValueAsDouble();
        inputs.hoodPositionRotations = hoodPositionRotations;
        inputs.hoodPositionDegrees = mechanismPositionToAngle(hoodPositionRotations).in(Degrees);

        inputs.hoodMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_hoodMotor.getAppliedControl());
        inputs.hoodMotorVelocityRPS = m_hoodVelocitySignal.getValueAsDouble();
        inputs.hoodMotorCurrentAmps = m_hoodCurrentSignal.getValueAsDouble();

        inputs.kickerMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_kickerMotor.getAppliedControl());
        inputs.kickerMotorVelocityRPS = m_kickerVelocitySignal.getValueAsDouble();
        inputs.kickerMotorCurrentAmps = m_kickerCurrentSignal.getValueAsDouble();
    }

    @Override
    public void idle() {
        m_flywheelMotor.disable();
        m_kickerMotor.disable();
    }

    @Override
    public void flywheelDutyCycle(double output) {
        m_flywheelMotor.setControl(m_flywheelDutyCycleRequest.withOutput(output));
    }
    @Override
    public void flywheelStop() {
        m_flywheelMotor.disable();
    }

    @Override
    public void setHoodPosition(double position) {
        m_hoodTargetPosition = position;
        m_hoodMotor.setControl(m_hoodPositionRequest.withPosition(position));
    }

    @Override
    public void kickerDutyCycle(double output) {
        m_kickerMotor.setControl(m_kickerDutyCycleRequest.withOutput(output));
    }
    @Override
    public void kickerStop() {
        m_kickerMotor.disable();
    }
}
