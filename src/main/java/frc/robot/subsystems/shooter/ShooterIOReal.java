package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.turret.TurretConstants.maxVelocity;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class ShooterIOReal implements ShooterIO {
    private final TalonFX m_flywheelMotorLeft;
    private final TalonFX m_flywheelMotorMiddleUpper;
    private final TalonFX m_flywheelMotorMiddleLower;
    private final TalonFX m_flywheelMotorRight;
    private final TalonFX m_hoodMotor;
    private final TalonFX m_kickerMotor;
    
    private double m_hoodTargetPosition = hoodPositionHome;

    private final StatusSignal<AngularVelocity> m_flywheelLeftVelocitySignal;
    private final StatusSignal<Current> m_flywheelLeftCurrentSignal;

    private final StatusSignal<AngularVelocity> m_flywheelMiddleUpperVelocitySignal;
    private final StatusSignal<Current> m_flywheelMiddleUpperCurrentSignal;

    private final StatusSignal<AngularVelocity> m_flywheelMiddleLowerVelocitySignal;
    private final StatusSignal<Current> m_flywheelMiddleLowerCurrentSignal;

    private final StatusSignal<AngularVelocity> m_flywheelRightVelocitySignal;
    private final StatusSignal<Current> m_flywheelRightCurrentSignal;

    private final StatusSignal<Angle> m_hoodPositionSignal;
    private final StatusSignal<AngularVelocity> m_hoodVelocitySignal;
    private final StatusSignal<Current> m_hoodCurrentSignal;

    private final StatusSignal<AngularVelocity> m_kickerVelocitySignal;
    private final StatusSignal<Current> m_kickerCurrentSignal;

    private final DutyCycleOut m_flywheelDutyCycleRequest = new DutyCycleOut(0d);
    private final MotionMagicVelocityVoltage m_flywheelVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(flywheelFOC);
    private final MotionMagicVoltage m_hoodPositionRequest = new MotionMagicVoltage(m_hoodTargetPosition);
    private final DutyCycleOut m_kickerDutyCycleRequest = new DutyCycleOut(0d);

    public ShooterIOReal() {
        m_flywheelMotorLeft = new TalonFX(flywheelMotorIdLeft, Constants.CANBUS);
        m_flywheelMotorMiddleUpper = new TalonFX(flywheelMotorIdMiddleUpper, Constants.CANBUS);
        m_flywheelMotorMiddleLower = new TalonFX(flywheelMotorIdMiddleLower, Constants.CANBUS);
        m_flywheelMotorRight = new TalonFX(flywheelMotorIdRight, Constants.CANBUS);
        m_hoodMotor = new TalonFX(hoodMotorId, Constants.CANBUS);
        m_kickerMotor = new TalonFX(kickerMotorId, Constants.CANBUS);

        m_flywheelLeftVelocitySignal = m_flywheelMotorLeft.getVelocity();
        m_flywheelLeftCurrentSignal = m_flywheelMotorLeft.getSupplyCurrent();

        m_flywheelMiddleUpperVelocitySignal = m_flywheelMotorMiddleUpper.getVelocity();
        m_flywheelMiddleUpperCurrentSignal = m_flywheelMotorMiddleUpper.getSupplyCurrent();

        m_flywheelMiddleLowerVelocitySignal = m_flywheelMotorMiddleLower.getVelocity();
        m_flywheelMiddleLowerCurrentSignal = m_flywheelMotorMiddleLower.getSupplyCurrent();

        m_flywheelRightVelocitySignal = m_flywheelMotorRight.getVelocity();
        m_flywheelRightCurrentSignal = m_flywheelMotorRight.getSupplyCurrent();

        m_hoodPositionSignal = m_hoodMotor.getPosition();
        m_hoodVelocitySignal = m_hoodMotor.getVelocity();
        m_hoodCurrentSignal = m_hoodMotor.getSupplyCurrent();

        m_kickerVelocitySignal = m_kickerMotor.getVelocity();
        m_kickerCurrentSignal = m_kickerMotor.getSupplyCurrent();

        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = flywheelNeutralMode;

        // flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // flywheelConfig.CurrentLimits.SupplyCurrentLimit = flywheelCurrentLimit;

        flywheelConfig.Slot0.kP = flywheelPidP;
        flywheelConfig.Slot0.kV = flywheelPidV;

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
        
        PhoenixUtil.tryUntilOk(5, () -> m_kickerMotor.getConfigurator().apply(kickerConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorLeft.getConfigurator().apply(flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedLeft))));
        // FIXME: I think withInverted on all these is redundant because they're followers but I don't want to break anything and we're short on time
        PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorMiddleUpper.getConfigurator().apply(flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedMiddleUpper))));
        PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorMiddleLower.getConfigurator().apply(flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedMiddleLower))));
        PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorRight.getConfigurator().apply(flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedRight))));
        PhoenixUtil.tryUntilOk(5, () -> m_hoodMotor.getConfigurator().apply(hoodConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d,
            m_flywheelLeftVelocitySignal,
            m_flywheelLeftCurrentSignal,
            m_flywheelMiddleUpperVelocitySignal,
            m_flywheelMiddleUpperCurrentSignal,
            m_flywheelMiddleLowerVelocitySignal,
            m_flywheelMiddleLowerCurrentSignal,
            m_flywheelRightVelocitySignal,
            m_flywheelRightCurrentSignal,
            m_hoodVelocitySignal,
            m_hoodCurrentSignal,
            m_kickerVelocitySignal,
            m_kickerCurrentSignal);

        m_hoodMotor.setPosition(m_hoodTargetPosition);

        m_flywheelMotorMiddleUpper.setControl(new Follower(flywheelMotorIdLeft, flywheelInvertedMiddleUpper == flywheelInvertedLeft ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
        m_flywheelMotorMiddleLower.setControl(new Follower(flywheelMotorIdLeft, flywheelInvertedMiddleLower == flywheelInvertedLeft ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
        m_flywheelMotorRight.setControl(new Follower(flywheelMotorIdLeft, flywheelInvertedRight == flywheelInvertedLeft ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_flywheelLeftVelocitySignal,
            m_flywheelLeftCurrentSignal,
            m_flywheelMiddleUpperVelocitySignal,
            m_flywheelMiddleUpperCurrentSignal,
            m_flywheelMiddleLowerVelocitySignal,
            m_flywheelMiddleLowerCurrentSignal,
            m_flywheelRightVelocitySignal,
            m_flywheelRightCurrentSignal,
            m_hoodVelocitySignal,
            m_hoodCurrentSignal,
            m_kickerVelocitySignal,
            m_kickerCurrentSignal);
        
        inputs.flywheelTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_flywheelMotorLeft.getAppliedControl());

        inputs.flywheelMotorLeftDutyCycle = PhoenixUtil.getRequestDutyCycle(m_flywheelMotorLeft.getAppliedControl());
        inputs.flywheelMotorLeftVelocityRPS = m_flywheelLeftVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorLeftCurrentAmps = m_flywheelLeftCurrentSignal.getValueAsDouble();

        inputs.flywheelMotorMiddleUpperDutyCycle = PhoenixUtil.getRequestDutyCycle(m_flywheelMotorMiddleUpper.getAppliedControl());
        inputs.flywheelMotorMiddleUpperVelocityRPS = m_flywheelMiddleUpperVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorMiddleUpperCurrentAmps = m_flywheelMiddleUpperCurrentSignal.getValueAsDouble();

        inputs.flywheelMotorMiddleLowerDutyCycle = PhoenixUtil.getRequestDutyCycle(m_flywheelMotorMiddleLower.getAppliedControl());
        inputs.flywheelMotorMiddleLowerVelocityRPS = m_flywheelMiddleLowerVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorMiddleLowerCurrentAmps = m_flywheelMiddleLowerCurrentSignal.getValueAsDouble();

        inputs.flywheelMotorRightDutyCycle = PhoenixUtil.getRequestDutyCycle(m_flywheelMotorRight.getAppliedControl());
        inputs.flywheelMotorRightVelocityRPS = m_flywheelRightVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorRightCurrentAmps = m_flywheelRightCurrentSignal.getValueAsDouble();


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
        m_flywheelMotorLeft.disable();
        m_kickerMotor.disable();
    }

    @Override
    public void flywheelDutyCycle(double output) {
        m_flywheelMotorLeft.setControl(m_flywheelDutyCycleRequest.withOutput(output));
    }
    @Override
    public void flywheelVelocity(AngularVelocity velocity) {
        m_flywheelMotorLeft.setControl(m_flywheelVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void flywheelStop() {
        m_flywheelMotorLeft.disable();
    }

    @Override
    public void setHoodPosition(Angle angle) {
        m_hoodTargetPosition = angleToMechanismPosition(angle);
        m_hoodMotor.setControl(m_hoodPositionRequest.withPosition(m_hoodTargetPosition));
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
