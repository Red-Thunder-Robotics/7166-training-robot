package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.turret.TurretConstants.distanceAboveFunnel;
import static frc.robot.subsystems.turret.TurretConstants.robotToTurretTransform;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.state_machine.ShooterState;
import frc.robot.state_machine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ConversionUtil;

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

        if (StateMachine.getShooterState() == ShooterState.Shooting && TurretSubsystem.instance.hasTarget()) {
            // final ShooterParams params = calculateParams(TurretSubsystem.instance.getProjectedTargetDistance(), RPM.of(3500d));
            // m_io.setHoodPosition(params.degrees);
        }
    }

    public boolean shouldIndex() {
        return m_inputs.flywheelMotorVelocityRPS >= shouldIndexVelocityThresholdRPS;
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

    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    private static record ShooterParams(double rpm, double degrees) { };

    private static final InterpolatingTreeMap<Double, ShooterParams> PARAM_MAP =
        new InterpolatingTreeMap<Double, ShooterParams>(InverseInterpolator.forDouble(), (ShooterParams start, ShooterParams end, double q) ->
            new ShooterParams(
                MathUtil.interpolate(start.rpm, end.rpm, q),
                MathUtil.interpolate(end.degrees, end.degrees, q)
            )
        );

    static {
        PARAM_MAP.put(1d, new ShooterParams(0d, 0d));
    }

    // https://github.com/hammerheads5000/2026Rebuilt/blob/6ecae474f5ed81970d8727d2fe6b17e945a1f08f/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java#L56C1-L61C6
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }

    // https://github.com/hammerheads5000/2026Rebuilt/blob/6ecae474f5ed81970d8727d2fe6b17e945a1f08f/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java#L100C1-L127C1
    public static record ShotData(LinearVelocity velocity, Angle angle, Translation3d target) {};
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }
    public static ShotData calculateShotFromFunnelClearance(
        Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget)
    {
        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double y_dist = predictedTarget
                .getMeasureZ()
                .minus(robotToTurretTransform.getMeasureZ())
                .in(Inches);
        double g = 386;
        double r = FieldConstants.FUNNEL_RADIUS.in(Inches)
                * x_dist
                / getDistanceToTarget(robot, actualTarget).in(Inches);
        double h = FieldConstants.FUNNEL_HEIGHT.plus(distanceAboveFunnel).in(Inches);
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
        return new ShotData(InchesPerSecond.of(v0), Radians.of(theta), predictedTarget);
    }


    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    public ShooterParams calculateParams(Translation3d target, Distance distance) {
        var shotData = calculateShotFromFunnelClearance(Drive.instance.getPose(), target, target);

        ShooterParams baseline = PARAM_MAP.get(distance.in(Meters));
        double baselineVelocity = distance.in(Meters) / calculateTimeOfFlight(shotData.velocity, Degrees.of(baseline.degrees), distance).in(Seconds);

        Translation2d futurePos = Drive.instance.getPose().getTranslation().plus(
            ConversionUtil.chassisSpeedsToTranslation2d(Drive.instance.getChassisSpeeds().times(latencyCompensationSeconds))
        );

        Translation2d toGoal = target.toTranslation2d().minus(futurePos);
        double distance2 = toGoal.getNorm();
        Translation2d targetDirection = toGoal.div(distance2);

        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        Translation2d shotVelocity = targetVelocity.minus(ConversionUtil.chassisSpeedsToTranslation2d(Drive.instance.getChassisSpeeds()));

        // Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        double velocityRatio = requiredVelocity / baselineVelocity;

        double rpmFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);

        double adjustedRpm = baseline.rpm * rpmFactor;

        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.degrees));
        double targetHorizFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0d, 1d);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));

        return new ShooterParams(adjustedRpm, adjustedHood);
    }
}
