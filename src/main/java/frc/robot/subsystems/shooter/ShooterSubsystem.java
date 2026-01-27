package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.turret.TurretConstants.distanceAboveFunnel;
import static frc.robot.subsystems.turret.TurretConstants.robotToTurretTransform;
import static frc.robot.subsystems.turret.TurretConstants.shouldIndexThresholdDegrees;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.shooter.ShooterConstants.InterpolationShooterParams;
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
            final InterpolationShooterParams params = calculateParams(TurretSubsystem.instance.getTargetPose().getTranslation());
            m_io.setHoodPosition(Degrees.of(params.degrees()));
            m_io.flywheelVelocity(RPM.of(params.rpm()));
        }
    }

    public Angle getHoodAngle() {
        return Degrees.of(m_inputs.hoodPositionDegrees);
    }

    @AutoLogOutput(key="ShooterShouldIndex")
    public boolean shouldIndex() {
        return Math.abs(m_inputs.flywheelTargetVelocityRPS - m_inputs.flywheelMotorVelocityRPS) <= shouldIndexVelocityThresholdRPS;
    }

    private LinearVelocity m_exitVelocity = MetersPerSecond.of(0d);
    public LinearVelocity getExitVelocity() {
        return m_exitVelocity;
    }
    // public LinearVelocity getExitVelocity() {
    //     return InchesPerSecond.of(m_inputs.flywheelMotorVelocityRPS * (2d * Math.PI) * flywheelRadius.in(Inches));
    // }

    public void stateUpdate(ShooterState shooterState) {
        switch (shooterState) {
            case Idle:
                // m_io.flywheelDutyCycle(flywheelOutput);
                m_io.flywheelStop();
                m_io.kickerStop();
                break;
            case Shooting:
                // m_io.flywheelDutyCycle(flywheelOutput);
                m_io.kickerDutyCycle(kickerOutput);
                break;
        }
    }

    // https://github.com/hammerheads5000/2026Rebuilt/blob/6ecae474f5ed81970d8727d2fe6b17e945a1f08f/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java#L56C1-L61C6
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }

    // https://github.com/hammerheads5000/2026Rebuilt/blob/6ecae474f5ed81970d8727d2fe6b17e945a1f08f/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java#L100C1-L127C1
    public static record PhysicsShotData(LinearVelocity velocity, Angle angle, Translation3d target) {};
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }
    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }
    public static PhysicsShotData calculateShotFromFunnelClearance(
        Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget)
    {
        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double z_dist = predictedTarget
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
        double D1 = z_dist;
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
        return new PhysicsShotData(InchesPerSecond.of(v0), Radians.of(theta), predictedTarget);
    }


    public InterpolationShooterParams calculateParams(Translation3d target) {
        final var robotPose = Drive.instance.getPose();
        final var fieldSpeeds = Drive.instance.getChassisSpeeds();
        
        // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
        // // FIXME: using shotData.velocity is inaccurate because we aren't targeting that (we're targeting baseline from parameter map)
        // // so we need to get a conversion from rotations per minute to exit velocity and then discard calculateShotFromFunnelClearance
        // var shotData = calculateShotFromFunnelClearance(robotPose, target, target);
        // m_exitVelocity = shotData.velocity;

        // final Distance distance = getDistanceToTarget(robotPose, target);

        // ShooterParams baseline = paramMap.map.get(distance.in(Meters));
        // double baselineVelocity = distance.in(Meters) / calculateTimeOfFlight(m_exitVelocity, Degrees.of(baseline.degrees()), distance).in(Seconds);

        // Translation2d futurePose = robotPose.getTranslation().plus(
        //     ConversionUtil.chassisSpeedsToTranslation2d(Drive.instance.getChassisSpeeds().times(latencyCompensationSeconds))
        // );

        // Translation2d toGoal = target.toTranslation2d().minus(futurePose);
        // Translation2d targetDirection = toGoal.div(toGoal.getNorm());

        // Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // Translation2d shotVelocity = targetVelocity.minus(ConversionUtil.chassisSpeedsToTranslation2d(Drive.instance.getChassisSpeeds()));

        // // Rotation2d turretAngle = shotVelocity.getAngle();
        // double requiredVelocity = shotVelocity.getNorm();

        // double velocityRatio = requiredVelocity / baselineVelocity;

        // double rpmFactor = Math.sqrt(velocityRatio);
        // double hoodFactor = Math.sqrt(velocityRatio);

        // double adjustedRpm = baseline.rpm() * rpmFactor;

        // double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.degrees()));
        // double targetHorizFromHood = baselineVelocity * hoodFactor;
        // double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0d, 1d);
        // double adjustedHood = Math.toDegrees(Math.acos(ratio));

        // return new ShooterParams(adjustedRpm, adjustedHood);

        // https://github.com/hammerheads5000/2026Rebuilt/blob/9a94e647443d8a5651b044449cc5ebb8195efc52/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java#L129
        final int iterations = 3;

        PhysicsShotData shot = calculateShotFromFunnelClearance(robotPose, target, target);
        Distance distance = getDistanceToTarget(robotPose, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.velocity, shot.angle, distance);
        Translation3d predictedTarget = target;

        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robotPose, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                    shot.velocity, shot.angle, getDistanceToTarget(robotPose, predictedTarget));
        }

        m_exitVelocity = shot.velocity;

        return new InterpolationShooterParams(
            RadiansPerSecond.of(m_exitVelocity.in(MetersPerSecond) / flywheelRadius.in(Meters)).in(RotationsPerSecond),
            shot.angle.in(Degrees)
        );
    }
}
