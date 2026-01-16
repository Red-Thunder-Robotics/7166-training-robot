package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public final class SimulationCommands {
    public static class SimFuelCommand extends Command {
        private static int globalId = 0;
        private int m_id = ++globalId;

        private final Timer m_timer = new Timer();

        private String m_name;
        private Pose3d m_initialPose = new Pose3d();
        private Pose3d m_pose = m_initialPose;
        private Twist3d m_initialVelocity;

        private Supplier<Pose3d> m_initialPoseSupplier;
        private Supplier<Twist3d> m_initialVelocitySupplier;

        public SimFuelCommand(Supplier<Pose3d> startPoseSupplier, Supplier<Twist3d> initialVelocitySupplier) {
            m_initialPoseSupplier = startPoseSupplier;
            m_initialVelocitySupplier = initialVelocitySupplier;

            m_name = "SimFuel" + m_id;
        }

        @Override
        public void initialize() {
            m_timer.restart();

            m_initialPose = m_initialPoseSupplier.get();
            m_pose = m_initialPose;
            m_initialVelocity = m_initialVelocitySupplier.get();
        }

        @Override
        public void execute() {
            final double t = m_timer.get();

            final double xpos = m_initialPose.getX() + m_initialVelocity.dx * t;
            final double ypos = m_initialPose.getY() + m_initialVelocity.dy * t;
            final double zpos = m_initialPose.getZ()
                    + m_initialVelocity.dz * t
                    - 0.5d * (9.81d) * t * t;

            m_pose = new Pose3d(new Translation3d(xpos, ypos, zpos), Rotation3d.kZero);
            
            Logger.recordOutput(m_name, m_pose);
        }

        @Override
        public boolean isFinished() {
            return m_pose.getZ() < 0d;
            // return m_timer.hasElapsed(3d);
        }
    }
}
