// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.ApriltagUtil;

public final class Constants {
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final int CONTROLLER = 0;

    public static final CANBus CANBUS = new CANBus("HighVoltageCANivore", "./logs/canivore.hoot");

    public static final boolean USE_TURRET = false;
    public static final boolean TWO_SHOOTER_MECHANISMS = !USE_TURRET;
    public static final boolean USE_NORTHSTAR = true;

    public static final class FieldConstants {
        public static final Distance APRIL_TAG_WIDTH = Inches.of(6.5d);

        public static final Distance FUNNEL_RADIUS = Inches.of(24d);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72d - 56.4d);

        public static final Distance FIELD_LENGTH = Meters.of(ApriltagUtil.fieldLayout.getFieldLength());
        public static final Distance FIELD_WIDTH = Meters.of(ApriltagUtil.fieldLayout.getFieldWidth());
    }
}
