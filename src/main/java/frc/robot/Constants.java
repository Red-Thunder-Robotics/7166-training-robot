// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
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

    private static final boolean USE_TWO_CONTROLLERS_DESIRED = false;
    public static final boolean USE_TWO_CONTROLLERS = !RobotBase.isSimulation() && USE_TWO_CONTROLLERS_DESIRED;
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = USE_TWO_CONTROLLERS ? 1 : DRIVER_CONTROLLER;

    public static final CANBus CANBUS = new CANBus("Canivore");

    public static final boolean USE_TURRET = false;
    public static final boolean USE_MACKINAC = true;

    public static final class FieldConstants {
        public static final Distance APRIL_TAG_WIDTH = Inches.of(6.5d);

        public static final Distance FUNNEL_RADIUS = Inches.of(24d);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72d - 56.4d);

        public static final Distance FIELD_LENGTH = Meters.of(ApriltagUtil.fieldLayout.getFieldLength());
        public static final Distance FIELD_WIDTH = Meters.of(ApriltagUtil.fieldLayout.getFieldWidth());

        public static final Distance FUEL_DIAMETER = Centimeters.of(15d);
    }

    public static final class DriveConstants {
        public static final Distance BUMPER_WIDTH = Inches.of(31d);
        public static final Distance BUMPER_LENGTH = BUMPER_WIDTH;

        public static final double SHOULD_INDEX_THRESHOLD_DEGREES = 4d;
    }
}
