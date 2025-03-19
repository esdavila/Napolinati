package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    public static final double LOOP_TIME = 0.020; // 20ms

    public static final class Drivetrain {

        public static final int LEFT_PRIMARY_MOTOR_ID = 3;
        public static final int LEFT_SECONDARY_MOTOR_ID = 4;
        public static final int RIGHT_PRIMARY_MOTOR_ID = 1;
        public static final int RIGHT_SECONDARY_MOTOR_ID = 2;

        public static final boolean LEFT_DRIVE_MOTORS_INVERTED = false;
        public static final boolean RIGHT_DRIVE_MOTORS_INVERTED = true;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22);

        public static final double GEARING = 8.45;
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3) / 0.9788265228;
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final int CURRENT_LIMIT = 60;
        public static final DCMotor GEARBOX_REPR = DCMotor.getNEO(2);
        public static final double MOI = 2;
        public static final double MASS_KG = 32;
        public static final double WHEEL_COF = 1.0;

        public static final double VELOCITY_kP = RobotBase.isReal() ? 1.5 : 1.4078;
        public static final double VELOCITY_kI = 0.0;
        public static final double VELOCITY_kD = 0.0;
        public static final double kS = RobotBase.isReal() ? 0.29971 : 0.057644;
        public static final double kV_LINEAR = RobotBase.isReal() ? 2.4 : 2.159;
        public static final double kA_LINEAR = RobotBase.isReal() ? 0.56555 : 0.24174;
        public static final double kV_ANGULAR = RobotBase.isReal() ? 1.2427 : 2.1403;
        public static final double kA_ANGULAR = RobotBase.isReal() ? 0.0859 : 0.42056;

        public static final double POSITION_kP = 1.5;
        public static final double POSITION_kI = 0.0;
        public static final double POSITION_kD = 0.0;
        public static final double ROTATION_kP = 2.0;
        public static final double ROTATION_kI = 0.0;
        public static final double ROTATION_kD = 0.0;

        public static final double RAMSETE_B = 3;
        public static final double RAMSETE_ZETA = 0.7;

        public static final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5;
        public static final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 20;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 40;

        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                MASS_KG, MOI,
                new ModuleConfig(
                        WHEEL_RADIUS_METERS,
                        MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                        WHEEL_COF, GEARBOX_REPR, CURRENT_LIMIT, 2),
                TRACK_WIDTH_METERS);
    }

    public static final class Controls {
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
        public static final double JOYSTICK_INPUT_DEADBAND = 0.1;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_CURVE_EXP = 3;
        public static final double JOYSTICK_ROT_LIMIT = 0.8;
    }
    
}
