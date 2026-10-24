package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Standalone bench test for combining Pedro Pathing pose data with Limelight
 * smart-group correction. This OpMode does not replace turretOdo.
 */
@TeleOp(name = "Test: Turret Kalman Filter", group = "Test")
public class turretKalmanFilterTest extends LinearOpMode {

    private static final String TURRET_SERVO_NAME = "turret";
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int LIMELIGHT_PIPELINE = 1;

    // Set this to match the alliance and coordinate convention used by the robot.
    private static final boolean RED_ALLIANCE = false;

    // Hive coordinates copied from turretOdo.java, in inches.
    private static final double LEFT_HIVE_X = RED_ALLIANCE ? 58.0 : 84.0;
    private static final double LEFT_HIVE_Y = RED_ALLIANCE ? 55.0 : 88.0;
    private static final double RIGHT_HIVE_X = RED_ALLIANCE ? 58.0 : 84.0;
    private static final double RIGHT_HIVE_Y = RED_ALLIANCE ? 88.0 : 55.0;

    // Set these to the robot's actual starting field pose before running.
    private static final double START_X_INCHES = 0.0;
    private static final double START_Y_INCHES = 0.0;
    private static final double START_HEADING_RADIANS = 0.0;

    private static final double HIVE_SWITCH_MARGIN_INCHES = 6.0;
    private static final double TURRET_ZERO_OFFSET_DEGREES = 0.0;

    // Set to -1 if the on-robot sign test shows the correction is reversed.
    private static final double TX_SIGN = 1.0;

    private static final double MIN_TURRET_ANGLE_DEGREES = -180.0;
    private static final double MAX_TURRET_ANGLE_DEGREES = 180.0;
    private static final double SERVO_CENTER = 0.5;

    // Kalman filter parameters, in degrees squared.
    private static final double PROCESS_NOISE_Q = 0.15;
    private static final double MEASUREMENT_NOISE_R = 9.0;
    private static final double MAX_INNOVATION_DEGREES = 35.0;
    private static final long CORRECTION_HOLD_MILLISECONDS = 300;

    private enum Hive {
        LEFT,
        RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turret = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        Limelight3A limelight =
                hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(
                START_X_INCHES,
                START_Y_INCHES,
                START_HEADING_RADIANS
        ));

        AngularKalmanFilter filter = new AngularKalmanFilter();
        Hive selectedHive = null;
        long lastValidTagTime = 0L;
        double commandedTurretAngleDegrees = 0.0;
        turret.setPosition(SERVO_CENTER);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.addLine("Standalone turret Kalman filter test");
        telemetry.addLine("Uses Follower from Constants.create(hardwareMap).");
        telemetry.addLine("Check Follower configuration and start pose.");
        telemetry.addLine("Press Play, then move the robot/turret safely.");
        telemetry.update();

        try {
            waitForStart();

            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.pose();

                double leftDistance = distance(
                        pose.x(), pose.y(), LEFT_HIVE_X, LEFT_HIVE_Y
                );
                double rightDistance = distance(
                        pose.x(), pose.y(), RIGHT_HIVE_X, RIGHT_HIVE_Y
                );

                Hive previousHive = selectedHive;
                selectedHive = chooseHive(
                        selectedHive, leftDistance, rightDistance
                );
                if (selectedHive != previousHive && previousHive != null) {
                    filter.reset();
                    lastValidTagTime = 0L;
                }

                double targetX = selectedHive == Hive.LEFT
                        ? LEFT_HIVE_X : RIGHT_HIVE_X;
                double targetY = selectedHive == Hive.LEFT
                        ? LEFT_HIVE_Y : RIGHT_HIVE_Y;

                double fieldBearingRadians = Math.atan2(
                        targetY - pose.y(),
                        targetX - pose.x()
                );
                double odometryAimDegrees = wrapDegrees(
                        Math.toDegrees(fieldBearingRadians - pose.heading())
                                + TURRET_ZERO_OFFSET_DEGREES
                );

                LLResult result = limelight.getLatestResult();
                Double selectedTxDegrees = null;

                if (result != null && result.isValid()) {
                    double groupTxDegrees = result.getTx();
                    if (Double.isFinite(groupTxDegrees)) {
                        selectedTxDegrees = groupTxDegrees;
                    }
                }

                if (selectedTxDegrees != null) {
                    // The camera rotates with the turret. tx is the remaining
                    // horizontal error from the current turret direction.
                    double observedAimDegrees = wrapDegrees(
                            commandedTurretAngleDegrees
                                    + TX_SIGN * selectedTxDegrees
                    );
                    double measuredCorrectionDegrees = wrapDegrees(
                            observedAimDegrees - odometryAimDegrees
                    );

                    filter.update(measuredCorrectionDegrees);
                    lastValidTagTime = System.currentTimeMillis();
                }

                boolean correctionIsFresh =
                        System.currentTimeMillis() - lastValidTagTime
                                <= CORRECTION_HOLD_MILLISECONDS;
                double correctionDegrees = correctionIsFresh
                        ? filter.getCorrectionDegrees() : 0.0;

                double desiredAngleDegrees = clamp(
                        wrapDegrees(odometryAimDegrees + correctionDegrees),
                        MIN_TURRET_ANGLE_DEGREES,
                        MAX_TURRET_ANGLE_DEGREES
                );

                // Calibrated mapping: -180 -> 0, 0 -> 0.5, +180 -> 1.
                double servoPosition = clamp(
                        SERVO_CENTER + desiredAngleDegrees / 360.0,
                        0.0,
                        1.0
                );
                turret.setPosition(servoPosition);
                commandedTurretAngleDegrees = desiredAngleDegrees;

                telemetry.addData("Selected hive", selectedHive);
                telemetry.addData("Left distance (in)", "%.1f", leftDistance);
                telemetry.addData("Right distance (in)", "%.1f", rightDistance);
                telemetry.addData(
                        "Pedro Pathing pose",
                        "x %.1f, y %.1f, heading %.1f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading())
                );
                telemetry.addData("Odometry aim (deg)", "%.1f", odometryAimDegrees);
                telemetry.addData("Smart group tx (deg)", selectedTxDegrees);
                telemetry.addData(
                        "Kalman correction (deg)",
                        correctionIsFresh
                                ? String.format("%.2f", filter.getCorrectionDegrees())
                                : "stale"
                );
                telemetry.addData("Commanded angle (deg)", "%.1f", desiredAngleDegrees);
                telemetry.addData("Servo position", "%.3f", servoPosition);
                telemetry.update();
            }
        } finally {
            limelight.stop();
        }
    }

    private static Hive chooseHive(
            Hive currentHive,
            double leftDistance,
            double rightDistance
    ) {
        if (currentHive == null) {
            return leftDistance <= rightDistance ? Hive.LEFT : Hive.RIGHT;
        }

        if (currentHive == Hive.LEFT
                && rightDistance + HIVE_SWITCH_MARGIN_INCHES < leftDistance) {
            return Hive.RIGHT;
        }
        if (currentHive == Hive.RIGHT
                && leftDistance + HIVE_SWITCH_MARGIN_INCHES < rightDistance) {
            return Hive.LEFT;
        }
        return currentHive;
    }

    private static double distance(
            double x1, double y1, double x2, double y2
    ) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    private static double wrapDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private static class AngularKalmanFilter {
        private double correctionDegrees;
        private double variance = 25.0;

        void update(double measurementDegrees) {
            double innovation = wrapDegrees(
                    measurementDegrees - correctionDegrees
            );

            if (Math.abs(innovation) > MAX_INNOVATION_DEGREES) {
                variance += PROCESS_NOISE_Q;
                return;
            }

            variance += PROCESS_NOISE_Q;
            double gain = variance / (variance + MEASUREMENT_NOISE_R);
            correctionDegrees = wrapDegrees(
                    correctionDegrees + gain * innovation
            );
            variance *= 1.0 - gain;
        }

        double getCorrectionDegrees() {
            return correctionDegrees;
        }

        void reset() {
            correctionDegrees = 0.0;
            variance = 25.0;
        }
    }
}
