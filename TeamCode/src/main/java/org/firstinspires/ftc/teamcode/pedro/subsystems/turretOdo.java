package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.math.Pose.interpolate;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class turretOdo {

    public enum EstadoTurret {
        MANUAL,
        AUTOMATICO
    }

    Pose upperPose = new Pose(0.0, 0.0, 50.0);

    private static final double CENTRO = 0.5;
    private static final double MAX_ANGLE_DEGREES = 180.0;
    private static final double SERVO_RANGE_DEGREES = 360.0;
    private static final double SERVO_DIRECTION = 1.0;
    private static final double MANUAL_SERVO_RATE = 0.5;

    private final Follower follower;
    private final Servo turret;
    //private Limelight3A limelight;

    private EstadoTurret estadoAtual = EstadoTurret.AUTOMATICO;
    private double targetX;
    private double targetY;
    private double servoPosition = CENTRO;
    private double angleDegrees;
    private static double distSelected;
    private double leftHiveDistance;
    private double rightHiveDistance;
    private double manualPosition = CENTRO;
    private long previousUpdateNanos;
    private String selectedHive = "Nenhuma";
    private boolean hasTarget;
    private boolean previousY;

    public turretOdo(HardwareMap hardwareMap, Follower follower) {
        if (follower == null) {
            throw new IllegalArgumentException("Follower nao pode ser null.");
        }
        this.follower = follower;
        turret = hardwareMap.get(Servo.class, "turret");
        //Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret.setPosition(CENTRO);
        previousUpdateNanos = System.nanoTime();

        //limelight.pipelineSwitch(1);
        //limelight.start();
    }

    public void selecionarHive() {
        selecionarHive(follower.pose());
    }

    private void selecionarHive(Pose pose) {
        double ratio = clamp(pose.x() / 50.0, -0.6, 0.6);

        double leftHiveX = 84.0 - ratio;
        double leftHiveY = 54.0;
        double rightHiveX = 81.0 - ratio;
        double rightHiveY = 91.0;

        leftHiveDistance = distance(pose.x(), pose.y(), leftHiveX, leftHiveY);
        rightHiveDistance = distance(pose.x(), pose.y(), rightHiveX, rightHiveY);

        if (pose.y() < 72.0) {
            targetX = leftHiveX;
            targetY = leftHiveY;
            selectedHive = "Esquerda";
            distSelected = leftHiveDistance;
        } else {
            targetX = rightHiveX;
            targetY = rightHiveY;
            selectedHive = "Direita";
            distSelected = rightHiveDistance;
        }

        hasTarget = true;
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        //LLResult result = limelight.getLatestResult();
        if (gamepad2.y && !previousY) {
            estadoAtual = estadoAtual == EstadoTurret.MANUAL
                    ? EstadoTurret.AUTOMATICO
                    : EstadoTurret.MANUAL;
        }
        previousY = gamepad2.y;

        if (estadoAtual == EstadoTurret.MANUAL) {
            updateManual(gamepad2);
            return;
        }

        Pose pose = follower.pose();
        if (gamepad2.left_bumper) {
//            if (!result.isValid() && result != null) {
//                //a lógica da limelight entra aqui
//            } else {
                selecionarHive(pose);
                if (!hasTarget) {
                    return;
                }

                double angle = Math.atan2(targetY - pose.y(), targetX - pose.x()) - pose.heading();
                angle = Math.atan2(Math.sin(angle), Math.cos(angle));
                angleDegrees = Math.toDegrees(angle);

                double limitedAngle = clamp(
                        SERVO_DIRECTION * angleDegrees,
                        -MAX_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES);
                servoPosition = clamp(
                        CENTRO + limitedAngle / SERVO_RANGE_DEGREES,
                        0.0,
                        1.0);
                manualPosition = servoPosition;
                turret.setPosition(servoPosition);
            //}
        }
    }

    private void updateManual(Gamepad gamepad) {
        long now = System.nanoTime();
        double elapsedSeconds = Math.min((now - previousUpdateNanos)
                / 1_000_000_000.0, 0.1);
        previousUpdateNanos = now;

        if (Math.abs(gamepad.right_stick_x) > 0.2) {
            manualPosition = clamp(
                    manualPosition + gamepad.right_stick_x
                            * MANUAL_SERVO_RATE * elapsedSeconds,
                    0.0,
                    1.0);
        } else if (gamepad.dpad_up) {
            manualPosition = CENTRO;
        }
        servoPosition = manualPosition;
        turret.setPosition(servoPosition);
    }

    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public String getSelectedHive() {
        return selectedHive;
    }

    public static double getDist() {
        return distSelected;
    }

    public double getLeftHiveDistance() {
        return leftHiveDistance;
    }

    public double getRightHiveDistance() {
        return rightHiveDistance;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }

    public double getServoPosition() {
        return servoPosition;
    }
}
