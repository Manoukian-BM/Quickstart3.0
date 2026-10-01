package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class turretOdo {
    private static final double CENTRO = 0.5;
    private static final double MAX_ANGLE_DEGREES = 150.0;
    private static final double SERVO_DIRECTION = 1.0;

    private final Follower follower;
    private final Servo servo;
    private final boolean redAlliance;

    private double targetX;
    private double targetY;
    private double servoPosition = CENTRO;
    private double angleDegrees;
    private String selectedHive = "Nenhuma";
    private boolean hasTarget;

    public turretOdo(
            HardwareMap hardwareMap,
            Follower follower,
            boolean redAlliance) {
        this.follower = follower;
        this.redAlliance = redAlliance;
        servo = hardwareMap.get(Servo.class, "servo1");
        servo.setPosition(CENTRO);
    }

    public void selectNearestHive() {
        Pose pose = follower.pose();

        double leftX = redAlliance ? 58.0 : 84.0;
        double leftY = redAlliance ? 55.0 : 88.0;
        double rightX = redAlliance ? 58.0 : 84.0;
        double rightY = redAlliance ? 88.0 : 55.0;

        double leftDx = leftX - pose.x();
        double leftDy = leftY - pose.y();
        double rightDx = rightX - pose.x();
        double rightDy = rightY - pose.y();

        // Comparação de distância por hipotenusa
        if (leftDx * leftDx + leftDy * leftDy
                <= rightDx * rightDx + rightDy * rightDy) {
            targetX = leftX;
            targetY = leftY;
            selectedHive = "Esquerda";
        } else {
            targetX = rightX;
            targetY = rightY;
            selectedHive = "Direita";
        }
        hasTarget = true;
    }

    public void update() {
        if (!hasTarget) {
            return;
        }

        //função de alinhamento da turret com a hive escolhida
        Pose pose = follower.pose();
        double angle = Math.atan2(targetY - pose.y(), targetX - pose.x())
                - pose.heading();
        angle = Math.atan2(Math.sin(angle), Math.cos(angle)); // normaliza o angulo no intervalo rad de -pi e +pi
        //evita que o mesmo número seja representado por valores diferentes (e.g 179 e −181)
        angleDegrees = Math.toDegrees(angle); //rad para graus

        double servoAngle = SERVO_DIRECTION * angleDegrees; // define como horario ou anti-horario
        double limitedAngle = Math.max(
                -MAX_ANGLE_DEGREES,
                Math.min(MAX_ANGLE_DEGREES, servoAngle)); //limite mecânico do servo (±150 graus)

        servoPosition = CENTRO + limitedAngle / 300.0; //converte angulo para posição do servo (0.0 a 1.0) (ex: limite de 300)
        servo.setPosition(servoPosition);
    }

    public String getSelectedHive() {
        return selectedHive;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }

    public double getServoPosition() {
        return servoPosition;
    }
}