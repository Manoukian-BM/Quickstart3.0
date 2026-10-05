package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class turretOdo {

    public enum EstadoTurret {
        MANUAL,
        AUTOMATICO
    }

    private EstadoTurret EstadoAtual = EstadoTurret.AUTOMATICO;
    boolean auto = true;
    boolean manual = false;
    private static final double CENTRO = 0.5;
    private static final double MAX_ANGLE_DEGREES = 150.0;
    private static final double SERVO_DIRECTION = 1.0;

    private final Follower follower;
    private final Servo turret;
    private final boolean redAlliance;

    private double targetX;
    private double targetY;
    private double servoPosition = CENTRO;
    private double angleDegrees;
    private String selectedHive = "Nenhuma";
    private boolean hasTarget;
    double posT = 0.5;

    public turretOdo(
            HardwareMap hardwareMap,
            Follower follower,
            boolean redAlliance) {
        this.follower = follower;
        this.redAlliance = redAlliance;
        turret = hardwareMap.get(Servo.class, "turret");
        turret.setPosition(CENTRO);
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

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        if (gamepad.y && manual) {
            EstadoAtual = EstadoTurret.AUTOMATICO;
        } else if (gamepad.y && auto) {
            EstadoAtual = EstadoTurret.MANUAL;
        }

        switch (EstadoAtual) {
            case MANUAL:
                if (Math.abs(gamepad2.right_stick_x) > 0.2) {
                    posT = 0.5 + gamepad.left_stick_y * 0.5;
                    turret.setPosition(posT);
                } else if (gamepad2.dpad_up) {
                    posT = 0.5;
                    turret.setPosition(posT);
                }

                break;
            case AUTOMATICO:
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
                turret.setPosition(servoPosition);

                break;
        }
        if (!hasTarget) {
            return;
        }
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