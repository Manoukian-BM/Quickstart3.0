package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Subsistemas
import org.firstinspires.ftc.teamcode.pedro.subsystems.IntakeSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.shooterSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.servoSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.driveTrainSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.HiveTurretAligner;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "TeleOp", group = "Advanced")
public class teleOp extends LinearOpMode {

    private enum DriveState {
        MANUAL,
        AUTO_ALIGN,
        HOLD,
        AUTO_ALIGN_ESTACIONAMENTO,
        HOLD_ESTACIONAMENTO
    }


    private static final long HOLD_TIME_MS = 800;
    private long holdStartTime = 0;
    private boolean pathIniciado = false;

    private Servo servo1;
    private RevBlinkinLedDriver led;

    private Follower follower;

    private ElapsedTime timer;

    double idleVelLonge = (2450 * 28) / 60.0;

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "servo1");

        double tempo = 0;

        boolean redAlliance = true; // false para Blue

        double hive1X = redAlliance ? 58.0 : 84.0;
        double hive1Y = redAlliance ? 55.0 : 88.0;

        double hive2X = redAlliance ? 58.0 : 84.0;
        double hive2Y = redAlliance ? 88.0 : 55.0;

        String chosenHive = "";
        double chosenHiveX = 0;
        double chosenHiveY = 0;

        telemetry.addLine("TeleOp pronto.");
        telemetry.update();

        waitForStart();

        timer = new ElapsedTime();

        while (opModeIsActive()) {

            Pose pose = follower.pose();

            double distanceToHive1 = Math.pow(hive1X - pose.x(), 2)
                    + Math.pow(hive1Y - pose.y(), 2);

            double distanceToHive2 = Math.pow(hive2X - pose.x(), 2)
                    + Math.pow(hive2Y - pose.y(), 2);

            // Escolhe a HIVE da aliança mais perto.
            if (gamepad1.a) {
                if (distanceToHive1 <= distanceToHive2) {
                    chosenHive = "Esquerda";
                    chosenHiveX = hive1X;
                    chosenHiveY = hive1Y;
                } else {
                    chosenHive = "Direita";
                    chosenHiveX = hive2X;
                    chosenHiveY = hive2Y;
                }

                // Calcula o ângulo entre o robô e a HIVE.
                double dx = chosenHiveX - pose.x();
                double dy = chosenHiveY - pose.y();

                double angleToHive = Math.atan2(dy, dx) - pose.heading();
                angleToHive = Math.atan2(Math.sin(angleToHive), Math.cos(angleToHive));

                double angleDegrees = Math.toDegrees(angleToHive);

                // Converte para o servo Taura: 0.5 = frente; curso aproximado de ±150°.
                double limitAngle = Math.max(-150.0, Math.min(150.0, angleDegrees));
                double servoPosition = 0.5 + limitAngle / 300.0;

                servo1.setPosition(servoPosition);

                telemetry.addData("Aliança", redAlliance ? "Red" : "Blue");
                telemetry.addData("HIVE escolhida", chosenHive);
                telemetry.addData("Distância HIVE esquerda", Math.sqrt(distanceToHive1));
                telemetry.addData("Distância HIVE direita", Math.sqrt(distanceToHive2));
                telemetry.addData("Ângulo relativo", angleDegrees);
                telemetry.addData("Servo", servoPosition);
                telemetry.update();
            }

            tempo = System.currentTimeMillis();

            follower.update();

            // ===== Subsistemas =====


            // ===== Telemetria =====

            telemetry.update();

        }
    }

    public void turretAlign() {
        // Coordenadas da HIVE no sistema de campo do Pedro Pathing

        // Red esquerda = (58, 55)
        // Red direita = (58, 88)
        // Blue esquerda = (84, 55)
        // Blue direita = (84, 88)
        double hiveX = 24.0;
        double hiveY = 36.0;

        // No loop, atualize a localização antes de ler a pose.
        // Se usar Follower, normalmente chame follower.update().
        Pose pose = follower.pose();

        double dx = hiveX - pose.x();
        double dy = hiveY - pose.y();

        // O heading do Pedro Pathing está em radianos.
        double angleToHive = Math.atan2(dy, dx) - pose.heading();

        // Normaliza o ângulo para -PI a +PI.
        angleToHive = Math.atan2(
                Math.sin(angleToHive),
                Math.cos(angleToHive)
        );

        double angleDegrees = Math.toDegrees(angleToHive);

        // O servo alcança aproximadamente -150° a +150°.
        // A posição 0.5 aponta para frente; o curso total de 300° ocupa 0.0 a 1.0.
        double limitedAngle = Math.max(-150.0, Math.min(150.0, angleDegrees));
        double servoPosition = 0.5 + limitedAngle / 300.0;

        servo1.setPosition(servoPosition);

        telemetry.addData("Angulo HIVE", angleDegrees);
        telemetry.addData("Posicao servo", servoPosition);
        telemetry.update();
    }
}