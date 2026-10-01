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
import org.firstinspires.ftc.teamcode.pedro.subsystems.turretOdo;

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

    private turretOdo turret;
    private boolean previousA;

    // true para Red; false para Blue
    private static final boolean RED_ALLIANCE = true;

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

        turret = new turretOdo(hardwareMap, follower, RED_ALLIANCE);

        telemetry.addLine("TeleOp pronto.");
        telemetry.update();

        waitForStart();

        timer = new ElapsedTime();

        while (opModeIsActive()) {


            Pose pose = follower.pose();

            tempo = System.currentTimeMillis();

            follower.update();

            // Escolhe a HIVE mais próxima somente quando A é pressionado.
            if (gamepad1.a && !previousA) {
                turret.selectNearestHive();
            }
            previousA = gamepad1.a;

            // Enquanto A estiver pressionado, a turret acompanha a HIVE escolhida.
            if (gamepad1.a) {
                turret.update();
            }

            telemetry.addData("HIVE escolhida", turret.getSelectedHive());
            telemetry.addData("Ângulo relativo", turret.getAngleDegrees());
            telemetry.addData("Posição do servo", turret.getServoPosition());
            telemetry.update();

            // ===== Subsistemas =====


            // ===== Telemetria =====

            telemetry.update();

        }
    }
}