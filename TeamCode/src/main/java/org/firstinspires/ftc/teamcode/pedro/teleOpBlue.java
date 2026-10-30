package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.subsystems.driveTrainSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.servoSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.shooterSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.turretOdo;

@TeleOp(name = "TeleOpBlue", group = "aaa teleOp oficial")
public class teleOpBlue extends LinearOpMode {

    private static final Pose START_POSE = new Pose(19, 119, Math.toRadians(-90));

    private driveTrainSub drive;
    private servoSub servo;
    private intakeSub intake;
    private shooterSub shooter;
    private turretOdo turret;
    private Follower follower;
    private ElapsedTime timer;
    private Servo servoAR;
    private Servo servoAL;
    private DistanceSensor distIntake1;
    private DistanceSensor distIntake2;
    private DistanceSensor distIntake3;
    private DistanceSensor distIntake4;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        if (follower == null) {
            telemetry.addLine("Pedro Pathing Follower nao configurado em Constants.create().");
            telemetry.update();
            requestOpModeStop();
            return;
        }
        follower.setPose(START_POSE);

        servo = new servoSub(hardwareMap);
        shooter = new shooterSub(hardwareMap, hardwareMap.get(DcMotorEx.class, "indexer"));
        intake = new intakeSub(hardwareMap);
        drive = new driveTrainSub(hardwareMap);
        turret = new turretOdo(hardwareMap, follower);

        distIntake1 = hardwareMap.get(DistanceSensor.class, "distIntake1");
        distIntake2 = hardwareMap.get(DistanceSensor.class, "distIntake2");
        distIntake3 = hardwareMap.get(DistanceSensor.class, "distIntake3");
        distIntake4 = hardwareMap.get(DistanceSensor.class, "distIntake4");
        servoAR = hardwareMap.get(Servo.class, "servoAR");
        servoAL = hardwareMap.get(Servo.class, "servoAL");

        servoAR.setPosition(0.93);
        servoAL.setPosition(0.83);
        timer = new ElapsedTime();

        telemetry.addLine("TeleOp pronto.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        timer.reset();
        while (opModeIsActive()) {
            double dI1 = distIntake1.getDistance(DistanceUnit.CM);
            double dI2 = distIntake2.getDistance(DistanceUnit.CM);
            double dI3 = distIntake3.getDistance(DistanceUnit.CM);
            double dI4 = distIntake4.getDistance(DistanceUnit.CM);

            follower.update();
            Pose currentPose = follower.pose();

            turret.update(gamepad1, gamepad2);
            double distanciaHive = turretOdo.getDist();

            servo.update(gamepad1, gamepad2, shooter.tiroPolen(), shooter.tiroPolen());
            intake.update(gamepad1, gamepad2, shooter.isAtirando(), dI1, dI2, dI3, dI4);
            shooter.update(gamepad1, gamepad2, turretOdo.getDist(), intake.isColetando(),
                    intake.isCuspindo(), turretOdo.getDist());
            drive.teleopUpdate(gamepad1, gamepad2);

            telemetry.addData("HIVE escolhida", turret.getSelectedHive());
            telemetry.addData("Distancia da HIVE", turret.getDist());
            telemetry.addData("Distancia HIVE direita", turret.getLeftHiveDistance());
            telemetry.addData("Distancia HIVE esquerda", turret.getRightHiveDistance());
            telemetry.addData("Angulo relativo", turret.getAngleDegrees());
            telemetry.addData("Posicao do servo", turret.getServoPosition());
            telemetry.addData("Follower ocupado", follower.isBusy());
            telemetry.addData("X", currentPose.x());
            telemetry.addData("Y", currentPose.y());
            telemetry.addData("Aangulo", Math.toDegrees(currentPose.heading()));
            telemetry.addData("Tempo", timer.seconds());
            telemetry.addData("right_stick_x", gamepad2.right_stick_x);
            telemetry.addData("RPM", shooter.getRPM());
//            telemetry.addData("DistIntake1", dI1);
//            telemetry.addData("DistIntake2", dI2);
//            telemetry.addData("DistIntake3", dI3);
//            telemetry.addData("DistIntake4", dI4);
            telemetry.update();
        }
    }
}
