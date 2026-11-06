package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private long lastTelemetryNs;
    private DistanceSensor distIntake1;
    private DistanceSensor distIntake2;
    private DistanceSensor distIntake3;
    private DistanceSensor distIntake4;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createTeleBlue(hardwareMap);
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

        // Mesma posicao de subir usada pelo botao do P2 em servoSub.
        servo.setArmRaised(true);
        timer = new ElapsedTime();

        telemetry.addLine("TeleOp pronto. P1 D-pad direito: reset na marca (9.1,130.4,90 graus).");
        telemetry.addLine("Segure LB do P2 para acompanhar o alvo. R1 ajusta pelas duas tags do meio. R3 alterna o modo manual.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        telemetry.setMsTransmissionInterval(200);
        timer.reset();
        try {
        while (opModeIsActive()) {
            double dI1 = distIntake1.getDistance(DistanceUnit.CM);
            double dI2 = distIntake2.getDistance(DistanceUnit.CM);
            double dI3 = distIntake3.getDistance(DistanceUnit.CM);
            double dI4 = distIntake4.getDistance(DistanceUnit.CM);

            // Atualizar a pose uma vez por ciclo, com conducao pelo Pedro.
            // Segurar L1 do P1 reduz os tres comandos da base para 50%.
            double driveScale = gamepad1.left_bumper ? 0.5 : 1.0;
            follower.manual(-gamepad1.left_stick_y * driveScale,
                    -gamepad1.left_stick_x * driveScale, -gamepad1.right_stick_x * driveScale);
            follower.update();

            turret.update(gamepad1, gamepad2);
            double distanciaHive = turretOdo.getDist();

            servo.update(gamepad1, gamepad2, shooter.tiroPolen(), shooter.tiroPolen());
            boolean tiroSolicitado = gamepad2.right_trigger > 0.2 || gamepad2.left_trigger > 0.2
                    || shooter.ownsFeedCycle();
            intake.update(gamepad1, gamepad2, tiroSolicitado, dI1, dI2, dI3, dI4);
            shooter.update(gamepad1, gamepad2, distanciaHive, intake.isColetando(),
                    intake.isCuspindo(), distanciaHive);
            // Os motores de conducao sao comandados pelo follower acima.

            long telemetryNow=System.nanoTime();
            if (telemetryNow-lastTelemetryNs>=200_000_000L) {
            lastTelemetryNs=telemetryNow;
            turret.addDiagnostics(telemetry);
            telemetry.addData("Distancia usada pelo RPM (cm)", "%.1f", distanciaHive * 2.54);
            telemetry.addData("Distancia HIVE esquerda", turret.getLeftHiveDistance());
            telemetry.addData("Distancia HIVE direita", turret.getRightHiveDistance());
            telemetry.addData("Angulo relativo", turret.getAngleDegrees());
            telemetry.addData("Posicao do servo", turret.getServoPosition());
            telemetry.addData("Follower ocupado", follower.isBusy());
            telemetry.addData("Tempo", timer.seconds());
            telemetry.addData("right_stick_x", gamepad2.right_stick_x);
            telemetry.addData("RPM alvo pela odometria", shooter.getTargetRPM());
            telemetry.addData("RPM shooterL", shooter.getRPM());
            telemetry.addData("Shooter pronto", shooter.isReady());
            telemetry.addData("Alimentacao do tiro", shooter.getFeedStatus());
            telemetry.addData("Ajuste fino mira (graus)", turret.getAutoTrimDegrees());
            telemetry.addData("Potencia shooterR/L", "%.2f / %.2f", shooter.getLeftPower(), shooter.getRightPower());
//            telemetry.addData("DistIntake1", dI1);
//            telemetry.addData("DistIntake2", dI2);
//            telemetry.addData("DistIntake3", dI3);
//            telemetry.addData("DistIntake4", dI4);
            telemetry.update();
            }
        }
        } finally {
            shooter.stop();
            turret.close();
        }
    }
}
