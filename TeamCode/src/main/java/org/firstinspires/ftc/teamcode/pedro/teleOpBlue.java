package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Subsistemas
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.subsystems.servoSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.turretOdo;
import org.firstinspires.ftc.teamcode.pedro.subsystems.shooterSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.driveTrainSub;

@TeleOp(name = "TeleOpBlue", group = "aaa teleOp oficial")
public class teleOpBlue extends LinearOpMode {

    // ===== Subsistemas ====

    private driveTrainSub drive;
    private servoSub servo;
    private intakeSub intake;
    private shooterSub shooter;
    private turretOdo turret;

    private enum DriveState {
        MANUAL,
        AUTO_ALIGN,
        HOLD,
        AUTO_ALIGN_ESTACIONAMENTO,
        HOLD_ESTACIONAMENTO
    }
    //private Pose startPose = new Pose(13, 120, Math.toRadians(-90));

    private long holdStartTime = 0;
    private boolean pathIniciado = false;

    private DistanceSensor distIntake1, distIntake2, distIntake3;

    private Follower follower;

    private ElapsedTime timer;

    double idleVel = (1900 * 28) / 60.0;
    private boolean previousA;
    double pose = 0.5;

    // true para Red; false para Blue
    private static final boolean RED_ALLIANCE = false;

    @Override
    public void runOpMode() {
        double tempo = 0;

        boolean redAlliance = true; // false para Blue

        double hive1X = redAlliance ? 58.0 : 84.0;
        double hive1Y = redAlliance ? 55.0 : 88.0;

        double hive2X = redAlliance ? 58.0 : 84.0;
        double hive2Y = redAlliance ? 88.0 : 55.0;

        String chosenHive = "";
        double chosenHiveX = 0;
        double chosenHiveY = 0;

        turret = new turretOdo(hardwareMap, follower, false);
        shooter = new shooterSub(hardwareMap, hardwareMap.get(DcMotorEx.class, "indexer"));
        intake = new intakeSub(hardwareMap);
        drive = new driveTrainSub(hardwareMap);

//        distIntake1 = hardwareMap.get(DistanceSensor.class, "distIntake1");
//        distIntake2 = hardwareMap.get(DistanceSensor.class, "distIntake2");
//        distIntake3 = hardwareMap.get(DistanceSensor.class, "distIntake3");

        telemetry.addLine("TeleOp pronto.");
        telemetry.update();

        waitForStart();

        timer = new ElapsedTime();

        while (opModeIsActive()) {

//            double dI1 = distIntake1.getDistance(DistanceUnit.CM);
//            double dI2 = distIntake2.getDistance(DistanceUnit.CM);
//            double dI3 = distIntake3.getDistance(DistanceUnit.CM);


            Pose pose = follower.pose();
            turret.getSelectedHive();

            tempo = System.currentTimeMillis();

            //follower.update();

            // Escolhe a HIVE mais próxima quando pressionado.
//            if (gamepad1.a && !previousA) {
//                turret.selectNearestHive();
//            }
//            previousA = gamepad1.a;
//
//            // Enquanto pressionado, a turret segue a HIVE.
//            if (gamepad1.a) {
//                turret.update();
//            }
//

            // ===== subsystems =====

            intake.update(gamepad1, gamepad2, shooter.isAtirando());
            shooter.update(gamepad1, gamepad2, intake.isColetando(), intake.isCuspindo());
            turret.update(gamepad1, gamepad2);
            drive.teleopUpdate(gamepad1, gamepad2);

            // ===== Telemetria =====
            telemetry.addData("HIVE escolhida", turret.getSelectedHive());
            telemetry.addData("Ângulo relativo", turret.getAngleDegrees());
            telemetry.addData("Posição do servo", turret.getServoPosition());
            telemetry.addData("Follower Busy", follower.isBusy());
//            telemetry.addData("DistIntake1", dI1);
//            telemetry.addData("DistIntake1", dI2);
//            telemetry.addData("DistIntake2", dI3);
            telemetry.addData("tempo: ", timer);
            telemetry.update();

            telemetry.update();

        }
    }
}