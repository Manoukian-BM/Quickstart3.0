package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Subsistemas
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.subsystems.servoSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.turretOdo;
import org.firstinspires.ftc.teamcode.pedro.subsystems.shooterSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.pedro.subsystems.driveTrainSub;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpBlue", group = "aaa teleOp oficial")
public class teleOpBlue extends LinearOpMode {

    // ===== Subsistemas ====

    private driveTrainSub drive;
    private servoSub servo;
    private intakeSub intake;
    private shooterSub shooter;
    private Servo turret;

    //private turretOdo turret;

    private Limelight3A limelight;
    //private Pose startPose = new Pose(13, 120, Math.toRadians(-90));

    private static final int[] TARGET_IDS = {42, 43, 44, 45};
    private static final int[] TARGET_ID2 = {38, 39, 40, 41};

    private GoBildaPinpointDriver pinpoint;

    private long holdStartTime = 0;
    private boolean pathIniciado = false;

    private DistanceSensor distIntake1, distIntake2, distIntake3, distIntake4;;

    //private Follower follower;

    private ElapsedTime timer;
    private Servo servoAR, servoAL;

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


        //turret = new turretOdo(hardwareMap, follower, false);
        servo = new servoSub(hardwareMap);
        shooter = new shooterSub(hardwareMap, hardwareMap.get(DcMotorEx.class, "indexer"));
        intake = new intakeSub(hardwareMap);
        drive = new driveTrainSub(hardwareMap);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        distIntake1 = hardwareMap.get(DistanceSensor.class, "distIntake1");
        distIntake2 = hardwareMap.get(DistanceSensor.class, "distIntake2");
        distIntake3 = hardwareMap.get(DistanceSensor.class, "distIntake3");
        distIntake4 = hardwareMap.get(DistanceSensor.class, "distIntake4");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        servoAR = hardwareMap.get(Servo.class, "servoAR");
        servoAL = hardwareMap.get(Servo.class, "servoAL");


        turret = hardwareMap.get(Servo.class, "turret");

        telemetry.addLine("TeleOp pronto.");
        telemetry.update();

        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(30);

        waitForStart();

        if (isStopRequested()) return;

        limelight.start();

        timer = new ElapsedTime();

        servoAR.setPosition(0.6);
        servoAL.setPosition(0.6);

        while (opModeIsActive()) {

            pinpoint.update();

            double dI1 = distIntake1.getDistance(DistanceUnit.CM);
            double dI2 = distIntake2.getDistance(DistanceUnit.CM);
            double dI3 = distIntake3.getDistance(DistanceUnit.CM);
            double dI4 = distIntake4.getDistance(DistanceUnit.CM);

            LLResult result = limelight.getLatestResult();
            //Pose pose = follower.pose();
            //turret.getSelectedHive();

            tempo = System.currentTimeMillis();

            //follower.update();

            // ===== subsystems =====

            if (result != null && result.isValid() && gamepad2.square) {

                // coleta todas as tags válidas detectadas nesse frame
                List<LLResultTypes.FiducialResult> validTags = new ArrayList<>();
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    for (int id : TARGET_IDS) {
                        if (tag.getFiducialId() == id) {
                            validTags.add(tag);
                            break;
                        }
                    }
                }

                if (!validTags.isEmpty()) {

                    // ordena pela posição tx e escolhe a do meio
                    validTags.sort((a, b) -> Double.compare(a.getTargetXDegrees(), b.getTargetXDegrees()));
                    LLResultTypes.FiducialResult middleTag = validTags.get(validTags.size() / 2);

                    double tx = middleTag.getTargetXDegrees();
                    double ty = middleTag.getTargetYDegrees();
                    double ta = middleTag.getTargetArea();

                    telemetry.addData("tag escolhida (meio)", middleTag.getFiducialId());
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);
                    telemetry.addData("ta", ta);

                    if (tx > 2.5) {
                        turret.setPosition(pose - 0.01);
                    } else if (tx < 2.5) {
                        turret.setPosition(pose + 0.01);
                    }
                }
            }

                servo.update(gamepad1, gamepad2, shooter.tiroPolen(), shooter.tiroPolen());
                intake.update(gamepad1, gamepad2, shooter.isAtirando(), dI1, dI2, dI3, dI4);
                shooter.update(gamepad1, gamepad2, 10,
                        intake.isColetando(), intake.isCuspindo());
                //turret.update(gamepad1, gamepad2);
                drive.teleopUpdate(gamepad1, gamepad2);

                // ===== Telemetria =====
//            telemetry.addData("HIVE escolhida", turret.getSelectedHive());
//            telemetry.addData("Ângulo relativo", turret.getAngleDegrees());
//            telemetry.addData("Posição do servo", turret.getServoPosition());
//            telemetry.addData("Follower ocupado", follower.isBusy());
                telemetry.addData("DistIntake1", dI1);
                telemetry.addData("DistIntake2", dI2);
                telemetry.addData("DistIntake3", dI3);
                telemetry.addData("DistIntake4", dI4);
            telemetry.addData("X", pinpoint.getEncoderX());
            telemetry.addData("Y", pinpoint.getEncoderY());
            telemetry.addData("Ang", pinpoint.getHeading(AngleUnit.DEGREES));
                telemetry.addData("tempo: ", timer);
                telemetry.addData("rpm", shooter.getRPM());
                telemetry.update();
            telemetry.update();

        }
    }
}