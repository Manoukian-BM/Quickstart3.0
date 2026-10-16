package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "ervo", group = "aaa teleOp oficial")
public class servo extends LinearOpMode {

    private Servo turretAjustR, turretAjustL;
    private Servo turret;
    private Servo servoAR, servoAL;
    private Servo servoR, servoL;
    private Servo servoIntk;


    double pose = 0.5;
    boolean nectar = false;
    boolean flower = false;
    boolean previousA = false;
    boolean previousB = false;

    @Override
    public void runOpMode() {


        turretAjustL = hardwareMap.get(Servo.class, "turretAjustL");
        turretAjustR = hardwareMap.get(Servo.class, "turretAjustR");

        turret = hardwareMap.get(Servo.class, "turret");

        servoAR = hardwareMap.get(Servo.class, "servoAR");
        servoAL = hardwareMap.get(Servo.class, "servoAL");

        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");

        servoIntk = hardwareMap.get(Servo.class, "servoIntk");

        telemetry.addLine("TeleOp pronto.");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.b && !previousB) {
                nectar = !nectar;
                double position = nectar ? 0.3 : 0.1;

                turretAjustL.setPosition(position);
                turretAjustR.setPosition(position + 0.03);
            }

            if (gamepad1.right_bumper) {
                turret.setPosition(1); //porta 2 exp
                telemetry.addData("turret", "posicao 1");
            } else if (gamepad1.left_bumper) {
                turret.setPosition(0);
                telemetry.addData("turret", "posicao 0");
            } else if (gamepad1.dpad_up) {
                turret.setPosition(0.5);
                telemetry.addData("turret", "0.5");
            }

            if (gamepad1.a && !previousA) {
                flower = !flower;
                double position = flower ? 1.0 : 0.0;

                servoAR.setPosition(position);
                servoAL.setPosition(position);
            }

            if (gamepad1.dpad_left) {
                servoR.setPosition(0.35);
                servoL.setPosition(0.35);
            } else if (gamepad1.dpad_right){
                servoR.setPosition(0.47);
                servoL.setPosition(0.47);
            }

            if (gamepad2.x) {
                servoIntk.setPosition(0.65); // porta 3 exp
                telemetry.addData("servoIntk", "posicao 0.65");
            } else {
                servoIntk.setPosition(1.0);
                telemetry.addData("servoIntk", "posicao 01.0");
            }

            previousA = gamepad1.a;
            previousB = gamepad1.b;
            telemetry.update();
        }
    }
}