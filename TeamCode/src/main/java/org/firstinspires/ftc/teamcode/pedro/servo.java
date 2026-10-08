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

                nectar = !nectar; //porta 0 e 1 exp
                double position = nectar ? 0.3 : 0.1;
                telemetry.addData("nectar", nectar);

                turretAjustL.setPosition(position);
                turretAjustR.setPosition(position);
            }

            if (gamepad1.a) {
                turret.setPosition(0); //porta 5 exp
                telemetry.addData("turret", "posicao 0");
            } else {
                turret.setPosition(0.2);
                telemetry.addData("turret", "posicao 0.2");
            }

            if (gamepad1.y) {
                servoAL.setPosition(0); //porta 5 exp
                telemetry.addData("servoAL", "posicao 0");
            } else {
                servoAL.setPosition(0.2);
                telemetry.addData("servoAL", "posicao 0.2");
            }

            if (gamepad1.x) {
                servoAR.setPosition(0); // porta 3 exp
                telemetry.addData("servoAR", "posicao 0");
            } else {
                servoAR.setPosition(0.2);
                telemetry.addData("servoAR", "posicao 0.2");
            }

            if (gamepad2.a) {
                servoR.setPosition(0); // porta 3 exp
                telemetry.addData("servoAR", "posicao 0");
            } else {
                servoR.setPosition(0.2);
                telemetry.addData("servoAR", "posicao 0.2");
            }

            if (gamepad2.y) {
                servoL.setPosition(0); // porta 3 exp
                telemetry.addData("servoAR", "posicao 0");
            } else {
                servoL.setPosition(0.2);
                telemetry.addData("servoAR", "posicao 0.2");
            }

            if (gamepad2.x) {
                servoIntk.setPosition(0); // porta 3 exp
                telemetry.addData("servoAR", "posicao 0");
            } else {
                servoIntk.setPosition(0.2);
                telemetry.addData("servoAR", "posicao 0.2");
            }

            previousB = gamepad1.b;
            telemetry.update();
        }
    }
}