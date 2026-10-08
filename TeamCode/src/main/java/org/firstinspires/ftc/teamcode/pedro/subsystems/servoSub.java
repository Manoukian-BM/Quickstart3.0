package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class servoSub {
    private final Servo turretAjustR, turretAjustL;
    private final Servo servoAR;
    private final Servo servoAL;
    private final Servo servoTurretR, servoTurretL;

    private RevColorSensorV3 distIntake4; // I2C 03 control

    private boolean previousA = false;
    private boolean previousB = false;
    boolean flower = false;
    boolean nectar = false;


    public servoSub(HardwareMap hardwareMap) {

        turretAjustR = hardwareMap.get(Servo.class, "turretAjustR");
        turretAjustL = hardwareMap.get(Servo.class, "turretAjustL");
        servoAR = hardwareMap.get(Servo.class, "servoAR");
        servoAL = hardwareMap.get(Servo.class, "servoAL");
        servoTurretR = hardwareMap.get(Servo.class, "servoR");
        servoTurretL = hardwareMap.get(Servo.class, "servoL");

        distIntake4 = hardwareMap.get(RevColorSensorV3.class, "distIntake4");
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, boolean shooting) {
        double blue = distIntake4.blue();
        double red = distIntake4.red();
        double green = distIntake4.green();

//        if (shooting) {
//
//            if (blue > 200 || red > 200) {
//                turretAjustR.setPosition(0.25);
//                turretAjustL.setPosition(0.25);
//            }
//        } else {
            if (gamepad.b && !previousB) {
                nectar = !nectar;
                double position = nectar ? 0.3 : 0.1;

                turretAjustL.setPosition(position);
                turretAjustR.setPosition(position + 0.03);
//            } if (gamepad.b && !previousB) {
//                nectar = !nectar;
//                double position = nectar ? 0.3 : 0.1;
//
//                turretAjustL.setPosition(position);
//                turretAjustR.setPosition(position);
//            }
            }

        if (gamepad.a && !previousA) {
            flower = !flower;
            double position = flower ? 1.0 : 0.0;

            servoAR.setPosition(position);
            servoAL.setPosition(position);
        }

        if (gamepad.dpad_left) {
            servoTurretR.setPosition(0.5);
            servoTurretL.setPosition(0.5);
        } else if (gamepad.dpad_right){
            servoTurretR.setPosition(0.32);
            servoTurretL.setPosition(0.32);
        }

        previousA = gamepad.a;
        previousB = gamepad.b;
    }
}
