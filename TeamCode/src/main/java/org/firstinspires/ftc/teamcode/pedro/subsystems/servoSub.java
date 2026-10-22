package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class servoSub {
    private final Servo turretAjustR, turretAjustL;
    private final Servo servoAR;
    private final Servo servoAL;
    private final Servo servoR;
    private final Servo servoIntk;

    private RevColorSensorV3 distIntake4; // I2C 03 control

    private boolean previousA = false;
    private boolean previousB = false;
    boolean flower = false;
    boolean nectar = false;


    public servoSub(HardwareMap hardwareMap) {

        turretAjustR = hardwareMap.get(Servo.class, "turretAjustR");
        turretAjustL = hardwareMap.get(Servo.class, "turretAjustL");
        servoIntk = hardwareMap.get(Servo.class, "servoIntk");
        servoAR = hardwareMap.get(Servo.class, "servoAR");
        servoAL = hardwareMap.get(Servo.class, "servoAL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        distIntake4 = hardwareMap.get(RevColorSensorV3.class, "distIntake4");
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, boolean polen, boolean nectar) {

//            if (gamepad.b && !previousB) {
//                nectar = !nectar;
//                double position = nectar ? 0.3 : 0.1;
//
//                turretAjustL.setPosition(position);
//                turretAjustR.setPosition(position + 0.03);
//            }

        if (gamepad2.a && !previousA) {
            flower = !flower;
            double position = flower ? 0.9 : 0.57; // 1 para trás, 0.5 para frente

            servoAR.setPosition(position);
            servoAL.setPosition(position);
        }


        if (gamepad.dpad_up) {
            servoIntk.setPosition(1.0);
        } else if (gamepad.dpad_down) {
            servoIntk.setPosition(0.65);
        }

        previousA = gamepad2.a;
        previousB = gamepad.b;
    }
}