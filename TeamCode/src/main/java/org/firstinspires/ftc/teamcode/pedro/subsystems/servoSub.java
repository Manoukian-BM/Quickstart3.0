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
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, boolean polen, boolean nectar) {

//            if (gamepad.b && !previousB) {
//                nectar = !nectar;
//                double position = nectar ? 0.3 : 0.1;
//
//                turretAjustL.setPosition(position);
//                turretAjustR.setPosition(position + 0.03);
//            }

        if (gamepad2.b && !previousA) {
            flower = !flower;
            double position = flower ? 0.56 : 0.83; // cima: 0.83, abaixo: 0.57

            servoAL.setPosition(position + 0.1);
            servoAR.setPosition(position);
        }


        if (gamepad.dpad_up) {
            servoIntk.setPosition(1.0);
        } else if (gamepad.dpad_down) {
            servoIntk.setPosition(0.65);
        }

        previousA = gamepad2.b;
        previousB = gamepad.b;
    }
}