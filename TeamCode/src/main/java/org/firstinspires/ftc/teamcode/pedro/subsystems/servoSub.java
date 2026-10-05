package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoSub {
    private final Servo servoR;
    private final Servo servoL;

    public servoSub(HardwareMap hardwareMap) {

        servoL = hardwareMap.get(Servo.class, "servoIntakeL");
        servoR = hardwareMap.get(Servo.class, "servoIntakeR");
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.b || gamepad.y || gamepad.right_trigger > 0.2 || gamepad.a) {
            // abrido
            servoR.setPosition(0.5);
            servoL.setPosition(0.5);
        } else {
            // fechado
            servoR.setPosition(0.72);
            servoL.setPosition(0.72);
        }

    }
    public void Intake(Gamepad gamepad, Gamepad gamepad2, double distIntake1Val, double distIntake2Val, double distIntake3Val) {
        if (!(gamepad2.dpad_right || gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad.b || gamepad.y || gamepad.x || gamepad.a)) {
            if (distIntake2Val <= 2 && distIntake1Val <= 2 && distIntake3Val <= 2 &&
                    !gamepad.left_bumper) {
                servoR.setPosition(0.45);
                servoL.setPosition(0.5);  //abrido
            }
        }

    }
}
