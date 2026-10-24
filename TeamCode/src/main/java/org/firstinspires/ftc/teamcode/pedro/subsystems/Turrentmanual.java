package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.teleOpBlue;
import org.firstinspires.ftc.teamcode.pedro.teleOpRed;

    public class  Turrentmanual {

    private final Servo turret, turret2;

    public enum EstadoIntake {
        IDLE, COLETANDO, CARREGADO, EXPELIR,
    }
    private EstadoIntake estadoAtual = EstadoIntake.IDLE;
    private boolean recemCarregado = false;
    double pos = 0.5;
    private RevBlinkinLedDriver.BlinkinPattern padraoAtual = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    public Turrentmanual(HardwareMap hardwareMap) {
        turret = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
    }

    public void update(Gamepad gamepad2) {

        recemCarregado = false;

        if (gamepad2.right_stick_x > 0.2 || gamepad2.right_stick_x < -0.2) {
            estadoAtual = EstadoIntake.COLETANDO;
        }


        switch (estadoAtual) {

            case COLETANDO:
            if (Math.abs(gamepad2.right_stick_x) > 0.2) {
                if (gamepad2.square) {
                pos = 0.5 + gamepad2.right_stick_x * 0.5;
                }
                turret.setPosition(pos);
                turret2.setPosition(pos);
            } else {
                pos = 0.5;
                turret.setPosition(pos);
                turret2.setPosition(pos);
            }

//                if (gamepad2.right_stick_x > 0.2) {
//                    turret.setPosition((gamepad2.right_stick_x * 0.9));
//                } else if (gamepad2.right_stick_x < -0.2) {
//                    turret.setPosition((gamepad2.right_stick_x * 0.9));
//                } else {
//                    turret.setPosition(0.5);
//                }
            break;
        }
    }

    public double posicao() {
        return pos;
    }
}
