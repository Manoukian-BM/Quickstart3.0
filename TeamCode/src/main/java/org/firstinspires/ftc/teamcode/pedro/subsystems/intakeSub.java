package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleOpBlue;
import org.firstinspires.ftc.teamcode.pedro.teleOpRed;

public class intakeSub {

    private DcMotor intake, indexer;
    RevBlinkinLedDriver led;
    teleOpRed redAlliance;
    teleOpBlue blueAlliance;

    public enum EstadoIntake {
        IDLE, COLETANDO, CARREGADO, EXPELIR
    }
    private EstadoIntake estadoAtual = EstadoIntake.IDLE;
    private boolean recemCarregado = false;
    private RevBlinkinLedDriver.BlinkinPattern padraoAtual = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    public intakeSub(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, boolean shooterAtivo /*double distIntake1Val, double distIntake2Val,
                       double distIntake3Val*/) {
        recemCarregado = false;

        if (gamepad2.left_bumper ) {
            estadoAtual = EstadoIntake.EXPELIR;
        } else if ((gamepad2.right_bumper)
                || (gamepad1.square))   {
            estadoAtual = EstadoIntake.COLETANDO;
        }  else {
            estadoAtual = EstadoIntake.IDLE;
        }

        /*if (distIntake1Val <= 7 && distIntake2Val <= 6 && distIntake3Val <= 3) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            ledVermelho1.off();
            ledVermelho2.off();
            ledVerde1.on();
            ledVerde2.on();
            gamepad2.rumble(200);

            if (shooterAtivo) {
                intake.setPower(-1.0);
            } else {
                shooterVerde.setPower(0);
                intake.setPower(0);
            }
        } */

        switch (estadoAtual) {
            case IDLE:
//                ledVerde1.off();
//                ledVerde2.off();
//                ledVermelho1.on();
//                ledVermelho2.on();
                intake.setPower(0);
                indexer.setPower(0);
                if (!shooterAtivo) {
                    indexer.setPower(0);
                }
                break;

            case COLETANDO:
//                ledVerde1.off();
//                ledVerde2.off();
//                ledVermelho1.on();
//                ledVermelho2.on();
                if (/*distIntake3Val <= 2*/ true) {
                    intake.setPower(-1.0);
                    indexer.setPower(-1.0);
                } else {
                    indexer.setPower(0);
                }
                break;

            case EXPELIR:
                intake.setPower(0.75);
                indexer.setPower(-1.0);
                break;

            case CARREGADO:
//                ledVermelho1.off();
//                ledVermelho2.off();
//                ledVerde1.on();
//                ledVerde2.on();
                gamepad2.rumble(200);

                if (shooterAtivo) {
                    intake.setPower(-1.0);
                } else {
                    indexer.setPower(0);
                    intake  .setPower(0);
                }
                break;
        }
    }

    public boolean isColetando() {
        return estadoAtual == EstadoIntake.COLETANDO;
    }

    public boolean isCuspindo() {
        return estadoAtual == EstadoIntake.EXPELIR;
    }

    public boolean isCarregado() {
        return estadoAtual == EstadoIntake.CARREGADO;
    }

    public boolean checkRumble() {
        return recemCarregado;
    }

    public String getEstado() {
        return estadoAtual.toString();
    }
}
