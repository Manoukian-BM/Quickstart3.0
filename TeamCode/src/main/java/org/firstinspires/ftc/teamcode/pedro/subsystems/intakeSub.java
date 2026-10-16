package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.teleOpBlue;
import org.firstinspires.ftc.teamcode.pedro.teleOpRed;

public class intakeSub {

    private DcMotor intake, indexer;
    private Servo servoIntk;
    RevBlinkinLedDriver led;
    teleOpRed redAlliance;
    teleOpBlue blueAlliance;

    double i = 0;
    public enum EstadoIntake {
        IDLE, COLETANDO, CARREGADO, EXPELIR
    }
    private EstadoIntake estadoAtual = EstadoIntake.IDLE;
    private boolean recemCarregado = false;
    private RevBlinkinLedDriver.BlinkinPattern padraoAtual = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    public intakeSub(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");

        servoIntk = hardwareMap.get(Servo.class, "servoIntk");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, boolean shooterAtivo,
                       double distIntake1Val, double distIntake2Val,
                       double distIntake3Val, double distIntake4Val) {

        recemCarregado = false;

        if (gamepad2.left_bumper || gamepad1.left_trigger > 0.2) {
            estadoAtual = EstadoIntake.EXPELIR;
        } else if ((gamepad1.right_trigger > 0.2)) {
            estadoAtual = EstadoIntake.COLETANDO;
        } else if (distIntake1Val < 4.2 && distIntake2Val < 2
                    && distIntake3Val < 3.5 && distIntake4Val < 3.5) {
            estadoAtual = EstadoIntake.CARREGADO;
        } else {
            estadoAtual = EstadoIntake.IDLE;
        }

        switch (estadoAtual) {
            case IDLE:
                servoIntk.setPosition(1.0);
                intake.setPower(0);
                //indexer.setPower(0);
                if (!shooterAtivo) {
                    indexer.setPower(0);
                }
                break;

            case COLETANDO:
                    intake.setPower(-1.0);
                    indexer.setPower(-0.2);
                break;

            case EXPELIR:
                intake.setPower(1.0);
                indexer.setPower(1.0);
                break;

            case CARREGADO:
                gamepad2.rumble(200);
                gamepad1.rumble(200);

                servoIntk.setPosition(0.65);

                if (shooterAtivo) {
                    intake.setPower(-1.0);
                } else {
                    indexer.setPower(0);
                    intake.setPower(0);
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
