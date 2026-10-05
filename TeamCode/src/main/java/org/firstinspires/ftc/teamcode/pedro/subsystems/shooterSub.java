package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooterSub {

    private final DcMotorEx lshooter;
    private final DcMotorEx rshooter;
    private final DcMotorEx indexer;
    private final DcMotor intake;
    double VelRPM = 0;
    double rpmR, correction, error, power;
    boolean shooterAtirando;
    boolean shooterReady;
    ElapsedTime timer = new ElapsedTime();

    public enum EstadoShooter {
        IDLE, ATIRANDO
    }

    private EstadoShooter estadoAtual = EstadoShooter.IDLE;
    private double rpmAtual = 0;

    public shooterSub(HardwareMap hardwareMap, DcMotorEx indexer) {
        lshooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        rshooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        this.indexer = indexer;
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, boolean intakeColetando, boolean intakeCuspindo) {

        if (gamepad.right_trigger > 0.2) {
            estadoAtual = EstadoShooter.ATIRANDO;
        } else {
            estadoAtual = EstadoShooter.IDLE;
        }

        rpmAtual = Math.abs((rshooter.getVelocity() / 28) * 60);
        double targetRPM;

        switch (estadoAtual) {
            case IDLE:
                if (!gamepad2.right_bumper) {
                    lshooter.setPower(0);
                    rshooter.setPower(0);
                }
                shooterReady = false;
                break;

            case ATIRANDO:
                targetRPM = 1900;
                rpmR = (Math.abs(rshooter.getVelocity() / 28) * 60.0);
                error = targetRPM - rpmR;
                correction = error * 0.0055;
                power = 0.6 + correction;
                power = Math.max(0, Math.min(power,1));
                rshooter.setPower(-power);
                lshooter.setPower(-power);

                if (rpmAtual >= 1900) {
                    indexer.setPower(1.0);;
                    intake.setPower(-1);
                } else {
                    indexer.setPower(0);
                    intake.setPower(0);
                }
                break;
        }
    }

    public boolean isAtirando() {
        shooterAtirando = estadoAtual != EstadoShooter.IDLE;
        return shooterAtirando;
    }

    public double getRPM() {
        return rpmAtual;
    }

    public String getEstado() {
        return estadoAtual.toString();
    }
}
