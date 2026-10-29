package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooterSub {
    private final DcMotorEx lshooter;
    private final DcMotorEx rshooter;
    private final DcMotorEx indexer;
    private final DcMotor intake;
    private final Servo servoR;
    private final Servo turretAjustR, turretAjustL;
    private final ColorRangeSensor distIntake4;
    double VelRPM = 0;
    double rpmR = 0;
    double correction, error, power;
    boolean shooterAtirando;
    boolean polen;
    boolean nectar;
    boolean shooterReady;
    ElapsedTime timer = new ElapsedTime();
    double shootTime = 0;

    private static final double MIN_DIST = 45.0;
    private static final double MAX_DIST = 55.0;
    private static final double MIN_RPM  = 1800.0;
    private static final double MAX_RPM  = 1850.0;

    double targetRPM;

    public enum EstadoShooter {
        IDLE, POLEN, NECTAR, POLEN_L, NECTAR_L, ACCEL
    }

    private EstadoShooter estadoAtual = EstadoShooter.IDLE;
    private double rpmAtual = 0;

    private double interpolateRpm(double distance) {
        // Limita a distância ao intervalo calibrado
        double d = Math.max(MIN_DIST, Math.min(distance, MAX_DIST));
        double fraction = (d - MIN_DIST) / (MAX_DIST - MIN_DIST);
        return MIN_RPM + fraction * (MAX_RPM - MIN_RPM);
    }

    public shooterSub(HardwareMap hardwareMap, DcMotorEx indexer) {
        turretAjustR = hardwareMap.get(Servo.class, "turretAjustR");
        turretAjustL = hardwareMap.get(Servo.class, "turretAjustL");

        distIntake4 = hardwareMap.get(ColorRangeSensor.class, "distIntake4");
        lshooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        rshooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        this.indexer = indexer != null
                ? indexer
                : hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servoR = hardwareMap.get(Servo.class, "servoR");

        this.indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.indexer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private double interpolateHiveRpm(double distance) {
        final double minDistance = 41.0;
        final double minRpm = 1820.0;
        final double maxDistance = 52.0;
        final double maxRpm = 1840.0;

        if (distance <= minDistance) {
            return minRpm;
        }
        if (distance >= maxDistance) {
            return maxRpm;
        }

        double fraction = (distance - minDistance) / (maxDistance - minDistance);
        return minRpm + fraction * (maxRpm - minRpm);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, double dist,
                       boolean intakeColetando, boolean intakeCuspindo,
                       double distanciaHIVE) {
        if (gamepad2.right_trigger > 0.2) {
            estadoAtual = EstadoShooter.POLEN;
        } else if (gamepad2.left_trigger > 0.2) {
            estadoAtual = EstadoShooter.NECTAR;
        } else if (gamepad2.dpad_right) {
            estadoAtual = EstadoShooter.POLEN_L;
//        } else if (gamepad2.dpad_left) {
//            estadoAtual = EstadoShooter.NECTAR_L;
//        } else if (gamepad2.right_bumper) {
//            estadoAtual = EstadoShooter.ACCEL;
        } else {
            estadoAtual = EstadoShooter.IDLE;
        }


        rpmAtual = Math.abs((rshooter.getVelocity() / 28) * 60);
        double targetRPM;

        switch (estadoAtual) {
            case IDLE:
                shooterAtirando = false;
                lshooter.setPower(-0.2);
                rshooter.setPower(-0.2);
                servoR.setPosition(0.27);
                shooterReady = false;
                break;

            case POLEN:
                shooterAtirando = true;
                shootTime = System.currentTimeMillis();
                targetRPM = interpolateRpm(distanciaHIVE);

                rpmR = Math.abs(rshooter.getVelocity() / 28) * 60.0;
                error = targetRPM - rpmR;
                correction = error * 0.0085;
                power = Math.max(0, Math.min(1 + correction, 1));
                rshooter.setPower(-power);
                lshooter.setPower(-power);
                turretAjustL.setPosition(0.655);
                turretAjustR.setPosition(0.095);
                servoR.setPosition(0.7);
                if (rpmAtual >= targetRPM) {
                    if ((System.currentTimeMillis() + shootTime) > System.currentTimeMillis() + 100) {
                        indexer.setPower(-0.6);
                        intake.setPower(-0.6);
                    }
                }
                break;

            case NECTAR:
                shooterAtirando = true;
                shootTime = System.currentTimeMillis();
                targetRPM = 1960;

                rpmR = Math.abs(rshooter.getVelocity() / 28) * 60.0;
                error = targetRPM - rpmR;
                correction = error * 0.0085;
                power = Math.max(0.05, Math.min(1 + correction, 1));
                rshooter.setPower(-power);
                lshooter.setPower(-power);
                turretAjustL.setPosition(0.866);
                turretAjustR.setPosition(0.32);
                servoR.setPosition(0.7);
                if (rpmAtual >= targetRPM) {
                    if ((System.currentTimeMillis() + shootTime) > System.currentTimeMillis() + 100) {
                        indexer.setPower(-1.0);
                        intake.setPower(-1.0);
                    }
                }
                break;

            case POLEN_L:
                shooterAtirando = true;
                shootTime = System.currentTimeMillis();
                targetRPM = 1850;

                rpmR = Math.abs(rshooter.getVelocity() / 28) * 60.0;
                error = targetRPM - rpmR;
                correction = error * 0.0085;
                power = Math.max(0, Math.min(1 + correction, 1));
                rshooter.setPower(-power);
                lshooter.setPower(-power);
                turretAjustL.setPosition(0.655);
                turretAjustR.setPosition(0.095);
                servoR.setPosition(0.7);
                if (rpmAtual >= targetRPM) {
                    if ((System.currentTimeMillis() + shootTime) > System.currentTimeMillis() + 100) {
                        indexer.setPower(-0.6);
                        intake.setPower(-0.6);
                    }
                }
                break;

            case NECTAR_L:
                shooterAtirando = true;
                shootTime = System.currentTimeMillis();
                targetRPM = 2050;
                rpmR = Math.abs(rshooter.getVelocity() / 28) * 60.0;
                error = targetRPM - rpmR;
                correction = error * 0.009;
                turretAjustR.setPosition(0.5);
                turretAjustL.setPosition(0.29);
                power = Math.max(0, Math.min(0.5 + correction, 1));
                rshooter.setPower(-power);
                lshooter.setPower(-power);

                servoR.setPosition(0.15);
                if (rpmAtual >= targetRPM) {
                    if ((System.currentTimeMillis() + shootTime) > System.currentTimeMillis() + 100) {
                        indexer.setPower(-0.95);
                        intake.setPower(-0.8);
                    }
                }
                break;

            case ACCEL:
                rshooter.setPower(-0.35);
                lshooter.setPower(-0.35);
        }
    }

    public boolean isAtirando() {
        shooterAtirando = estadoAtual != EstadoShooter.IDLE;
        return shooterAtirando;
    }

    public boolean tiroPolen() {
        polen = estadoAtual == EstadoShooter.POLEN;
        return polen;
    }

    public boolean tiroNectar() {
        nectar = estadoAtual == EstadoShooter.NECTAR;
        return nectar;
    }

    public double getRPM() {
        return rpmAtual;
    }

    public String getEstado() {
        return estadoAtual.toString();
    }
}
