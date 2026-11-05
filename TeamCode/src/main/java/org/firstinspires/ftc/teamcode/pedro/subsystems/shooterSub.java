package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooterSub {
    private static final double TICKS_PER_REV = 28.0;
    private static final double MIN_DIST = 45.0;
    private static final double MAX_DIST = 55.0;
    private static final double MIN_RPM = 1775.0;
    private static final double MAX_RPM = 1805.0;
    private static final double NECTAR_MIN_RPM = 1890.0;
    private static final double NECTAR_MAX_RPM = 1940.0;
    private static final double EXTRA_DISTANCE_RPM_LIMIT = 150.0;
    private static final double NEAR_RPM_BONUS = 120.0;
    private static final double BONUS_RPM_PER_5_INCHES = 10.0;
    private static final double RPM_KP = 0.00035;
    private static final double RPM_KI = 0.00012;
    private static final double RPM_KD = 0.000008;
    private static final double RPM_KV = 0.00030;
    private static final double RPM_I_ZONE = 300.0;
    private static final double MAX_INTEGRAL_POWER = 0.12;
    private static final double DERIVATIVE_FILTER_SECONDS = 0.04;
    private final DcMotorEx lshooter;
    private final DcMotorEx rshooter;
    private final DcMotorEx indexer;
    private final DcMotor intake;
    private final Servo servoR;
    private final Servo turretAjustR, turretAjustL;
    private final ColorRangeSensor distIntake4;
    private final VoltageSensor battery;
    private double integralPower;
    private double previousRpm;
    private double filteredDerivative;
    private long previousRpmTime;
    private boolean rpmControllerInitialized;
    double VelRPM = 0;
    double rpmR = 0;
    double correction, error, power;
    boolean shooterAtirando;
    boolean polen;
    boolean nectar;
    boolean shooterReady;
    ElapsedTime timer = new ElapsedTime();
    double shootTime = 0;
    double targetRPM;

    public enum EstadoShooter {
        IDLE, POLEN, NECTAR, POLEN_L, NECTAR_L, ACCEL
    }

    private EstadoShooter estadoAtual = EstadoShooter.IDLE;
    private EstadoShooter estadoAnterior = EstadoShooter.IDLE;
    private double rpmAtual = 0;

    private void resetRpmController() {
        integralPower = 0;
        filteredDerivative = 0;
        rpmControllerInitialized = false;
    }

    private double calculateShooterPower(double target, double measuredRpm, double voltage, long now) {
        if (!Double.isFinite(target) || target <= 0 || !Double.isFinite(measuredRpm)) {
            resetRpmController();
            return 0;
        }

        double dt = rpmControllerInitialized ? (now - previousRpmTime) / 1e9 : 0;
        if (dt <= 0 || dt > 0.1) {
            integralPower = 0;
            filteredDerivative = 0;
            dt = 0;
        } else {
            double derivative = (measuredRpm - previousRpm) / dt;
            double alpha = dt / (DERIVATIVE_FILTER_SECONDS + dt);
            filteredDerivative += alpha * (derivative - filteredDerivative);
        }

        rpmControllerInitialized = true;
        previousRpmTime = now;
        previousRpm = measuredRpm;
        double rpmError = target - measuredRpm;
        if (rpmError > 500.0) {
            integralPower = 0;
            return 1.0;
        }

        if (Math.abs(rpmError) > RPM_I_ZONE) integralPower = 0;
        double candidateIntegral = clamp(
                integralPower + RPM_KI * rpmError * dt,
                -MAX_INTEGRAL_POWER,
                MAX_INTEGRAL_POWER);
        double validVoltage = Double.isFinite(voltage) && voltage >= 9 && voltage <= 15
                ? voltage : 12.0;
        double voltageCompensation = 12.0 / validVoltage;
        double basePower = RPM_KV * target + RPM_KP * rpmError - RPM_KD * filteredDerivative;
        double candidateOutput = (basePower + candidateIntegral) * voltageCompensation;

        if (Math.abs(rpmError) <= RPM_I_ZONE
                && ((candidateOutput >= 0 && candidateOutput <= 1)
                || (candidateOutput > 1 && rpmError < 0)
                || (candidateOutput < 0 && rpmError > 0))) {
            integralPower = candidateIntegral;
        }
        return clamp((basePower + integralPower) * voltageCompensation, 0, 1);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double interpolateRpm(double distance, boolean nectar) {
        return targetRpmForDistance(distance, nectar) - 10.0;
    }

    static double targetRpmForDistance(double distance, boolean nectar) {
        if (!Double.isFinite(distance) || distance <= 0) return Double.NaN;
        double d = Math.max(MIN_DIST, distance);
        double fraction = (d - MIN_DIST) / (MAX_DIST - MIN_DIST);
        double min = nectar ? NECTAR_MIN_RPM : MIN_RPM;
        double max = nectar ? NECTAR_MAX_RPM : MAX_RPM;
        double baseRpm = Math.min(min + fraction * (max - min), max + EXTRA_DISTANCE_RPM_LIMIT);
        double bonusRpm = NEAR_RPM_BONUS
                + ((d - MIN_DIST) / 5.0) * BONUS_RPM_PER_5_INCHES;
        return baseRpm + bonusRpm;
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
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        this.indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.indexer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, double dist,
                       boolean intakeColetando, boolean intakeCuspindo,
                       double distanciaHIVE) {
        if (gamepad2.right_trigger > 0.2 || gamepad.cross) {
            estadoAtual = EstadoShooter.POLEN;
        } else if (gamepad2.left_trigger > 0.2) {
            estadoAtual = EstadoShooter.NECTAR;
        } else if (gamepad2.dpad_right) {
            estadoAtual = EstadoShooter.POLEN_L;
        } else {
            estadoAtual = EstadoShooter.IDLE;
        }

        rpmAtual = Math.abs(rshooter.getVelocity()) * 60.0 / TICKS_PER_REV;
        if (estadoAtual != estadoAnterior) {
            resetRpmController();
            estadoAnterior = estadoAtual;
        }

        boolean nectar = estadoAtual == EstadoShooter.NECTAR || estadoAtual == EstadoShooter.NECTAR_L;
        boolean shooting = estadoAtual == EstadoShooter.POLEN || estadoAtual == EstadoShooter.NECTAR
                || estadoAtual == EstadoShooter.POLEN_L || estadoAtual == EstadoShooter.NECTAR_L;
        if (shooting) {
            targetRPM = interpolateRpm(distanciaHIVE, nectar);
            if (!Double.isFinite(targetRPM) || !Double.isFinite(rpmAtual)) {
                power = 0;
                rshooter.setPower(0);
                lshooter.setPower(0);
                indexer.setPower(0);
                intake.setPower(0);
                servoR.setPosition(0.27);
                return;
            }
            double voltage = battery == null ? 12.0 : battery.getVoltage();
            power = calculateShooterPower(targetRPM, rpmAtual, voltage, System.nanoTime());
            rshooter.setPower(-power);
            lshooter.setPower(-power);
        } else {
            targetRPM = 0;
            resetRpmController();
        }

        switch (estadoAtual) {
            case IDLE:
                shooterAtirando = false;
                power = 0.2;
                lshooter.setPower(-0.2);
                rshooter.setPower(-0.2);
                servoR.setPosition(0.27);
                shooterReady = false;
                break;

            case POLEN:
                shooterAtirando = true;
                shootTime = System.currentTimeMillis();
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
                turretAjustR.setPosition(0.5);
                turretAjustL.setPosition(0.29);

                servoR.setPosition(0.15);
                if (rpmAtual >= targetRPM) {
                    if ((System.currentTimeMillis() + shootTime) > System.currentTimeMillis() + 100) {
                        indexer.setPower(-0.95);
                        intake.setPower(-0.8);
                    }
                }
                break;

            case ACCEL:
                power = 0.35;
                rshooter.setPower(-0.35);
                lshooter.setPower(-0.35);
                targetRPM = 0;
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

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getPower() {
        return power;
    }
}
