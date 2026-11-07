package org.firstinspires.ftc.teamcode.pedro.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class auto8p2h extends OpMode {

    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final long FEED_TIME_MS = 2200;
    private static final double SHOOTER_SPINUP_DISTANCE = 12.0;

    private Follower follower;

    private DcMotorEx lshooter;
    private DcMotorEx rshooter;
    private DcMotor intake;
    private DcMotor indexer;
    private Servo servoR;
    private Servo servoAR;
    private Servo servoAL;
    private Servo turretAjustR;
    private Servo turretAjustL;
    private Servo turretR;
    private Servo turretL;
    private Servo servoIntk;
    private VoltageSensor battery;
    private DistanceSensor distIntake1;
    private DistanceSensor distIntake2;
    private DistanceSensor distIntake3;
    private DistanceSensor distIntake4;

    private int pathState;
    private double rpmAtual;
    private double ShooterRPM;
    private double rightShooterRPM;
    private double shooterPower;
    private boolean alimentando;
    private boolean coletando;
    private long inicioColeta;
    private long feedStartTime;
    private int activeShotState = Integer.MIN_VALUE;
    private int activeShotRpm;
    private long inicioAtraso;

    private final ShooterPid shooterPid = new ShooterPid();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(32, 130, 90);
    private final Pose tiro1 = poseFactory.of(32, 127, 90);
    private final Pose coleta = poseFactory.of(40, 115, 90);
    private final Pose coletaControl1 = poseFactory.of(32, 122, 0);
    private final Pose tiro2 = poseFactory.of(46.7, 130, 90);
    private final Pose tiro2Control1 = poseFactory.of(47.0274, 117.6367, 0);
    private final Pose park = poseFactory.of(14.2439, 120.5177, 90);
    private final Pose parkControl1 = poseFactory.of(47.1013, 118.9202, 0);


    public Path coleta() {
        return Paths.curve(startPose, coletaControl1, coleta).constant(coleta);
    }

    public Path tiro2() {
        return Paths.curve(coleta, tiro2Control1, tiro2).constant(tiro2);
    }

    public Path park() {
        return Paths.curve(tiro2, parkControl1, park).constant(park);
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        lshooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        rshooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        servoR = hardwareMap.get(Servo.class, "servoR");
        turretAjustR = hardwareMap.get(Servo.class, "turretAjustR");
        turretAjustL = hardwareMap.get(Servo.class, "turretAjustL");
        turretR = hardwareMap.get(Servo.class, "turretR");
        turretL = hardwareMap.get(Servo.class, "turretL");
        servoAR = hardwareMap.get(Servo.class, "servoAR");
        servoAL = hardwareMap.get(Servo.class, "servoAL");
        servoIntk = hardwareMap.get(Servo.class, "servoIntk");
        distIntake1 = hardwareMap.get(DistanceSensor.class, "distIntake1");
        distIntake2 = hardwareMap.get(DistanceSensor.class, "distIntake2");
        distIntake3 = hardwareMap.get(DistanceSensor.class, "distIntake3");
        distIntake4 = hardwareMap.get(DistanceSensor.class, "distIntake4");
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        lshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretR.setPosition(0.9);
        turretL.setPosition(0.9);
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        pathState = 0;
        inicioAtraso = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long tempoDecorrido = System.currentTimeMillis() - inicioAtraso;
        if (tempoDecorrido > 10000) {
            telemetry.addData("Iniciando em", (10000 - tempoDecorrido) / 1000.0);
            telemetry.update();
            follower.update();
            autonomousPathUpdate();
        } else {
            telemetry.addData("Iniciando em", (10000 - tempoDecorrido) / 1000.0);
            telemetry.update();
        }
//        follower.update();
//        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("RPM alvo", activeShotRpm);
        telemetry.addData("RPM shooter direito", rightShooterRPM);
        telemetry.addData("RPM medido", ShooterRPM);
        telemetry.addData("Potencia shooter", shooterPower);
        telemetry.addData("Alimentando", alimentando);
        telemetry.update();

        turretR.setPosition(0.9);
        turretL.setPosition(0.9);
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public boolean isCloseToTarget(Pose target, double toleranceInInches) {
        double distance = Math.sqrt(
                Math.pow(target.x() - follower.pose().x(), 2) +
                        Math.pow(target.y() - follower.pose().y(), 2)
        );
        return distance < toleranceInInches;
    }

    public void autonomousPathUpdate() {
        double tolerance = 1.0;
        double idleVel = (180 * SHOOTER_TICKS_PER_REV) / 60.0;
        servoAR.setPosition(0.5);
        servoAL.setPosition(0.6);
        turretAjustL.setPosition(0.08);
        turretAjustR.setPosition(0.08);

        switch (pathState) {
            case 0:
                rshooter.setVelocity(-idleVel);
                lshooter.setVelocity(-idleVel);
                setPathState(10);
                break;

            case 10:
                shooter(2, 1825, !follower.isBusy() || isCloseToTarget(tiro1, tolerance));
                break;

            case 2:
                intake.setPower(-1.0);
                indexer.setPower(-1.0);
                servoR.setPosition(0.3);
                servoIntk.setPosition(1.0);
                shooterIdle(idleVel);
                follower.follow(coleta());
                setPathState(20);
                break;

            case 20:
                indexer.setPower(0);
                follower.follow(tiro2());
                setPathState(3);
                break;


            case 3:
                if (!follower.isBusy() || isCloseToTarget(coleta, tolerance)) {
                    intake.setPower(-1.0);
                    indexer.setPower(-0.6);
                    servoAR.setPosition(0.58);
                    servoAL.setPosition(0.58);

                    if (!coletando) {
                        coletando = true;
                        inicioColeta = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - inicioColeta >= 2150) {
                        intake.setPower(-0.2);
                        indexer.setPower(0);
                        coletando = false;
                        setPathState(30);
                    }

                    servoAR.setPosition(0.56);
                    servoAL.setPosition(0.56);
                }
                break;

            case 30:
                boolean chegou = !follower.isBusy()
                        || isCloseToTarget(tiro2, tolerance);
                if (chegou
                        || isCloseToTarget(tiro2, SHOOTER_SPINUP_DISTANCE)) {
                    turretR.setPosition(0.92);
                    turretL.setPosition(0.92);
                    shooter(4, 1825, chegou);
                }
                break;

            case 4:
                intake.setPower(0);
                indexer.setPower(0);
                servoR.setPosition(0.3);
                shooterIdle(idleVel);
                follower.follow(park());
                setPathState(50);
                break;

            case 50:
                if (!follower.isBusy() || isCloseToTarget(park, tolerance)) {
                    rshooter.setPower(0);
                    lshooter.setPower(0);
                    setPathState(51);
                }
                break;

            case 51:
                intake.setPower(0);
                indexer.setPower(0);
                rshooter.setPower(0);
                lshooter.setPower(0);
                setPathState(-1);
                break;
        }

    }

    private void shooterIdle(double idleVel) {
        resetShooterControl();
        activeShotState = Integer.MIN_VALUE;
        lshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rshooter.setVelocity(-idleVel);
        lshooter.setVelocity(-idleVel);
    }

    public void shooter(int estado, int rpm, boolean podeAlimentar) {
        turretAjustL.setPosition(0.08);
        turretAjustR.setPosition(0.08);
        servoR.setPosition(0.7);

        if (activeShotState != estado || activeShotRpm != rpm) {
            resetShooterControl();
            activeShotState = estado;
            activeShotRpm = rpm;
            lshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        long now = System.nanoTime();
        rightShooterRPM = Math.abs(rshooter.getVelocity()) * 60.0 / SHOOTER_TICKS_PER_REV;
        rpmAtual = rightShooterRPM;
        ShooterRPM = rpmAtual;

        double voltage = battery == null ? 12.0 : battery.getVoltage();
        shooterPower = shooterPid.update(rpm, rightShooterRPM, voltage, now);
        lshooter.setPower(-shooterPower);
        rshooter.setPower(-shooterPower);

        long nowMs = System.currentTimeMillis();

        if (!alimentando) {
            if (!podeAlimentar || rightShooterRPM < rpm) {
                intake.setPower(0);
                indexer.setPower(0);
                return;
            }

            alimentando = true;
            feedStartTime = nowMs;
        }

        if (nowMs - feedStartTime < FEED_TIME_MS) {
            intake.setPower(-0.6);
            indexer.setPower(-0.6);
            return;
        }

        intake.setPower(0);
        indexer.setPower(0);
        setPathState(estado);
        resetShooterControl();
    }

    private void resetShooterControl() {
        shooterPid.reset();
        feedStartTime = 0;
        alimentando = false;
    }

    private static class ShooterPid {
        private static final double KP = 0.00035;
        private static final double KI = 0.00012;
        private static final double KD = 0.000008;
        private static final double KV = 0.00030;
        private static final double I_ZONE_RPM = 300.0;
        private static final double MAX_INTEGRAL_POWER = 0.12;
        private static final double DERIVATIVE_FILTER_SECONDS = 0.04;
        private double integralPower;
        private double previousRpm;
        private double filteredDerivative;
        private long previousTime;
        private boolean initialized;

        double update(double targetRpm, double measuredRpm, double voltage, long now) {
            double dt = initialized ? (now - previousTime) / 1e9 : 0;
            if (dt <= 0 || dt > 0.1) {
                integralPower = 0;
                filteredDerivative = 0;
                dt = 0;
            } else {
                double derivative = (measuredRpm - previousRpm) / dt;
                double alpha = dt / (DERIVATIVE_FILTER_SECONDS + dt);
                filteredDerivative += alpha * (derivative - filteredDerivative);
            }

            initialized = true;
            previousTime = now;
            previousRpm = measuredRpm;
            double error = targetRpm - measuredRpm;
            if (Math.abs(error) > I_ZONE_RPM) integralPower = 0;
            double candidateIntegral = clamp(
                    integralPower + KI * error * dt,
                    -MAX_INTEGRAL_POWER,
                    MAX_INTEGRAL_POWER);
            double validVoltage = Double.isFinite(voltage) && voltage >= 9 && voltage <= 15
                    ? voltage : 12.0;
            double voltageCompensation = 12.0 / validVoltage;
            double basePower = KV * targetRpm + KP * error - KD * filteredDerivative;
            double candidateOutput = (basePower + candidateIntegral) * voltageCompensation;

            if (Math.abs(error) <= I_ZONE_RPM
                    && ((candidateOutput >= 0 && candidateOutput <= 1)
                    || (candidateOutput > 1 && error < 0)
                    || (candidateOutput < 0 && error > 0))) {
                integralPower = candidateIntegral;
            }
            return clamp((basePower + integralPower) * voltageCompensation, 0, 1);
        }

        void reset() {
            integralPower = 0;
            previousRpm = 0;
            filteredDerivative = 0;
            previousTime = 0;
            initialized = false;
        }

        private static double clamp(double value, double minimum, double maximum) {
            return Math.max(minimum, Math.min(value, maximum));
        }
    }
}
