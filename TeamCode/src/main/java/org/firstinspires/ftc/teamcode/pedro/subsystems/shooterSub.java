package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class shooterSub {
    // Preserva a conversao existente; confirmar 28 ticks por volta do eixo do shooter.
    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE = 0.0; // Liberar somente ao atingir o RPM alvo.
    private static final long HOOD_SETTLE_NS = 300_000_000L;
    private static final long GATE_OPEN_NS = 150_000_000L;
    private static final double GATE_CLOSED = 0.27, GATE_OPEN = 0.7;
    private static final double MIN_DIST = 45, MAX_DIST = 55;
    private static final double NEAR_RPM_BONUS = 120.0;
    private static final double BONUS_RPM_PER_5_INCHES = 10.0;
    private static final double POLLEN_BASE_RPM = 1680.0;
    private static final double POLLEN_RPM_PER_INCH = 8.0;
    private static final double POLLEN_FAR_RPM_BONUS = 10.0;
    // Extensao inicial alem de 55 in: calibrar no robo antes de ampliar este teto.
    private static final double EXTRA_DISTANCE_RPM_LIMIT = 150.0;
    // Curva inicial do nectar, ainda precisa ser calibrada com tiros reais.
    private static final double NECTAR_MIN_RPM = 1785, NECTAR_MAX_RPM = 1865;
    private final DcMotorEx lshooter, rshooter, indexer;
    private final DcMotor intake;
    private final Servo servoR, turretAjustR, turretAjustL;
    private final VoltageSensor battery;
    // Encoder fisico shooterL (campo rshooter) comanda a potencia dos dois motores.
    private final ShooterRpmController sharedPid = new ShooterRpmController();
    public enum EstadoShooter { IDLE, POLEN, NECTAR, POLEN_L, NECTAR_L, ACCEL }
    private EstadoShooter estadoAtual = EstadoShooter.IDLE;
    private long shotStart, gateStart, feedStart;
    private boolean gateOpening, shooterReady, feeding;
    private double targetRPM, rpmAtual, leftPower, rightPower;
    private String feedStatus = "Repouso";

    public static double targetRpmForDistance(double distance, boolean nectar) {
        if (!Double.isFinite(distance) || distance <= 0) return Double.NaN;
        double d = Math.max(MIN_DIST, distance);
        double fraction = (d - MIN_DIST) / (MAX_DIST - MIN_DIST);
        double min = nectar ? NECTAR_MIN_RPM : POLLEN_BASE_RPM;
        double max = nectar ? NECTAR_MAX_RPM
                : POLLEN_BASE_RPM + POLLEN_RPM_PER_INCH * (MAX_DIST - MIN_DIST);
        double baseRPM = Math.min(min + fraction * (max - min), max + EXTRA_DISTANCE_RPM_LIMIT);
        // Acrescimo sobre a curva base: +120 ate 45 in; mais +10 a cada 5 in adicionais.
        // Variacao continua entre as distancias, sem somar novamente o ajuste anterior.
        double bonusRPM = NEAR_RPM_BONUS
                + ((d - MIN_DIST) / 5.0) * BONUS_RPM_PER_5_INCHES;
        if (!nectar) bonusRPM += POLLEN_FAR_RPM_BONUS * Math.min(fraction, 1.0);
        return baseRPM + bonusRPM;
    }

    public shooterSub(HardwareMap hardwareMap, DcMotorEx indexer) {
        turretAjustR = hardwareMap.get(Servo.class, "turretAjustR");
        turretAjustL = hardwareMap.get(Servo.class, "turretAjustL");
        lshooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        rshooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        this.indexer = indexer != null ? indexer : hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servoR = hardwareMap.get(Servo.class, "servoR");
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
        this.indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        for (DcMotorEx motor : new DcMotorEx[]{lshooter, rshooter}) {
            // O PID deste codigo comanda potencia; nao empilhar com PID interno do Hub.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.indexer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, double dist,
                       boolean intakeColetando, boolean intakeCuspindo, double distanciaHIVE) {
        updateAtTime(gamepad2, distanciaHIVE, System.nanoTime());
    }

    void updateAtTime(Gamepad gamepad2, double distanciaHIVE, long now) {
        EstadoShooter requested = gamepad2.right_trigger > 0.2 ? EstadoShooter.POLEN
                : gamepad2.left_trigger > 0.2 ? EstadoShooter.NECTAR : EstadoShooter.IDLE;
        if (requested != estadoAtual) {
            resetControl();
            shotStart = now;
            targetRPM = Double.NaN;
            // Limpar a alimentacao ao entrar/sair do tiro, sem sobrescrever coleta em repouso.
            indexer.setPower(0);
            intake.setPower(0);
        }
        estadoAtual = requested;
        rpmAtual = Math.abs(rshooter.getVelocity()) * 60 / TICKS_PER_REV;

        if (estadoAtual == EstadoShooter.IDLE) {
            feedStatus = "Repouso";
            targetRPM = 0;
            resetControl();
            setWheelPower(0.2, 0.2);
            servoR.setPosition(GATE_CLOSED);
            return;
        }
        boolean nectar = estadoAtual == EstadoShooter.NECTAR || estadoAtual == EstadoShooter.NECTAR_L;
        // Guarda o primeiro alvo valido ate soltar o gatilho ou trocar de tiro.
        if (!Double.isFinite(targetRPM)) targetRPM = targetRpmForDistance(distanciaHIVE, nectar);
        if (!Double.isFinite(targetRPM) || !Double.isFinite(rpmAtual)
                || !Double.isFinite(distanciaHIVE) || distanciaHIVE <= 0) {
            indexer.setPower(0);
            intake.setPower(0);
            feedStatus = "Distancia ou encoder invalido";
            resetControl();
            setWheelPower(0, 0);
            servoR.setPosition(GATE_CLOSED);
            return;
        }
        if (nectar) {
            turretAjustL.setPosition(0.866);
            turretAjustR.setPosition(0.32);
        } else {
            turretAjustL.setPosition(0.655);
            turretAjustR.setPosition(0.095);
        }
        double voltage = battery == null ? 12 : battery.getVoltage();
        double sharedPower = sharedPid.update(targetRPM, rpmAtual, voltage, now);
        setWheelPower(sharedPower, sharedPower);
        // RPM so autoriza o inicio. A queda entre bolas nao interrompe a rajada.
        if (!shooterReady && rpmAtual >= targetRPM - RPM_TOLERANCE)
            shooterReady = true;
        if (!shooterReady || now - shotStart < HOOD_SETTLE_NS) {
            indexer.setPower(0);
            intake.setPower(0);
            feedStatus = !shooterReady ? "Aguardando primeiro RPM: erro " + Math.round(targetRPM - rpmAtual)
                    : "Aguardando ajustes";
            servoR.setPosition(GATE_CLOSED);
            return;
        }
        feedStatus = "Abrindo servo R";
        servoR.setPosition(GATE_OPEN);
        if (!gateOpening) {
            gateStart = now;
            gateOpening = true;
            indexer.setPower(0);
            intake.setPower(0);
        }
        if (now - gateStart >= GATE_OPEN_NS) {
            if (!feeding) { feedStart = now; feeding = true; }
            feedStatus = "Alimentacao continua: " + Math.round((now - feedStart) / 1e8) / 10.0 + " s";
            indexer.setPower(nectar ? -0.8 : -0.50);
            intake.setPower(nectar ? -0.8 : -0.50);
        }
    }

    private void setWheelPower(double left, double right) {
        leftPower = left;
        rightPower = right;
        lshooter.setPower(-left);
        rshooter.setPower(-right);
    }
    private void resetControl() {
        sharedPid.reset();
        feeding = false;
        gateOpening = false;
        shooterReady = false;
    }
    public void stop() {
        resetControl();
        estadoAtual = EstadoShooter.IDLE;
        targetRPM = 0;
        setWheelPower(0, 0);
        indexer.setPower(0);
        intake.setPower(0);
        servoR.setPosition(GATE_CLOSED);
    }
    public boolean ownsFeedCycle() { return gateOpening; }
    public boolean isAtirando() { return estadoAtual != EstadoShooter.IDLE; }
    public boolean tiroPolen() { return estadoAtual == EstadoShooter.POLEN || estadoAtual == EstadoShooter.POLEN_L; }
    public boolean tiroNectar() { return estadoAtual == EstadoShooter.NECTAR || estadoAtual == EstadoShooter.NECTAR_L; }
    public String getFeedStatus() { return feedStatus; }
    public double getTargetRPM() { return targetRPM; }
    public double getRPM() { return rpmAtual; }
    public boolean isReady() { return shooterReady; }
    public double getLeftPower() { return leftPower; }
    public double getRightPower() { return rightPower; }
    public String getEstado() { return estadoAtual.toString(); }
}