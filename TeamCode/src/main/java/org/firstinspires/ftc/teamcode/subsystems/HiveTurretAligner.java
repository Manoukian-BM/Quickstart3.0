package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Alinha uma turret acionada por um servo Taura BL35 com uma HIVE detectada
 * pela Limelight3A.
 *
 * <p>Escala angular do Taura:</p>
 * <ul>
 *     <li>0.0 = 90 graus horario</li>
 *     <li>0.5 = 0 graus, apontado para frente</li>
 *     <li>1.0 = 90 graus anti-horario</li>
 * </ul>
 *
 * <p>A Limelight deve estar configurada para detectar AprilTags. O valor de
 * {@code tx} e usado como erro angular horizontal da tag.</p>
 */
public final class HiveTurretAligner {
    public enum Status {
        NO_RESULT,
        NO_HIVE,
        SEARCHING,
        ALIGNED
    }

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_CENTER = 0.5;
    private static final double DEGREES_PER_SERVO_UNIT = 180.0;

    private final Limelight3A limelight;
    private final Servo turret;
    private final Set<Integer> hiveTagIds;
    private final double positionPerDegree;
    private final double correctionSign;
    private final double deadbandDegrees;

    private double targetPosition;
    private double lastTxDegrees;
    private int lastTagId = -1;
    private Status status = Status.NO_RESULT;

    /**
     * @param limelight Limelight3A configurada para detectar AprilTags
     * @param turret servo Taura BL35 em modo angular
     * @param centerPosition posicao do servo quando a camera aponta para frente;
     *                       normalmente 0.5
     * @param positionPerDegree ganho em posicao do servo por grau de erro;
     *                           o valor inicial sugerido e 0.0045
     * @param correctionSign use 1 ou -1 conforme o sentido de tx da Limelight
     * @param deadbandDegrees erro angular aceito como alinhado
     * @param hiveTagIds IDs das tags que representam as HIVEs
     */
    public HiveTurretAligner(
            Limelight3A limelight,
            Servo turret,
            double centerPosition,
            double positionPerDegree,
            double correctionSign,
            double deadbandDegrees,
            int... hiveTagIds) {
        if (limelight == null || turret == null) {
            throw new IllegalArgumentException("Limelight e servo sao obrigatorios.");
        }
        if (positionPerDegree <= 0 || deadbandDegrees < 0) {
            throw new IllegalArgumentException("Ganho e tolerancia devem ser validos.");
        }
        if (correctionSign != 1 && correctionSign != -1) {
            throw new IllegalArgumentException("correctionSign deve ser 1 ou -1.");
        }
        if (hiveTagIds == null || hiveTagIds.length == 0) {
            throw new IllegalArgumentException("Informe ao menos um ID de HIVE.");
        }

        this.limelight = limelight;
        this.turret = turret;
        this.positionPerDegree = positionPerDegree;
        this.correctionSign = correctionSign;
        this.deadbandDegrees = deadbandDegrees;
        this.targetPosition = clipServoPosition(centerPosition);

        this.hiveTagIds = new HashSet<>();
        for (int id : hiveTagIds) {
            this.hiveTagIds.add(id);
        }
        turret.setPosition(this.targetPosition);
    }

    public void start() {
        limelight.start();
    }

    /**
     * Processa uma leitura e atualiza o servo. Deve ser chamado uma vez por
     * ciclo do OpMode.
     *
     * <p>Como o BL35 tem 180 graus entre as posicoes 0.0 e 1.0,
     * 1 grau corresponde a aproximadamente 1/180 = 0.00556 de posicao.
     * O ganho fica configuravel para compensar montagem, folga e calibracao.</p>
     */
    public Status update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            status = Status.NO_RESULT;
            return status;
        }

        LLResultTypes.FiducialResult target = findLargestHive(result.getFiducialResults());
        if (target == null) {
            status = Status.NO_HIVE;
            return status;
        }

        lastTagId = target.getFiducialId();
        lastTxDegrees = target.getTargetXDegrees();

        if (Math.abs(lastTxDegrees) <= deadbandDegrees) {
            status = Status.ALIGNED;
            return status;
        }

        targetPosition = clipServoPosition(
                targetPosition + correctionSign * lastTxDegrees * positionPerDegree);
        turret.setPosition(targetPosition);
        status = Status.SEARCHING;
        return status;
    }

    private LLResultTypes.FiducialResult findLargestHive(
            List<LLResultTypes.FiducialResult> fiducials) {
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (!hiveTagIds.contains(fiducial.getFiducialId())) {
                continue;
            }
            if (best == null || fiducial.getTargetArea() > best.getTargetArea()) {
                best = fiducial;
            }
        }
        return best;
    }

    private double clipServoPosition(double position) {
        return Range.clip(position, SERVO_MIN, SERVO_MAX);
    }

    /**
     * Converte um angulo Taura (-90 a +90 graus) para posicao do servo.
     * Angulo positivo representa anti-horario.
     */
    public static double angleToServoPosition(double angleDegrees) {
        return Range.clip(
                SERVO_CENTER + angleDegrees / DEGREES_PER_SERVO_UNIT,
                SERVO_MIN,
                SERVO_MAX);
    }

    public boolean isAligned() {
        return status == Status.ALIGNED;
    }

    public Status getStatus() {
        return status;
    }

    public double getLastTxDegrees() {
        return lastTxDegrees;
    }

    public int getLastTagId() {
        return lastTagId;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void stop() {
        limelight.stop();
    }
}
