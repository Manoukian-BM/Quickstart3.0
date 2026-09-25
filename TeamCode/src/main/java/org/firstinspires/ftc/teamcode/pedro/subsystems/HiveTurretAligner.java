package org.firstinspires.ftc.teamcode.pedro.subsystems;

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
 * Escala angular do Taura BL35:
 * 0.0 = 90 graus horario
 * 0.5 = 0 graus, apontado para frente
 * 1.0 = 90 graus anti-horario
 *
 * Somente as AprilTags 39, 40, 43 e 44 sao aceitas.
 */
public final class HiveTurretAligner {

    public enum Status {
        NO_RESULT,
        NO_HIVE,
        SEARCHING,
        ALIGNED
    }

    private static final double SERVO_ESQ = 0.0;
    private static final double SERVO_DIR = 1.0;
    private static final double SERVO_CENTRO = 0.5;
    private static final double DEGREES_PER_SERVO_UNIT = 180.0;

    // Somente estes ‘IDs’ serao considerados HIVEs.
    private static final int HIVE_TAG_39 = 39;
    private static final int HIVE_TAG_40 = 40;
    private static final int HIVE_TAG_43 = 43;
    private static final int HIVE_TAG_44 = 44;

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
     * @param turret Servo Taura BL35 configurado em modo angular
     * @param centerPosition Posicao central do servo.
     *                       Para o Taura BL35, normalmente 0.5
     * @param positionPerDegree Ganho em posicao do servo por grau de erro.
     *                          Valor inicial sugerido: 0.0045
     * @param correctionSign Use 1 ou -1 conforme o sentido de correcao
     * @param deadbandDegrees Erro angular considerado alinhado
     */
    public HiveTurretAligner(
            Limelight3A limelight,
            Servo turret,
            double centerPosition,
            double positionPerDegree,
            double correctionSign,
            double deadbandDegrees) {

        if (limelight == null || turret == null) {
            throw new IllegalArgumentException(
                    "Limelight e servo sao obrigatorios.");
        }

        if (positionPerDegree <= 0 || deadbandDegrees < 0) {
            throw new IllegalArgumentException(
                    "Ganho e tolerancia devem ser validos.");
        }

        if (correctionSign != 1 && correctionSign != -1) {
            throw new IllegalArgumentException(
                    "correctionSign deve ser 1 ou -1.");
        }

        this.limelight = limelight;
        this.turret = turret;
        this.positionPerDegree = positionPerDegree;
        this.correctionSign = correctionSign;
        this.deadbandDegrees = deadbandDegrees;

        this.targetPosition = clipServoPosition(centerPosition);

        hiveTagIds = new HashSet<>();
        hiveTagIds.add(HIVE_TAG_39);
        hiveTagIds.add(HIVE_TAG_40);
        hiveTagIds.add(HIVE_TAG_43);
        hiveTagIds.add(HIVE_TAG_44);

        turret.setPosition(this.targetPosition);
    }

    /**
     * Inicia a leitura da Limelight.
     */
    public void start() {
        limelight.start();
    }

    /**
     * Le a Limelight e atualiza a posicao da turret.
     *
     * Este metodo deve ser chamado repetidamente dentro do loop do OpMode.
     */
    public Status update() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            status = Status.NO_RESULT;
            return status;
        }

        LLResultTypes.FiducialResult target =
                findLargestHive(result.getFiducialResults());

        if (target == null) {
            status = Status.NO_HIVE;
            return status;
        }

        lastTagId = target.getFiducialId();
        lastTxDegrees = target.getTargetXDegrees();

        /*
         * Se o erro horizontal estiver dentro da tolerancia,
         * a turret ja esta alinhada.
         */
        if (Math.abs(lastTxDegrees) <= deadbandDegrees) {
            status = Status.ALIGNED;
            return status;
        }

        /*
         * Correcao proporcional:
         *
         * nova posicao =
         * posicao atual + sentido * erro angular * ganho
         */
        targetPosition = clipServoPosition(
                targetPosition
                        + correctionSign
                        * lastTxDegrees
                        * positionPerDegree
        );

        turret.setPosition(targetPosition);

        status = Status.SEARCHING;
        return status;
    }

    /**
     * Procura somente as tags 39, 40, 43 e 44.
     *
     * Se mais de uma estiver visivel, escolhe a de maior area,
     * normalmente a mais proxima ou mais bem detectada.
     */
    private LLResultTypes.FiducialResult findLargestHive(
            List<LLResultTypes.FiducialResult> fiducials) {

        LLResultTypes.FiducialResult best = null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int detectedId = fiducial.getFiducialId();

            // Ignora qualquer AprilTag que nao seja 39, 40, 43 ou 44.
            if (!hiveTagIds.contains(detectedId)) {
                continue;
            }

            if (best == null
                    || fiducial.getTargetArea() > best.getTargetArea()) {
                best = fiducial;
            }
        }

        return best;
    }

    /**
     * Garante que a posicao enviada ao servo fique entre 0.0 e 1.0.
     */
    private double clipServoPosition(double position) {
        return Range.clip(position, SERVO_ESQ, SERVO_DIR);
    }

    /**
     * Converte um angulo Taura para a posicao do servo.
     *
     * Angulo positivo = anti-horario
     * Angulo negativo = horario
     *
     * Exemplos:
     *  90 graus  -> 1.0
     *   0 graus  -> 0.5
     * -90 graus  -> 0.0
     */
    public static double angleToServoPosition(double angleDegrees) {
        return Range.clip(
                SERVO_CENTRO
                        + angleDegrees / DEGREES_PER_SERVO_UNIT,
                SERVO_ESQ,
                SERVO_DIR
        );
    }

    /**
     * Retorna true quando a turret esta alinhada dentro da tolerancia.
     */
    public boolean isAligned() {
        return status == Status.ALIGNED;
    }

    /**
     * Retorna o estado atual do alinhamento.
     */
    public Status getStatus() {
        return status;
    }

    /**
     * Retorna o ultimo erro horizontal da tag em graus.
     */
    public double getLastTxDegrees() {
        return lastTxDegrees;
    }

    /**
     * Retorna o ID da última HIVE detectada.
     */
    public int getLastTagId() {
        return lastTagId;
    }

    /**
     * Retorna a ultima posicao comandada ao servo.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Para a leitura da Limelight.
     */
    public void stop() {
        limelight.stop();
    }
}