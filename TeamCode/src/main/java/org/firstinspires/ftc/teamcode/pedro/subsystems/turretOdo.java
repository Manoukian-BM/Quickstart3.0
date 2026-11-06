package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;



public class turretOdo {

    public enum EstadoTurret { MANUAL, AUTOMATICO }
    private EstadoTurret estadoAtual = EstadoTurret.AUTOMATICO;
    private static final double SERVO_RATE_PER_SECOND = 0.25;
    private double manualPosition = 0.5;
    private long previousUpdateNanos = System.nanoTime();
    private boolean previousR3, previousRightBumper, previousReset;
    private static final double SERVO_MIN = 0.0, SERVO_MAX = 1.0;
    private TurretGeometry.Aim aim;
    private boolean servoSaturated;
    private double autoTrim;
    private static final double AUTO_TRIM_RATE = 0.02;
    private static final double MAX_AUTO_TRIM = 0.05;
    private final BlueHiveFineAdjustment fineAim = new BlueHiveFineAdjustment();
    private Limelight3A limelight;
    private boolean cameraStarted;
    private static final int VISION_PIPELINE = 1;
    // Calibrar o eixo horizontal da camera com a linha de tiro real.
    // Zero pressupoe alinhamento; nao e uma medida mecanica.
    private static final double CAMERA_AIM_TX_DEGREES = 0.0;
    private String cameraStatus = "Camera opcional, aguardando R1 do P2";
    private double visionError = Double.NaN;
    private int visionTagId = -1;

    private static final double CENTRO = 0.5;
    private static final double SERVO_RANGE_DEGREES = 360.0;
    private static final double AIM_REFRESH_TOLERANCE_DEGREES = 0.6;
    private static final double SERVO_DIRECTION = -1.0;

    private final Follower follower;
    private final Servo turretR, turretL;

    private static final double OPENING_SWITCH_MARGIN_IN = 5.0 / 2.54;
    private int selectedCell = -1;
    private int lockedCell = -1;
    private double lockedBaseAngle;
    private double targetX;
    private double targetY;
    private double servoPosition = CENTRO;
    private double desiredPosition = CENTRO;
    private double angleDegrees;
    private static double distSelected;
    private double leftHiveDistance;
    private double rightHiveDistance;
    private String selectedHive = "Nenhuma";

    // +X afasta da lateral esquerda; +Y aproxima da parede amarela da foto.
    // As duas aberturas usam as medidas reais e o mesmo referencial do reset.
    double leftHiveX = BlueHiveField.BLUE_X_IN;
    double leftHiveY = BlueHiveField.SCORING_Y_IN;
    double rightHiveX = BlueHiveField.BLUE_X_IN;
    double rightHiveY = BlueHiveField.AUDIENCE_Y_IN;

    public turretOdo(HardwareMap hardwareMap, Follower follower) {
        if (follower == null) {
            throw new IllegalArgumentException("Follower nao pode ser null.");
        }
        this.follower = follower;
        turretR = hardwareMap.get(Servo.class, "turretR");
        turretL = hardwareMap.get(Servo.class, "turretL");
        double lastPosition = turretR.getPosition();
        if (Double.isFinite(lastPosition)) servoPosition = clamp(lastPosition, SERVO_MIN, SERVO_MAX);
        desiredPosition = servoPosition;
        manualPosition = servoPosition;
        limelight=hardwareMap.tryGet(Limelight3A.class,"limelight");
        if (limelight==null) limelight=hardwareMap.tryGet(Limelight3A.class,"limelight3A");
    }

    public void selecionarHive() {
        selecionarHive(follower.pose());
    }

    private void selecionarHive(Pose pose) {

        leftHiveDistance = distance(pose.x(), pose.y(), leftHiveX, leftHiveY);
        rightHiveDistance = distance(pose.x(), pose.y(), rightHiveX, rightHiveY);

        // Perto da linha central, mantenha a abertura escolhida enquanto a pose oscila.
        double fromCenter = pose.y() - BlueHiveField.CENTER_Y_IN;
        if (selectedCell < 0 || Math.abs(fromCenter) > OPENING_SWITCH_MARGIN_IN) {
            selectedCell = fromCenter < 0 ? 0 : 1;
        }
        if (selectedCell == 0) {
            targetX = leftHiveX;
            targetY = leftHiveY;
            selectedHive = "Azul / lado scoring (42-45)";
            distSelected = leftHiveDistance;
        } else {
            targetX = rightHiveX;
            targetY = rightHiveY;
            selectedHive = "Azul / lado publico (38-41)";
            distSelected = rightHiveDistance;
        }
    }

    private double odometryAngle(Pose pose) {
        return aim.relativeAngle + TurretGeometry.fieldCorrectionRadians(pose.x(), selectedCell == 1);
    }

    private boolean lockMatchesCurrentAim(Pose pose) {
        return lockedCell >= 0 && lockedCell == selectedCell && aim != null && aim.valid
                && Math.abs(Math.toDegrees(TurretGeometry.normalize(odometryAngle(pose) - lockedBaseAngle)))
                <= AIM_REFRESH_TOLERANCE_DEGREES;
    }

    private void commandLockedAim() {
        double correctedAngle = TurretGeometry.normalize(lockedBaseAngle
                + Math.toRadians(fineAim.degrees(lockedCell)));
        angleDegrees = Math.toDegrees(correctedAngle);
        double raw = CENTRO + SERVO_DIRECTION * angleDegrees / SERVO_RANGE_DEGREES + autoTrim;
        servoSaturated = raw < SERVO_MIN || raw > SERVO_MAX;
        desiredPosition = TurretGeometry.servoCommand(correctedAngle, CENTRO, SERVO_DIRECTION,
                SERVO_RANGE_DEGREES, autoTrim, SERVO_MIN, SERVO_MAX);
        setServoPosition(desiredPosition);
    }

    private void advanceServo(double elapsedSeconds) {
        double remaining = desiredPosition - servoPosition;
        if (remaining == 0) return;
        double maxStep = SERVO_RATE_PER_SECOND * elapsedSeconds;
        double next = Math.abs(remaining) <= maxStep ? desiredPosition
                : servoPosition + Math.copySign(maxStep, remaining);
        setServoPosition(next);
    }

    private void setServoPosition(double next) {
        if (next == servoPosition) return;
        servoPosition = next;
        turretR.setPosition(servoPosition);
        turretL.setPosition(servoPosition);
    }

    /** LB do P2 acompanha pela odometria enquanto pressionado. R1 solicita ajuste fino. */
    public void update(Gamepad gamepad, Gamepad gamepad2) {
        if (gamepad.dpad_right && !previousReset) {
            follower.setX(BlueHiveField.RESET_X_IN);
            follower.setY(BlueHiveField.RESET_Y_IN);
            follower.setHeading(BlueHiveField.RESET_HEADING_RADIANS);
            selectedCell = -1;
            lockedCell = -1;
            desiredPosition = servoPosition;
            manualPosition = servoPosition;
            fineAim.clearAll();
        }
        previousReset = gamepad.dpad_right;
        // Ler depois de qualquer redefinicao da pose para nao mirar com a pose anterior.
        Pose pose = follower.pose();
        boolean validPose=Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
        if (validPose) {
            selecionarHive(pose);
            aim = TurretGeometry.calculate(pose.x(), pose.y(), pose.heading(), targetX, targetY);
        } else {
            distSelected = Double.NaN;
            aim = null;
        }
        long now = System.nanoTime();
        double elapsedSeconds = Math.min((now - previousUpdateNanos) / 1_000_000_000.0, 0.05);
        previousUpdateNanos = now;
        boolean modeChanged = gamepad2.right_stick_button && !previousR3;
        if (modeChanged) {
            estadoAtual = estadoAtual == EstadoTurret.AUTOMATICO
                    ? EstadoTurret.MANUAL : EstadoTurret.AUTOMATICO;
            // Comecar o ajuste a partir da posicao realmente comandada.
            manualPosition = servoPosition;
            desiredPosition = servoPosition;
            lockedCell = -1;
            if (fineAim.isActive()) fineAim.cancel("Troca manual/automatico; ajuste cancelado");
        }
        previousR3 = gamepad2.right_stick_button;
        boolean rightBumper = gamepad2.right_bumper;
        boolean request = rightBumper && !previousRightBumper;
        previousRightBumper = rightBumper;
        if (estadoAtual == EstadoTurret.MANUAL) {
            if (gamepad2.dpad_up) {
                manualPosition = CENTRO;
            } else if (Math.abs(gamepad2.right_stick_x) > 0.2) {
                manualPosition = clamp(manualPosition - gamepad2.right_stick_x
                        * SERVO_RATE_PER_SECOND * elapsedSeconds, 0.0, 1.0);
            }
            angleDegrees=(manualPosition-CENTRO)*SERVO_RANGE_DEGREES/SERVO_DIRECTION;
            servoSaturated=false;
            desiredPosition = manualPosition;
            advanceServo(elapsedSeconds);
            if (request) fineAim.cancel("Modo manual: use R3 para voltar a odometria");
        } else {
            boolean manualTrim = Math.abs(gamepad2.right_stick_x) > 0.2;
            if (gamepad2.dpad_up) {
                autoTrim = 0;
                int cellToClear = lockedCell >= 0 ? lockedCell : selectedCell;
                if (cellToClear >= 0) fineAim.clear(cellToClear);
            } else if (manualTrim) {
                if (fineAim.isActive()) fineAim.cancel("Ajuste manual tem prioridade");
                autoTrim = clamp(autoTrim - gamepad2.right_stick_x * AUTO_TRIM_RATE
                        * elapsedSeconds, -MAX_AUTO_TRIM, MAX_AUTO_TRIM);
            }
            if (gamepad2.left_bumper && aim != null && aim.valid) {
                lockedCell = selectedCell;
                lockedBaseAngle = odometryAngle(pose);
                commandLockedAim();
            } else if (lockedCell >= 0 && (gamepad2.dpad_up || manualTrim)) {
                // Ajustes manuais atuam sobre o ultimo angulo escolhido com LB.
                commandLockedAim();
            }
            if (gamepad2.dpad_up || manualTrim) return;
            if (request) {
                visionTagId = -1;
                visionError = Double.NaN;
                if (!lockMatchesCurrentAim(pose)) {
                    fineAim.cancel("Segure LB do P2 para alinhar antes de usar R1");
                } else if (!servoSaturated && startCamera()) {
                    fineAim.request(lockedCell, now, pose.x(), pose.y(), pose.heading(), servoPosition);
                } else {
                    fineAim.cancel("Sem ajuste visual. Confira limite e camera");
                }
            }
            if (fineAim.isActive()) {
                if (!lockMatchesCurrentAim(pose)) {
                    fineAim.cancel("Pose mudou. Atualize a mira com LB do P2");
                    return;
                }
                fineAim.track(now, lockedCell, pose.x(), pose.y(), pose.heading(), servoPosition);
                if (updateVision(now)) commandLockedAim();
            }
        }
    }

    private boolean startCamera() {
        if (limelight==null) { cameraStatus="Limelight nao configurada; odometria continua"; return false; }
        if (cameraStarted) return true;
        try {
            limelight.setPollRateHz(30);
            if (!limelight.pipelineSwitch(VISION_PIPELINE)) {
                cameraStatus="Falha ao selecionar pipeline AprilTag 1"; return false;
            }
            limelight.start();
            cameraStarted=true;
            cameraStatus="Pipeline AprilTag 1";
            return true;
        } catch (RuntimeException e) {
            cameraStatus="Camera indisponivel; odometria continua";
            return false;
        }
    }

    private boolean updateVision(long now) {
        if (!fineAim.isActive() || limelight==null) return false;
        try {
            LLResult result=limelight.getLatestResult();
            if (result==null) { cameraStatus="Aguardando imagem"; return false; }
            long age=result.getStaleness();
            double latency=result.getCaptureLatency()+result.getTargetingLatency();
            if (age<0 || age>150 || !Double.isFinite(latency) || latency<0 || age+latency>150
                    || result.getPipelineIndex()!=VISION_PIPELINE) {
                cameraStatus="Imagem atrasada ou pipeline incorreto"; return false;
            }
            int firstId = lockedCell == 0 ? 43 : 39;
            visionError = result.isValid()
                    ? middleTagCenter(result.getFiducialResults(), firstId) - CAMERA_AIM_TX_DEGREES
                    : Double.NaN;
            visionTagId = Double.isFinite(visionError) ? firstId : -1;
            String middleTags = firstId + " e " + (firstId + 1);
            cameraStatus = Double.isFinite(visionError) ? "Tags centrais " + middleTags
                    : "Aguardando as tags centrais " + middleTags;
            return fineAim.observe(now, result.getTimestamp(), visionTagId, visionError);
        } catch (RuntimeException e) {
            fineAim.cancel("Leitura da camera falhou; mantida odometria");
            cameraStatus="Camera indisponivel";
            return false;
        }
    }

    private static double middleTagCenter(List<FiducialResult> tags, int firstId) {
        if (tags == null) return Double.NaN;
        double first = Double.NaN;
        double second = Double.NaN;
        for (FiducialResult tag : tags) {
            if (tag == null) continue;
            int id = tag.getFiducialId();
            if (id != firstId && id != firstId + 1) continue;
            double tx = tag.getTargetXDegreesNoCrosshair();
            if (!(Math.abs(tx) < 80)) continue;
            double imageX = Math.tan(Math.toRadians(tx));
            if (id == firstId) first = imageX;
            else second = imageX;
        }
        return Math.toDegrees(Math.atan((first + second) / 2.0));
    }

    public void close() { if (limelight!=null) limelight.stop(); }
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getAutoTrimDegrees() { return autoTrim * SERVO_RANGE_DEGREES; }

    /** Chamado pelo TeleOp no bloco de telemetria limitado a 5 Hz. */
    public void addDiagnostics(Telemetry telemetry) {
        Pose p=follower.pose();
        telemetry.addData("ROBO da lateral oposta / parede amarela (cm)", "%.1f / %.1f",
                (BlueHiveField.RIGHT_WALL_X_IN - p.x()) * 2.54,
                (BlueHiveField.BACK_WALL_Y_IN - p.y()) * 2.54);
        if (estadoAtual == EstadoTurret.AUTOMATICO) {
            if (lockedCell < 0) {
                telemetry.addData("Torreta (LB P2)", "Aguardando LB");
            } else {
                String tags = lockedCell == 0 ? "42-45" : "38-41";
                String status = fineAim.hasCorrection(lockedCell) ? "Com ajuste visual" : "Pela odometria";
                telemetry.addData("Torreta (LB P2)", "Abertura %s | %s", tags,
                        lockMatchesCurrentAim(p) ? status : "Segure LB para acompanhar");
            }
        }
        if (aim != null && aim.valid) {
            telemetry.addData("Alvo pela pose atual", "%s | %s", estadoAtual, selectedHive);
            telemetry.addData("CENTRO da lateral oposta / parede amarela (cm)", "%.1f / %.1f",
                    (BlueHiveField.RIGHT_WALL_X_IN - targetX) * 2.54,
                    (BlueHiveField.BACK_WALL_Y_IN - targetY) * 2.54);
            telemetry.addData("DISTANCIA torreta ao centro (cm)", "%.1f", aim.distance * 2.54);
        } else {
            telemetry.addData("Alvo pela pose atual", aim == null ? "Pose invalida" : "Torreta sobre o alvo");
        }
        telemetry.addData("PINPOINT pose (in/in/deg)","%.2f / %.2f / %.1f",p.x(),p.y(),Math.toDegrees(p.heading()));
        telemetry.addData("PINPOINT offsets X/Y (mm)","%.1f / %.1f",
                org.firstinspires.ftc.teamcode.pedro.Constants.TELEBLUE_X_POD_OFFSET_MM,
                org.firstinspires.ftc.teamcode.pedro.Constants.TELEBLUE_Y_POD_OFFSET_MM);
        telemetry.addData("TORRETA local frente/esq (mm)","-90 / 0");
        telemetry.addData("ALVO azul X/Y (in)","%.2f / %.2f",targetX,targetY);
        telemetry.addData("ALVO 38-41 lateral/fundo (cm)","%.0f / %.0f",
                BlueHiveField.OPENINGS_FROM_LEFT_CM,BlueHiveField.AUDIENCE_FROM_BACK_CM);
        telemetry.addData("ALVO 42-45 lateral/oposta (cm)","%.0f / %.0f",
                BlueHiveField.OPENINGS_FROM_LEFT_CM,BlueHiveField.SCORING_FROM_OPPOSITE_CM);
        telemetry.addData("CAMPO comprimento (cm) / centro Y (in)","%.0f / %.2f",
                BlueHiveField.FIELD_LENGTH_CM,BlueHiveField.CENTER_Y_IN);
        if (aim!=null) {
            telemetry.addData("TORRETA global X/Y (in)","%.2f / %.2f",aim.turretX,aim.turretY);
            telemetry.addData("ALVO distancia do eixo (in)",aim.distance);
            telemetry.addData("ALVO ang global/rel (deg)","%.1f / %.1f",Math.toDegrees(aim.globalAngle),Math.toDegrees(aim.relativeAngle));
            telemetry.addData("TORRETA calculo valido",aim.valid);
        }
        if (aim != null && aim.valid) {
            telemetry.addData("Correcao por posicao (deg)",
                    Math.toDegrees(TurretGeometry.fieldCorrectionRadians(p.x(), selectedCell == 1)));
        }
        telemetry.addData("SERVO ang desejado (deg)",angleDegrees);
        telemetry.addData("SERVO comando / centro", "%.4f / %.4f",servoPosition,CENTRO);
        telemetry.addData("SERVO limites / saturado", "%.2f..%.2f / %s",SERVO_MIN,SERVO_MAX,servoSaturated);
        telemetry.addData("Ajuste visual (P2 R1)",fineAim.status());
        telemetry.addData("Limelight",cameraStatus);
        if (visionTagId >= 0) telemetry.addData("AprilTags centrais usadas", "%d e %d", visionTagId, visionTagId + 1);
        telemetry.addData("Ultimo erro visual (deg)",visionError);
        telemetry.addData("Ajuste visual scoring/publico (deg)","%.2f / %.2f",fineAim.degrees(0),fineAim.degrees(1));
    }
    public EstadoTurret getEstado() { return estadoAtual; }

    public String getSelectedHive() {
        return selectedHive;
    }
    public static double getDist() {
        return distSelected;
    }

    public double getLeftHiveDistance() {
        return leftHiveDistance;
    }

    public double getRightHiveDistance() {
        return rightHiveDistance;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }

    public double getServoPosition() {
        return servoPosition;
    }
}
