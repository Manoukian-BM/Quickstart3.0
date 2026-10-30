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
import java.util.ArrayList;
import java.util.List;



public class turretOdo {

    public enum EstadoTurret { MANUAL, AUTOMATICO }
    private EstadoTurret estadoAtual = EstadoTurret.AUTOMATICO;
    private static final double MANUAL_SERVO_RATE = 0.5;
    private double manualPosition = 0.5;
    private long previousUpdateNanos = System.nanoTime();
    private boolean previousR3, previousTriangle, previousReset;
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
    private String cameraStatus = "Camera opcional, aguardando triangulo";
    private double visionError = Double.NaN;
    private int visionFirstId;

    private static final double CENTRO = 0.5;
    private static final double SERVO_RANGE_DEGREES = 360.0;
    private static final double SERVO_DIRECTION = -1.0;

    private final Follower follower;
    private final Servo turretR, turretL;

    private double targetX;
    private double targetY;
    private double servoPosition = CENTRO;
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
        turretR.setPosition(CENTRO);
        turretL.setPosition(CENTRO);
        limelight=hardwareMap.tryGet(Limelight3A.class,"limelight");
        if (limelight==null) limelight=hardwareMap.tryGet(Limelight3A.class,"limelight3A");
    }

    public void selecionarHive() {
        selecionarHive(follower.pose());
    }

    private void selecionarHive(Pose pose) {

        leftHiveDistance = distance(pose.x(), pose.y(), leftHiveX, leftHiveY);
        rightHiveDistance = distance(pose.x(), pose.y(), rightHiveX, rightHiveY);

        if (pose.y() < TurretGeometry.FIELD_CENTER_Y_IN) {
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

    private void alinharPelaOdometria(Pose pose) {
        selecionarHive(pose);
        aim=TurretGeometry.calculate(pose.x(),pose.y(),pose.heading(),targetX,targetY);
        if (!aim.valid) return; // Segurar ultimo comando valido; nao enviar NaN ao servo.
        double correctedAngle=TurretGeometry.normalize(aim.relativeAngle
                + TurretGeometry.fieldCorrectionRadians(pose.x(),pose.y())
                + Math.toRadians(fineAim.degrees(cell(pose))));
        angleDegrees=Math.toDegrees(correctedAngle);
        double raw=CENTRO+SERVO_DIRECTION*angleDegrees/SERVO_RANGE_DEGREES+autoTrim;
        servoSaturated=raw<SERVO_MIN || raw>SERVO_MAX;
        servoPosition=TurretGeometry.servoCommand(correctedAngle,CENTRO,SERVO_DIRECTION,
                SERVO_RANGE_DEGREES,autoTrim,SERVO_MIN,SERVO_MAX);
        turretR.setPosition(servoPosition);
        turretL.setPosition(servoPosition);
    }

    /** Odometra continuamente; triangulo pede apenas um ajuste fino opcional. */
    public void update(Gamepad gamepad, Gamepad gamepad2) {
        if (gamepad.dpad_right && !previousReset) {
            follower.setX(BlueHiveField.RESET_X_IN);
            follower.setY(BlueHiveField.RESET_Y_IN);
            follower.setHeading(BlueHiveField.RESET_HEADING_RADIANS);
            fineAim.clearAll();
        }
        previousReset = gamepad.dpad_right;
        // Ler depois de qualquer redefinicao da pose para nao mirar com a pose anterior.
        Pose pose = follower.pose();
        boolean validPose=Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
        if (!validPose) {
            distSelected=Double.NaN;
            aim=TurretGeometry.calculate(Double.NaN,0,0,targetX,targetY);
        }
        long now = System.nanoTime();
        double elapsedSeconds = Math.min((now - previousUpdateNanos) / 1_000_000_000.0, 0.1);
        previousUpdateNanos = now;
        if (gamepad2.right_stick_button && !previousR3) {
            estadoAtual = estadoAtual == EstadoTurret.AUTOMATICO
                    ? EstadoTurret.MANUAL : EstadoTurret.AUTOMATICO;
            // Comecar o ajuste a partir da posicao realmente comandada.
            manualPosition = turretR.getPosition();
            if (fineAim.isActive()) fineAim.cancel("Troca manual/automatico; ajuste cancelado");
        }
        previousR3 = gamepad2.right_stick_button;
        boolean triangle=gamepad2.triangle || gamepad2.y;
        boolean request=triangle && !previousTriangle;
        previousTriangle=triangle;
        if (estadoAtual == EstadoTurret.MANUAL) {
            if (validPose) {
                selecionarHive(pose);
                aim=TurretGeometry.calculate(pose.x(),pose.y(),pose.heading(),targetX,targetY);
            }
            if (gamepad2.dpad_up) {
                manualPosition = CENTRO;
            } else if (Math.abs(gamepad2.right_stick_x) > 0.2) {
                manualPosition = clamp(manualPosition - gamepad2.right_stick_x
                        * MANUAL_SERVO_RATE * elapsedSeconds, 0.0, 1.0);
            }
            angleDegrees=(manualPosition-CENTRO)*SERVO_RANGE_DEGREES/SERVO_DIRECTION;
            servoSaturated=false;
            servoPosition = manualPosition;
            turretR.setPosition(servoPosition);
            turretL.setPosition(servoPosition);
            if (request) fineAim.cancel("Modo manual: use R3 para voltar a odometria");
        } else {
            if (gamepad2.dpad_up) {
                autoTrim = 0;
                fineAim.clear(cell(pose));
            } else if (Math.abs(gamepad2.right_stick_x) > 0.2) {
                if (fineAim.isActive()) fineAim.cancel("Ajuste manual tem prioridade");
                autoTrim = clamp(autoTrim - gamepad2.right_stick_x * AUTO_TRIM_RATE
                        * elapsedSeconds, -MAX_AUTO_TRIM, MAX_AUTO_TRIM);
            }
            if (validPose) alinharPelaOdometria(pose);
            else aim=TurretGeometry.calculate(Double.NaN,0,0,targetX,targetY);
            if (request && !gamepad2.dpad_up && Math.abs(gamepad2.right_stick_x)<=0.2) {
                if (validPose && !servoSaturated && startCamera())
                    fineAim.request(cell(pose),now,pose.x(),pose.y(),pose.heading(),servoPosition);
                else fineAim.cancel("Sem ajuste visual: confira pose, limite e camera");
            }
            if (gamepad2.dpad_up || Math.abs(gamepad2.right_stick_x)>0.2) return;
            if (fineAim.isActive()) {
                fineAim.track(now,cell(pose),pose.x(),pose.y(),pose.heading(),servoPosition);
                updateVision(now,pose);
                if (validPose) alinharPelaOdometria(pose);
            }
        }
    }

    // Referencial confirmado pelo reset: +Y aponta para a parede amarela da foto.
    // Com a colmeia azul a esquerda, scoring fica em Y baixo e publico em Y alto.
    private static int cell(Pose pose) { return pose.y()<TurretGeometry.FIELD_CENTER_Y_IN ? 0 : 1; }

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

    private void updateVision(long now,Pose pose) {
        if (!fineAim.isActive() || limelight==null) return;
        try {
            LLResult result=limelight.getLatestResult();
            if (result==null) { cameraStatus="Aguardando imagem"; return; }
            long age=result.getStaleness();
            double latency=result.getCaptureLatency()+result.getTargetingLatency();
            if (age<0 || age>150 || !Double.isFinite(latency) || latency<0 || age+latency>150
                    || result.getPipelineIndex()!=VISION_PIPELINE) {
                cameraStatus="Imagem atrasada ou pipeline incorreto"; return;
            }
            List<BlueHiveTagCenter.Detection> detections=new ArrayList<>();
            List<FiducialResult> tags=result.getFiducialResults();
            if (result.isValid() && tags!=null) for (FiducialResult tag:tags) {
                if (tag!=null) detections.add(new BlueHiveTagCenter.Detection(tag.getFiducialId(),
                        tag.getTargetXDegreesNoCrosshair(),tag.getTargetYDegreesNoCrosshair()));
            }
            visionFirstId=cell(pose)==0 ? 42 : 38;
            visionError=BlueHiveTagCenter.horizontalDegrees(
                    detections.toArray(new BlueHiveTagCenter.Detection[0]),visionFirstId)
                    - CAMERA_AIM_TX_DEGREES;
            cameraStatus=Double.isFinite(visionError) ? "Quatro tags azuis confirmadas"
                    : "Precisa das quatro tags "+visionFirstId+"-"+(visionFirstId+3);
            fineAim.observe(now,result.getTimestamp(),visionError);
        } catch (RuntimeException e) {
            fineAim.cancel("Leitura da camera falhou; mantida odometria");
            cameraStatus="Camera indisponivel";
        }
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
        telemetry.addData("Correcao por posicao (deg)",
                Math.toDegrees(TurretGeometry.fieldCorrectionRadians(p.x(),p.y())));
        telemetry.addData("SERVO ang desejado (deg)",angleDegrees);
        telemetry.addData("SERVO comando / centro", "%.4f / %.4f",servoPosition,CENTRO);
        telemetry.addData("SERVO limites / saturado", "%.2f..%.2f / %s",SERVO_MIN,SERVO_MAX,servoSaturated);
        telemetry.addData("Ajuste visual (P2 triangulo)",fineAim.status());
        telemetry.addData("Limelight",cameraStatus);
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
