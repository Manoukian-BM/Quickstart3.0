package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Controla automaticamente a turret apontando para a HIVE mais próxima.
 *
 * A classe usa:
 * - Pose do Pinpoint/Pedro Pathing para saber a posição do robô;
 * - Encoder do motor para saber o ângulo atual da turret;
 * - PID para controlar o motor;
 * - Histerese para evitar troca constante entre as duas HIVEs.
 *
 * Todas as unidades angulares internas são RADIANOS.
 */
public class AutoHiveTurret {

    /**
     * Identifica qual HIVE está sendo usada.
     */
    public enum Hive {
        LEFT,
        RIGHT
    }

    /*
     * Substituir essas coordenadas pelas reais das HIVEs.
     *
     * Os valores devem:
     * - estar no sistema de coordenadas do Pedro Pathing;
     * - usar as mesmas unidades do Pose;
     * - ser medidos a partir da mesma origem usada pelo Pinpoint.
     *
     * O terceiro valor é o heading da HIVE e não é utilizado neste cálculo.
     */
    private static final Pose LEFT_HIVE = new Pose(
            84.0,
            84.0,
            Math.toRadians(0.0)
    );

    private static final Pose RIGHT_HIVE = new Pose(
            84.0,
            52.0,
            Math.toRadians(0.0)
    );

    /*
     * Margem mínima para trocar de HIVE.
     *
     * Exemplo:
     * se a HIVE atual está a 50 unidades e a outra está a 48,
     * a turret não troca.
     *
     * Isso evita troca constante quando o robô está perto do meio do campo.
     */
    private static final double HIVE_SWITCH_MARGIN = 8.0;

    /*
     * Limites mecânicos opcionais da turret.
     *
     * Estes valores são relativos ao zero da turret.
     * Se a turret puder girar 360 graus, deixe LIMITES como false.
     */
    private static final boolean LIMITES = false;
    private static final double MIN_TURRET_ANGLE = Math.toRadians(-135.0);
    private static final double MAX_TURRET_ANGLE = Math.toRadians(135.0);

    private final Localizer localizer;
    private final DcMotorEx turretMotor;

    /*
     * Configuração do encoder.
     *
     * TURRET_TICKS_PER_REV:
     * quantidade de pulsos do encoder do motor em uma volta do motor.
     *
     * TURRET_GEAR_RATIO:
     * quantas voltas o motor faz para a turret completar uma volta.
     *
     * Exemplo:
     * - motor: 537.7 ticks por volta;
     * - redução 2:1;
     * - o motor gira duas vezes para a turret girar uma vez.
     *
     * Nesse caso:
     * TURRET_TICKS_PER_REV: 537.7.
     * TURRET_GEAR_RATIO = 2.0.
     */
    private final double turretTicksPerRev;
    private final double turretGearRatio;

    /*
     * Posição do encoder quando a turret está no seu zero mecânico.
     *
     * Essa posição é capturada ao criar o objeto.
     * Portanto, antes de iniciar o teste, coloque a turret na posição zero.
     */
    private final int encoderZeroPosition;

    /*
     * Se a turret girar para o lado contrário ao esperado,
     * altere este valor para -1.
     */
    private final double motorDirection;

    /*
     * Ganhos do PID.
     *
     * Comece usando apenas KP.
     * Depois ajuste KD e, por último, KI se necessário.
     */
    private final double kP;
    private final double kI;
    private final double kD;

    private final ElapsedTime timer = new ElapsedTime();

    private Hive selectedHive = Hive.RIGHT;

    private double integral;
    private double previousError;

    private double desiredTurretAngle;
    private double turretAngle;
    private double error;
    private double motorPower;

    /**
     * Construtor para o primeiro teste.
     *
     * @param localizer Localizer do Pedro Pathing configurado com o Pinpoint
     * @param turretMotor motor da turret
     * @param turretTicksPerRev ticks do encoder por volta do motor
     * @param turretGearRatio redução entre motor e turret
     * @param motorDirection 1.0 ou -1.0
     * @param kP ganho proporcional
     * @param kI ganho integral
     * @param kD ganho derivativo
     */
    public AutoHiveTurret(
            Localizer localizer,
            DcMotorEx turretMotor,
            double turretTicksPerRev,
            double turretGearRatio,
            double motorDirection,
            double kP,
            double kI,
            double kD) {

        if (localizer == null) {
            throw new IllegalArgumentException("localizer não pode ser null");
        }

        if (turretMotor == null) {
            throw new IllegalArgumentException("turretMotor não pode ser null");
        }

        if (turretTicksPerRev <= 0.0) {
            throw new IllegalArgumentException(
                    "turretTicksPerRev deve ser positivo"
            );
        }

        if (turretGearRatio <= 0.0) {
            throw new IllegalArgumentException(
                    "turretGearRatio deve ser positivo"
            );
        }

        if (motorDirection != 1.0 && motorDirection != -1.0) {
            throw new IllegalArgumentException(
                    "motorDirection deve ser 1.0 ou -1.0"
            );
        }

        this.localizer = localizer;
        this.turretMotor = turretMotor;
        this.turretTicksPerRev = turretTicksPerRev;
        this.turretGearRatio = turretGearRatio;
        this.motorDirection = motorDirection;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        /*
         * Antes de criar esta classe, posicione fisicamente a turret no zero.
         */
        this.encoderZeroPosition = turretMotor.getCurrentPosition();

        turretMotor.setZeroPowerBehavior(
                DcMotorEx.ZeroPowerBehavior.BRAKE
        );

        turretMotor.setMode(
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER
        );

        timer.reset();
    }

    /**
     * Deve ser chamado uma vez por ciclo do OpMode.
     *
     * Este método:
     * 1. atualiza o Pinpoint;
     * 2. seleciona a HIVE mais próxima;
     * 3. calcula o ângulo até ela;
     * 4. lê o encoder da turret;
     * 5. executa o PID;
     * 6. aplica potência ao motor.
     */
    public void update() {
        /*
         * O Pinpoint precisa ser atualizado antes de ler a pose.
         */
        localizer.update();

        Pose robotPose = localizer.pose();

        selectNearestHive(robotPose);

        Pose selectedHivePose = getSelectedHivePose();

        /*
         * Ângulo global entre o robô e a HIVE.
         */
        double angleToHive = Math.atan2(
                selectedHivePose.y() - robotPose.y(),
                selectedHivePose.x() - robotPose.x()
        );

        /*
         * A turret está presa ao robô.
         *
         * Por isso, subtraímos o heading do robô para transformar
         * o ângulo global em um ângulo relativo ao chassi.
         */
        desiredTurretAngle = normalizeAngle(
                angleToHive - robotPose.heading()
        );

        turretAngle = getTurretAngleRadians();

        error = normalizeAngle(
                desiredTurretAngle - turretAngle
        );

        /*
         * Se houver limites mecânicos, impede que a turret tente
         * alcançar um ângulo impossível.
         */
        if (LIMITES
                && (desiredTurretAngle < MIN_TURRET_ANGLE
                || desiredTurretAngle > MAX_TURRET_ANGLE)) {

            motorPower = 0.0;
            turretMotor.setPower(0.0);
            resetPid();
            return;
        }

        motorPower = calculatePid(error);

        /*
         * Limita a potência para evitar movimentos muito agressivos.
         *
         * Comece com 0.5 durante os testes.
         */
        motorPower = Range.clip(motorPower, -0.5, 0.5);

        turretMotor.setPower(motorPower * motorDirection);
    }

    /**
     * Seleciona automaticamente a HIVE mais próxima.
     *
     * A histerese evita que a seleção fique alternando quando
     * as duas HIVEs possuem distâncias semelhantes.
     */
    private void selectNearestHive(Pose robotPose) {
        double distanceToLeftSquared = squaredDistance(
                robotPose,
                LEFT_HIVE
        );

        double distanceToRightSquared = squaredDistance(
                robotPose,
                RIGHT_HIVE
        );

        Hive nearestHive;

        if (distanceToLeftSquared <= distanceToRightSquared) {
            nearestHive = Hive.LEFT;
        } else {
            nearestHive = Hive.RIGHT;
        }

        /*
         * Se a HIVE mais próxima já está selecionada, não há nada a fazer.
         */
        if (nearestHive == selectedHive) {
            return;
        }

        double selectedHiveDistanceSquared;
        double nearestHiveDistanceSquared;

        if (selectedHive == Hive.LEFT) {
            selectedHiveDistanceSquared = distanceToLeftSquared;
            nearestHiveDistanceSquared = distanceToRightSquared;
        } else {
            selectedHiveDistanceSquared = distanceToRightSquared;
            nearestHiveDistanceSquared = distanceToLeftSquared;
        }

        /*
         * Só troca quando a nova HIVE for significativamente mais próxima.
         */
        double marginSquared = HIVE_SWITCH_MARGIN * HIVE_SWITCH_MARGIN;

        if (selectedHiveDistanceSquared
                - nearestHiveDistanceSquared > marginSquared) {

            selectedHive = nearestHive;

            /*
             * Zera o integral quando o alvo muda.
             * Isso evita que o PID carregue erro acumulado da HIVE anterior.
             */
            resetPid();
        }
    }

    /**
     * Retorna a coordenada da HIVE atualmente selecionada.
     */
    private Pose getSelectedHivePose() {
        if (selectedHive == Hive.LEFT) {
            return LEFT_HIVE;
        }

        return RIGHT_HIVE;
    }

    /**
     * Calcula distância ao quadrado.
     *
     * Não usamos sqrt porque, para comparar distâncias,
     * o valor quadrático é suficiente e mais barato.
     */
    private double squaredDistance(Pose first, Pose second) {
        double deltaX = second.x() - first.x();
        double deltaY = second.y() - first.y();

        return deltaX * deltaX + deltaY * deltaY;
    }

    /**
     * Converte a posição atual do encoder em radianos.
     */
    private double getTurretAngleRadians() {
        int currentEncoderPosition = turretMotor.getCurrentPosition();

        int encoderDelta = currentEncoderPosition - encoderZeroPosition;

        /*
         * Primeiro converte ticks para voltas do motor.
         * Depois converte voltas para voltas da turret.
         * Finalmente converte voltas para radianos.
         */
        double motorRevolutions =
                encoderDelta / turretTicksPerRev;

        double turretRevolutions =
                motorRevolutions / turretGearRatio;

        return turretRevolutions * 2.0 * Math.PI;
    }

    /**
     * Controlador PID.
     */
    private double calculatePid(double currentError) {
        double deltaTime = timer.seconds();
        timer.reset();

        /*
         * Proteção contra um primeiro ciclo muito pequeno
         * ou uma leitura de tempo inválida.
         */
        if (deltaTime <= 0.0 || deltaTime > 0.5) {
            deltaTime = 0.02;
        }

        /*
         * Zona morta para reduzir tremores quando já está alinhado.
         */
        double deadband = Math.toRadians(1.0);

        if (Math.abs(currentError) < deadband) {
            currentError = 0.0;
        }

        /*
         * Componente integral.
         *
         * O limite impede windup, que pode causar uma arrancada
         * forte depois que a turret fica presa.
         */
        integral += currentError * deltaTime;
        integral = Range.clip(integral, -0.5, 0.5);

        /*
         * Componente derivativa.
         */
        double derivative =
                (currentError - previousError) / deltaTime;

        previousError = currentError;

        return kP * currentError
                + kI * integral
                + kD * derivative;
    }

    /**
     * Zera o estado acumulado do PID.
     */
    private void resetPid() {
        integral = 0.0;
        previousError = 0.0;
        timer.reset();
    }

    /**
     * Permite selecionar manualmente uma HIVE, caso necessário.
     */
    public void setSelectedHive(Hive hive) {
        if (hive == null) {
            throw new IllegalArgumentException("hive não pode ser null");
        }

        selectedHive = hive;
        resetPid();
    }

    /**
     * Retorna a HIVE atualmente selecionada.
     */
    public Hive getSelectedHive() {
        return selectedHive;
    }

    /**
     * Retorna o ângulo desejado da turret em radianos.
     */
    public double getDesiredTurretAngle() {
        return desiredTurretAngle;
    }

    /**
     * Retorna o ângulo atual da turret em radianos.
     */
    public double getTurretAngle() {
        return turretAngle;
    }

    /**
     * Retorna o erro angular atual em radianos.
     */
    public double getError() {
        return error;
    }

    /**
     * Retorna a potência aplicada ao motor.
     */
    public double getMotorPower() {
        return motorPower;
    }

    /**
     * Para a turret.
     */
    public void stop() {
        motorPower = 0.0;
        turretMotor.setPower(0.0);
        resetPid();
    }

    /**
     * Mantém ângulos no intervalo (-PI, PI].
     */
    private double normalizeAngle(double angle) {
        return Math.atan2(
                Math.sin(angle),
                Math.cos(angle)
        );
    }
}