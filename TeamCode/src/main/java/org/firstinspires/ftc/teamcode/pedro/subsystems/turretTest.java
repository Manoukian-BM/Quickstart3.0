package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TesteTurretUltraPlanetary", group = "Turret")
public class turretTest extends LinearOpMode {

    // PARÂMETROS DE REDUÇÃO E ENCODER (Ajustar aqui se mudar o ‘hardware’)
    // Motor HD Hex base = 28 ticks. Com cartucho 3:1 = 28 * 3 = 84
    static final double TICKS_PER_MOTOR_REV = 84.0;

    // Se a turret tiver uma engrenagem externa ligada ao motor, inserir a relação aqui.
    // Exemplo: se a engrenagem da turret for 4x maior que a do motor, coloque 4.0. Se for direta, use 1.0.
    static final double TURRET_GEAR_RATIO   = 1.0;

    static final double TICKS_PER_DEGREE    = (TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO) / 360.0;

    // Alvos de posicionamento predefinidos (em graus)
    double posicaoAlvoGraus = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Mapeamento do hardware (ID configurado no Driver Station)
        // Configuração do Atuador
        DcMotorEx motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");

        // Configurações iniciais de segurança e comportamento
        motorTurret.setDirection(DcMotor.Direction.FORWARD); // Altere para REVERSE se girar invertido
        motorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Segura a turret no lugar ao parar

        // Reinicia o encoder para garantir que a posição atual seja o "Zero" do robô
        motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Configura para o modo de malha fechada por posição
        motorTurret.setTargetPosition(0);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Aguardando Inicialização...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // EXEMPLO DE CONTROLE POR BOTOEIRA (Gamepad 1 ou 2)
            // Define posições lógicas de teste usando os botões
            if (gamepad1.a) {
                posicaoAlvoGraus = 0.0;    // Posição centralizada
            } else if (gamepad1.b) {
                posicaoAlvoGraus = 90.0;   // 90 graus para a direita
            } else if (gamepad1.x) {
                posicaoAlvoGraus = -90.0;  // 90 graus para a esquerda
            }

            // Conversão matemática dinâmica de Graus → Ticks
            int targetTicks = (int) (posicaoAlvoGraus * TICKS_PER_DEGREE);

            // Envia o comando para o motor
            motorTurret.setTargetPosition(targetTicks);

            // Define a velocidade máxima que a turret usará para buscar a posição (0.0 a 1.0)
            motorTurret.setPower(0.5);

            // Telemetria para acompanhar os testes na Driver
            telemetry.addData("Alvo (Graus)", posicaoAlvoGraus);
            telemetry.addData("Alvo (Ticks)", targetTicks);
            telemetry.addData("Posição Atual (Ticks)", motorTurret.getCurrentPosition());
            telemetry.addData("Erro de Posição", targetTicks - motorTurret.getCurrentPosition());
            telemetry.update();
        }
    }
}
