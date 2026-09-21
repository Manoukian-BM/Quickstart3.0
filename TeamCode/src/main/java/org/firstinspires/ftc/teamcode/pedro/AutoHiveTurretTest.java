package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.subsystems.AutoHiveTurret;

@TeleOp(name = "Auto Hive Turret Test", group = "Test")
public class AutoHiveTurretTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx turretMotor =
                hardwareMap.get(DcMotorEx.class, "turretMotor");

        /*
         * Esta linha precisa ser substituída pela criação real
         * do seu Localizer Pinpoint.
         */
        Localizer localizer = criarLocalizerPinpoint();

        /*
         * Valores da sua montagem.
         */
        double turretTicksPerRev = 537.7;
        double turretGearRatio = 2.0;
        double motorDirection = 1.0;

        /*
         * Ganhos iniciais do PID.
         */
        double kP = 1.5;
        double kI = 0.0;
        double kD = 0.05;

        AutoHiveTurret turret = new AutoHiveTurret(
                localizer,
                turretMotor,
                turretTicksPerRev,
                turretGearRatio,
                motorDirection,
                kP,
                kI,
                kD
        );

        telemetry.addLine("Posicione a turret no zero.");
        telemetry.addLine("Pressione PLAY para iniciar.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            turret.update();

            telemetry.addData(
                    "HIVE",
                    turret.getSelectedHive()
            );

            telemetry.addData(
                    "Erro graus",
                    Math.toDegrees(turret.getError())
            );

            telemetry.addData(
                    "Potencia",
                    turret.getMotorPower()
            );

            telemetry.update();
        }

        turret.stop();
    }

    private Localizer criarLocalizerPinpoint() {
        /*
         * Aqui deve entrar o código real de criação do seu
         * Localizer configurado com o Pinpoint.
         */
        throw new UnsupportedOperationException(
                "Configure aqui o Localizer Pinpoint"
        );
    }
}