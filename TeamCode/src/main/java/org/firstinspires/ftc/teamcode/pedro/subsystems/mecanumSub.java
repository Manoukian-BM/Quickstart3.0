package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class mecanumSub {

    private final DcMotor roda0;
    private final DcMotor roda1;
    private final DcMotor roda2;
    private final DcMotor roda3;
    private Limelight3A limelight3A;
    RevBlinkinLedDriver led;
    double ID_TAG;
    double integral = 0;
    double lastError = 0;
    double timer = 0;

    public mecanumSub(HardwareMap hardwareMap) {

        roda0 = hardwareMap.get(DcMotor.class, "roda0");
        roda1 = hardwareMap.get(DcMotor.class, "roda1");
        roda2 = hardwareMap.get(DcMotor.class, "roda2");
        roda3 = hardwareMap.get(DcMotor.class, "roda3");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        roda0.setDirection(DcMotor.Direction.REVERSE);
        roda1.setDirection(DcMotor.Direction.REVERSE);
        roda2.setDirection(DcMotor.Direction.REVERSE);
        roda3.setDirection(DcMotor.Direction.FORWARD);

        /*try {
            limelight3A = hardwareMap.get(Limelight3A.class, "limelight3A");
            limelight3A.setPollRateHz(90);
            limelight3A.pipelineSwitch(0);
            limelight3A.start();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }*/
    }

    public void teleopUpdate(Gamepad gamepad, Gamepad gamepad2) {
        double divisor;
        if (gamepad.left_bumper) divisor = 3.2;       // Lento
        else if (gamepad.right_bumper) divisor = 0.1; // Turbo
        else divisor = 1.18;                           // Normal

        double vertical = -(gamepad.left_stick_y / divisor);
        double horizontal = -(gamepad.left_stick_x / divisor);
        double pivot = (gamepad.right_stick_x / divisor);

        roda1.setPower(-pivot + (vertical + (horizontal)));
        roda3.setPower(-pivot + vertical - (horizontal));
        roda0.setPower(pivot + vertical - (horizontal));
        roda2.setPower(pivot + (vertical + horizontal));
    }

    public void aplicarPotencia(double r0, double r2, double r1, double r3) {
        roda0.setPower(r0);
        roda2.setPower(r2);
        roda1.setPower(r1);
        roda3.setPower(r3);
    }

    public void stop() {
        if (limelight3A != null) limelight3A.stop();
        aplicarPotencia(0,0,0,0);
    }

    public void Depuration(Telemetry telemetry) {
        LLResult result = limelight3A.getLatestResult();
        telemetry.addData("Tx", result.getTx());
        telemetry.addData("Ty", result.getTy());
        telemetry.addData("Ta", result.getTa());
        telemetry.update();
    }
}
