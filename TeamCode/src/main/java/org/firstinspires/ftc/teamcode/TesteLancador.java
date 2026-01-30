package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TesteLancador extends OpMode {

    DcMotor launcher;

    // ===== TOGGLE =====
    boolean launcherAtivo = false;
    boolean lbAnterior = false;

    @Override
    public void init() {

        launcher = hardwareMap.get(DcMotor.class, "launcher");

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // ===== TOGGLE DO LAUNCHER (LB) =====
        boolean lbAtual = gamepad1.left_bumper;

        if (lbAtual && !lbAnterior) {
            launcherAtivo = !launcherAtivo;
        }
        lbAnterior = lbAtual;

        launcher.setPower(launcherAtivo ? 1.0 : 0);

        telemetry.addData("Launcher Ativo", launcherAtivo);
        telemetry.update();
    }
}
