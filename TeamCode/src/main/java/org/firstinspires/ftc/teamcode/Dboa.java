package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class Dboa extends OpMode {

    DcMotor fe, fd, te, td;

    double AzulFE, AzulTD, VermelhoTE, VermelhoFD;
    double SpinMode = 1;

    VoltageSensor batteryVoltage;

    @Override
    public void init() {

        fd = hardwareMap.get(DcMotor.class, "FD");
        fe = hardwareMap.get(DcMotor.class, "FE");
        te = hardwareMap.get(DcMotor.class, "TE");
        td = hardwareMap.get(DcMotor.class, "TD");

        // motores do lado esquerdo invertidos
        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void loop() {

        // ===== LOCOMOÇÃO =====
        double rotRobo = gamepad1.right_stick_x;

        // CORREÇÃO AQUI
        double x = gamepad1.left_stick_x;     // direita = direita
        double y = -gamepad1.left_stick_y;    // frente = frente

        double FAzul = y + x;
        double FVermelho = y - x;

        if (gamepad1.right_stick_button) {
            SpinMode = (SpinMode == 0) ? 1 : 0;
        }

        if (SpinMode == 0) {
            AzulFE = FAzul + (rotRobo * 1.8);
            AzulTD = FAzul - (rotRobo * 1.8);
            VermelhoTE = FVermelho + (rotRobo * 1.8);
            VermelhoFD = FVermelho - (rotRobo * 1.8);
        } else {
            AzulFE = FAzul + rotRobo;
            AzulTD = FAzul - rotRobo;
            VermelhoTE = FVermelho + rotRobo;
            VermelhoFD = FVermelho - rotRobo;
        }

        fd.setPower(compensar(VermelhoFD));
        te.setPower(compensar(VermelhoTE));
        fe.setPower(compensar(AzulFE));
        td.setPower(compensar(AzulTD));
    }
}
