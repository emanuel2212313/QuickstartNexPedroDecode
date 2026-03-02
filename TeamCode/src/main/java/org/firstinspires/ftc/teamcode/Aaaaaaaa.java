package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Aaaaaaaa extends OpMode {

    DcMotor cameraMotor;

    @Override
    public void init() {

        cameraMotor = hardwareMap.get(DcMotor.class, "cameraMotor");

        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        // NÃO aplica força nenhuma
        cameraMotor.setPower(0);

        telemetry.addData("Posicao Encoder", cameraMotor.getCurrentPosition());
        telemetry.update();
    }
}