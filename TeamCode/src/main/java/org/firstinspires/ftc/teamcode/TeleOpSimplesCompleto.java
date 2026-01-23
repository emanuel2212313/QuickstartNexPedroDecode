package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TeleOpSimplesCompleto extends OpMode {


    DcMotor fe, fd, te, td;

    DcMotor intake1, intake2, launcher;

    double AzulFE, AzulTD, VermelhoTE, VermelhoFD;
    double SpinMode = 1;


    Follower follower;

    @Override
    public void init() {

        // ----- MOTORES DO DRIVE -----
        fd = hardwareMap.get(DcMotor.class, "FD");
        fe = hardwareMap.get(DcMotor.class, "FE");
        te = hardwareMap.get(DcMotor.class, "TE");
        td = hardwareMap.get(DcMotor.class, "TD");


        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        launcher = hardwareMap.get(DcMotor.class, "launcher");


        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {

        // ----- MOVIMENTO -----
        double xEsquerdo = -gamepad1.left_stick_x;
        double yEsquerdo = -gamepad1.left_stick_y;

        double FAzul = yEsquerdo + xEsquerdo;
        double FVermelho = yEsquerdo - xEsquerdo;

        // Alternar modo de giro
        if (gamepad1.right_stick_button) {
            SpinMode = (SpinMode == 0) ? 1 : 0;
        }

        if (SpinMode == 0) {
            AzulFE = FAzul + (gamepad1.right_stick_x * 1.8);
            AzulTD = FAzul - (gamepad1.right_stick_x * 1.8);
            VermelhoTE = FVermelho + (gamepad1.right_stick_x * 1.8);
            VermelhoFD = FVermelho - (gamepad1.right_stick_x * 1.8);
        } else {
            AzulFE = FAzul - gamepad1.right_stick_x;
            AzulTD = FAzul + gamepad1.right_stick_x;
            VermelhoTE = FVermelho - gamepad1.right_stick_x;
            VermelhoFD = FVermelho + gamepad1.right_stick_x;
        }

        fd.setPower(VermelhoFD);
        te.setPower(VermelhoTE);
        fe.setPower(AzulFE);
        td.setPower(AzulTD);

        // ----- INTAKE -----
        if (gamepad1.right_trigger > 0.1) {
            intake1.setPower(0.25);
            intake2.setPower(0.25);
        } else {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        // ----- LANÇADOR -----
        if (gamepad1.left_trigger > 0.1) {
            launcher.setPower(1.0);
        } else {
            launcher.setPower(0);
        }
    }
}
