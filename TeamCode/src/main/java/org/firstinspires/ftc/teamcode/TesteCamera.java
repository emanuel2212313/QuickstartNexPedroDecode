package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class TesteCamera extends OpMode {

    DcMotor cameraMotor;
    Limelight3A limelight;

    final int TARGET_ID = 21;

    final double kP = 0.030;
    final double kD = 0.003;
    final double DEAD_DEG = 0.4;
    final double MAX_POWER = 0.6;

    double ultimoErro = 0;
    double ultimoGiroCamera = 0;
    double tempoUltimaTag = 0;

    @Override
    public void init() {
        cameraMotor = hardwareMap.get(DcMotor.class, "cameraMotor");
        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }

    @Override
    public void loop() {

        double txDegrees = 0;
        boolean viuTag = false;

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        if (fr.getFiducialId() == TARGET_ID) {
                            txDegrees = fr.getTargetXDegrees();
                            viuTag = true;
                            break;
                        }
                    }
                }
            }
        }

        if (viuTag) {
            double erro = txDegrees;
            double derivada = erro - ultimoErro;

            double giroCamera = (erro * kP) + (derivada * kD);

            if (Math.abs(erro) < DEAD_DEG) giroCamera = 0;

            giroCamera = Math.max(-MAX_POWER, Math.min(MAX_POWER, giroCamera));

            cameraMotor.setPower(giroCamera);

            ultimoErro = erro;
            ultimoGiroCamera = giroCamera;
            tempoUltimaTag = getRuntime();

        } else {
            if (getRuntime() - tempoUltimaTag < 0.25) {
                cameraMotor.setPower(ultimoGiroCamera * 0.6);
            } else {
                cameraMotor.setPower(0);
                ultimoErro = 0;
            }
        }

        telemetry.addData("Tag Vista", viuTag);
        telemetry.addData("tx", txDegrees);
        telemetry.update();
    }
}
