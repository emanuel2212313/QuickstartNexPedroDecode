package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@TeleOp
public class TesteCamOnly extends OpMode {

    // ===== MOTOR DA CÂMERA =====
    DcMotor cameraMotor;

    // ===== LIMELIGHT =====
    Limelight3A limelight;
    final int TARGET_ID = 21;

    // ===== goBILDA 19.2:1 =====
    final double TICKS_PER_REV = 537.7;
    final double FOV_DEG = 54.0; // FOV horizontal Limelight
    final double TICKS_PER_DEG = TICKS_PER_REV / FOV_DEG;

    // ===== MEMÓRIA DO GOL =====
    double targetTicks = 0;
    boolean temMemoria = false;

    // ===== PID DA CÂMERA =====
    final double kP = 0.008;
    final double kD = 0.0004;
    final double DEAD_TICKS = 10;
    final double MAX_POWER = 0.5;

    double ultimoErro = 0;

    // ===== VOLTAGEM =====
    VoltageSensor batteryVoltage;

    @Override
    public void init() {

        cameraMotor = hardwareMap.get(DcMotor.class, "cameraMotor");
        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void loop() {

        // ===== LEITURA DA TAG =====
        boolean viuTag = false;
        double txDeg = 0;

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                if (tags != null) {
                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getFiducialId() == TARGET_ID) {
                            txDeg = fr.getTargetXDegrees();
                            viuTag = true;
                            break;
                        }
                    }
                }
            }
        }

        int currentTicks = cameraMotor.getCurrentPosition();

        // ===== ATUALIZA MEMÓRIA (quando vê a tag) =====
        if (viuTag) {
            double deltaTicks = txDeg * TICKS_PER_DEG;
            targetTicks = currentTicks + deltaTicks;
            temMemoria = true;
        }

        // ===== CONTROLE ODOMÉTRICO =====
        double power = 0;

        if (temMemoria) {
            double erro = targetTicks - currentTicks;
            double derivada = erro - ultimoErro;
            ultimoErro = erro;

            if (Math.abs(erro) > DEAD_TICKS) {
                power = (erro * kP) + (derivada * kD);
            }
        }

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        cameraMotor.setPower(compensar(power));

        // ===== TELEMETRIA =====
        telemetry.addData("Tag Vista", viuTag);
        telemetry.addData("Memoria Ativa", temMemoria);
        telemetry.addData("Target (ticks)", targetTicks);
        telemetry.addData("Atual (ticks)", currentTicks);
        telemetry.update();
    }
}
