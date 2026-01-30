package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.*;

import java.util.List;

public class CameraSubsystem {

    DcMotor cameraMotor;
    Limelight3A limelight;
    VoltageSensor battery;

    final int TARGET_ID = 20;

    final double TICKS_POR_GRAU = 28.68;
    final double kP = 0.015;
    final double kD = 0.004;
    final double DEAD_DEG = 0.4;
    final double MAX_POWER = 0.6;

    double anguloCamera = 0;
    double anguloGol = 0;
    boolean golConhecido = false;
    double ultimoErro = 0;

    public CameraSubsystem(HardwareMap hw, VoltageSensor battery) {
        this.battery = battery;

        cameraMotor = hw.get(DcMotor.class, "cameraMotor");
        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            limelight = hw.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }

    private double compensar(double p) {
        double fator = 13.0 / battery.getVoltage();
        return Math.max(-1, Math.min(1, p * fator));
    }

    public void update() {

        // ODOMETRIA
        anguloCamera = cameraMotor.getCurrentPosition() / TICKS_POR_GRAU;

        // VISÃO
        boolean viuTag = false;
        double txDegrees = 0;

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

        // MEMÓRIA DO GOL
        if (viuTag) {
            anguloGol = anguloCamera + txDegrees;
            golConhecido = true;
        }

        // CONTROLE PID
        double giroCamera;

        if (golConhecido) {
            double erro = anguloGol - anguloCamera;
            double derivada = erro - ultimoErro;
            ultimoErro = erro;

            giroCamera = (erro * kP) + (derivada * kD);

            if (Math.abs(erro) < DEAD_DEG) {
                giroCamera = 0;
            }
        } else {
            // varredura inicial
            giroCamera = 0.12;
        }

        giroCamera = Math.max(-MAX_POWER, Math.min(MAX_POWER, giroCamera));
        cameraMotor.setPower(compensar(giroCamera));
    }

    // Telemetria opcional
    public double getAnguloCamera() { return anguloCamera; }
    public double getAnguloGol() { return anguloGol; }
    public boolean isGolConhecido() { return golConhecido; }
}
