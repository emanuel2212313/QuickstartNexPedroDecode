package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    DcMotor intake;
    DcMotor intake2;
    DistanceSensor sensor;

    boolean manual = false;
    boolean travado = false;

    public IntakeSubsystem(DcMotor intake, DcMotor intake2, DistanceSensor sensor) {
        this.intake = intake;
        this.intake2 = intake2;
        this.sensor = sensor;
    }

    public void alternarManual() {
        manual = !manual;
    }

    public double getDistancia() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public void update(boolean launcherAtivo, boolean podeAlimentar, boolean ignorarSensor) {

        double distancia = getDistancia();

        if (!ignorarSensor) {
            if (distancia < 8) travado = true;
        } else travado = false;

        double power;

        if (launcherAtivo)
            power = podeAlimentar ? 0.7 : 0;
        else
            power = manual ? 0.7 : 0;

        intake.setPower(power);

        if (!travado)
            intake2.setPower(power);
        else
            intake2.setPower(0);
    }
}
