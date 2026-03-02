package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
@Autonomous(name = "Autonomo")
public class Adkakld extends NextFTCOpMode {


    private final Pose firstpose = new Pose(21.500, 124.500, Math.toRadians(143));
    private final Pose secondpose = new Pose(51.500, 91.000, Math.toRadians(143));
    private final Pose thirdpose = new Pose(46.00,84.00,Math.toRadians(180));

    private PathChain firtpath;

    private void buildPaths() {
        firtpath = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(firstpose, secondpose))
                .setLinearHeadingInterpolation(firstpose.getHeading(), secondpose.getHeading())
                .build();

    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(firtpath)
        );
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(firstpose);
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {

        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {

    }
}