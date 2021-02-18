package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.botfunctionality.AutoBot;

@Autonomous(name = "Wobble Delivery", group = "Linear Opmode")
public class DeliverWobbleOp extends AutoBot {
    @Override
    public void runOpMode(){
        boolean runOnce = true;
        initAutoBot();
        composeTelemetry();
        waitForStart();
        while(opModeIsActive() && runOnce){
            executeDeliveryScript();
            runOnce = false;
        }
    }

    //TODO: make this a lamda so it can be run with other scripts
    public void executeDeliveryScript(){
        driveDist(1.4, DistanceUnit.METER);
        telemetry.update();
        sleep(5000);

        rotate(90, 0.7);
        telemetry.update();
        sleep(5000);

        rotate(-90,0.7);
        telemetry.update();
        sleep(5000);

        driveDist(-1.4, DistanceUnit.METER);
        telemetry.update();
    }
}
