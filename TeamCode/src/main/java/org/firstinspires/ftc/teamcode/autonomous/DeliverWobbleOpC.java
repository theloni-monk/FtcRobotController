package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.botfunctionality.PlaybackBot;

@Autonomous(name = "Wobble Delivery C", group = "Auto")
public class DeliverWobbleOpC extends PlaybackBot {
    final String fileName = "C";
    @Override
    public void runOpMode(){
        boolean runOnce = true;
        initPlaybackBot(fileName);
        waitForStart();
        while(opModeIsActive() && runOnce){
            executeDeliveryScript();
            runOnce = false;
        }
    }

    //TODO: make this a lamda so it can be run with other scripts
    public void executeDeliveryScript(){
        executePlaybackLogic();
    }
}
