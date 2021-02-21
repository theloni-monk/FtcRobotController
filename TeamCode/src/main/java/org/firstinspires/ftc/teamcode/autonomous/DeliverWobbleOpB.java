package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.botfunctionality.PlaybackBot;

@Autonomous(name = "Wobble Delivery B", group = "Auto")
public class DeliverWobbleOpB extends PlaybackBot {
    final String fileName = "B";
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
