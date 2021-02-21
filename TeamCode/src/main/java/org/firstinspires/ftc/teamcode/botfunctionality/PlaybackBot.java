package org.firstinspires.ftc.teamcode.botfunctionality;

import com.qualcomm.robotcore.util.RobotLog;
import com.thoughtworks.xstream.core.util.Base64Encoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public abstract class PlaybackBot extends HolonomicDriveBot {

    ObjectInputStream objInStream;
    HashMap<Long, String> timeMap;

    public PlaybackBot(){}

    public void initData(String bs64){
        try{
            Base64Encoder decoder = new Base64Encoder();
            byte [] data = decoder.decode(bs64);
            ByteArrayInputStream bis = new ByteArrayInputStream(data);
            objInStream = new ObjectInputStream(bis);
            String datastring = new String(data);
            RobotLog.vv("bin data", datastring);
            timeMap = (HashMap<Long, String>) objInStream.readObject();
            RobotLog.vv("map",timeMap.toString());
            objInStream.close();
        } catch (IOException | ClassNotFoundException e){
            //TODO: error handling
        }
    }

    public void initPlaybackBot(String name){
        initDriveOp();
        initData(name);
    }

    public void executePlaybackLogic(){
        long initTime = System.currentTimeMillis();
        RobotLog.vv("map",timeMap.toString());
        // Iterator
        Iterator<Map.Entry<Long, String>> mapIterator
                = timeMap.entrySet().iterator();

        try {
            // Iterating every set of entry in the HashMap
            while (mapIterator.hasNext() && opModeIsActive()) {
                Map.Entry<Long, String> mapEntry = mapIterator.next();
                while (mapEntry.getKey() < System.currentTimeMillis() - initTime) {
                    Thread.sleep(3);
                }
                RobotLog.vv("playback", mapEntry.getValue().toString());
                String doubles[] = mapEntry.getValue().split(",");
                runLeftMotorVel(Double.valueOf(doubles[0]), AngleUnit.RADIANS);
                runRightMotorVel(Double.valueOf(doubles[0]), AngleUnit.RADIANS);
            }
        } catch(InterruptedException e){}
    }


}
