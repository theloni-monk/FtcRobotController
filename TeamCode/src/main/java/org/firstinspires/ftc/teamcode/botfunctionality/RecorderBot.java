package org.firstinspires.ftc.teamcode.botfunctionality;

import android.content.Context;
import android.content.ContextWrapper;
import android.util.Base64;

import com.qualcomm.robotcore.util.RobotLog;
import com.thoughtworks.xstream.core.util.Base64Encoder;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.HashMap;

public abstract class RecorderBot extends HolonomicDriveBot{
    ObjectOutputStream objOutStream;
    HashMap<Long, String> outputMap;

    double prevVelL;
    double prevVelR;

    protected long initTime = 0;

    public RecorderBot(){

    }

    public void initMap(){
        outputMap = new HashMap<>();


    }

    public void initRecorderBot(){
        initDriveOp();
        initMap();
    }

    protected void executeRecorderLogic(){
        if(initTime == 0) { initTime = System.currentTimeMillis();}
        if(prevVelL != this.leftAngVel || prevVelR != this.rightAngVel){
            outputMap.put(System.currentTimeMillis() - initTime, this.leftAngVel+","+ this.rightAngVel);
        }
    }

    protected void saveFile(){
        try{
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            objOutStream = new ObjectOutputStream(bos);
            objOutStream.writeObject(outputMap);
            objOutStream.flush();
            objOutStream.close();

            byte[] data = bos.toByteArray();
            Base64Encoder encoder = new Base64Encoder();
            String bs64 = encoder.encode(data);
            RobotLog.vv("Map",outputMap.toString());
            RobotLog.vv("s", "\n\n\n");
            RobotLog.vv("Wobble", bs64);
            RobotLog.vv("s","\n\n\n");
        }catch(IOException e){

        }
    }

}
