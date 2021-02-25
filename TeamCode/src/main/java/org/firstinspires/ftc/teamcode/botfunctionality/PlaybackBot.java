package org.firstinspires.ftc.teamcode.botfunctionality;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.reflect.Type;
import java.util.Map;
import java.util.TreeMap;

public abstract class PlaybackBot extends HolonomicDriveBot {

    TreeMap<Long, String> timeMap;
    Gson gson;

    public PlaybackBot() {
    }

    public void initData(String jsonstr) {
        gson = new Gson();
        RobotLog.vv("jsonstr", jsonstr);
        Type type = new TypeToken<TreeMap<Long, String>>() {
        }.getType();
        timeMap = (TreeMap<Long, String>) gson.fromJson(jsonstr, type);
        RobotLog.vv("map", timeMap.toString());

    }

    public void initPlaybackBot(String json) {
        initDriveOp();
        initData(json);
    }

    public void executePlaybackLogic() {
        long initTime = System.currentTimeMillis();
        RobotLog.vv("map", timeMap.toString());

        for (Map.Entry<Long, String> mapEntry : timeMap.entrySet()) {
            // Iterating every set of entry in the TreeMap
            while (mapEntry.getKey() > System.currentTimeMillis() - initTime && opModeIsActive()) {
                RobotLog.vv("key", String.valueOf(mapEntry.getKey()));
                RobotLog.vv("time", String.valueOf(System.currentTimeMillis() - initTime));
                try {
                    Thread.sleep(2);
                } catch (Exception e) {
                }
            }
            if(!opModeIsActive()) break;

            RobotLog.vv("playback", mapEntry.getValue());
            String doubles[] = mapEntry.getValue().split(",");
            runLeftMotorVel(Double.valueOf(doubles[0]), AngleUnit.RADIANS);
            runRightMotorVel(Double.valueOf(doubles[1]), AngleUnit.RADIANS);
        }


    }


}
