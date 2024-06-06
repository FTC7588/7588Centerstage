package org.firstinspires.ftc.teamcode.opmodes.auto.state;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;

public class LogicNode {
    public ArrayList <AutoIf> conditionsList;

    private String name;

    public LogicNode(String name){
        this.name = name;
        conditionsList = new ArrayList<>();
    }

    public interface AutoCondition{
        boolean autoCondition();
    }

    private class AutoIf {
        AutoCondition condition;
        Runnable preTransitionMethod;
        LogicNode transitionNode;

        public AutoIf(AutoCondition condition, Runnable preTransition, LogicNode transitionNode){
            this.condition = condition;
            this.preTransitionMethod = preTransition;
            this.transitionNode = transitionNode;
        }
    }

    public void addCondition(AutoCondition condition, Runnable preTransitionMethod, LogicNode transitionNode){
        conditionsList.add(new AutoIf(condition, preTransitionMethod, transitionNode));
    }

    public void addPositionCondition(DrivetrainSubsystem drive, double tolerance, Pose2d nextPose, LogicNode nextNode){
        conditionsList.add(new AutoIf(
                ()->drive.reachedTarget(tolerance),
                ()->drive.setTargetPose(nextPose),
                nextNode
        ));
    }

    public void addPositionCondition(DrivetrainSubsystem drive, double tolerance, Pose2d nextPose, Runnable preTransitionMethod ,LogicNode nextNode){
        conditionsList.add(new AutoIf(
                ()->drive.reachedTarget(tolerance),
                ()->{
                    drive.setTargetPose(nextPose);
                    preTransitionMethod.run();
                }
                , nextNode));
    }



    public void run(){
        for(AutoIf autoIf:conditionsList){
            if(autoIf.condition.autoCondition()){
                autoIf.preTransitionMethod.run();
                transition(autoIf.transitionNode);
                break;
            }
        }
    }

    private void transition(LogicNode transitionNode){
        conditionsList = transitionNode.conditionsList;
        name = transitionNode.name;
    }

    @Override
    public String toString(){
        return name;
    }

}