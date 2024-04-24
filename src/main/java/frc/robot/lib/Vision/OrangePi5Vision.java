// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.Vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.GD;

/** Add your docs here. */
public class OrangePi5Vision {
    PhotonCamera m_noteCam;
    PhotonCamera m_aprilCam;
    PhotonPipelineResult resultsNote;
    PhotonPipelineResult resultsApril;
    public static double m_noteYaw;
    public static double m_aprilYaw;
    public static double m_aprilArea;
    List<PhotonTrackedTarget> m_aprilTargets;
    PhotonTrackedTarget m_aprilTarget;
    public OrangePi5Vision(){
       
      m_aprilCam = new PhotonCamera("N300");
      m_aprilCam.setPipelineIndex(0);
      m_aprilCam.setDriverMode(false);

       m_noteCam = new PhotonCamera("NexiGo");
       m_noteCam.setPipelineIndex(0);
       m_noteCam.setDriverMode(false);

    }
    public PhotonPipelineResult getNoteResults(){
        return m_noteCam.getLatestResult();
    }
    public boolean hasNoteTargets(){
        return getNoteResults().hasTargets();
    }

    public PhotonPipelineResult getAprilResults(){
        return m_aprilCam.getLatestResult();
    }
    public boolean hasAprilTargets(){
        return getAprilResults().hasTargets();
    }

    /**
     * 
     * @return The Yaw in degrees with Positive right
     */
    private double getCameraNoteYaw(){
        resultsNote = m_noteCam.getLatestResult();
        
        if(resultsNote.hasTargets()){
            return -resultsNote.getBestTarget().getYaw();
        }else {
            return 180.0;
        }
    }

    private double getCameraAprilYaw(){
        resultsApril = m_aprilCam.getLatestResult();

        m_aprilTarget = resultsApril.getBestTarget();
        if(resultsApril.hasTargets()){
            if(GD.G_Alliance == Alliance.Blue && m_aprilTarget.getFiducialId() == 11){
                return -resultsApril.getBestTarget().getYaw();
            }else if(GD.G_Alliance == Alliance.Blue && m_aprilTarget.getFiducialId() == 16){
                m_aprilArea = resultsApril.getBestTarget().getArea();
                return -resultsApril.getBestTarget().getYaw();
            }else {
                return 0;
            }
            
        }else {
            return 180.0;
        }
    }
    public double getNoteYaw(){
        return m_noteYaw;
    }
    public double getAprilYaw(){
        return m_aprilYaw;
    }
    public void findNote(){
        m_noteYaw = getCameraNoteYaw();
    }
    public void findApril(){
        m_aprilYaw = getCameraAprilYaw();
    }
    public double getAprilArea(){
        return m_aprilArea;
    }
}
