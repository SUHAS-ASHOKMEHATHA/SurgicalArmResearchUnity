using System;
using UnityEngine;

[Serializable]
public sealed class VlmGroundTruthSnapshot
{
    public string timestampUtc;
    public float simulationTime;
    public string scene;
    public string anatomicalTarget;
    public string operationSite;
    public string activeTool;
    public string procedureStage;
    public string operationStatus;
    public string contactState;
    public string controlMode;
    public string difficulty;
    public string anatomyMotion;
    public string targetHealth;
    public string cameraView;
    public bool procedureRunning;
    public bool procedurePaused;
    public bool emergencyStopActive;
    public bool operationPassed;
    public float procedureElapsed;
    public float procedureDuration;
    public float targetError;
    public float surfaceClearance;
    public float nearestObstacleDistance;
    public string nearestObstacleName;
    public Vector3 toolTipPosition;
    public Vector3 targetPosition;
    public float[] jointAngles;

    public string StateSignature()
    {
        return string.Join(
            "|",
            anatomicalTarget,
            operationSite,
            activeTool,
            procedureStage,
            contactState,
            controlMode,
            cameraView,
            procedureRunning,
            procedurePaused,
            emergencyStopActive,
            operationPassed);
    }
}

[Serializable]
public sealed class VlmInferenceRequest
{
    public string frameId;
    public string imageBase64;
    public string prompt;
}

[Serializable]
public sealed class VlmPrediction
{
    public string frameId;
    public string model;
    public string target;
    public string tool;
    public string stage;
    public string contactState;
    public string safetyConcern;
    public float confidence;
    public float latencyMs;
    public bool validJson;
    public string rawResponse;
}

[Serializable]
public sealed class VlmEvaluationRecord
{
    public string frameId;
    public string timestampUtc;
    public string imagePath;
    public string groundTruthPath;
    public string predictionPath;
    public string expectedTarget;
    public string predictedTarget;
    public string expectedTool;
    public string predictedTool;
    public string expectedStage;
    public string predictedStage;
    public string expectedContactState;
    public string predictedContactState;
    public bool targetCorrect;
    public bool toolCorrect;
    public bool stageCorrect;
    public bool contactCorrect;
    public bool validJson;
    public bool hallucinationDetected;
    public float confidence;
    public float latencyMs;
}