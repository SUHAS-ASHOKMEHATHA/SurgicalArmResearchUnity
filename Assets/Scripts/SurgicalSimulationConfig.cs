using System;
using UnityEngine;

public enum ProcedureDifficulty
{
    Beginner,
    Intermediate,
    Advanced
}

public enum SurgicalControlMode
{
    Automatic,
    Manual,
    SharedControl
}

public enum AnatomyMotionMode
{
    Normal,
    BreathingAndHeartbeat
}

public enum TargetHealthState
{
    Healthy,
    SimulatedAbnormality
}

public enum SurgicalCameraView
{
    Isometric,
    Top,
    Side,
    Endoscope
}

public enum OptimizationMethod
{
    GeneticAlgorithm,
    RandomSearch,
    NeuralPredictionSimulation
}

[Serializable]
public sealed class SurgicalSimulationConfig
{
    public ProcedureDifficulty Difficulty = ProcedureDifficulty.Intermediate;
    public SurgicalControlMode ControlMode = SurgicalControlMode.Automatic;
    public AnatomyMotionMode AnatomyMotion = AnatomyMotionMode.Normal;
    public TargetHealthState TargetHealth = TargetHealthState.Healthy;
    public SurgicalCameraView CameraView = SurgicalCameraView.Isometric;
    public OptimizationMethod Optimizer = OptimizationMethod.GeneticAlgorithm;

    public float OperationDuration = 40f;
    public float ArmSpeed = 1f;
    public float Acceleration = 1f;
    public float Smoothness = 0.85f;
    public float ContactTolerance = 0.12f;
    public float SafetyDistance = 0.65f;
    public float TargetSizeScale = 1f;
    public Vector3 TargetOffset = Vector3.zero;
    public float MaximumToolDeviation = 0.12f;
    public float CollisionSensitivity = 1f;
    public float PseudoHapticWarningDistance = 0.65f;
    public float SharedControlAuthority = 0.7f;

    public float AccuracyWeight = 1f;
    public float WorkspaceWeight = 0.12f;
    public float ClearanceWeight = 0.2f;
    public float ManipulabilityWeight = 0.18f;

    public bool SingularityAvoidance = true;
    public bool SelfCollisionAvoidance = true;
    public bool PatientCollisionAvoidance = true;
    public bool RemoteCenterConstraint = true;
    public bool AutomaticRecovery = true;
    public bool ShowTrajectoryTrail = true;
    public bool ShowSafetyZone = true;
    public bool ShowAnatomyOverlay = true;
    public bool WarningSound = true;
    public bool VisionTrackingSimulation = true;
    public bool AdaptiveControlSimulation = true;

    public readonly float[] JointMinimumDegrees = { -90f, -90f, -90f, -90f, -90f, -90f };
    public readonly float[] JointMaximumDegrees = { 90f, 90f, 90f, 90f, 90f, 90f };

    public void ApplyDifficultyPreset(ProcedureDifficulty difficulty)
    {
        Difficulty = difficulty;
        switch (difficulty)
        {
            case ProcedureDifficulty.Beginner:
                ArmSpeed = 0.75f;
                Smoothness = 1f;
                ContactTolerance = 0.15f;
                SafetyDistance = 0.75f;
                MaximumToolDeviation = 0.15f;
                CollisionSensitivity = 1.25f;
                break;
            case ProcedureDifficulty.Advanced:
                ArmSpeed = 1.2f;
                Smoothness = 0.7f;
                ContactTolerance = 0.08f;
                SafetyDistance = 0.5f;
                MaximumToolDeviation = 0.08f;
                CollisionSensitivity = 1.5f;
                break;
            default:
                ArmSpeed = 1f;
                Smoothness = 0.85f;
                ContactTolerance = 0.12f;
                SafetyDistance = 0.65f;
                MaximumToolDeviation = 0.12f;
                CollisionSensitivity = 1f;
                break;
        }

        PseudoHapticWarningDistance = SafetyDistance;
        Validate();
    }

    public void Validate()
    {
        OperationDuration = Mathf.Clamp(OperationDuration, 20f, 120f);
        ArmSpeed = Mathf.Clamp(ArmSpeed, 0.25f, 2f);
        Acceleration = Mathf.Clamp(Acceleration, 0.25f, 2f);
        Smoothness = Mathf.Clamp01(Smoothness);
        ContactTolerance = Mathf.Clamp(ContactTolerance, 0.03f, 0.25f);
        SafetyDistance = Mathf.Clamp(SafetyDistance, ContactTolerance + 0.05f, 1.5f);
        TargetSizeScale = Mathf.Clamp(TargetSizeScale, 0.5f, 2f);
        MaximumToolDeviation = Mathf.Clamp(MaximumToolDeviation, 0.03f, 0.3f);
        CollisionSensitivity = Mathf.Clamp(CollisionSensitivity, 0.5f, 2f);
        PseudoHapticWarningDistance = Mathf.Clamp(
            PseudoHapticWarningDistance,
            ContactTolerance + 0.05f,
            1.5f);
        SharedControlAuthority = Mathf.Clamp01(SharedControlAuthority);
        AccuracyWeight = Mathf.Max(0.01f, AccuracyWeight);
        WorkspaceWeight = Mathf.Max(0f, WorkspaceWeight);
        ClearanceWeight = Mathf.Max(0f, ClearanceWeight);
        ManipulabilityWeight = Mathf.Max(0f, ManipulabilityWeight);

        if (JointMinimumDegrees.Length != 6 || JointMaximumDegrees.Length != 6)
        {
            throw new InvalidOperationException("Exactly six joint limits are required.");
        }

        for (int index = 0; index < JointMinimumDegrees.Length; index++)
        {
            JointMinimumDegrees[index] = Mathf.Clamp(JointMinimumDegrees[index], -180f, 180f);
            JointMaximumDegrees[index] = Mathf.Clamp(
                JointMaximumDegrees[index],
                JointMinimumDegrees[index],
                180f);
        }
    }
}