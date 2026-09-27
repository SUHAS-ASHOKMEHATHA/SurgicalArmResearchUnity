using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;

public sealed class SurgicalArmResearchController : MonoBehaviour
{
    private const int JointCount = ArmKinematicsModel.JointCount;
    private const int WorkspaceSampleCount = 900;
    private const int PopulationSize = 72;
    private const int GenerationCount = 60;
    private const float MaximumMicroExtension = 0.35f;
    private const float OperationObstaclePenetration = 0.03f;
    private const float WorkspaceVoxelSize = 0.2f;
    private const float SurgicalProcedureDuration = 40f;
    private const float SurgicalApproachDuration = 4.5f;

    private static readonly Vector3[] LocalJointAxes =
    {
        Vector3.up,
        Vector3.right,
        Vector3.right,
        Vector3.forward,
        Vector3.right,
        Vector3.up
    };

    // These are explicit simulation assumptions, not manufacturer limits.
    private static readonly OperationSite[] AvailableOperationSites =
    {
        OperationSite.Head,
        OperationSite.Heart,
        OperationSite.Eye,
        OperationSite.Spine,
        OperationSite.LeftShoulder,
        OperationSite.RightShoulder,
        OperationSite.LeftElbow,
        OperationSite.RightElbow,
        OperationSite.Abdomen,
        OperationSite.LeftHand,
        OperationSite.RightHand,
        OperationSite.LeftLeg,
        OperationSite.RightLeg
    };

    private readonly Collider[] overlapBuffer = new Collider[64];
    private readonly HashSet<Collider> ignoredRobotColliders = new HashSet<Collider>();
    private readonly List<WorkspacePoint> workspacePoints = new List<WorkspacePoint>();
    private readonly Dictionary<OperationSite, OperationPreflight> operationPreflights =
        new Dictionary<OperationSite, OperationPreflight>();

    private RobotMove robotMove;
    private Transform[] joints;
    private Quaternion[] neutralLocalRotations;
    private Transform endEffector;
    private ArmKinematicsModel kinematics;
    private WorkspaceSummary workspaceSummary;
    private OptimizationResult optimizationResult;
    private ParticleSystem workspaceCloud;
    private Material workspaceMaterial;
    private GameObject targetMarker;
    private GameObject optimizedMarker;
    private LineRenderer microStageLine;
    private GameObject feedbackProbe;
    private Coroutine armMotion;
    private Coroutine feedbackProof;
    private Coroutine operationRoutine;
    private SurgicalTrainingMannequin mannequin;
    private SurgicalToolVisualizer surgicalTool;
    private SurgicalFeedbackVisualizer feedbackVisualizer;
    private SurgicalSimulationConfig simulationConfig = new SurgicalSimulationConfig();
    private SurgicalTrajectoryRecorder trajectoryRecorder;
    private SurgicalTrialDatabase trialDatabase;
    private SyntheticVisionTracker visionTracker = new SyntheticVisionTracker();
    private SimulatedNeuralTrajectoryPredictor neuralPredictor =
        new SimulatedNeuralTrajectoryPredictor();
    private SharedControlCoordinator sharedControl = new SharedControlCoordinator();
    private Ros2DigitalTwinSimulation ros2Simulation = new Ros2DigitalTwinSimulation();
    private Camera presentationCamera;
    private Bounds presentationBounds;
    private Vector3 presentationViewDirection;
    private Vector3 isometricViewDirection;
    private FeedbackState feedbackState;
    private OperationSite selectedOperation;
    private PresentationPhase presentationPhase = PresentationPhase.Selection;
    private SetupTab setupTab = SetupTab.Operation;
    private string nearestObstacleName = "none";
    private string operationStatus = "Select an operation to begin.";
    private string surgicalStage = "Waiting";
    private string anatomicalTargetName = "None";
    private string operatorId = "Operator 1";
    private string safetyMessage = "Safety monitoring active";
    private float nearestObstacleDistance = 0.65f;
    private float currentManipulability;
    private float maximumReach;
    private float neutralKinematicsError;
    private float operationTargetError;
    private float operationSurfaceClearance;
    private float activeMicroExtension;
    private float appliedPoseError;
    private float procedureStartTime;
    private float procedureElapsed;
    private float procedureDurationMeasured;
    private float maximumProcedureDeviation;
    private Vector3 activeSurgicalTarget;
    private Transform activeTargetTransform;
    private Collider activeTargetCollider;
    private readonly float[] manualJointAngles = new float[JointCount];
    private bool initialized;
    private bool cloudVisible;
    private bool operationMenuVisible;
    private bool operationProofPassed;
    private bool proofSawCaution;
    private bool proofSawContact;
    private bool procedureRunning;
    private bool procedureTimelinePassed;
    private bool procedurePaused;
    private bool cancelRequested;
    private bool emergencyStopActive;
    private bool replayRunning;
    private bool previousContactState;
    private int surgicalToolChanges;
    private int surgicalMotionCycles;
    private int collisionEvents;
    private float lastTelemetryPublishTime;
    private float randomSearchDistance;
    private float neuralPredictedCorrection;
    private TrialComparison trialComparison = new TrialComparison();
    private Vector2 setupScrollPosition;
    private Vector2 operationScrollPosition;

    public VlmGroundTruthSnapshot CreateVlmGroundTruthSnapshot()
    {
        return new VlmGroundTruthSnapshot
        {
            timestampUtc = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture),
            simulationTime = Time.time,
            scene = SceneManager.GetActiveScene().name,
            anatomicalTarget = anatomicalTargetName,
            operationSite = selectedOperation.ToString(),
            activeTool = surgicalTool != null ? surgicalTool.ActiveToolName : "No tool",
            procedureStage = surgicalStage,
            operationStatus = operationStatus,
            contactState = feedbackState.ToString(),
            controlMode = simulationConfig.ControlMode.ToString(),
            difficulty = simulationConfig.Difficulty.ToString(),
            anatomyMotion = simulationConfig.AnatomyMotion.ToString(),
            targetHealth = simulationConfig.TargetHealth.ToString(),
            cameraView = simulationConfig.CameraView.ToString(),
            procedureRunning = procedureRunning,
            procedurePaused = procedurePaused,
            emergencyStopActive = emergencyStopActive,
            operationPassed = operationProofPassed,
            procedureElapsed = procedureElapsed,
            procedureDuration = simulationConfig.OperationDuration,
            targetError = operationTargetError,
            surfaceClearance = operationSurfaceClearance,
            nearestObstacleDistance = nearestObstacleDistance,
            nearestObstacleName = nearestObstacleName,
            toolTipPosition = initialized ? GetCurrentToolTipPosition() : Vector3.zero,
            targetPosition = activeSurgicalTarget,
            jointAngles = initialized ? GetCurrentJointAngles() : new float[JointCount]
        };
    }
    private string proofReportPath = "pending";
    private Vector2 resultsScrollPosition;

    public void Initialize(RobotMove moveController, Transform toolTransform)
    {
        if (initialized || moveController == null || toolTransform == null)
        {
            return;
        }

        robotMove = moveController;
        endEffector = toolTransform;
        joints = new[]
        {
            robotMove.J1.transform,
            robotMove.J2.transform,
            robotMove.J3.transform,
            robotMove.J4.transform,
            robotMove.J5.transform,
            robotMove.J6.transform
        };

        neutralLocalRotations = new Quaternion[JointCount];
        Vector3[] jointPositions = new Vector3[JointCount];
        Vector3[] jointAxes = new Vector3[JointCount];

        maximumReach = MaximumMicroExtension;
        for (int index = 0; index < JointCount; index++)
        {
            neutralLocalRotations[index] = joints[index].localRotation;
            jointPositions[index] = joints[index].position;
            jointAxes[index] = joints[index].TransformDirection(LocalJointAxes[index]);

            if (index > 0)
            {
                maximumReach += Vector3.Distance(jointPositions[index - 1], jointPositions[index]);
            }

            foreach (Collider robotCollider in joints[index].GetComponentsInChildren<Collider>(true))
            {
                ignoredRobotColliders.Add(robotCollider);
            }
        }

        maximumReach += Vector3.Distance(jointPositions[JointCount - 1], endEffector.position);
        foreach (Collider toolCollider in endEffector.GetComponentsInChildren<Collider>(true))
        {
            ignoredRobotColliders.Add(toolCollider);
        }

        kinematics = new ArmKinematicsModel(jointPositions, jointAxes, endEffector.position);
        simulationConfig.ApplyDifficultyPreset(ProcedureDifficulty.Intermediate);
        initialized = true;

        mannequin = SurgicalTrainingMannequin.Create(
            new Vector3(-5.474f, 1.11f, 20.0969f),
            Quaternion.Euler(0f, -24.176f, 0f));
        Physics.SyncTransforms();
        ConfigureFocusedPresentation();
        surgicalTool = new SurgicalToolVisualizer();
        feedbackVisualizer = new SurgicalFeedbackVisualizer(gameObject);
        trajectoryRecorder = new SurgicalTrajectoryRecorder();
        string resultsDirectory = Path.GetFullPath(Path.Combine(
            Application.dataPath,
            "..",
            "ResearchResults"));
        trialDatabase = new SurgicalTrialDatabase(resultsDirectory);
        trialComparison = trialDatabase.Compare();
        CreateResearchMarkers();
        AnalyseWorkspace();
        RunOperationPreflights();
        ExportProofData();
        StartCoroutine(CaptureSelectionScreenshot());
    }

    private void Update()
    {
        if (!initialized)
        {
            return;
        }

        UpdateLiveMetrics();
        UpdateMicroStageVisual();
        UpdateSurgicalToolVisual();
        UpdateAnatomyAndTracking();
        UpdateResearchTelemetry();
        UpdateSafetyFeedbackVisual();
        if (procedureRunning)
        {
            procedureElapsed = Mathf.Min(
                simulationConfig.OperationDuration,
                procedureElapsed + (procedurePaused ? 0f : Time.deltaTime));
        }
    }

    private void AnalyseWorkspace()
    {
        workspacePoints.Clear();
        System.Random random = new System.Random(20260911);
        HashSet<Vector3Int> baselineVoxels = new HashSet<Vector3Int>();
        HashSet<Vector3Int> macroMicroVoxels = new HashSet<Vector3Int>();
        float maximumManipulability = 0f;

        for (int sampleIndex = 0; sampleIndex < WorkspaceSampleCount; sampleIndex++)
        {
            float[] angles = CreateRandomAngles(random);
            ArmKinematicSample baseline = kinematics.Evaluate(angles);
            ArmKinematicSample extended = kinematics.Evaluate(angles, MaximumMicroExtension);
            float baselineClearance = GetClosestObstacleDistance(baseline.ToolTipPosition, out _);
            float extendedClearance = GetClosestObstacleDistance(extended.ToolTipPosition, out _);

            workspacePoints.Add(new WorkspacePoint(
                baseline.ToolTipPosition,
                baseline.TranslationalManipulability,
                baselineClearance,
                false));
            workspacePoints.Add(new WorkspacePoint(
                extended.ToolTipPosition,
                extended.TranslationalManipulability,
                extendedClearance,
                true));

            baselineVoxels.Add(ToVoxel(baseline.ToolTipPosition));
            macroMicroVoxels.Add(ToVoxel(baseline.ToolTipPosition));
            macroMicroVoxels.Add(ToVoxel(extended.ToolTipPosition));
            maximumManipulability = Mathf.Max(
                maximumManipulability,
                baseline.TranslationalManipulability,
                extended.TranslationalManipulability);
        }

        int singularSamples = 0;
        int collisionRiskSamples = 0;
        float singularThreshold = maximumManipulability * 0.08f;
        foreach (WorkspacePoint point in workspacePoints)
        {
            if (point.Manipulability <= singularThreshold)
            {
                singularSamples++;
            }

            if (point.Clearance <= simulationConfig.ContactTolerance)
            {
                collisionRiskSamples++;
            }
        }

        workspaceSummary = new WorkspaceSummary(
            baselineVoxels.Count,
            macroMicroVoxels.Count,
            maximumManipulability,
            singularSamples,
            collisionRiskSamples);

        CreateWorkspaceCloud();

        ArmKinematicSample neutralSample = kinematics.Evaluate(new float[JointCount]);
        neutralKinematicsError = Vector3.Distance(
            neutralSample.EndEffectorPosition,
            endEffector.position);
        bool neutralCheckPassed = neutralKinematicsError < 0.0001f;
        Debug.Log(
            $"Research proof: neutral FK error={neutralKinematicsError:F6}; " +
            $"workspace baseline={baselineVoxels.Count} voxels; " +
            $"macro-micro={macroMicroVoxels.Count} voxels; " +
            $"self-check={(neutralCheckPassed ? "PASS" : "FAIL")}");
    }

    private void OptimizeTargetPose(
        Vector3 targetPosition,
        Collider allowedContactCollider,
        string targetName)
    {
        if (workspaceSummary == null)
        {
            return;
        }

        targetMarker.transform.position = targetPosition;
        System.Random random = new System.Random(314159 + trialDatabase.Trials.Count);
        Candidate[] population = new Candidate[PopulationSize];

        population[0] = EvaluateCandidate(
            new float[JointCount],
            0f,
            targetPosition,
            allowedContactCollider);
        for (int index = 1; index < population.Length; index++)
        {
            population[index] = EvaluateCandidate(
                CreateRandomAngles(random),
                NextFloat(random, 0f, MaximumMicroExtension),
                targetPosition,
                allowedContactCollider);
        }

        Array.Sort(population, (left, right) => left.Score.CompareTo(right.Score));
        randomSearchDistance = population[0].Distance;
        neuralPredictedCorrection = neuralPredictor.PredictCorrection(
            targetName.Replace(" PREFLIGHT", string.Empty),
            trialDatabase.Trials);

        int generations = simulationConfig.Optimizer == OptimizationMethod.RandomSearch
            ? 0
            : simulationConfig.Optimizer == OptimizationMethod.NeuralPredictionSimulation
                ? GenerationCount / 2
                : GenerationCount;

        for (int generation = 0; generation < generations; generation++)
        {
            Array.Sort(population, (left, right) => left.Score.CompareTo(right.Score));
            Candidate[] nextGeneration = new Candidate[PopulationSize];
            nextGeneration[0] = population[0].Clone();
            nextGeneration[1] = population[1].Clone();

            int parentPoolSize = PopulationSize / 3;
            float generationProgress = generation / Mathf.Max(1f, generations - 1f);
            float mutationRate = Mathf.Lerp(0.35f, 0.08f, generationProgress);
            if (simulationConfig.Optimizer == OptimizationMethod.NeuralPredictionSimulation)
            {
                mutationRate *= Mathf.Lerp(0.7f, 1f, neuralPredictedCorrection / 0.08f);
            }
            for (int index = 2; index < PopulationSize; index++)
            {
                Candidate parentA = population[random.Next(parentPoolSize)];
                Candidate parentB = population[random.Next(parentPoolSize)];
                float[] childAngles = new float[JointCount];

                for (int jointIndex = 0; jointIndex < JointCount; jointIndex++)
                {
                    float value = random.NextDouble() < 0.5
                        ? parentA.Angles[jointIndex]
                        : parentB.Angles[jointIndex];

                    if (random.NextDouble() < mutationRate)
                    {
                        float range = simulationConfig.JointMaximumDegrees[jointIndex]
                            - simulationConfig.JointMinimumDegrees[jointIndex];
                        value += NextFloat(random, -0.1f * range, 0.1f * range);
                    }

                    childAngles[jointIndex] = Mathf.Clamp(
                        value,
                        simulationConfig.JointMinimumDegrees[jointIndex],
                        simulationConfig.JointMaximumDegrees[jointIndex]);
                }

                float extension = random.NextDouble() < 0.5
                    ? parentA.MicroExtension
                    : parentB.MicroExtension;
                if (random.NextDouble() < mutationRate)
                {
                    extension += NextFloat(
                        random,
                        -0.12f * MaximumMicroExtension,
                        0.12f * MaximumMicroExtension);
                }

                nextGeneration[index] = EvaluateCandidate(
                    childAngles,
                    Mathf.Clamp(extension, 0f, MaximumMicroExtension),
                    targetPosition,
                    allowedContactCollider);
            }

            population = nextGeneration;
        }

        Array.Sort(population, (left, right) => left.Score.CompareTo(right.Score));
        Candidate best = population[0];
        float baselineDistance = Vector3.Distance(
            kinematics.Evaluate(new float[JointCount]).ToolTipPosition,
            targetPosition);

        optimizationResult = new OptimizationResult(
            best.Angles,
            best.MicroExtension,
            best.Sample,
            baselineDistance,
            best.Distance,
            best.Clearance);

        optimizedMarker.transform.position = best.Sample.ToolTipPosition;
        bool improved = best.Distance <= baselineDistance;
        bool passed = allowedContactCollider == null
            ? improved
            : best.Distance <= simulationConfig.ContactTolerance;
        Debug.Log(
            $"Research proof: {targetName} optimization baseline distance={baselineDistance:F3}; " +
            $"optimized distance={best.Distance:F3}; clearance={best.Clearance:F3}; " +
            $"micro extension={best.MicroExtension:F3}; result={(passed ? "PASS" : "REVIEW")}");
    }

    private Candidate EvaluateCandidate(
        float[] angles,
        float microExtension,
        Vector3 targetPosition,
        Collider allowedContactCollider)
    {
        ArmKinematicSample sample = kinematics.Evaluate(angles, microExtension);
        float distance = Vector3.Distance(sample.ToolTipPosition, targetPosition);
        float clearance = GetClosestObstacleDistance(
            sample.ToolTipPosition,
            out _,
            allowedContactCollider);
        float normalizedDistance = distance / Mathf.Max(maximumReach, 0.001f);
        float normalizedManipulability = workspaceSummary.MaximumManipulability > 0f
            ? Mathf.Clamp01(sample.TranslationalManipulability / workspaceSummary.MaximumManipulability)
            : 0f;
        float clearancePenalty = 0f;

        if (allowedContactCollider != null && clearance <= OperationObstaclePenetration)
        {
            clearancePenalty = 1f;
        }
        else if (allowedContactCollider == null
            && clearance <= simulationConfig.ContactTolerance)
        {
            clearancePenalty = 1f;
        }
        else if (allowedContactCollider == null
            && clearance < simulationConfig.PseudoHapticWarningDistance)
        {
            clearancePenalty = simulationConfig.ClearanceWeight
                * (simulationConfig.PseudoHapticWarningDistance - clearance)
                / simulationConfig.PseudoHapticWarningDistance;
        }

        float footprintPenalty = simulationConfig.WorkspaceWeight
            * (1f - microExtension / MaximumMicroExtension);
        float selfCollisionPenalty = 0f;
        if (simulationConfig.SelfCollisionAvoidance)
        {
            for (int first = 0; first < JointCount - 2; first++)
            {
                for (int second = first + 2; second < JointCount; second++)
                {
                    float separation = Vector3.Distance(
                        sample.JointPositions[first],
                        sample.JointPositions[second]);
                    if (separation < 0.08f)
                    {
                        selfCollisionPenalty += (0.08f - separation) * 5f;
                    }
                }
            }
        }

        float rcmPenalty = 0f;
        if (simulationConfig.RemoteCenterConstraint)
        {
            Vector3 remoteCenter = targetPosition + Vector3.up * 0.2f;
            Vector3 toolLine = sample.ToolDirection.normalized;
            float lineDistance = Vector3.Cross(
                remoteCenter - sample.EndEffectorPosition,
                toolLine).magnitude;
            rcmPenalty = Mathf.Max(0f, lineDistance - 0.08f) * 0.15f;
        }

        float singularityPenalty = simulationConfig.SingularityAvoidance
            ? simulationConfig.ManipulabilityWeight * (1f - normalizedManipulability)
            : 0f;
        float score = simulationConfig.AccuracyWeight * normalizedDistance
            + singularityPenalty
            + clearancePenalty
            + footprintPenalty
            + selfCollisionPenalty
            + rcmPenalty;

        return new Candidate(
            (float[])angles.Clone(),
            microExtension,
            sample,
            distance,
            clearance,
            score);
    }

    private void UpdateLiveMetrics()
    {
        Vector3[] positions = new Vector3[JointCount];
        Vector3[] axes = new Vector3[JointCount];
        for (int index = 0; index < JointCount; index++)
        {
            positions[index] = joints[index].position;
            axes[index] = joints[index].TransformDirection(LocalJointAxes[index]);
        }

        currentManipulability = ArmKinematicsModel.CalculateTranslationalManipulability(
            positions,
            axes,
            GetCurrentToolTipPosition());
        nearestObstacleDistance = GetClosestObstacleDistance(
            GetCurrentToolTipPosition(),
            out nearestObstacleName);

        if (nearestObstacleDistance <= simulationConfig.ContactTolerance)
        {
            feedbackState = FeedbackState.Contact;
            proofSawContact = true;
        }
        else if (nearestObstacleDistance < simulationConfig.PseudoHapticWarningDistance)
        {
            feedbackState = FeedbackState.Caution;
            proofSawCaution = true;
        }
        else
        {
            feedbackState = FeedbackState.Clear;
        }
    }

    private float GetClosestObstacleDistance(
        Vector3 position,
        out string obstacleName,
        Collider ignoredCollider = null)
    {
        obstacleName = "none";
        float minimumDistance = simulationConfig.PseudoHapticWarningDistance;
        int count = Physics.OverlapSphereNonAlloc(
            position,
            simulationConfig.PseudoHapticWarningDistance,
            overlapBuffer,
            ~0,
            QueryTriggerInteraction.Ignore);

        for (int index = 0; index < count; index++)
        {
            Collider candidate = overlapBuffer[index];
            if (candidate == null
                || candidate == ignoredCollider
                || ignoredRobotColliders.Contains(candidate))
            {
                continue;
            }

            bool belongsToRobot = false;
            for (int jointIndex = 0; jointIndex < JointCount; jointIndex++)
            {
                if (candidate.transform == joints[jointIndex]
                    || candidate.transform.IsChildOf(joints[jointIndex]))
                {
                    belongsToRobot = true;
                    break;
                }
            }

            if (belongsToRobot)
            {
                continue;
            }

            Vector3 closestPoint;
            MeshCollider meshCollider = candidate as MeshCollider;
            if (meshCollider != null && !meshCollider.convex)
            {
                closestPoint = candidate.bounds.ClosestPoint(position);
            }
            else
            {
                closestPoint = candidate.ClosestPoint(position);
            }

            float distance = Vector3.Distance(position, closestPoint);
            if (distance < minimumDistance)
            {
                minimumDistance = distance;
                obstacleName = candidate.gameObject.name;
            }
        }

        return minimumDistance;
    }

    private void CreateWorkspaceCloud()
    {
        if (workspaceCloud != null)
        {
            Destroy(workspaceCloud.gameObject);
        }

        GameObject cloudObject = new GameObject("Research Workspace Cloud");
        workspaceCloud = cloudObject.AddComponent<ParticleSystem>();
        ParticleSystem.MainModule main = workspaceCloud.main;
        main.loop = false;
        main.playOnAwake = false;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.maxParticles = workspacePoints.Count;
        main.startLifetime = float.MaxValue;
        main.startSpeed = 0f;
        main.startSize = 0.045f;

        ParticleSystem.EmissionModule emission = workspaceCloud.emission;
        emission.enabled = false;
        ParticleSystemRenderer particleRenderer = workspaceCloud.GetComponent<ParticleSystemRenderer>();
        Shader shader = Shader.Find("Universal Render Pipeline/Particles/Unlit");
        if (shader == null)
        {
            shader = Shader.Find("Sprites/Default");
        }

        if (shader != null)
        {
            workspaceMaterial = new Material(shader);
            particleRenderer.material = workspaceMaterial;
        }

        ParticleSystem.Particle[] particles = new ParticleSystem.Particle[workspacePoints.Count];
        float maximumManipulability = Mathf.Max(workspaceSummary.MaximumManipulability, 0.000001f);
        for (int index = 0; index < workspacePoints.Count; index++)
        {
            WorkspacePoint point = workspacePoints[index];
            float normalizedManipulability = Mathf.Clamp01(point.Manipulability / maximumManipulability);
            Color color;

            if (point.Clearance <= simulationConfig.ContactTolerance)
            {
                color = new Color(0.95f, 0.15f, 0.1f, 0.85f);
            }
            else if (normalizedManipulability < 0.08f)
            {
                color = new Color(1f, 0.7f, 0.05f, 0.75f);
            }
            else if (point.IsMicroExtended)
            {
                color = new Color(0.1f, 0.75f, 1f, 0.45f);
            }
            else
            {
                color = Color.Lerp(
                    new Color(0.15f, 0.8f, 0.3f, 0.55f),
                    new Color(0.1f, 1f, 0.75f, 0.8f),
                    normalizedManipulability);
            }

            particles[index].position = point.Position;
            particles[index].startColor = color;
            particles[index].startSize = point.IsMicroExtended ? 0.035f : 0.05f;
            particles[index].startLifetime = float.MaxValue;
            particles[index].remainingLifetime = float.MaxValue;
        }

        workspaceCloud.SetParticles(particles, particles.Length);
        workspaceCloud.gameObject.SetActive(cloudVisible);
    }

    private void CreateResearchMarkers()
    {
        targetMarker = CreateMarker(
            "Research Target",
            new Color(0.1f, 0.9f, 1f, 1f),
            0.16f);
        optimizedMarker = CreateMarker(
            "Optimized Tool Position",
            new Color(0.15f, 1f, 0.35f, 1f),
            0.1f);

        GameObject lineObject = new GameObject("Virtual Micro Stage");
        microStageLine = lineObject.AddComponent<LineRenderer>();
        microStageLine.positionCount = 2;
        microStageLine.startWidth = 0.04f;
        microStageLine.endWidth = 0.025f;
        microStageLine.startColor = new Color(0.1f, 0.9f, 1f, 1f);
        microStageLine.endColor = new Color(0.15f, 1f, 0.35f, 1f);
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null)
        {
            shader = Shader.Find("Sprites/Default");
        }

        if (shader != null)
        {
            microStageLine.material = new Material(shader);
        }
        targetMarker.SetActive(false);
        optimizedMarker.SetActive(false);
        microStageLine.enabled = false;
    }

    private void ConfigureFocusedPresentation()
    {
        Transform robotVisualRoot = joints[0];
        while (robotVisualRoot.parent != null && robotVisualRoot.name != "robot")
        {
            robotVisualRoot = robotVisualRoot.parent;
        }

        HashSet<Renderer> focusedRenderers = new HashSet<Renderer>();
        foreach (Renderer robotRenderer in robotVisualRoot.GetComponentsInChildren<Renderer>(true))
        {
            focusedRenderers.Add(robotRenderer);
        }
        foreach (Renderer mannequinRenderer in mannequin.GetComponentsInChildren<Renderer>(true))
        {
            focusedRenderers.Add(mannequinRenderer);
        }

        foreach (Renderer sceneRenderer in FindObjectsByType<Renderer>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None))
        {
            sceneRenderer.enabled = focusedRenderers.Contains(sceneRenderer);
        }

        foreach (Canvas legacyCanvas in FindObjectsByType<Canvas>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None))
        {
            legacyCanvas.enabled = false;
        }

        presentationCamera = FindFirstObjectByType<Camera>();
        if (presentationCamera == null || focusedRenderers.Count == 0)
        {
            return;
        }

        bool boundsInitialized = false;
        presentationBounds = new Bounds();
        foreach (Renderer focusedRenderer in focusedRenderers)
        {
            if (!focusedRenderer.enabled)
            {
                continue;
            }

            if (!boundsInitialized)
            {
                presentationBounds = focusedRenderer.bounds;
                boundsInitialized = true;
            }
            else
            {
                presentationBounds.Encapsulate(focusedRenderer.bounds);
            }
        }

        if (!boundsInitialized)
        {
            return;
        }

        presentationViewDirection = (
            presentationCamera.transform.position - presentationBounds.center).normalized;
        if (presentationViewDirection.sqrMagnitude < 0.001f)
        {
            presentationViewDirection = new Vector3(0.65f, 0.4f, 1f).normalized;
        }
        isometricViewDirection = presentationViewDirection;

        presentationCamera.nearClipPlane = 0.05f;
        presentationCamera.clearFlags = CameraClearFlags.SolidColor;
        presentationCamera.backgroundColor = new Color(0.035f, 0.045f, 0.055f, 1f);
        FrameFocusedCamera(new Rect(0.4f, 0f, 0.6f, 1f));
        Debug.Log($"Focused presentation: {focusedRenderers.Count} arm/mannequin renderers visible.");
    }

    private void FrameFocusedCamera(Rect viewport)
    {
        if (presentationCamera == null)
        {
            return;
        }

        presentationCamera.rect = viewport;
        float viewportAspect = Mathf.Max(
            0.1f,
            Screen.width * viewport.width / Mathf.Max(1f, Screen.height * viewport.height));
        float verticalHalfAngle = presentationCamera.fieldOfView * 0.5f * Mathf.Deg2Rad;
        float horizontalHalfAngle = Mathf.Atan(Mathf.Tan(verticalHalfAngle) * viewportAspect);
        float limitingHalfAngle = Mathf.Min(verticalHalfAngle, horizontalHalfAngle);
        float cameraDistance = presentationBounds.extents.magnitude
            / Mathf.Max(Mathf.Sin(limitingHalfAngle), 0.1f) * 1.12f;

        presentationCamera.transform.position = presentationBounds.center
            + presentationViewDirection * cameraDistance;
        presentationCamera.transform.LookAt(presentationBounds.center);
    }

    private void ApplyCameraView()
    {
        if (presentationCamera == null)
        {
            return;
        }

        switch (simulationConfig.CameraView)
        {
            case SurgicalCameraView.Top:
                presentationViewDirection = Vector3.up;
                break;
            case SurgicalCameraView.Side:
                presentationViewDirection = Vector3.forward;
                break;
            case SurgicalCameraView.Endoscope:
                Vector3 target = activeTargetTransform != null
                    ? activeTargetTransform.position
                    : mannequin.HeartTargetPosition;
                presentationCamera.rect = new Rect(0f, 0f, 1f, 1f);
                presentationCamera.transform.position = target
                    + new Vector3(0.18f, 0.16f, 0.22f);
                presentationCamera.transform.LookAt(target);
                return;
            default:
                presentationViewDirection = isometricViewDirection;
                break;
        }

        Rect viewport = presentationPhase == PresentationPhase.Selection
            ? new Rect(0.4f, 0f, 0.6f, 1f)
            : new Rect(0f, 0f, 1f, 1f);
        FrameFocusedCamera(viewport);
    }

    private void ApplyConfigurationAndReanalyse()
    {
        simulationConfig.Validate();
        mannequin.ConfigureTargetAppearance(
            null,
            simulationConfig.TargetSizeScale,
            Vector3.zero,
            simulationConfig.TargetHealth,
            simulationConfig.ShowAnatomyOverlay);
        AnalyseWorkspace();
        RunOperationPreflights();
        ExportProofData();
        safetyMessage = "Configuration applied and all targets rechecked";
    }

    private static GameObject CreateMarker(string markerName, Color color, float scale)
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = markerName;
        marker.transform.localScale = Vector3.one * scale;
        Collider markerCollider = marker.GetComponent<Collider>();
        markerCollider.enabled = false;
        Destroy(markerCollider);

        Renderer markerRenderer = marker.GetComponent<Renderer>();
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader != null)
        {
            markerRenderer.material = new Material(shader);
        }
        markerRenderer.material.color = color;
        return marker;
    }

    private void ApplyOptimizedPose()
    {
        if (optimizationResult == null)
        {
            return;
        }

        if (armMotion != null)
        {
            StopCoroutine(armMotion);
        }

        activeMicroExtension = optimizationResult.MicroExtension;
        armMotion = StartCoroutine(AnimateJointPose(optimizationResult.Angles, 2.5f));
    }

    private void RestoreNeutralPose()
    {
        if (armMotion != null)
        {
            StopCoroutine(armMotion);
        }

        activeMicroExtension = 0f;
        armMotion = StartCoroutine(AnimateJointPose(new float[JointCount], 2.5f));
    }

    private IEnumerator AnimateJointPose(float[] targetAngles, float duration)
    {
        Quaternion[] startRotations = new Quaternion[JointCount];
        Quaternion[] targetRotations = new Quaternion[JointCount];
        for (int index = 0; index < JointCount; index++)
        {
            startRotations[index] = joints[index].localRotation;
            targetRotations[index] = neutralLocalRotations[index]
                * Quaternion.AngleAxis(targetAngles[index], LocalJointAxes[index]);
        }

        float elapsed = 0f;
        while (elapsed < duration)
        {
            if (cancelRequested || emergencyStopActive)
            {
                armMotion = null;
                yield break;
            }
            if (procedurePaused)
            {
                yield return null;
                continue;
            }

            elapsed += Time.deltaTime * simulationConfig.Acceleration;
            float linearProgress = Mathf.Clamp01(elapsed / duration);
            float smoothProgress = Mathf.SmoothStep(0f, 1f, linearProgress);
            float progress = Mathf.Lerp(
                linearProgress,
                smoothProgress,
                simulationConfig.Smoothness);
            for (int index = 0; index < JointCount; index++)
            {
                joints[index].localRotation = Quaternion.Slerp(
                    startRotations[index],
                    targetRotations[index],
                    progress);
            }
            yield return null;
        }

        armMotion = null;
        if (optimizationResult != null && targetAngles == optimizationResult.Angles)
        {
            appliedPoseError = Vector3.Distance(
                endEffector.position,
                optimizationResult.Sample.EndEffectorPosition);
            Debug.Log($"Research proof: applied-pose FK error={appliedPoseError:F4} Unity units.");
        }
    }

    private void RunFeedbackProof()
    {
        if (feedbackProof != null)
        {
            StopCoroutine(feedbackProof);
        }

        feedbackProof = StartCoroutine(FeedbackProofSequence());
    }

    private void StartOperation(OperationSite operationSite)
    {
        if (operationRoutine != null)
        {
            return;
        }

        operationMenuVisible = false;
        presentationPhase = PresentationPhase.Operating;
        FrameFocusedCamera(new Rect(0.4f, 0f, 0.6f, 1f));
        microStageLine.enabled = true;
        OperationProfile profile = GetOperationProfile(operationSite);
        anatomicalTargetName = profile.AnatomicalName;
        activeTargetTransform = profile.TargetTransform;
        activeTargetCollider = profile.TargetCollider;
        mannequin.ConfigureTargetAppearance(
            activeTargetTransform,
            simulationConfig.TargetSizeScale,
            simulationConfig.TargetOffset,
            simulationConfig.TargetHealth,
            simulationConfig.ShowAnatomyOverlay);
        activeSurgicalTarget = activeTargetTransform.position;
        procedureStartTime = Time.time;
        procedureElapsed = 0f;
        procedureDurationMeasured = 0f;
        maximumProcedureDeviation = 0f;
        surgicalToolChanges = 0;
        surgicalMotionCycles = 0;
        procedureTimelinePassed = false;
        procedureRunning = true;
        procedurePaused = false;
        cancelRequested = false;
        emergencyStopActive = false;
        collisionEvents = 0;
        safetyMessage = "Safety monitoring active";
        feedbackVisualizer.ResetState();
        trajectoryRecorder.Begin(simulationConfig.ShowTrajectoryTrail);
        ChangeSurgicalTool(SurgicalToolType.Endoscope);
        operationRoutine = StartCoroutine(ConductOperation(operationSite));
    }

    public void RunHeadOperationDemo()
    {
        StartOperation(OperationSite.Head);
    }

    public void RunLegOperationDemo()
    {
        StartOperation(OperationSite.LeftLeg);
    }

    private float[] GetCommandedOperationAngles()
    {
        float[] commandedAngles = (float[])optimizationResult.Angles.Clone();
        float manualInfluence;
        switch (simulationConfig.ControlMode)
        {
            case SurgicalControlMode.Manual:
                manualInfluence = 1f;
                break;
            case SurgicalControlMode.SharedControl:
                manualInfluence = 1f - simulationConfig.SharedControlAuthority;
                break;
            default:
                manualInfluence = 0f;
                break;
        }

        for (int index = 0; index < commandedAngles.Length; index++)
        {
            commandedAngles[index] = Mathf.Clamp(
                commandedAngles[index] + manualJointAngles[index] * manualInfluence,
                simulationConfig.JointMinimumDegrees[index],
                simulationConfig.JointMaximumDegrees[index]);
        }
        return commandedAngles;
    }

    private IEnumerator ConductOperation(OperationSite operationSite)
    {
        selectedOperation = operationSite;
        operationProofPassed = false;
        proofSawCaution = false;
        proofSawContact = false;
        surgicalStage = "Robotic positioning";
        operationStatus = $"Positioning above the {anatomicalTargetName.ToLowerInvariant()} target...";

        RestoreNeutralPose();
        while (armMotion != null)
        {
            yield return null;
        }

        OperationProfile profile = GetOperationProfile(operationSite);
        Collider targetCollider = profile.TargetCollider;
        Vector3 targetPosition = profile.TargetPosition;
        string targetName = profile.AnatomicalName.ToUpperInvariant();

        operationStatus = $"Planning a safe trajectory to the {targetName.ToLowerInvariant()}...";
        OptimizeTargetPose(targetPosition, targetCollider, targetName);
        yield return null;

        operationStatus = $"Approaching the {targetName.ToLowerInvariant()} with navigation probe...";
        activeMicroExtension = optimizationResult.MicroExtension;
        float[] commandedAngles = GetCommandedOperationAngles();
        armMotion = StartCoroutine(AnimateJointPose(
            commandedAngles,
            SurgicalApproachDuration / simulationConfig.ArmSpeed));
        while (armMotion != null)
        {
            if (cancelRequested)
            {
                yield break;
            }
            yield return null;
        }

        Quaternion[] contactPose = CaptureJointRotations();
        yield return StartCoroutine(RunSurgicalMotionTimeline(contactPose, operationSite));
        RestoreJointRotations(contactPose);
        procedureDurationMeasured = Time.time - procedureStartTime;
        procedureRunning = false;
        trajectoryRecorder.End();
        procedureTimelinePassed = procedureElapsed >= simulationConfig.OperationDuration - 0.05f
            && surgicalToolChanges >= 5
            && surgicalMotionCycles >= 4;
        ChangeSurgicalTool(SurgicalToolType.None);

        Vector3 toolTipPosition = GetCurrentToolTipPosition();
        operationTargetError = Vector3.Distance(toolTipPosition, targetPosition);
        operationSurfaceClearance = Vector3.Distance(
            toolTipPosition,
            targetCollider.ClosestPoint(toolTipPosition));
        operationProofPassed = operationTargetError <= simulationConfig.ContactTolerance
            && operationSurfaceClearance <= simulationConfig.ContactTolerance
            && maximumProcedureDeviation <= simulationConfig.MaximumToolDeviation
            && procedureTimelinePassed;
        operationStatus = operationProofPassed
            ? $"PASS: {anatomicalTargetName} procedure completed within tolerance."
            : $"REVIEW: {anatomicalTargetName} procedure requires validation.";

        Debug.Log(
            $"Operation proof: site={targetName}; target error={operationTargetError:F4}; " +
            $"surface clearance={operationSurfaceClearance:F4}; " +
            $"tolerance={simulationConfig.ContactTolerance:F4}; " +
            $"duration={procedureDurationMeasured:F2}s; tools={surgicalToolChanges}; " +
            $"motion cycles={surgicalMotionCycles}; " +
            $"result={(operationProofPassed ? "PASS" : "FAIL")}");
        SaveTrialRecord();
        ExportProofData();
        presentationPhase = PresentationPhase.Results;
        FrameFocusedCamera(new Rect(0.46f, 0f, 0.54f, 1f));
        yield return new WaitForEndOfFrame();
        string screenshotPath = Path.GetFullPath(Path.Combine(
            Application.dataPath,
            "..",
            "ResearchResults",
            "surgery_result.png"));
        ScreenCapture.CaptureScreenshot(screenshotPath);
        Debug.Log($"Operation result screenshot saved to {screenshotPath}");
        operationRoutine = null;
    }

    private void PauseProcedure()
    {
        if (!procedureRunning || emergencyStopActive)
        {
            return;
        }
        procedurePaused = true;
        safetyMessage = "Procedure paused by operator";
    }

    private void ResumeProcedure()
    {
        if (!procedureRunning || emergencyStopActive)
        {
            return;
        }
        procedurePaused = false;
        safetyMessage = "Safety monitoring active";
    }

    private void CancelProcedure(bool emergency)
    {
        cancelRequested = true;
        emergencyStopActive = emergency;
        procedurePaused = false;
        procedureRunning = false;
        replayRunning = false;
        trajectoryRecorder.End();
        StopAllCoroutines();
        operationRoutine = null;
        armMotion = null;
        feedbackProof = null;
        surgicalTool.SetTool(SurgicalToolType.None);
        RestoreJointRotations(neutralLocalRotations);
        activeMicroExtension = 0f;
        microStageLine.enabled = false;
        operationProofPassed = false;
        safetyMessage = emergency
            ? "EMERGENCY STOP - all motion inhibited"
            : "Procedure cancelled; automatic recovery completed";
        operationStatus = safetyMessage;
        presentationPhase = emergency
            ? PresentationPhase.Emergency
            : PresentationPhase.Selection;
    }

    private void ClearEmergencyStop()
    {
        emergencyStopActive = false;
        cancelRequested = false;
        selectedOperation = OperationSite.None;
        operationStatus = "Select an operation to begin.";
        safetyMessage = "Safety monitoring active";
        presentationPhase = PresentationPhase.Selection;
        FrameFocusedCamera(new Rect(0.4f, 0f, 0.6f, 1f));
    }

    private void ApplyManualPose()
    {
        if (procedureRunning || emergencyStopActive)
        {
            return;
        }

        if (armMotion != null)
        {
            StopCoroutine(armMotion);
        }
        cancelRequested = false;
        armMotion = StartCoroutine(AnimateJointPose(
            (float[])manualJointAngles.Clone(),
            2f / simulationConfig.ArmSpeed));
    }

    private void ReturnHome()
    {
        if (procedureRunning)
        {
            CancelProcedure(false);
            return;
        }

        cancelRequested = false;
        for (int index = 0; index < manualJointAngles.Length; index++)
        {
            manualJointAngles[index] = 0f;
        }
        RestoreNeutralPose();
        safetyMessage = "Arm returning to home pose";
    }

    private void StartSlowMotionReplay()
    {
        if (replayRunning || procedureRunning || trajectoryRecorder.SampleCount < 2)
        {
            return;
        }
        StartCoroutine(ReplayRecordedProcedure());
    }

    private IEnumerator ReplayRecordedProcedure()
    {
        replayRunning = true;
        presentationPhase = PresentationPhase.Operating;
        surgicalStage = "Slow-motion replay";
        operationStatus = "Replaying the recorded surgical trajectory at 0.35x speed.";
        yield return trajectoryRecorder.Replay(joints, surgicalTool.SetTool, 0.35f);
        surgicalTool.SetTool(SurgicalToolType.None);
        replayRunning = false;
        presentationPhase = PresentationPhase.Results;
    }

    private void SaveTrialRecord()
    {
        float accuracyScore = 1f - Mathf.Clamp01(
            operationTargetError / Mathf.Max(simulationConfig.ContactTolerance, 0.001f));
        float precisionScore = 1f - Mathf.Clamp01(
            maximumProcedureDeviation
            / Mathf.Max(simulationConfig.MaximumToolDeviation, 0.001f));
        float safetyScore = Mathf.Clamp01(1f - collisionEvents * 0.12f);
        float completionScore = procedureTimelinePassed ? 1f : 0f;
        float score = 100f * (
            0.4f * accuracyScore
            + 0.25f * precisionScore
            + 0.2f * safetyScore
            + 0.15f * completionScore);

        SurgicalTrialRecord trial = new SurgicalTrialRecord
        {
            TrialId = Guid.NewGuid().ToString("N"),
            TimestampUtc = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture),
            OperatorId = operatorId,
            AnatomicalTarget = anatomicalTargetName,
            Difficulty = simulationConfig.Difficulty.ToString(),
            ControlMode = simulationConfig.ControlMode.ToString(),
            AnatomyMotion = simulationConfig.AnatomyMotion.ToString(),
            TargetHealth = simulationConfig.TargetHealth.ToString(),
            Optimizer = simulationConfig.Optimizer.ToString(),
            Duration = procedureDurationMeasured,
            TargetError = operationTargetError,
            SurfaceClearance = operationSurfaceClearance,
            MaximumDeviation = maximumProcedureDeviation,
            Manipulability = currentManipulability,
            ToolChanges = surgicalToolChanges,
            MotionCycles = surgicalMotionCycles,
            CollisionEvents = collisionEvents,
            Passed = operationProofPassed,
            Score = score
        };
        trialDatabase.Save(trial);
        trialComparison = trialDatabase.Compare();
    }

    private IEnumerator RunSurgicalMotionTimeline(
        Quaternion[] contactPose,
        OperationSite operationSite)
    {
        SurgicalToolType currentTool = SurgicalToolType.Endoscope;
        int previousCycle = -1;

        while (procedureElapsed < simulationConfig.OperationDuration)
        {
            if (cancelRequested)
            {
                yield break;
            }
            if (procedurePaused)
            {
                yield return null;
                continue;
            }

            float elapsed = procedureElapsed;
            float timeScale = simulationConfig.OperationDuration / SurgicalProcedureDuration;
            SurgicalToolType nextTool;
            float phaseStart;
            float phaseFrequency;
            float amplitudeScale;

            if (elapsed < 5f * timeScale)
            {
                nextTool = SurgicalToolType.Endoscope;
                phaseStart = 0f;
                phaseFrequency = 0.3f;
                amplitudeScale = 0.12f;
                surgicalStage = $"{anatomicalTargetName} endoscopic inspection";
            }
            else if (elapsed < 10f * timeScale)
            {
                nextTool = SurgicalToolType.NavigationProbe;
                phaseStart = 5f * timeScale;
                phaseFrequency = 0.45f;
                amplitudeScale = 0.22f;
                surgicalStage = $"{anatomicalTargetName} surface mapping";
            }
            else if (elapsed < 16f * timeScale)
            {
                nextTool = SurgicalToolType.MicroScalpel;
                phaseStart = 10f * timeScale;
                phaseFrequency = 0.75f;
                amplitudeScale = 0.42f;
                surgicalStage = GetAccessStage(operationSite);
            }
            else if (elapsed < 22f * timeScale)
            {
                nextTool = SurgicalToolType.SimulatedCautery;
                phaseStart = 16f * timeScale;
                phaseFrequency = 0.62f;
                amplitudeScale = 0.3f;
                surgicalStage = $"{anatomicalTargetName} simulated cautery pass";
            }
            else if (elapsed < 28f * timeScale)
            {
                nextTool = SurgicalToolType.IrrigationAspirator;
                phaseStart = 22f * timeScale;
                phaseFrequency = 0.55f;
                amplitudeScale = 0.34f;
                surgicalStage = $"{anatomicalTargetName} irrigation and aspiration";
            }
            else if (elapsed < 34f * timeScale)
            {
                nextTool = SurgicalToolType.MicroForceps;
                phaseStart = 28f * timeScale;
                phaseFrequency = 0.68f;
                amplitudeScale = 0.3f;
                surgicalStage = GetManipulationStage(operationSite);
            }
            else if (elapsed < 38f * timeScale)
            {
                nextTool = SurgicalToolType.ClosureInstrument;
                phaseStart = 34f * timeScale;
                phaseFrequency = 0.82f;
                amplitudeScale = 0.26f;
                surgicalStage = $"{anatomicalTargetName} closure gesture";
            }
            else
            {
                nextTool = SurgicalToolType.Endoscope;
                phaseStart = 38f * timeScale;
                phaseFrequency = 0.35f;
                amplitudeScale = 0.12f;
                surgicalStage = $"{anatomicalTargetName} final verification";
            }

            if (nextTool != currentTool)
            {
                ChangeSurgicalTool(nextTool);
                currentTool = nextTool;
            }

            operationStatus = $"{surgicalStage} using {surgicalTool.ActiveToolName}.";
            float phaseTime = elapsed - phaseStart;
            float primaryWave = Mathf.Sin(phaseTime * Mathf.PI * 2f * phaseFrequency);
            float secondaryWave = Mathf.Sin(
                phaseTime * Mathf.PI * 2f * phaseFrequency * 0.5f + Mathf.PI * 0.5f);
            float siteScale = GetOperationProfile(operationSite).MotionScale;
            float adaptiveScale = simulationConfig.AdaptiveControlSimulation
                ? Mathf.Clamp(
                    simulationConfig.MaximumToolDeviation
                    / Mathf.Max(maximumProcedureDeviation, 0.001f),
                    0.25f,
                    1f)
                : 1f;
            float motionScale = siteScale * adaptiveScale;

            joints[3].localRotation = contactPose[3] * Quaternion.AngleAxis(
                primaryWave * amplitudeScale * motionScale,
                LocalJointAxes[3]);
            joints[4].localRotation = contactPose[4] * Quaternion.AngleAxis(
                secondaryWave * amplitudeScale * 0.7f * motionScale,
                LocalJointAxes[4]);
            joints[5].localRotation = contactPose[5] * Quaternion.AngleAxis(
                -primaryWave * amplitudeScale * 1.2f * motionScale,
                LocalJointAxes[5]);

            int cycle = Mathf.FloorToInt(phaseTime * phaseFrequency);
            if (cycle != previousCycle && cycle >= 0)
            {
                surgicalMotionCycles++;
                previousCycle = cycle;
            }

            float deviation = Vector3.Distance(GetCurrentToolTipPosition(), activeSurgicalTarget);
            maximumProcedureDeviation = Mathf.Max(maximumProcedureDeviation, deviation);
            yield return null;
        }

        procedureElapsed = simulationConfig.OperationDuration;
    }

    private Quaternion[] CaptureJointRotations()
    {
        Quaternion[] rotations = new Quaternion[JointCount];
        for (int index = 0; index < JointCount; index++)
        {
            rotations[index] = joints[index].localRotation;
        }
        return rotations;
    }

    private void RestoreJointRotations(Quaternion[] rotations)
    {
        for (int index = 0; index < JointCount; index++)
        {
            joints[index].localRotation = rotations[index];
        }
    }

    private void ChangeSurgicalTool(SurgicalToolType toolType)
    {
        surgicalTool.SetTool(toolType);
        surgicalToolChanges++;
        Debug.Log(
            $"Surgical phase: t={procedureElapsed:F1}s; target={anatomicalTargetName}; " +
            $"tool={surgicalTool.ActiveToolName}");
    }

    private void RunOperationPreflights()
    {
        operationPreflights.Clear();
        foreach (OperationSite site in AvailableOperationSites)
        {
            OperationProfile profile = GetOperationProfile(site);
            mannequin.ConfigureTargetAppearance(
                profile.TargetTransform,
                simulationConfig.TargetSizeScale,
                simulationConfig.TargetOffset,
                simulationConfig.TargetHealth,
                simulationConfig.ShowAnatomyOverlay);
            profile = GetOperationProfile(site);
            OptimizeTargetPose(
                profile.TargetPosition,
                profile.TargetCollider,
                $"{profile.AnatomicalName.ToUpperInvariant()} PREFLIGHT");
            float surfaceClearance = Vector3.Distance(
                optimizationResult.Sample.ToolTipPosition,
                profile.TargetCollider.ClosestPoint(
                    optimizationResult.Sample.ToolTipPosition));
            operationPreflights[site] = new OperationPreflight(
                optimizationResult.OptimizedDistance,
                surfaceClearance,
                optimizationResult.OptimizedDistance <= simulationConfig.ContactTolerance
                    && surfaceClearance <= simulationConfig.ContactTolerance);
        }
                mannequin.ConfigureTargetAppearance(
                    null,
                    simulationConfig.TargetSizeScale,
                    Vector3.zero,
                    simulationConfig.TargetHealth,
                    simulationConfig.ShowAnatomyOverlay);
    }

    private OperationProfile GetOperationProfile(OperationSite operationSite)
    {
        switch (operationSite)
        {
            case OperationSite.Head:
                return new OperationProfile(
                    "Brain",
                    mannequin.HeadCollider,
                        mannequin.BrainTarget,
                    mannequin.HeadTargetPosition,
                    0.72f);
            case OperationSite.Heart:
                return new OperationProfile(
                    "Heart",
                    mannequin.HeartCollider,
                    mannequin.HeartTarget,
                    mannequin.HeartTargetPosition,
                    0.68f);
            case OperationSite.Eye:
                return new OperationProfile(
                    "Eye",
                    mannequin.EyeCollider,
                    mannequin.EyeTarget,
                    mannequin.EyeTargetPosition,
                    0.55f);
            case OperationSite.Spine:
                return new OperationProfile(
                    "Spine",
                    mannequin.SpineCollider,
                    mannequin.SpineTarget,
                    mannequin.SpineTargetPosition,
                    0.65f);
            case OperationSite.LeftShoulder:
                return new OperationProfile(
                    "Left shoulder",
                    mannequin.LeftShoulderCollider,
                    mannequin.LeftShoulderTarget,
                    mannequin.LeftShoulderTargetPosition,
                    0.78f);
            case OperationSite.RightShoulder:
                return new OperationProfile(
                    "Right shoulder",
                    mannequin.RightShoulderCollider,
                    mannequin.RightShoulderTarget,
                    mannequin.RightShoulderTargetPosition,
                    0.78f);
            case OperationSite.LeftElbow:
                return new OperationProfile(
                    "Left elbow",
                    mannequin.LeftElbowCollider,
                    mannequin.LeftElbowTarget,
                    mannequin.LeftElbowTargetPosition,
                    0.8f);
            case OperationSite.RightElbow:
                return new OperationProfile(
                    "Right elbow",
                    mannequin.RightElbowCollider,
                    mannequin.RightElbowTarget,
                    mannequin.RightElbowTargetPosition,
                    0.8f);
            case OperationSite.Abdomen:
                return new OperationProfile(
                    "Abdomen",
                    mannequin.AbdomenCollider,
                    mannequin.AbdomenTarget,
                    mannequin.AbdomenTargetPosition,
                    0.7f);
            case OperationSite.LeftHand:
                return new OperationProfile(
                    "Left hand",
                    mannequin.LeftHandCollider,
                    mannequin.LeftHandTarget,
                    mannequin.LeftHandTargetPosition,
                    0.82f);
            case OperationSite.RightHand:
                return new OperationProfile(
                    "Right hand",
                    mannequin.RightHandCollider,
                    mannequin.RightHandTarget,
                    mannequin.RightHandTargetPosition,
                    0.82f);
            case OperationSite.LeftLeg:
                return new OperationProfile(
                    "Left knee",
                    mannequin.LeftLegCollider,
                    mannequin.LeftKneeTarget,
                    mannequin.LeftLegTargetPosition,
                    1f);
            case OperationSite.RightLeg:
                return new OperationProfile(
                    "Right knee",
                    mannequin.RightLegCollider,
                    mannequin.RightKneeTarget,
                    mannequin.RightLegTargetPosition,
                    1f);
            default:
                throw new ArgumentOutOfRangeException(
                    nameof(operationSite),
                    operationSite,
                    "A valid operation site is required.");
        }
    }

    private string GetAccessStage(OperationSite operationSite)
    {
        switch (operationSite)
        {
            case OperationSite.Head:
                return "Controlled cranial access pass";
            case OperationSite.Heart:
                return "Controlled cardiac access pass";
            case OperationSite.Eye:
                return "Controlled ophthalmic access pass";
            case OperationSite.Spine:
                return "Controlled spinal access pass";
            case OperationSite.LeftShoulder:
            case OperationSite.RightShoulder:
                return "Controlled shoulder access pass";
            case OperationSite.LeftElbow:
            case OperationSite.RightElbow:
                return "Controlled elbow access pass";
            case OperationSite.Abdomen:
                return "Controlled abdominal access pass";
            case OperationSite.LeftHand:
            case OperationSite.RightHand:
                return "Controlled hand access pass";
            default:
                return "Controlled knee access pass";
        }
    }

    private string GetManipulationStage(OperationSite operationSite)
    {
        switch (operationSite)
        {
            case OperationSite.Head:
                return "Brain micro-dissection gesture";
            case OperationSite.Heart:
                return "Cardiac tissue manipulation gesture";
            case OperationSite.Eye:
                return "Ophthalmic microsurgery gesture";
            case OperationSite.Spine:
                return "Spinal alignment gesture";
            case OperationSite.LeftShoulder:
            case OperationSite.RightShoulder:
                return "Shoulder tissue-alignment gesture";
            case OperationSite.LeftElbow:
            case OperationSite.RightElbow:
                return "Elbow tendon-alignment gesture";
            case OperationSite.Abdomen:
                return "Abdominal tissue manipulation gesture";
            case OperationSite.LeftHand:
            case OperationSite.RightHand:
                return "Hand tendon-alignment gesture";
            default:
                return "Knee ligament-alignment gesture";
        }
    }

    private IEnumerator FeedbackProofSequence()
    {
        if (feedbackProbe != null)
        {
            Destroy(feedbackProbe);
        }

        proofSawCaution = false;
        proofSawContact = false;
        feedbackProbe = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        feedbackProbe.name = "Pseudo-Haptic Test Obstacle";
        feedbackProbe.transform.localScale = Vector3.one * 0.24f;
        Renderer probeRenderer = feedbackProbe.GetComponent<Renderer>();
        probeRenderer.material.color = new Color(1f, 0.25f, 0.08f, 1f);

        Vector3 direction = Camera.main != null
            ? Camera.main.transform.right.normalized
            : Vector3.right;
        Vector3 start = endEffector.position
            + direction * (simulationConfig.PseudoHapticWarningDistance + 0.45f);
        float duration = 3f;
        float elapsed = 0f;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            feedbackProbe.transform.position = Vector3.Lerp(
                start,
                endEffector.position,
                Mathf.SmoothStep(0f, 1f, elapsed / duration));
            yield return null;
        }

        yield return new WaitForSeconds(1f);
        elapsed = 0f;
        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            feedbackProbe.transform.position = Vector3.Lerp(
                endEffector.position,
                start,
                Mathf.SmoothStep(0f, 1f, elapsed / duration));
            yield return null;
        }

        Destroy(feedbackProbe);
        feedbackProbe = null;
        feedbackProof = null;
        bool passed = proofSawCaution && proofSawContact;
        Debug.Log(
            $"Research proof: visual pseudo-haptic states caution={proofSawCaution}, " +
            $"contact={proofSawContact}; result={(passed ? "PASS" : "FAIL")}");
        ExportProofData();
    }

    private void UpdateMicroStageVisual()
    {
        if (microStageLine == null || optimizationResult == null)
        {
            return;
        }

        Vector3 toolDirection = endEffector.position - joints[JointCount - 1].position;
        if (toolDirection.sqrMagnitude < 0.000001f)
        {
            toolDirection = endEffector.up;
        }

        Vector3 virtualTip = endEffector.position
            + toolDirection.normalized * activeMicroExtension;
        microStageLine.SetPosition(0, endEffector.position);
        microStageLine.SetPosition(1, virtualTip);
    }

    private void UpdateSurgicalToolVisual()
    {
        if (surgicalTool == null || (!procedureRunning && !replayRunning))
        {
            return;
        }

        Vector3 fallbackDirection = endEffector.position - joints[JointCount - 1].position;
        if (fallbackDirection.sqrMagnitude < 0.000001f)
        {
            fallbackDirection = endEffector.up;
        }

        surgicalTool.UpdatePose(
            GetCurrentToolTipPosition(),
            activeSurgicalTarget,
            fallbackDirection.normalized);
    }

    private void UpdateAnatomyAndTracking()
    {
        if (mannequin == null)
        {
            return;
        }

        mannequin.UpdateAnatomyMotion(
            Time.time,
            simulationConfig.AnatomyMotion,
            simulationConfig.TargetHealth);

        if (activeTargetTransform == null || !procedureRunning)
        {
            return;
        }

        activeSurgicalTarget = simulationConfig.VisionTrackingSimulation
            ? visionTracker.Track(
                activeTargetTransform,
                procedureElapsed,
                simulationConfig.AnatomyMotion,
                simulationConfig.TargetHealth)
            : activeTargetTransform.position;
    }

    private void UpdateResearchTelemetry()
    {
        bool inContact = feedbackState == FeedbackState.Contact;
        if (inContact && !previousContactState)
        {
            collisionEvents++;
        }
        previousContactState = inContact;

        bool intendedTarget = activeTargetCollider != null
            && nearestObstacleName == activeTargetCollider.gameObject.name;
        bool unsafePenetration = procedureRunning
            && simulationConfig.PatientCollisionAvoidance
            && nearestObstacleDistance <= 0.005f
            && !intendedTarget;
        bool severeSingularity = procedureRunning
            && procedureElapsed > SurgicalApproachDuration
            && simulationConfig.SingularityAvoidance
            && workspaceSummary != null
            && currentManipulability < workspaceSummary.MaximumManipulability * 0.01f;
        if (simulationConfig.AutomaticRecovery && (unsafePenetration || severeSingularity))
        {
            safetyMessage = unsafePenetration
                ? $"Unsafe collision near {nearestObstacleName}; recovering"
                : "Severe singularity detected; recovering";
            CancelProcedure(false);
            return;
        }

        if (procedureRunning && !procedurePaused && trajectoryRecorder != null)
        {
            trajectoryRecorder.Record(
                joints,
                GetCurrentToolTipPosition(),
                surgicalTool.ActiveToolType);
        }

        if (Time.time - lastTelemetryPublishTime >= 0.1f)
        {
            ros2Simulation.Publish(GetCurrentJointAngles(), GetCurrentToolTipPosition());
            lastTelemetryPublishTime = Time.time;
        }
    }

    private void UpdateSafetyFeedbackVisual()
    {
        if (feedbackVisualizer == null)
        {
            return;
        }

        int severity = feedbackState == FeedbackState.Contact
            ? 2
            : feedbackState == FeedbackState.Caution ? 1 : 0;
        feedbackVisualizer.Update(
            activeSurgicalTarget,
            simulationConfig.SafetyDistance,
            procedureRunning && simulationConfig.ShowSafetyZone,
            severity,
            simulationConfig.WarningSound);
    }

    private float[] GetCurrentJointAngles()
    {
        float[] angles = new float[JointCount];
        for (int index = 0; index < JointCount; index++)
        {
            Quaternion delta = Quaternion.Inverse(neutralLocalRotations[index])
                * joints[index].localRotation;
            delta.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f)
            {
                angle -= 360f;
            }
            angles[index] = Vector3.Dot(axis, LocalJointAxes[index]) < 0f ? -angle : angle;
        }
        return angles;
    }

    private Vector3 GetCurrentToolTipPosition()
    {
        Vector3 toolDirection = endEffector.position - joints[JointCount - 1].position;
        if (toolDirection.sqrMagnitude < 0.000001f)
        {
            toolDirection = endEffector.up;
        }

        return endEffector.position + toolDirection.normalized * activeMicroExtension;
    }

    private void ExportProofData()
    {
        if (workspaceSummary == null || optimizationResult == null)
        {
            return;
        }

        try
        {
            string directory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "ResearchResults"));
            Directory.CreateDirectory(directory);
            string workspacePath = Path.Combine(directory, "workspace_samples.csv");
            StringBuilder csv = new StringBuilder();
            csv.AppendLine("model,x,y,z,translational_manipulability,clearance");
            foreach (WorkspacePoint point in workspacePoints)
            {
                csv.Append(point.IsMicroExtended ? "macro_micro" : "baseline").Append(',')
                    .Append(point.Position.x.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                    .Append(point.Position.y.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                    .Append(point.Position.z.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                    .Append(point.Manipulability.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                    .Append(point.Clearance.ToString("F6", CultureInfo.InvariantCulture)).AppendLine();
            }
            File.WriteAllText(workspacePath, csv.ToString());

            proofReportPath = Path.Combine(directory, "proof_summary.txt");
            StringBuilder report = new StringBuilder();
            report.AppendLine("SIMULATION RESEARCH PROOF - NOT A FORCE MEASUREMENT");
            report.AppendLine($"Operator: {operatorId}");
            report.AppendLine($"Difficulty: {simulationConfig.Difficulty}");
            report.AppendLine($"Control mode: {simulationConfig.ControlMode}");
            report.AppendLine($"Anatomy motion: {simulationConfig.AnatomyMotion}");
            report.AppendLine($"Target health: {simulationConfig.TargetHealth}");
            report.AppendLine($"Optimizer: {simulationConfig.Optimizer}");
            report.AppendLine($"Camera view: {simulationConfig.CameraView}");
            report.AppendLine($"Workspace samples: {WorkspaceSampleCount}");
            report.AppendLine($"Baseline occupied voxels: {workspaceSummary.BaselineVoxelCount}");
            report.AppendLine($"Macro-micro occupied voxels: {workspaceSummary.MacroMicroVoxelCount}");
            report.AppendLine($"Maximum translational manipulability: {workspaceSummary.MaximumManipulability:F6}");
            report.AppendLine($"Low-manipulability samples: {workspaceSummary.SingularSampleCount}");
            report.AppendLine($"Contact-risk samples: {workspaceSummary.CollisionRiskSampleCount}");
            report.AppendLine($"Baseline target distance: {optimizationResult.BaselineDistance:F6}");
            report.AppendLine($"Optimized target distance: {optimizationResult.OptimizedDistance:F6}");
            report.AppendLine($"Optimized clearance: {optimizationResult.Clearance:F6}");
            report.AppendLine($"Optimized virtual micro extension: {optimizationResult.MicroExtension:F6}");
            report.AppendLine($"Random-search baseline distance: {randomSearchDistance:F6}");
            report.AppendLine($"Simulated neural correction: {neuralPredictedCorrection:F6}");
            report.AppendLine($"Pseudo-haptic caution observed: {proofSawCaution}");
            report.AppendLine($"Pseudo-haptic contact observed: {proofSawContact}");
            foreach (OperationSite site in AvailableOperationSites)
            {
                OperationPreflight preflight = operationPreflights[site];
                report.AppendLine(
                    $"{site} target preflight error: {preflight.Error:F6}");
                report.AppendLine(
                    $"{site} surface preflight clearance: {preflight.SurfaceClearance:F6}");
                report.AppendLine(
                    $"{site} target reachable: {preflight.Passed}");
            }
            report.AppendLine($"Selected mannequin operation: {selectedOperation}");
            report.AppendLine($"Operation target error: {operationTargetError:F6}");
            report.AppendLine($"Operation surface clearance: {operationSurfaceClearance:F6}");
            report.AppendLine($"Operation contact tolerance: {simulationConfig.ContactTolerance:F6}");
            report.AppendLine($"Procedure target: {anatomicalTargetName}");
            report.AppendLine($"Procedure duration: {procedureDurationMeasured:F3}");
            report.AppendLine($"Procedure duration requirement: {simulationConfig.OperationDuration:F3}");
            report.AppendLine($"Surgical tool changes: {surgicalToolChanges}");
            report.AppendLine($"Surgical motion cycles: {surgicalMotionCycles}");
            report.AppendLine($"Maximum procedural target deviation: {maximumProcedureDeviation:F6}");
            report.AppendLine($"Collision events: {collisionEvents}");
            report.AppendLine($"Procedure timeline passed: {procedureTimelinePassed}");
            report.AppendLine($"Operation proof passed: {operationProofPassed}");
            report.AppendLine($"Recorded trajectory samples: {trajectoryRecorder.SampleCount}");
            report.AppendLine($"Saved trials: {trialComparison.TrialCount}");
            report.AppendLine($"Average trial score: {trialComparison.AverageScore:F6}");
            report.AppendLine($"Synthetic vision tracking: {simulationConfig.VisionTrackingSimulation}");
            report.AppendLine($"Adaptive control simulation: {simulationConfig.AdaptiveControlSimulation}");
            report.AppendLine($"ROS 2 status: {ros2Simulation.Status}");
            report.AppendLine($"Mock joint-state publishes: {ros2Simulation.PublishedJointStates}");
            report.AppendLine("Real force sensing, physical haptics, ROS 2 middleware, and neural inference are not connected.");
            File.WriteAllText(proofReportPath, report.ToString());
            Debug.Log($"Research proof exported to {proofReportPath}");
        }
        catch (Exception exception)
        {
            Debug.LogWarning($"Research proof export failed: {exception.Message}");
        }
    }

    private IEnumerator CaptureSelectionScreenshot()
    {
        yield return new WaitForEndOfFrame();
        string screenshotPath = Path.GetFullPath(Path.Combine(
            Application.dataPath,
            "..",
            "ResearchResults",
            "body_region_selection.png"));
        ScreenCapture.CaptureScreenshot(screenshotPath);
        Debug.Log($"Body-region selection screenshot saved to {screenshotPath}");
    }

    private void OnGUI()
    {
        if (!initialized || workspaceSummary == null || optimizationResult == null)
        {
            return;
        }

        if (presentationPhase == PresentationPhase.Selection)
        {
            DrawSelectionPanel();
        }
        else if (presentationPhase == PresentationPhase.Operating)
        {
            DrawOperatingPanel();
        }
        else if (presentationPhase == PresentationPhase.Emergency)
        {
            DrawEmergencyPanel();
        }
        else
        {
            DrawResultsPanel();
        }
    }

    private void DrawEmergencyPanel()
    {
        float width = Mathf.Min(520f, Screen.width - 32f);
        float left = (Screen.width - width) * 0.5f;
        float top = Mathf.Max(16f, (Screen.height - 190f) * 0.5f);
        Color previousColor = GUI.color;
        GUI.color = new Color(0.35f, 0.04f, 0.04f, 1f);
        GUI.DrawTexture(new Rect(left, top, width, 190f), Texture2D.whiteTexture);
        GUI.color = previousColor;
        GUILayout.BeginArea(new Rect(left + 12f, top + 12f, width - 24f, 166f));
        GUILayout.Label("EMERGENCY STOP ACTIVE", CreateCenteredStyle(22, FontStyle.Bold));
        GUILayout.Label("All simulated robot motion is inhibited and the arm has recovered to home.", CreateCenteredStyle(14));
        GUILayout.Space(16f);
        if (GUILayout.Button("CLEAR EMERGENCY STOP", GUILayout.Height(44f)))
        {
            ClearEmergencyStop();
        }
        GUILayout.EndArea();
    }

    private void DrawSelectionPanel()
    {
        Color previousGuiColor = GUI.color;
        GUI.color = new Color(0.025f, 0.03f, 0.038f, 1f);
        GUI.DrawTexture(
            new Rect(0f, 0f, Screen.width * 0.4f, Screen.height),
            Texture2D.whiteTexture);
        GUI.color = previousGuiColor;

        float width = Mathf.Min(410f, Screen.width * 0.4f - 20f);
        float height = Screen.height - 24f;
        float left = 10f;
        float top = 12f;

        GUILayout.BeginArea(new Rect(left, top, width, height), GUI.skin.box);
        GUILayout.Label("SURGICAL ARM SIMULATION", CreateCenteredStyle(18, FontStyle.Bold));
        setupTab = (SetupTab)GUILayout.Toolbar(
            (int)setupTab,
            new[] { "OPERATION", "CONFIG", "MANUAL", "RESEARCH" },
            GUILayout.Height(30f));
        GUILayout.Space(6f);
        setupScrollPosition = GUILayout.BeginScrollView(setupScrollPosition);

        switch (setupTab)
        {
            case SetupTab.Configuration:
                DrawConfigurationTab();
                break;
            case SetupTab.ManualControl:
                DrawManualControlTab();
                break;
            case SetupTab.Research:
                DrawResearchTab();
                break;
            default:
                DrawOperationTab();
                break;
        }

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private void DrawOperationTab()
    {
        GUILayout.Label("Select a mannequin region for examination.", CreateCenteredStyle(13));
        GUILayout.Space(8f);

        if (!operationMenuVisible)
        {
            if (GUILayout.Button("CONDUCT SURGERY SIMULATION", GUILayout.Height(42f)))
            {
                operationMenuVisible = true;
            }
        }
        else
        {
            GUILayout.Label("Choose the operation area:", CreateCenteredStyle(13, FontStyle.Bold));
            GUILayout.BeginHorizontal();
            DrawOperationButton("HEAD / BRAIN", OperationSite.Head);
            DrawOperationButton("MID BODY / HEART", OperationSite.Heart);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            DrawOperationButton("EYE", OperationSite.Eye);
            DrawOperationButton("SPINE", OperationSite.Spine);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            DrawOperationButton("LEFT SHOULDER", OperationSite.LeftShoulder);
            DrawOperationButton("RIGHT SHOULDER", OperationSite.RightShoulder);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            DrawOperationButton("LEFT ELBOW", OperationSite.LeftElbow);
            DrawOperationButton("RIGHT ELBOW", OperationSite.RightElbow);
            GUILayout.EndHorizontal();
            DrawOperationButton("ABDOMEN", OperationSite.Abdomen);
            GUILayout.BeginHorizontal();
            DrawOperationButton("LEFT HAND", OperationSite.LeftHand);
            DrawOperationButton("RIGHT HAND", OperationSite.RightHand);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            DrawOperationButton("LEFT LEG / KNEE", OperationSite.LeftLeg);
            DrawOperationButton("RIGHT LEG / KNEE", OperationSite.RightLeg);
            GUILayout.EndHorizontal();
            if (GUILayout.Button("CANCEL", GUILayout.Height(34f)))
            {
                operationMenuVisible = false;
            }
        }
    }

    private void DrawConfigurationTab()
    {
        GUILayout.Label("PROCEDURE PROFILE", CreateCenteredStyle(14, FontStyle.Bold));
        GUILayout.Label("Difficulty");
        ProcedureDifficulty difficulty = (ProcedureDifficulty)GUILayout.Toolbar(
            (int)simulationConfig.Difficulty,
            new[] { "BEGINNER", "INTERMEDIATE", "ADVANCED" });
        if (difficulty != simulationConfig.Difficulty)
        {
            simulationConfig.ApplyDifficultyPreset(difficulty);
        }

        GUILayout.Label("Control mode");
        simulationConfig.ControlMode = (SurgicalControlMode)GUILayout.Toolbar(
            (int)simulationConfig.ControlMode,
            new[] { "AUTO", "MANUAL", "SHARED" });
        GUILayout.Label("Anatomy");
        simulationConfig.AnatomyMotion = (AnatomyMotionMode)GUILayout.Toolbar(
            (int)simulationConfig.AnatomyMotion,
            new[] { "NORMAL", "MOVING" });
        simulationConfig.TargetHealth = (TargetHealthState)GUILayout.Toolbar(
            (int)simulationConfig.TargetHealth,
            new[] { "HEALTHY", "ABNORMAL" });

        simulationConfig.OperationDuration = DrawSlider(
            "Operation duration (s)", simulationConfig.OperationDuration, 20f, 120f);
        simulationConfig.ArmSpeed = DrawSlider("Arm speed", simulationConfig.ArmSpeed, 0.25f, 2f);
        simulationConfig.Acceleration = DrawSlider("Acceleration", simulationConfig.Acceleration, 0.25f, 2f);
        simulationConfig.Smoothness = DrawSlider("Smoothness", simulationConfig.Smoothness, 0f, 1f);
        simulationConfig.ContactTolerance = DrawSlider("Contact tolerance", simulationConfig.ContactTolerance, 0.03f, 0.25f);
        simulationConfig.SafetyDistance = DrawSlider("Safety distance", simulationConfig.SafetyDistance, 0.2f, 1.5f);
        simulationConfig.TargetSizeScale = DrawSlider("Target size", simulationConfig.TargetSizeScale, 0.5f, 2f);
        simulationConfig.MaximumToolDeviation = DrawSlider("Maximum deviation", simulationConfig.MaximumToolDeviation, 0.03f, 0.3f);
        simulationConfig.CollisionSensitivity = DrawSlider("Collision sensitivity", simulationConfig.CollisionSensitivity, 0.5f, 2f);
        simulationConfig.PseudoHapticWarningDistance = DrawSlider("Warning distance", simulationConfig.PseudoHapticWarningDistance, 0.15f, 1.5f);

        GUILayout.Label("Target offset");
        simulationConfig.TargetOffset.x = DrawSlider("X", simulationConfig.TargetOffset.x, -0.1f, 0.1f);
        simulationConfig.TargetOffset.y = DrawSlider("Y", simulationConfig.TargetOffset.y, -0.1f, 0.1f);
        simulationConfig.TargetOffset.z = DrawSlider("Z", simulationConfig.TargetOffset.z, -0.1f, 0.1f);

        if (GUILayout.Button("APPLY AND RECHECK TARGETS", GUILayout.Height(36f)))
        {
            ApplyConfigurationAndReanalyse();
        }
    }

    private void DrawManualControlTab()
    {
        GUILayout.Label("MANUAL / SHARED JOINT OFFSETS", CreateCenteredStyle(14, FontStyle.Bold));
        GUILayout.Label($"Mode: {simulationConfig.ControlMode}");
        GUILayout.Label("Offsets are added to the optimized target pose so the selected operation remains reachable.");
        for (int index = 0; index < JointCount; index++)
        {
            manualJointAngles[index] = DrawSlider(
                $"J{index + 1} offset",
                manualJointAngles[index],
                -20f,
                20f);
        }

        if (simulationConfig.ControlMode == SurgicalControlMode.SharedControl)
        {
            simulationConfig.SharedControlAuthority = DrawSlider(
                "Autonomous authority",
                simulationConfig.SharedControlAuthority,
                0f,
                1f);
        }

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("PREVIEW OFFSETS", GUILayout.Height(36f)))
        {
            ApplyManualPose();
        }
        if (GUILayout.Button("RETURN HOME", GUILayout.Height(36f)))
        {
            ReturnHome();
        }
        GUILayout.EndHorizontal();

        GUILayout.Label("Joint limits");
        for (int index = 0; index < JointCount; index++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"J{index + 1}", GUILayout.Width(28f));
            simulationConfig.JointMinimumDegrees[index] = GUILayout.HorizontalSlider(
                simulationConfig.JointMinimumDegrees[index], -180f, 0f);
            simulationConfig.JointMaximumDegrees[index] = GUILayout.HorizontalSlider(
                simulationConfig.JointMaximumDegrees[index], 0f, 180f);
            GUILayout.Label(
                $"{simulationConfig.JointMinimumDegrees[index]:F0} / {simulationConfig.JointMaximumDegrees[index]:F0}",
                GUILayout.Width(72f));
            GUILayout.EndHorizontal();
        }
    }

    private void DrawResearchTab()
    {
        GUILayout.Label("OPTIMIZATION AND SAFETY", CreateCenteredStyle(14, FontStyle.Bold));
        simulationConfig.Optimizer = (OptimizationMethod)GUILayout.Toolbar(
            (int)simulationConfig.Optimizer,
            new[] { "GA", "RANDOM", "NEURAL SIM" });
        simulationConfig.AccuracyWeight = DrawSlider("Accuracy weight", simulationConfig.AccuracyWeight, 0.01f, 2f);
        simulationConfig.WorkspaceWeight = DrawSlider("Workspace weight", simulationConfig.WorkspaceWeight, 0f, 1f);
        simulationConfig.ClearanceWeight = DrawSlider("Clearance weight", simulationConfig.ClearanceWeight, 0f, 1f);
        simulationConfig.ManipulabilityWeight = DrawSlider("Manipulability weight", simulationConfig.ManipulabilityWeight, 0f, 1f);

        simulationConfig.SingularityAvoidance = GUILayout.Toggle(simulationConfig.SingularityAvoidance, "Singularity avoidance");
        simulationConfig.SelfCollisionAvoidance = GUILayout.Toggle(simulationConfig.SelfCollisionAvoidance, "Self-collision avoidance");
        simulationConfig.PatientCollisionAvoidance = GUILayout.Toggle(simulationConfig.PatientCollisionAvoidance, "Patient-collision avoidance");
        simulationConfig.RemoteCenterConstraint = GUILayout.Toggle(simulationConfig.RemoteCenterConstraint, "Remote-center constraint");
        simulationConfig.AutomaticRecovery = GUILayout.Toggle(simulationConfig.AutomaticRecovery, "Automatic recovery");
        simulationConfig.VisionTrackingSimulation = GUILayout.Toggle(simulationConfig.VisionTrackingSimulation, "Synthetic vision tracking");
        simulationConfig.AdaptiveControlSimulation = GUILayout.Toggle(simulationConfig.AdaptiveControlSimulation, "Adaptive control simulation");
        simulationConfig.ShowTrajectoryTrail = GUILayout.Toggle(simulationConfig.ShowTrajectoryTrail, "Tool-path trail");
        simulationConfig.ShowSafetyZone = GUILayout.Toggle(simulationConfig.ShowSafetyZone, "Target safety zone");
        simulationConfig.ShowAnatomyOverlay = GUILayout.Toggle(simulationConfig.ShowAnatomyOverlay, "Anatomy overlay");
        simulationConfig.WarningSound = GUILayout.Toggle(simulationConfig.WarningSound, "Warning sound");

        GUILayout.Label("Camera view");
        simulationConfig.CameraView = (SurgicalCameraView)GUILayout.Toolbar(
            (int)simulationConfig.CameraView,
            new[] { "ISO", "TOP", "SIDE", "SCOPE" });
        if (GUILayout.Button("APPLY CAMERA"))
        {
            ApplyCameraView();
        }

        GUILayout.Label("Operator ID");
        operatorId = GUILayout.TextField(operatorId, 32);
        GUILayout.Space(8f);
        GUILayout.Label($"Trials: {trialComparison.TrialCount} | Passed: {trialComparison.PassedCount}");
        GUILayout.Label($"Average score: {trialComparison.AverageScore:F3} | Best: {trialComparison.BestScore:F3}");
        GUILayout.Label($"Average target error: {trialComparison.AverageTargetError:F4}");
        DrawMetricBar("Average score", trialComparison.AverageScore / 100f, new Color(0.15f, 0.8f, 0.45f));
        GUILayout.Label($"GA result: {optimizationResult.OptimizedDistance:F4}");
        GUILayout.Label($"Random-search baseline: {randomSearchDistance:F4}");
        GUILayout.Label($"Neural correction estimate: {neuralPredictedCorrection:F4}");
        GUILayout.Label(ros2Simulation.Status);
        GUILayout.Label($"Mock ROS 2 publishes: {ros2Simulation.PublishedJointStates}");
        if (GUILayout.Button("REANALYSE WORKSPACE AND TARGETS", GUILayout.Height(36f)))
        {
            ApplyConfigurationAndReanalyse();
        }
    }

    private static float DrawSlider(string label, float value, float minimum, float maximum)
    {
        GUILayout.Label($"{label}: {value:F2}");
        return GUILayout.HorizontalSlider(value, minimum, maximum);
    }

    private static void DrawMetricBar(string label, float normalizedValue, Color color)
    {
        GUILayout.Label(label);
        Rect background = GUILayoutUtility.GetRect(0f, 16f, GUILayout.ExpandWidth(true));
        GUI.DrawTexture(background, Texture2D.grayTexture);
        Color previousColor = GUI.color;
        GUI.color = color;
        GUI.DrawTexture(
            new Rect(background.x, background.y, background.width * Mathf.Clamp01(normalizedValue), background.height),
            Texture2D.whiteTexture);
        GUI.color = previousColor;
    }

    private void DrawOperationButton(string label, OperationSite operationSite)
    {
        OperationPreflight preflight;
        bool available = operationPreflights.TryGetValue(operationSite, out preflight)
            && preflight.Passed;
        bool previousEnabled = GUI.enabled;
        GUI.enabled = available;
        string statusLabel = available ? "PASS" : "REVIEW";
        if (GUILayout.Button(
            $"{label}\n{statusLabel}  error {preflight?.Error ?? 0f:F3}",
            GUILayout.Height(48f),
            GUILayout.ExpandWidth(true)))
        {
            StartOperation(operationSite);
        }
        GUI.enabled = previousEnabled;
    }

    private void DrawOperatingPanel()
    {
        Color previousGuiColor = GUI.color;
        GUI.color = new Color(0.025f, 0.03f, 0.038f, 1f);
        GUI.DrawTexture(
            new Rect(0f, 0f, Screen.width * 0.4f, Screen.height),
            Texture2D.whiteTexture);
        GUI.color = previousGuiColor;

        float width = Mathf.Min(410f, Screen.width * 0.4f - 20f);
        float height = Screen.height - 24f;
        GUILayout.BeginArea(new Rect(10f, 12f, width, height), GUI.skin.box);
        operationScrollPosition = GUILayout.BeginScrollView(operationScrollPosition);
        GUILayout.Label(
            $"{anatomicalTargetName.ToUpperInvariant()} PROCEDURE IN PROGRESS",
            CreateCenteredStyle(18, FontStyle.Bold));
        GUILayout.Label(operationStatus, CreateCenteredStyle(13));
        GUILayout.Label(
            $"Tool: {surgicalTool.ActiveToolName}  |  Stage: {surgicalStage}",
            CreateCenteredStyle(12));
        GUILayout.Label(
            $"Time remaining: {Mathf.Max(0f, simulationConfig.OperationDuration - procedureElapsed):F1} seconds",
            CreateCenteredStyle(14, FontStyle.Bold));

        Rect progressRect = GUILayoutUtility.GetRect(0f, 12f, GUILayout.ExpandWidth(true));
        GUI.DrawTexture(progressRect, Texture2D.grayTexture);
        Color progressColor = GUI.color;
        GUI.color = new Color(0.1f, 0.75f, 0.9f, 1f);
        GUI.DrawTexture(
            new Rect(
                progressRect.x,
                progressRect.y,
                progressRect.width * Mathf.Clamp01(
                    procedureElapsed / simulationConfig.OperationDuration),
                progressRect.height),
            Texture2D.whiteTexture);
        GUI.color = progressColor;

        Rect statusRect = GUILayoutUtility.GetRect(0f, 30f, GUILayout.ExpandWidth(true));
        float pulse = 0.75f + 0.25f * Mathf.Sin(Time.unscaledTime * 8f);
        Color statusColor = feedbackState == FeedbackState.Contact
            ? new Color(0.95f * pulse, 0.12f, 0.08f, 1f)
            : feedbackState == FeedbackState.Caution
                ? new Color(1f, 0.72f * pulse, 0.08f, 1f)
                : new Color(0.12f, 0.62f, 0.28f, 1f);
        Color previousColor = GUI.color;
        GUI.color = statusColor;
        GUI.DrawTexture(statusRect, Texture2D.whiteTexture);
        GUI.color = previousColor;
        GUI.Label(statusRect, feedbackState.ToString().ToUpperInvariant(), CreateStatusStyle());
        float contactIntensity = 1f - Mathf.Clamp01(
            nearestObstacleDistance / simulationConfig.PseudoHapticWarningDistance);
        GUILayout.Label(
            $"Contact intensity: {contactIntensity * 100f:F0}% | Location: {nearestObstacleName}",
            CreateCenteredStyle(12));
        if (nearestObstacleDistance < simulationConfig.ContactTolerance * 0.5f)
        {
            GUILayout.Label("EXCESSIVE CONTACT-RISK WARNING", CreateCenteredStyle(13, FontStyle.Bold));
        }

        GUILayout.BeginHorizontal();
        if (GUILayout.Button(procedurePaused ? "RESUME" : "PAUSE", GUILayout.Height(34f)))
        {
            if (procedurePaused)
            {
                ResumeProcedure();
            }
            else
            {
                PauseProcedure();
            }
        }
        if (GUILayout.Button("CANCEL", GUILayout.Height(34f)))
        {
            CancelProcedure(false);
        }
        Color previousBackground = GUI.backgroundColor;
        GUI.backgroundColor = new Color(0.9f, 0.12f, 0.08f);
        if (GUILayout.Button("EMERGENCY STOP", GUILayout.Height(34f)))
        {
            CancelProcedure(true);
        }
        GUI.backgroundColor = previousBackground;
        GUILayout.EndHorizontal();
        GUILayout.Space(8f);
        GUILayout.Label(safetyMessage, CreateCenteredStyle(12, FontStyle.Bold));
        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private void DrawResultsPanel()
    {
        bool workspaceProofPassed = neutralKinematicsError < 0.0001f
            && workspaceSummary.MacroMicroVoxelCount > workspaceSummary.BaselineVoxelCount
            && optimizationResult.OptimizedDistance < optimizationResult.BaselineDistance
            && appliedPoseError < 0.0001f;
        bool visualProofPassed = proofSawCaution
            && proofSawContact
            && operationSurfaceClearance <= simulationConfig.ContactTolerance;
        bool allProofsPassed = operationProofPassed
            && workspaceProofPassed
            && visualProofPassed
            && procedureTimelinePassed;

        Color previousGuiColor = GUI.color;
        GUI.color = new Color(0.025f, 0.03f, 0.038f, 1f);
        GUI.DrawTexture(
            new Rect(0f, 0f, Screen.width * 0.46f, Screen.height),
            Texture2D.whiteTexture);
        GUI.color = previousGuiColor;

        float width = Mathf.Min(420f, Screen.width * 0.46f - 20f);
        float height = Screen.height - 24f;
        GUILayout.BeginArea(new Rect(12f, 12f, width, height), GUI.skin.box);

        GUIStyle titleStyle = CreateCenteredStyle(21, FontStyle.Bold);
        titleStyle.normal.textColor = allProofsPassed
            ? new Color(0.2f, 0.85f, 0.42f)
            : new Color(1f, 0.68f, 0.15f);
        GUILayout.Label(
            allProofsPassed ? "SURGERY SIMULATION COMPLETED" : "SIMULATION REQUIRES REVIEW",
            titleStyle);
        GUILayout.Label(
            $"Procedure: {anatomicalTargetName.ToUpperInvariant()} ({selectedOperation.ToString().ToUpperInvariant()})",
            CreateCenteredStyle(15, FontStyle.Bold));
        GUILayout.Space(10f);
        resultsScrollPosition = GUILayout.BeginScrollView(
            resultsScrollPosition,
            GUILayout.ExpandHeight(true));

        GUILayout.Label($"1. KINEMATICS + WORKSPACE: {(workspaceProofPassed ? "PASS" : "REVIEW")}");
        GUILayout.Label($"Forward-kinematics validation error: {neutralKinematicsError:F6}");
        GUILayout.Label($"Applied-pose error: {appliedPoseError:F6}");
        GUILayout.Label($"Workspace improvement: {workspaceSummary.BaselineVoxelCount} -> {workspaceSummary.MacroMicroVoxelCount} voxels");
        GUILayout.Label($"Target optimization: {optimizationResult.BaselineDistance:F4} -> {optimizationResult.OptimizedDistance:F4}");
        GUILayout.Space(10f);

        GUILayout.Label($"2. VISUAL PSEUDO-HAPTICS: {(visualProofPassed ? "PASS" : "REVIEW")}");
        GUILayout.Label($"Caution state observed: {(proofSawCaution ? "YES" : "NO")}");
        GUILayout.Label($"Contact state observed: {(proofSawContact ? "YES" : "NO")}");
        GUILayout.Label($"Surface clearance: {operationSurfaceClearance:F4}");
        GUILayout.Space(10f);

        GUILayout.Label($"3. SURGICAL MOTION TIMELINE: {(procedureTimelinePassed ? "PASS" : "REVIEW")}");
        GUILayout.Label($"Measured duration: {procedureDurationMeasured:F2} seconds");
        GUILayout.Label($"Tool changes: {surgicalToolChanges}");
        GUILayout.Label($"Precision motion cycles: {surgicalMotionCycles}");
        GUILayout.Label($"Maximum target deviation: {maximumProcedureDeviation:F4}");
        GUILayout.Label($"Collision events: {collisionEvents}");
        GUILayout.Space(10f);

        GUILayout.Label("OPTIMIZER COMPARISON");
        GUILayout.Label($"Selected method: {simulationConfig.Optimizer}");
        GUILayout.Label($"Random-search error: {randomSearchDistance:F4}");
        GUILayout.Label($"Final optimized error: {optimizationResult.OptimizedDistance:F4}");
        GUILayout.Label($"Neural correction estimate: {neuralPredictedCorrection:F4}");
        GUILayout.Space(10f);

        GUILayout.Label("TRIAL HISTORY");
        GUILayout.Label($"Trials: {trialComparison.TrialCount} | Passed: {trialComparison.PassedCount}");
        GUILayout.Label($"Average score: {trialComparison.AverageScore:F3} | Best: {trialComparison.BestScore:F3}");
        DrawMetricBar("Average score", trialComparison.AverageScore / 100f, new Color(0.15f, 0.8f, 0.45f));
        GUILayout.Space(10f);

        GUILayout.Label($"OPERATION RESULT: {(operationProofPassed ? "PASS" : "REVIEW")}");
        GUILayout.Label($"Target error: {operationTargetError:F4}");
        GUILayout.Label($"Allowed tolerance: {simulationConfig.ContactTolerance:F4}");
        GUILayout.Label(operationStatus);
        GUILayout.Space(10f);
        GUILayout.Label("Simulation evidence only - no physical force was measured.");
        GUILayout.EndScrollView();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("SLOW-MOTION REPLAY", GUILayout.Height(38f)))
        {
            StartSlowMotionReplay();
        }
        if (GUILayout.Button("RETURN HOME", GUILayout.Height(38f)))
        {
            ReturnHome();
        }
        GUILayout.EndHorizontal();

        if (GUILayout.Button("NEW OPERATION", GUILayout.Height(38f)))
        {
            RestoreNeutralPose();
            FrameFocusedCamera(new Rect(0.4f, 0f, 0.6f, 1f));
            microStageLine.enabled = false;
            selectedOperation = OperationSite.None;
            operationMenuVisible = false;
            operationProofPassed = false;
            operationStatus = "Select an operation to begin.";
            surgicalStage = "Waiting";
            anatomicalTargetName = "None";
            procedureElapsed = 0f;
            presentationPhase = PresentationPhase.Selection;
        }
        GUILayout.EndArea();
    }

    private static GUIStyle CreateCenteredStyle(int fontSize, FontStyle fontStyle = FontStyle.Normal)
    {
        GUIStyle style = new GUIStyle(GUI.skin.label)
        {
            alignment = TextAnchor.MiddleCenter,
            fontSize = fontSize,
            fontStyle = fontStyle,
            wordWrap = true
        };
        style.normal.textColor = Color.white;
        return style;
    }

    private static GUIStyle CreateStatusStyle()
    {
        GUIStyle style = CreateCenteredStyle(14, FontStyle.Bold);
        style.normal.textColor = Color.white;
        return style;
    }

    private void OnDestroy()
    {
        if (workspaceMaterial != null)
        {
            Destroy(workspaceMaterial);
        }

        if (surgicalTool != null)
        {
            surgicalTool.Destroy();
        }
        if (feedbackVisualizer != null)
        {
            feedbackVisualizer.Destroy();
        }

    }

    private float[] CreateRandomAngles(System.Random random)
    {
        float[] angles = new float[JointCount];
        for (int index = 0; index < JointCount; index++)
        {
            angles[index] = NextFloat(
                random,
                simulationConfig.JointMinimumDegrees[index],
                simulationConfig.JointMaximumDegrees[index]);
        }
        return angles;
    }

    private static float NextFloat(System.Random random, float minimum, float maximum)
    {
        return minimum + (float)random.NextDouble() * (maximum - minimum);
    }

    private static Vector3Int ToVoxel(Vector3 position)
    {
        return new Vector3Int(
            Mathf.FloorToInt(position.x / WorkspaceVoxelSize),
            Mathf.FloorToInt(position.y / WorkspaceVoxelSize),
            Mathf.FloorToInt(position.z / WorkspaceVoxelSize));
    }

    private enum FeedbackState
    {
        Clear,
        Caution,
        Contact
    }

    private enum OperationSite
    {
        None,
        Head,
        Heart,
        Eye,
        Spine,
        LeftShoulder,
        RightShoulder,
        LeftElbow,
        RightElbow,
        Abdomen,
        LeftHand,
        RightHand,
        LeftLeg,
        RightLeg
    }

    private enum PresentationPhase
    {
        Selection,
        Operating,
        Results,
        Emergency
    }

    private enum SetupTab
    {
        Operation,
        Configuration,
        ManualControl,
        Research
    }

    private sealed class WorkspacePoint
    {
        public Vector3 Position { get; }
        public float Manipulability { get; }
        public float Clearance { get; }
        public bool IsMicroExtended { get; }

        public WorkspacePoint(Vector3 position, float manipulability, float clearance, bool isMicroExtended)
        {
            Position = position;
            Manipulability = manipulability;
            Clearance = clearance;
            IsMicroExtended = isMicroExtended;
        }
    }

    private sealed class OperationProfile
    {
        public string AnatomicalName { get; }
        public Collider TargetCollider { get; }
        public Transform TargetTransform { get; }
        public Vector3 TargetPosition { get; }
        public float MotionScale { get; }

        public OperationProfile(
            string anatomicalName,
            Collider targetCollider,
            Transform targetTransform,
            Vector3 targetPosition,
            float motionScale)
        {
            AnatomicalName = anatomicalName;
            TargetCollider = targetCollider;
            TargetTransform = targetTransform;
            TargetPosition = targetPosition;
            MotionScale = motionScale;
        }
    }

    private sealed class OperationPreflight
    {
        public float Error { get; }
        public float SurfaceClearance { get; }
        public bool Passed { get; }

        public OperationPreflight(float error, float surfaceClearance, bool passed)
        {
            Error = error;
            SurfaceClearance = surfaceClearance;
            Passed = passed;
        }
    }

    private sealed class WorkspaceSummary
    {
        public int BaselineVoxelCount { get; }
        public int MacroMicroVoxelCount { get; }
        public float MaximumManipulability { get; }
        public int SingularSampleCount { get; }
        public int CollisionRiskSampleCount { get; }

        public WorkspaceSummary(
            int baselineVoxelCount,
            int macroMicroVoxelCount,
            float maximumManipulability,
            int singularSampleCount,
            int collisionRiskSampleCount)
        {
            BaselineVoxelCount = baselineVoxelCount;
            MacroMicroVoxelCount = macroMicroVoxelCount;
            MaximumManipulability = maximumManipulability;
            SingularSampleCount = singularSampleCount;
            CollisionRiskSampleCount = collisionRiskSampleCount;
        }
    }

    private sealed class Candidate
    {
        public float[] Angles { get; }
        public float MicroExtension { get; }
        public ArmKinematicSample Sample { get; }
        public float Distance { get; }
        public float Clearance { get; }
        public float Score { get; }

        public Candidate(
            float[] angles,
            float microExtension,
            ArmKinematicSample sample,
            float distance,
            float clearance,
            float score)
        {
            Angles = angles;
            MicroExtension = microExtension;
            Sample = sample;
            Distance = distance;
            Clearance = clearance;
            Score = score;
        }

        public Candidate Clone()
        {
            return new Candidate(
                (float[])Angles.Clone(),
                MicroExtension,
                Sample,
                Distance,
                Clearance,
                Score);
        }
    }

    private sealed class OptimizationResult
    {
        public float[] Angles { get; }
        public float MicroExtension { get; }
        public ArmKinematicSample Sample { get; }
        public float BaselineDistance { get; }
        public float OptimizedDistance { get; }
        public float Clearance { get; }

        public OptimizationResult(
            float[] angles,
            float microExtension,
            ArmKinematicSample sample,
            float baselineDistance,
            float optimizedDistance,
            float clearance)
        {
            Angles = angles;
            MicroExtension = microExtension;
            Sample = sample;
            BaselineDistance = baselineDistance;
            OptimizedDistance = optimizedDistance;
            Clearance = clearance;
        }
    }
}

public static class SurgicalArmResearchBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void AttachToMainArm()
    {
        if (SceneManager.GetActiveScene().name != "RobotControl")
        {
            return;
        }

        GameObject robotObject = GameObject.Find("Robot1");
        GameObject endEffectorObject = GameObject.Find("AnimationEE");
        if (robotObject == null || endEffectorObject == null)
        {
            Debug.LogError("Research monitor could not find Robot1 or AnimationEE.");
            return;
        }

        RobotMove robotMove = robotObject.GetComponent<RobotMove>();
        if (robotMove == null)
        {
            Debug.LogError("Research monitor could not find RobotMove on Robot1.");
            return;
        }

        SurgicalArmResearchController controller =
            robotObject.GetComponent<SurgicalArmResearchController>();
        if (controller == null)
        {
            controller = robotObject.AddComponent<SurgicalArmResearchController>();
        }

        controller.Initialize(robotMove, endEffectorObject.transform);
    }
}