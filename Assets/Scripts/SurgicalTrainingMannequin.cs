using UnityEngine;
using System.Collections.Generic;

public sealed class SurgicalTrainingMannequin : MonoBehaviour
{
    private static readonly Vector3 HeadLocalPosition = new Vector3(-0.22f, 0f, 0f);
    private static readonly Vector3 HeartLocalPosition = new Vector3(-0.055f, 0.105f, -0.035f);
    private static readonly Vector3 LeftHandLocalPosition = new Vector3(-0.025f, 0.04f, -0.31f);
    private static readonly Vector3 RightHandLocalPosition = new Vector3(-0.025f, 0.04f, 0.31f);
    private static readonly Vector3 LeftKneeLocalPosition = new Vector3(0.32f, 0.03f, -0.075f);
    private static readonly Vector3 RightKneeLocalPosition = new Vector3(0.32f, 0.03f, 0.075f);
    private const float HeadSurfaceOffset = 0.1f;

    private readonly List<Transform> surgicalTargets = new List<Transform>();
    private readonly Dictionary<Transform, Vector3> targetBasePositions =
        new Dictionary<Transform, Vector3>();
    private readonly Dictionary<Transform, Vector3> targetBaseScales =
        new Dictionary<Transform, Vector3>();
    private readonly Dictionary<Transform, Color> targetBaseColors =
        new Dictionary<Transform, Color>();
    private Vector3 basePosition;

    public Collider HeadCollider { get; private set; }
    public Collider HeartCollider { get; private set; }
    public Collider LeftHandCollider { get; private set; }
    public Collider RightHandCollider { get; private set; }
    public Collider LeftLegCollider { get; private set; }
    public Collider RightLegCollider { get; private set; }
    public Collider EyeCollider { get; private set; }
    public Collider SpineCollider { get; private set; }
    public Collider LeftShoulderCollider { get; private set; }
    public Collider RightShoulderCollider { get; private set; }
    public Collider LeftElbowCollider { get; private set; }
    public Collider RightElbowCollider { get; private set; }
    public Collider AbdomenCollider { get; private set; }
    public Transform BrainTarget { get; private set; }
    public Transform HeartTarget { get; private set; }
    public Transform LeftHandTarget { get; private set; }
    public Transform RightHandTarget { get; private set; }
    public Transform LeftKneeTarget { get; private set; }
    public Transform RightKneeTarget { get; private set; }
    public Transform EyeTarget { get; private set; }
    public Transform SpineTarget { get; private set; }
    public Transform LeftShoulderTarget { get; private set; }
    public Transform RightShoulderTarget { get; private set; }
    public Transform LeftElbowTarget { get; private set; }
    public Transform RightElbowTarget { get; private set; }
    public Transform AbdomenTarget { get; private set; }
    public Vector3 HeadTargetPosition => BrainTarget.position;
    public Vector3 HeartTargetPosition => HeartTarget.position;
    public Vector3 LeftHandTargetPosition => LeftHandTarget.position;
    public Vector3 RightHandTargetPosition => RightHandTarget.position;
    public Vector3 LeftLegTargetPosition => LeftKneeTarget.position;
    public Vector3 RightLegTargetPosition => RightKneeTarget.position;
    public Vector3 EyeTargetPosition => EyeTarget.position;
    public Vector3 SpineTargetPosition => SpineTarget.position;
    public Vector3 LeftShoulderTargetPosition => LeftShoulderTarget.position;
    public Vector3 RightShoulderTargetPosition => RightShoulderTarget.position;
    public Vector3 LeftElbowTargetPosition => LeftElbowTarget.position;
    public Vector3 RightElbowTargetPosition => RightElbowTarget.position;
    public Vector3 AbdomenTargetPosition => AbdomenTarget.position;

    public static SurgicalTrainingMannequin Create(Vector3 headTargetPosition, Quaternion rotation)
    {
        GameObject root = new GameObject("Surgical Training Mannequin");
        Vector3 rootPosition = headTargetPosition
            - rotation * HeadLocalPosition
            - Vector3.up * HeadSurfaceOffset;
        root.transform.SetPositionAndRotation(rootPosition, rotation);
        SurgicalTrainingMannequin mannequin = root.AddComponent<SurgicalTrainingMannequin>();
        mannequin.Build();
        mannequin.basePosition = root.transform.position;
        return mannequin;
    }

    private void Build()
    {
        Color skinColor = new Color(0.74f, 0.67f, 0.61f, 1f);
        Color gownColor = new Color(0.08f, 0.42f, 0.48f, 1f);
        Color gownAccentColor = new Color(0.08f, 0.58f, 0.62f, 1f);
        Color brainColor = new Color(0.95f, 0.28f, 0.42f, 1f);
        Color heartColor = new Color(0.9f, 0.12f, 0.22f, 1f);
        Color handColor = new Color(0.95f, 0.7f, 0.25f, 1f);
        Color kneeColor = new Color(0.12f, 0.78f, 0.92f, 1f);
        Color eyeColor = new Color(0.45f, 0.95f, 0.62f, 1f);
        Color spineColor = new Color(0.65f, 0.42f, 0.95f, 1f);
        Color jointColor = new Color(0.95f, 0.48f, 0.16f, 1f);
        Color abdomenColor = new Color(0.95f, 0.82f, 0.2f, 1f);

        GameObject head = CreatePart(
            "Head Operation Target",
            PrimitiveType.Sphere,
            HeadLocalPosition,
            new Vector3(0.18f, 0.16f, 0.18f),
            Quaternion.identity,
            skinColor);
        HeadCollider = head.GetComponent<Collider>();
        BrainTarget = CreateTargetMarker(
            "Brain Surgical Site",
            HeadLocalPosition + Vector3.up * HeadSurfaceOffset,
            new Vector3(0.08f, 0.025f, 0.08f),
            brainColor).transform;
        EyeCollider = HeadCollider;
        EyeTarget = CreateTargetMarker(
            "Eye Surgical Site",
            HeadLocalPosition + new Vector3(-0.025f, 0.09f, -0.055f),
            new Vector3(0.032f, 0.014f, 0.032f),
            eyeColor).transform;

        CreatePart(
            "Neck",
            PrimitiveType.Cylinder,
            new Vector3(-0.145f, 0f, 0f),
            new Vector3(0.065f, 0.045f, 0.065f),
            Quaternion.Euler(0f, 0f, 90f),
            skinColor);

        GameObject torso = CreatePart(
            "Upper Torso",
            PrimitiveType.Capsule,
            new Vector3(-0.025f, 0f, 0f),
            new Vector3(0.16f, 0.19f, 0.18f),
            Quaternion.Euler(0f, 0f, 90f),
            gownColor);
        HeartCollider = torso.GetComponent<Collider>();
        HeartTarget = CreateTargetMarker(
            "Heart Surgical Site",
            HeartLocalPosition,
            new Vector3(0.065f, 0.022f, 0.065f),
            heartColor).transform;
        SpineCollider = HeartCollider;
        SpineTarget = CreateTargetMarker(
            "Spine Surgical Site",
            new Vector3(0.015f, 0.105f, 0.095f),
            new Vector3(0.04f, 0.014f, 0.12f),
            spineColor).transform;

        LeftShoulderCollider = HeartCollider;
        LeftShoulderTarget = CreateTargetMarker(
            "Left Shoulder Surgical Site",
            new Vector3(-0.075f, 0.085f, -0.14f),
            new Vector3(0.045f, 0.016f, 0.045f),
            jointColor).transform;
        RightShoulderCollider = HeartCollider;
        RightShoulderTarget = CreateTargetMarker(
            "Right Shoulder Surgical Site",
            new Vector3(-0.075f, 0.085f, 0.14f),
            new Vector3(0.045f, 0.016f, 0.045f),
            jointColor).transform;

        GameObject abdomen = CreatePart(
            "Abdomen",
            PrimitiveType.Capsule,
            new Vector3(0.105f, 0f, 0f),
            new Vector3(0.145f, 0.13f, 0.16f),
            Quaternion.Euler(0f, 0f, 90f),
            gownAccentColor);
        AbdomenCollider = abdomen.GetComponent<Collider>();
        AbdomenTarget = CreateTargetMarker(
            "Abdomen Surgical Site",
            new Vector3(0.105f, 0.09f, 0f),
            new Vector3(0.065f, 0.018f, 0.075f),
            abdomenColor).transform;

        CreatePart(
            "Hip",
            PrimitiveType.Sphere,
            new Vector3(0.15f, 0f, 0f),
            new Vector3(0.18f, 0.12f, 0.2f),
            Quaternion.identity,
            gownColor);

        CreatePart(
            "Left Upper Leg",
            PrimitiveType.Capsule,
            new Vector3(0.255f, 0f, -0.075f),
            new Vector3(0.08f, 0.13f, 0.08f),
            Quaternion.Euler(0f, 0f, 90f),
            gownColor);

        GameObject leftKnee = CreatePart(
            "Left Knee",
            PrimitiveType.Sphere,
            LeftKneeLocalPosition,
            new Vector3(0.1f, 0.08f, 0.1f),
            Quaternion.identity,
            skinColor);
        LeftLegCollider = leftKnee.GetComponent<Collider>();
        LeftKneeTarget = CreateTargetMarker(
            "Left Knee Surgical Site",
            LeftKneeLocalPosition + Vector3.up * 0.055f,
            new Vector3(0.055f, 0.018f, 0.055f),
            kneeColor).transform;

        CreatePart(
            "Left Lower Leg",
            PrimitiveType.Capsule,
            new Vector3(0.43f, 0f, -0.075f),
            new Vector3(0.07f, 0.13f, 0.07f),
            Quaternion.Euler(0f, 0f, 90f),
            skinColor);

        CreatePart(
            "Right Upper Leg",
            PrimitiveType.Capsule,
            new Vector3(0.255f, 0f, 0.075f),
            new Vector3(0.08f, 0.13f, 0.08f),
            Quaternion.Euler(0f, 0f, 90f),
            gownColor);

        GameObject rightKnee = CreatePart(
            "Right Knee",
            PrimitiveType.Sphere,
            RightKneeLocalPosition,
            new Vector3(0.1f, 0.08f, 0.1f),
            Quaternion.identity,
            skinColor);
        RightLegCollider = rightKnee.GetComponent<Collider>();
        RightKneeTarget = CreateTargetMarker(
            "Right Knee Surgical Site",
            RightKneeLocalPosition + Vector3.up * 0.055f,
            new Vector3(0.055f, 0.018f, 0.055f),
            kneeColor).transform;

        CreatePart(
            "Right Lower Leg",
            PrimitiveType.Capsule,
            new Vector3(0.43f, 0f, 0.075f),
            new Vector3(0.07f, 0.13f, 0.07f),
            Quaternion.Euler(0f, 0f, 90f),
            skinColor);

        CreatePart(
            "Left Foot",
            PrimitiveType.Capsule,
            new Vector3(0.56f, 0f, -0.075f),
            new Vector3(0.065f, 0.08f, 0.085f),
            Quaternion.Euler(0f, 0f, 90f),
            skinColor);

        CreatePart(
            "Right Foot",
            PrimitiveType.Capsule,
            new Vector3(0.56f, 0f, 0.075f),
            new Vector3(0.065f, 0.08f, 0.085f),
            Quaternion.Euler(0f, 0f, 90f),
            skinColor);

        CreatePart(
            "Left Upper Arm",
            PrimitiveType.Capsule,
            new Vector3(-0.055f, 0f, -0.17f),
            new Vector3(0.06f, 0.11f, 0.06f),
            Quaternion.Euler(90f, 0f, 0f),
            gownColor);

        GameObject leftForearm = CreatePart(
            "Left Forearm",
            PrimitiveType.Capsule,
            new Vector3(-0.035f, 0f, -0.255f),
            new Vector3(0.05f, 0.09f, 0.05f),
            Quaternion.Euler(90f, 0f, 0f),
            skinColor);
        LeftElbowCollider = leftForearm.GetComponent<Collider>();
        LeftElbowTarget = CreateTargetMarker(
            "Left Elbow Surgical Site",
            new Vector3(-0.04f, 0.045f, -0.225f),
            new Vector3(0.04f, 0.014f, 0.04f),
            jointColor).transform;

        GameObject leftHand = CreatePart(
            "Left Hand",
            PrimitiveType.Sphere,
            LeftHandLocalPosition,
            new Vector3(0.09f, 0.055f, 0.075f),
            Quaternion.identity,
            skinColor);
        LeftHandCollider = leftHand.GetComponent<Collider>();
        LeftHandTarget = CreateTargetMarker(
            "Left Hand Surgical Site",
            LeftHandLocalPosition + Vector3.up * 0.045f,
            new Vector3(0.045f, 0.015f, 0.045f),
            handColor).transform;

        CreatePart(
            "Right Upper Arm",
            PrimitiveType.Capsule,
            new Vector3(-0.055f, 0f, 0.17f),
            new Vector3(0.06f, 0.11f, 0.06f),
            Quaternion.Euler(90f, 0f, 0f),
            gownColor);

        GameObject rightForearm = CreatePart(
            "Right Forearm",
            PrimitiveType.Capsule,
            new Vector3(-0.035f, 0f, 0.255f),
            new Vector3(0.05f, 0.09f, 0.05f),
            Quaternion.Euler(90f, 0f, 0f),
            skinColor);
        RightElbowCollider = rightForearm.GetComponent<Collider>();
        RightElbowTarget = CreateTargetMarker(
            "Right Elbow Surgical Site",
            new Vector3(-0.04f, 0.045f, 0.225f),
            new Vector3(0.04f, 0.014f, 0.04f),
            jointColor).transform;

        GameObject rightHand = CreatePart(
            "Right Hand",
            PrimitiveType.Sphere,
            RightHandLocalPosition,
            new Vector3(0.09f, 0.055f, 0.075f),
            Quaternion.identity,
            skinColor);
        RightHandCollider = rightHand.GetComponent<Collider>();
        RightHandTarget = CreateTargetMarker(
            "Right Hand Surgical Site",
            RightHandLocalPosition + Vector3.up * 0.045f,
            new Vector3(0.045f, 0.015f, 0.045f),
            handColor).transform;

        SetHighlightedTarget(null, true);
    }

    private GameObject CreatePart(
        string partName,
        PrimitiveType primitiveType,
        Vector3 localPosition,
        Vector3 localScale,
        Quaternion localRotation,
        Color color)
    {
        GameObject part = GameObject.CreatePrimitive(primitiveType);
        part.name = partName;
        part.transform.SetParent(transform, false);
        part.transform.localPosition = localPosition;
        part.transform.localRotation = localRotation;
        part.transform.localScale = localScale;

        Renderer partRenderer = part.GetComponent<Renderer>();
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader != null)
        {
            partRenderer.material = new Material(shader);
        }
        partRenderer.material.color = color;
        if (partRenderer.material.HasProperty("_Smoothness"))
        {
            partRenderer.material.SetFloat("_Smoothness", 0.42f);
        }
        return part;
    }

    private GameObject CreateTargetMarker(
        string markerName,
        Vector3 localPosition,
        Vector3 localScale,
        Color color)
    {
        GameObject marker = CreatePart(
            markerName,
            PrimitiveType.Cylinder,
            localPosition,
            localScale,
            Quaternion.identity,
            color);
        Collider markerCollider = marker.GetComponent<Collider>();
        markerCollider.enabled = false;
        surgicalTargets.Add(marker.transform);
        targetBasePositions[marker.transform] = localPosition;
        targetBaseScales[marker.transform] = localScale;
        targetBaseColors[marker.transform] = color;
        return marker;
    }

    public void SetHighlightedTarget(Transform target, bool showAnatomyOverlay)
    {
        foreach (Transform surgicalTarget in surgicalTargets)
        {
            Renderer targetRenderer = surgicalTarget.GetComponent<Renderer>();
            bool selected = surgicalTarget == target;
            targetRenderer.enabled = selected || (showAnatomyOverlay && target == null);
            if (selected)
            {
                targetRenderer.material.EnableKeyword("_EMISSION");
                targetRenderer.material.SetColor(
                    "_EmissionColor",
                    targetRenderer.material.color * 1.4f);
            }
            else
            {
                targetRenderer.material.SetColor("_EmissionColor", Color.black);
            }
        }
    }

    public void ConfigureTargetAppearance(
        Transform selectedTarget,
        float targetScale,
        Vector3 targetOffset,
        TargetHealthState healthState,
        bool showAnatomyOverlay)
    {
        SetHighlightedTarget(selectedTarget, showAnatomyOverlay);
        foreach (Transform surgicalTarget in surgicalTargets)
        {
            float selectedScale = surgicalTarget == selectedTarget ? targetScale : 1f;
            surgicalTarget.localPosition = targetBasePositions[surgicalTarget]
                + (surgicalTarget == selectedTarget ? targetOffset : Vector3.zero);
            surgicalTarget.localScale = targetBaseScales[surgicalTarget] * selectedScale;
            Renderer renderer = surgicalTarget.GetComponent<Renderer>();
            if (surgicalTarget == selectedTarget
                && healthState == TargetHealthState.SimulatedAbnormality)
            {
                renderer.material.color = new Color(1f, 0.15f, 0.08f, 1f);
            }
            else
            {
                renderer.material.color = targetBaseColors[surgicalTarget];
            }
        }
    }

    public void UpdateAnatomyMotion(
        float elapsed,
        AnatomyMotionMode motionMode,
        TargetHealthState healthState)
    {
        if (motionMode == AnatomyMotionMode.Normal)
        {
            transform.position = basePosition;
            return;
        }

        float breathing = Mathf.Sin(elapsed * Mathf.PI * 0.45f) * 0.008f;
        float heartbeatAmplitude = healthState == TargetHealthState.SimulatedAbnormality
            ? 0.006f
            : 0.003f;
        float heartbeat = Mathf.Sin(elapsed * Mathf.PI * 2f * 1.15f) * heartbeatAmplitude;
        transform.position = basePosition + Vector3.up * (breathing + heartbeat);
    }
}