using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public sealed class SurgicalTrajectoryRecorder
{
    private readonly List<TrajectorySample> samples = new List<TrajectorySample>();
    private readonly LineRenderer trail;
    private readonly Material trailMaterial;
    private float startTime;
    private float lastRecordTime;

    public int SampleCount => samples.Count;
    public bool IsRecording { get; private set; }
    public bool TrailVisible
    {
        get => trail.enabled;
        set => trail.enabled = value;
    }

    public SurgicalTrajectoryRecorder()
    {
        GameObject trailObject = new GameObject("Surgical Tool Path Trail");
        trail = trailObject.AddComponent<LineRenderer>();
        trail.useWorldSpace = true;
        trail.startWidth = 0.012f;
        trail.endWidth = 0.006f;
        trail.positionCount = 0;
        trail.numCornerVertices = 3;
        trail.numCapVertices = 3;
        trail.startColor = new Color(0.1f, 0.85f, 1f, 0.85f);
        trail.endColor = new Color(1f, 0.35f, 0.15f, 0.75f);

        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null)
        {
            shader = Shader.Find("Sprites/Default");
        }
        if (shader != null)
        {
            trailMaterial = new Material(shader);
            trail.material = trailMaterial;
        }
        trail.enabled = false;
    }

    public void Begin(bool showTrail)
    {
        samples.Clear();
        trail.positionCount = 0;
        trail.enabled = showTrail;
        startTime = Time.time;
        lastRecordTime = float.NegativeInfinity;
        IsRecording = true;
    }

    public void Record(Transform[] joints, Vector3 toolPosition, SurgicalToolType toolType)
    {
        if (!IsRecording || joints == null || joints.Length != 6)
        {
            return;
        }

        if (Time.time - lastRecordTime < 1f / 30f)
        {
            return;
        }
        lastRecordTime = Time.time;

        Quaternion[] rotations = new Quaternion[6];
        for (int index = 0; index < rotations.Length; index++)
        {
            rotations[index] = joints[index].localRotation;
        }

        samples.Add(new TrajectorySample(
            Time.time - startTime,
            rotations,
            toolPosition,
            toolType));

        if (trail.enabled)
        {
            trail.positionCount++;
            trail.SetPosition(trail.positionCount - 1, toolPosition);
        }
    }

    public void End()
    {
        IsRecording = false;
    }

    public IEnumerator Replay(
        Transform[] joints,
        Action<SurgicalToolType> changeTool,
        float speedScale)
    {
        if (samples.Count < 2)
        {
            yield break;
        }

        speedScale = Mathf.Clamp(speedScale, 0.1f, 2f);
        SurgicalToolType activeTool = SurgicalToolType.None;
        for (int sampleIndex = 0; sampleIndex < samples.Count; sampleIndex++)
        {
            TrajectorySample sample = samples[sampleIndex];
            for (int jointIndex = 0; jointIndex < 6; jointIndex++)
            {
                joints[jointIndex].localRotation = sample.JointRotations[jointIndex];
            }

            if (sample.ToolType != activeTool)
            {
                activeTool = sample.ToolType;
                changeTool?.Invoke(activeTool);
            }

            if (sampleIndex + 1 < samples.Count)
            {
                float delay = (samples[sampleIndex + 1].Time - sample.Time) / speedScale;
                if (delay > 0f)
                {
                    yield return new WaitForSeconds(delay);
                }
            }
        }
    }

    public void Destroy()
    {
        if (trail != null)
        {
            UnityEngine.Object.Destroy(trail.gameObject);
        }
        if (trailMaterial != null)
        {
            UnityEngine.Object.Destroy(trailMaterial);
        }
    }

    private sealed class TrajectorySample
    {
        public float Time { get; }
        public Quaternion[] JointRotations { get; }
        public Vector3 ToolPosition { get; }
        public SurgicalToolType ToolType { get; }

        public TrajectorySample(
            float time,
            Quaternion[] jointRotations,
            Vector3 toolPosition,
            SurgicalToolType toolType)
        {
            Time = time;
            JointRotations = jointRotations;
            ToolPosition = toolPosition;
            ToolType = toolType;
        }
    }
}