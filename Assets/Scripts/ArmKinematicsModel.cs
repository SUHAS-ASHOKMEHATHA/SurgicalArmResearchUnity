using System;
using UnityEngine;

public sealed class ArmKinematicsModel
{
    public const int JointCount = 6;

    private readonly Vector3[] neutralJointPositions;
    private readonly Vector3[] neutralJointAxes;
    private readonly Vector3 neutralEndEffectorPosition;
    private readonly Vector3 neutralToolDirection;

    public ArmKinematicsModel(
        Vector3[] jointPositions,
        Vector3[] jointAxes,
        Vector3 endEffectorPosition)
    {
        if (jointPositions == null || jointPositions.Length != JointCount)
        {
            throw new ArgumentException("Six joint positions are required.", nameof(jointPositions));
        }

        if (jointAxes == null || jointAxes.Length != JointCount)
        {
            throw new ArgumentException("Six joint axes are required.", nameof(jointAxes));
        }

        neutralJointPositions = (Vector3[])jointPositions.Clone();
        neutralJointAxes = new Vector3[JointCount];
        for (int index = 0; index < JointCount; index++)
        {
            neutralJointAxes[index] = jointAxes[index].normalized;
        }

        neutralEndEffectorPosition = endEffectorPosition;
        Vector3 toolVector = endEffectorPosition - jointPositions[JointCount - 1];
        neutralToolDirection = toolVector.sqrMagnitude > 0.000001f
            ? toolVector.normalized
            : Vector3.up;
    }

    public ArmKinematicSample Evaluate(float[] jointAnglesDegrees, float microExtension = 0f)
    {
        if (jointAnglesDegrees == null || jointAnglesDegrees.Length != JointCount)
        {
            throw new ArgumentException("Six joint angles are required.", nameof(jointAnglesDegrees));
        }

        Vector3[] positions = (Vector3[])neutralJointPositions.Clone();
        Vector3[] axes = (Vector3[])neutralJointAxes.Clone();
        Vector3 endEffectorPosition = neutralEndEffectorPosition;
        Vector3 toolDirection = neutralToolDirection;

        for (int jointIndex = 0; jointIndex < JointCount; jointIndex++)
        {
            Quaternion rotation = Quaternion.AngleAxis(jointAnglesDegrees[jointIndex], axes[jointIndex]);
            Vector3 pivot = positions[jointIndex];

            for (int childIndex = jointIndex + 1; childIndex < JointCount; childIndex++)
            {
                positions[childIndex] = pivot + rotation * (positions[childIndex] - pivot);
                axes[childIndex] = rotation * axes[childIndex];
            }

            endEffectorPosition = pivot + rotation * (endEffectorPosition - pivot);
            toolDirection = rotation * toolDirection;
        }

        Vector3 toolTipPosition = endEffectorPosition + toolDirection.normalized * microExtension;
        float manipulability = CalculateTranslationalManipulability(positions, axes, toolTipPosition);

        return new ArmKinematicSample(
            positions,
            axes,
            endEffectorPosition,
            toolTipPosition,
            toolDirection.normalized,
            manipulability);
    }

    public static float CalculateTranslationalManipulability(
        Vector3[] jointPositions,
        Vector3[] jointAxes,
        Vector3 toolPosition)
    {
        float m00 = 0f;
        float m01 = 0f;
        float m02 = 0f;
        float m11 = 0f;
        float m12 = 0f;
        float m22 = 0f;

        for (int index = 0; index < JointCount; index++)
        {
            Vector3 column = Vector3.Cross(
                jointAxes[index].normalized,
                toolPosition - jointPositions[index]);

            m00 += column.x * column.x;
            m01 += column.x * column.y;
            m02 += column.x * column.z;
            m11 += column.y * column.y;
            m12 += column.y * column.z;
            m22 += column.z * column.z;
        }

        float determinant =
            m00 * (m11 * m22 - m12 * m12)
            - m01 * (m01 * m22 - m12 * m02)
            + m02 * (m01 * m12 - m11 * m02);

        return Mathf.Sqrt(Mathf.Max(0f, determinant));
    }
}

public sealed class ArmKinematicSample
{
    public Vector3[] JointPositions { get; }
    public Vector3[] JointAxes { get; }
    public Vector3 EndEffectorPosition { get; }
    public Vector3 ToolTipPosition { get; }
    public Vector3 ToolDirection { get; }
    public float TranslationalManipulability { get; }

    public ArmKinematicSample(
        Vector3[] jointPositions,
        Vector3[] jointAxes,
        Vector3 endEffectorPosition,
        Vector3 toolTipPosition,
        Vector3 toolDirection,
        float translationalManipulability)
    {
        JointPositions = jointPositions;
        JointAxes = jointAxes;
        EndEffectorPosition = endEffectorPosition;
        ToolTipPosition = toolTipPosition;
        ToolDirection = toolDirection;
        TranslationalManipulability = translationalManipulability;
    }
}