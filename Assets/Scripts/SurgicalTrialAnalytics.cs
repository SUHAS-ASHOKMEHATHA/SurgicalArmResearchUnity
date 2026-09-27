using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[Serializable]
public sealed class SurgicalTrialRecord
{
    public string TrialId;
    public string TimestampUtc;
    public string OperatorId;
    public string AnatomicalTarget;
    public string Difficulty;
    public string ControlMode;
    public string AnatomyMotion;
    public string TargetHealth;
    public string Optimizer;
    public float Duration;
    public float TargetError;
    public float SurfaceClearance;
    public float MaximumDeviation;
    public float Manipulability;
    public int ToolChanges;
    public int MotionCycles;
    public int CollisionEvents;
    public bool Passed;
    public float Score;
}

[Serializable]
internal sealed class SurgicalTrialCollection
{
    public List<SurgicalTrialRecord> Trials = new List<SurgicalTrialRecord>();
}

public sealed class SurgicalTrialDatabase
{
    private readonly string jsonPath;
    private readonly string csvPath;
    private readonly SurgicalTrialCollection collection;

    public IReadOnlyList<SurgicalTrialRecord> Trials => collection.Trials;

    public SurgicalTrialDatabase(string resultsDirectory)
    {
        Directory.CreateDirectory(resultsDirectory);
        jsonPath = Path.Combine(resultsDirectory, "surgical_trials.json");
        csvPath = Path.Combine(resultsDirectory, "surgical_trials.csv");
        collection = Load();
    }

    public void Save(SurgicalTrialRecord trial)
    {
        collection.Trials.Add(trial);
        File.WriteAllText(jsonPath, JsonUtility.ToJson(collection, true));
        ExportCsv();
    }

    public TrialComparison Compare()
    {
        if (collection.Trials.Count == 0)
        {
            return new TrialComparison();
        }

        float totalScore = 0f;
        float totalError = 0f;
        float bestScore = float.MinValue;
        int passed = 0;
        foreach (SurgicalTrialRecord trial in collection.Trials)
        {
            totalScore += trial.Score;
            totalError += trial.TargetError;
            bestScore = Mathf.Max(bestScore, trial.Score);
            if (trial.Passed)
            {
                passed++;
            }
        }

        return new TrialComparison
        {
            TrialCount = collection.Trials.Count,
            PassedCount = passed,
            AverageScore = totalScore / collection.Trials.Count,
            BestScore = bestScore,
            AverageTargetError = totalError / collection.Trials.Count
        };
    }

    private SurgicalTrialCollection Load()
    {
        if (!File.Exists(jsonPath))
        {
            return new SurgicalTrialCollection();
        }

        try
        {
            SurgicalTrialCollection loaded = JsonUtility.FromJson<SurgicalTrialCollection>(
                File.ReadAllText(jsonPath));
            if (loaded == null)
            {
                return new SurgicalTrialCollection();
            }

            foreach (SurgicalTrialRecord trial in loaded.Trials)
            {
                if (trial.Score > 0f && trial.Score <= 1f)
                {
                    trial.Score *= 100f;
                }
            }
            return loaded;
        }
        catch
        {
            return new SurgicalTrialCollection();
        }
    }

    private void ExportCsv()
    {
        using (StreamWriter writer = new StreamWriter(csvPath, false))
        {
            writer.WriteLine(
                "trial_id,timestamp_utc,operator,target,difficulty,control_mode," +
                "anatomy_motion,target_health,optimizer,duration,target_error," +
                "surface_clearance,max_deviation,manipulability,tool_changes," +
                "motion_cycles,collision_events,passed,score");
            foreach (SurgicalTrialRecord trial in collection.Trials)
            {
                writer.WriteLine(string.Join(",", new[]
                {
                    trial.TrialId,
                    trial.TimestampUtc,
                    Escape(trial.OperatorId),
                    Escape(trial.AnatomicalTarget),
                    trial.Difficulty,
                    trial.ControlMode,
                    trial.AnatomyMotion,
                    trial.TargetHealth,
                    trial.Optimizer,
                    Format(trial.Duration),
                    Format(trial.TargetError),
                    Format(trial.SurfaceClearance),
                    Format(trial.MaximumDeviation),
                    Format(trial.Manipulability),
                    trial.ToolChanges.ToString(CultureInfo.InvariantCulture),
                    trial.MotionCycles.ToString(CultureInfo.InvariantCulture),
                    trial.CollisionEvents.ToString(CultureInfo.InvariantCulture),
                    trial.Passed ? "true" : "false",
                    Format(trial.Score)
                }));
            }
        }
    }

    private static string Format(float value)
    {
        return value.ToString("F6", CultureInfo.InvariantCulture);
    }

    private static string Escape(string value)
    {
        return string.IsNullOrEmpty(value) ? "unknown" : value.Replace(",", "_");
    }
}

public sealed class TrialComparison
{
    public int TrialCount;
    public int PassedCount;
    public float AverageScore;
    public float BestScore;
    public float AverageTargetError;
}