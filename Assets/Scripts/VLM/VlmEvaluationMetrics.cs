using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

public sealed class VlmEvaluationMetrics
{
    private static readonly HashSet<string> KnownTargets = new HashSet<string>
    {
        "none", "brain", "eye", "heart", "spine", "abdomen",
        "left shoulder", "right shoulder", "left elbow", "right elbow",
        "left hand", "right hand", "left knee", "right knee"
    };

    private static readonly HashSet<string> KnownTools = new HashSet<string>
    {
        "no tool", "camera / endoscope", "navigation probe", "micro-scalpel",
        "simulated cautery", "irrigation / aspiration tool", "micro-forceps",
        "closure instrument"
    };

    private static readonly HashSet<string> KnownContacts = new HashSet<string>
    {
        "clear", "caution", "contact"
    };

    private readonly List<VlmEvaluationRecord> records = new List<VlmEvaluationRecord>();
    private readonly string csvPath;
    private readonly string summaryPath;

    public int Count => records.Count;
    public VlmEvaluationSummary Summary => CalculateSummary();

    public VlmEvaluationMetrics(string outputDirectory)
    {
        Directory.CreateDirectory(outputDirectory);
        csvPath = Path.Combine(outputDirectory, "vlm_evaluations.csv");
        summaryPath = Path.Combine(outputDirectory, "vlm_summary.json");
    }

    public VlmEvaluationRecord Evaluate(
        VlmGroundTruthSnapshot truth,
        VlmPrediction prediction,
        string imagePath,
        string truthPath,
        string predictionPath)
    {
        string expectedTarget = Normalize(truth.anatomicalTarget);
        string expectedTool = Normalize(truth.activeTool);
        string expectedStage = Normalize(truth.procedureStage);
        string expectedContact = Normalize(truth.contactState);
        string predictedTarget = Normalize(prediction.target);
        string predictedTool = Normalize(prediction.tool);
        string predictedStage = Normalize(prediction.stage);
        string predictedContact = Normalize(prediction.contactState);

        bool hallucination = !KnownTargets.Contains(predictedTarget)
            || !KnownTools.Contains(predictedTool)
            || !KnownContacts.Contains(predictedContact)
            || string.IsNullOrWhiteSpace(predictedStage);

        VlmEvaluationRecord record = new VlmEvaluationRecord
        {
            frameId = prediction.frameId,
            timestampUtc = truth.timestampUtc,
            imagePath = imagePath,
            groundTruthPath = truthPath,
            predictionPath = predictionPath,
            expectedTarget = truth.anatomicalTarget,
            predictedTarget = prediction.target,
            expectedTool = truth.activeTool,
            predictedTool = prediction.tool,
            expectedStage = truth.procedureStage,
            predictedStage = prediction.stage,
            expectedContactState = truth.contactState,
            predictedContactState = prediction.contactState,
            targetCorrect = expectedTarget == predictedTarget,
            toolCorrect = expectedTool == predictedTool,
            stageCorrect = LabelsMatch(expectedStage, predictedStage),
            contactCorrect = expectedContact == predictedContact,
            validJson = prediction.validJson,
            hallucinationDetected = hallucination,
            confidence = Mathf.Clamp01(prediction.confidence),
            latencyMs = Mathf.Max(0f, prediction.latencyMs)
        };
        records.Add(record);
        Export();
        return record;
    }

    private void Export()
    {
        using (StreamWriter writer = new StreamWriter(csvPath, false))
        {
            writer.WriteLine(
                "frame_id,timestamp_utc,image_path,expected_target,predicted_target," +
                "target_correct,expected_tool,predicted_tool,tool_correct," +
                "expected_stage,predicted_stage,stage_correct,expected_contact," +
                "predicted_contact,contact_correct,valid_json,hallucination," +
                "confidence,latency_ms");
            foreach (VlmEvaluationRecord record in records)
            {
                writer.WriteLine(string.Join(",", new[]
                {
                    Escape(record.frameId),
                    Escape(record.timestampUtc),
                    Escape(record.imagePath),
                    Escape(record.expectedTarget),
                    Escape(record.predictedTarget),
                    Bool(record.targetCorrect),
                    Escape(record.expectedTool),
                    Escape(record.predictedTool),
                    Bool(record.toolCorrect),
                    Escape(record.expectedStage),
                    Escape(record.predictedStage),
                    Bool(record.stageCorrect),
                    Escape(record.expectedContactState),
                    Escape(record.predictedContactState),
                    Bool(record.contactCorrect),
                    Bool(record.validJson),
                    Bool(record.hallucinationDetected),
                    Number(record.confidence),
                    Number(record.latencyMs)
                }));
            }
        }

        File.WriteAllText(summaryPath, JsonUtility.ToJson(CalculateSummary(), true));
    }

    private VlmEvaluationSummary CalculateSummary()
    {
        if (records.Count == 0)
        {
            return new VlmEvaluationSummary();
        }

        return new VlmEvaluationSummary
        {
            sampleCount = records.Count,
            targetAccuracy = Ratio(record => record.targetCorrect),
            toolAccuracy = Ratio(record => record.toolCorrect),
            stageAccuracy = Ratio(record => record.stageCorrect),
            contactAccuracy = Ratio(record => record.contactCorrect),
            targetMacroF1 = MacroF1(
                records.Select(record => Normalize(record.expectedTarget)),
                records.Select(record => Normalize(record.predictedTarget))),
            toolMacroF1 = MacroF1(
                records.Select(record => Normalize(record.expectedTool)),
                records.Select(record => Normalize(record.predictedTool))),
            validJsonRate = Ratio(record => record.validJson),
            hallucinationRate = Ratio(record => record.hallucinationDetected),
            averageConfidence = records.Average(record => record.confidence),
            averageLatencyMs = records.Average(record => record.latencyMs)
        };
    }

    private float Ratio(Func<VlmEvaluationRecord, bool> predicate)
    {
        return records.Count(record => predicate(record)) / (float)records.Count;
    }

    private static float MacroF1(IEnumerable<string> expected, IEnumerable<string> predicted)
    {
        string[] expectedValues = expected.ToArray();
        string[] predictedValues = predicted.ToArray();
        HashSet<string> labels = new HashSet<string>(expectedValues);
        labels.UnionWith(predictedValues);
        labels.Remove(string.Empty);
        if (labels.Count == 0)
        {
            return 0f;
        }

        float total = 0f;
        foreach (string label in labels)
        {
            int truePositive = 0;
            int falsePositive = 0;
            int falseNegative = 0;
            for (int index = 0; index < expectedValues.Length; index++)
            {
                bool expectedMatch = expectedValues[index] == label;
                bool predictedMatch = predictedValues[index] == label;
                if (expectedMatch && predictedMatch)
                {
                    truePositive++;
                }
                else if (!expectedMatch && predictedMatch)
                {
                    falsePositive++;
                }
                else if (expectedMatch)
                {
                    falseNegative++;
                }
            }

            float precision = truePositive / (float)Mathf.Max(1, truePositive + falsePositive);
            float recall = truePositive / (float)Mathf.Max(1, truePositive + falseNegative);
            total += precision + recall > 0f
                ? 2f * precision * recall / (precision + recall)
                : 0f;
        }
        return total / labels.Count;
    }

    private static bool LabelsMatch(string expected, string predicted)
    {
        return expected == predicted
            || (!string.IsNullOrEmpty(expected)
                && !string.IsNullOrEmpty(predicted)
                && (expected.Contains(predicted) || predicted.Contains(expected)));
    }

    private static string Normalize(string value)
    {
        return string.IsNullOrWhiteSpace(value)
            ? string.Empty
            : value.Trim().ToLowerInvariant().Replace("_", " ");
    }

    private static string Bool(bool value)
    {
        return value ? "true" : "false";
    }

    private static string Number(float value)
    {
        return value.ToString("F6", CultureInfo.InvariantCulture);
    }

    private static string Escape(string value)
    {
        string safeValue = value ?? string.Empty;
        return $"\"{safeValue.Replace("\"", "\"\"")}\"";
    }
}

[Serializable]
public sealed class VlmEvaluationSummary
{
    public int sampleCount;
    public float targetAccuracy;
    public float toolAccuracy;
    public float stageAccuracy;
    public float contactAccuracy;
    public float targetMacroF1;
    public float toolMacroF1;
    public float validJsonRate;
    public float hallucinationRate;
    public float averageConfidence;
    public float averageLatencyMs;
}