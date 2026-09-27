using System;
using System.Collections;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.SceneManagement;

[Serializable]
public sealed class VlmObserverSettings
{
    public bool captureEnabled = true;
    public bool inferenceEnabled;
    public bool captureStateChanges = true;
    public bool captureIdleFrames;
    public bool showOverlay;
    public string endpoint = "http://127.0.0.1:8765/infer";
    public float captureIntervalSeconds = 2f;
    public int captureWidth = 768;
    public int captureHeight = 432;
    public int jpegQuality = 88;
    public int maximumFramesPerSession = 500;

    public void Validate()
    {
        captureIntervalSeconds = Mathf.Clamp(captureIntervalSeconds, 0.25f, 30f);
        captureWidth = Mathf.Clamp(captureWidth, 320, 1920);
        captureHeight = Mathf.Clamp(captureHeight, 180, 1080);
        jpegQuality = Mathf.Clamp(jpegQuality, 40, 100);
        maximumFramesPerSession = Mathf.Clamp(maximumFramesPerSession, 1, 10000);
        if (string.IsNullOrWhiteSpace(endpoint))
        {
            endpoint = "http://127.0.0.1:8765/infer";
        }
    }
}

public sealed class VlmSceneObserver : MonoBehaviour
{
    private const string Prompt =
        "Act as a read-only observer of a simulated surgical robot scene. " +
        "Identify the anatomical target, active tool, procedure stage, contact state, " +
        "and any visible safety concern. Allowed targets: none, brain, eye, heart, " +
        "spine, abdomen, left shoulder, right shoulder, left elbow, right elbow, " +
        "left hand, right hand, left knee, right knee. Allowed contact states: " +
        "clear, caution, contact. Return only one JSON object with fields target, " +
        "tool, stage, contactState, safetyConcern, and confidence.";

    private SurgicalArmResearchController controller;
    private Camera observerCamera;
    private VlmObserverSettings settings;
    private VlmEvaluationMetrics metrics;
    private string outputDirectory;
    private string framesDirectory;
    private string groundTruthDirectory;
    private string predictionsDirectory;
    private string configPath;
    private string previousSignature = string.Empty;
    private string lastStatus = "Waiting for scene";
    private string lastFrameId = "none";
    private float nextCaptureTime;
    private bool captureInProgress;
    private bool overlayVisible;
    private int frameCounter;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Install()
    {
        if (SceneManager.GetActiveScene().name != "RobotControl"
            || FindAnyObjectByType<VlmSceneObserver>() != null)
        {
            return;
        }

        GameObject observerObject = new GameObject("Read-Only VLM Observer");
        observerObject.AddComponent<VlmSceneObserver>();
    }

    private IEnumerator Start()
    {
        outputDirectory = Path.GetFullPath(Path.Combine(
            Application.dataPath,
            "..",
            "VlmResults"));
        framesDirectory = Path.Combine(outputDirectory, "frames");
        groundTruthDirectory = Path.Combine(outputDirectory, "ground_truth");
        predictionsDirectory = Path.Combine(outputDirectory, "predictions");
        configPath = Path.Combine(outputDirectory, "vlm_config.json");
        Directory.CreateDirectory(framesDirectory);
        Directory.CreateDirectory(groundTruthDirectory);
        Directory.CreateDirectory(predictionsDirectory);
        settings = LoadOrCreateSettings();
        metrics = new VlmEvaluationMetrics(outputDirectory);
        overlayVisible = settings.showOverlay;

        while (controller == null || observerCamera == null)
        {
            controller = FindAnyObjectByType<SurgicalArmResearchController>();
            observerCamera = FindAnyObjectByType<Camera>();
            yield return null;
        }

        lastStatus = settings.inferenceEnabled
            ? "Observer ready; VLM bridge enabled"
            : "Dataset capture ready; inference disabled";
        nextCaptureTime = Time.time;
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F9))
        {
            overlayVisible = !overlayVisible;
        }

        if (controller == null || observerCamera == null || settings == null
            || !settings.captureEnabled || captureInProgress
            || frameCounter >= settings.maximumFramesPerSession)
        {
            return;
        }

        VlmGroundTruthSnapshot snapshot = controller.CreateVlmGroundTruthSnapshot();
        string signature = snapshot.StateSignature();
        bool stateChanged = settings.captureStateChanges && signature != previousSignature;
        bool intervalReached = Time.time >= nextCaptureTime
            && (snapshot.procedureRunning || settings.captureIdleFrames);
        if (!stateChanged && !intervalReached)
        {
            return;
        }

        previousSignature = signature;
        nextCaptureTime = Time.time + settings.captureIntervalSeconds;
        StartCoroutine(CaptureAndEvaluate(snapshot));
    }

    private IEnumerator CaptureAndEvaluate(VlmGroundTruthSnapshot snapshot)
    {
        captureInProgress = true;
        yield return new WaitForEndOfFrame();

        string frameId = $"frame_{DateTime.UtcNow:yyyyMMdd_HHmmss_fff}_{frameCounter:D5}";
        frameCounter++;
        lastFrameId = frameId;
        string imagePath = Path.Combine(framesDirectory, frameId + ".jpg");
        string truthPath = Path.Combine(groundTruthDirectory, frameId + ".json");
        string predictionPath = Path.Combine(predictionsDirectory, frameId + ".json");

        byte[] imageBytes = CaptureCameraJpeg();
        if (imageBytes == null || imageBytes.Length == 0)
        {
            lastStatus = "Frame capture failed";
            captureInProgress = false;
            yield break;
        }

        File.WriteAllBytes(imagePath, imageBytes);
        File.WriteAllText(truthPath, JsonUtility.ToJson(snapshot, true));
        if (!settings.inferenceEnabled)
        {
            lastStatus = $"Saved dataset frame {frameCounter}";
            captureInProgress = false;
            yield break;
        }

        VlmInferenceRequest request = new VlmInferenceRequest
        {
            frameId = frameId,
            imageBase64 = Convert.ToBase64String(imageBytes),
            prompt = Prompt
        };
        byte[] requestBytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(request));
        using (UnityWebRequest webRequest = new UnityWebRequest(settings.endpoint, "POST"))
        {
            webRequest.uploadHandler = new UploadHandlerRaw(requestBytes);
            webRequest.downloadHandler = new DownloadHandlerBuffer();
            webRequest.SetRequestHeader("Content-Type", "application/json");
            yield return webRequest.SendWebRequest();

            if (webRequest.result != UnityWebRequest.Result.Success)
            {
                lastStatus = $"VLM bridge unavailable: {webRequest.error}";
                captureInProgress = false;
                yield break;
            }

            string responseJson = webRequest.downloadHandler.text;
            VlmPrediction prediction;
            try
            {
                prediction = JsonUtility.FromJson<VlmPrediction>(responseJson);
                if (prediction == null)
                {
                    throw new InvalidDataException("Empty VLM response.");
                }
                prediction.validJson = true;
            }
            catch (Exception exception)
            {
                prediction = new VlmPrediction
                {
                    frameId = frameId,
                    validJson = false,
                    rawResponse = responseJson,
                    model = "unparsed"
                };
                lastStatus = $"Invalid VLM JSON: {exception.Message}";
            }

            prediction.frameId = frameId;
            prediction.rawResponse = responseJson;
            File.WriteAllText(predictionPath, JsonUtility.ToJson(prediction, true));
            if (prediction.model == "dry-run-no-vlm")
            {
                lastStatus = "Dry-run bridge test passed; excluded from VLM metrics";
                captureInProgress = false;
                yield break;
            }

            VlmEvaluationRecord record = metrics.Evaluate(
                snapshot,
                prediction,
                imagePath,
                truthPath,
                predictionPath);
            lastStatus = record.hallucinationDetected
                ? "Prediction evaluated; hallucination flagged"
                : "Prediction evaluated successfully";
        }

        captureInProgress = false;
    }

    private byte[] CaptureCameraJpeg()
    {
        RenderTexture previousTarget = observerCamera.targetTexture;
        RenderTexture previousActive = RenderTexture.active;
        Rect previousRect = observerCamera.rect;
        RenderTexture renderTexture = RenderTexture.GetTemporary(
            settings.captureWidth,
            settings.captureHeight,
            24,
            RenderTextureFormat.ARGB32);
        Texture2D texture = new Texture2D(
            settings.captureWidth,
            settings.captureHeight,
            TextureFormat.RGB24,
            false);

        try
        {
            observerCamera.rect = new Rect(0f, 0f, 1f, 1f);
            observerCamera.targetTexture = renderTexture;
            RenderTexture.active = renderTexture;
            observerCamera.Render();
            texture.ReadPixels(
                new Rect(0f, 0f, settings.captureWidth, settings.captureHeight),
                0,
                0);
            texture.Apply(false, false);
            return texture.EncodeToJPG(settings.jpegQuality);
        }
        finally
        {
            observerCamera.targetTexture = previousTarget;
            observerCamera.rect = previousRect;
            RenderTexture.active = previousActive;
            RenderTexture.ReleaseTemporary(renderTexture);
            Destroy(texture);
        }
    }

    private VlmObserverSettings LoadOrCreateSettings()
    {
        VlmObserverSettings loaded = null;
        if (File.Exists(configPath))
        {
            try
            {
                loaded = JsonUtility.FromJson<VlmObserverSettings>(File.ReadAllText(configPath));
            }
            catch (Exception exception)
            {
                Debug.LogWarning($"VLM config could not be read: {exception.Message}");
            }
        }

        loaded = loaded ?? new VlmObserverSettings();
        loaded.Validate();
        File.WriteAllText(configPath, JsonUtility.ToJson(loaded, true));
        return loaded;
    }

    private void OnGUI()
    {
        if (!overlayVisible || settings == null)
        {
            return;
        }

        VlmEvaluationSummary summary = metrics != null
            ? metrics.Summary
            : new VlmEvaluationSummary();
        float width = Mathf.Min(360f, Screen.width - 24f);
        GUILayout.BeginArea(
            new Rect(Screen.width - width - 12f, Screen.height - 190f, width, 178f),
            GUI.skin.box);
        GUILayout.Label("VLM SUPERVISORY OBSERVER");
        GUILayout.Label(lastStatus);
        GUILayout.Label($"Frame: {lastFrameId} | Captured: {frameCounter}");
        GUILayout.Label($"Evaluated: {summary.sampleCount} | Valid JSON: {summary.validJsonRate:P1}");
        GUILayout.Label($"Target: {summary.targetAccuracy:P1} | Tool: {summary.toolAccuracy:P1}");
        GUILayout.Label($"Stage: {summary.stageAccuracy:P1} | Contact: {summary.contactAccuracy:P1}");
        GUILayout.Label($"Hallucination rate: {summary.hallucinationRate:P1}");
        GUILayout.Label("Read-only observer; no robot command channel");
        GUILayout.EndArea();
    }
}