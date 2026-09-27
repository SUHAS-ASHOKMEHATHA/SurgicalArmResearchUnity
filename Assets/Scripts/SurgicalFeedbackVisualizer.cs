using UnityEngine;

public sealed class SurgicalFeedbackVisualizer
{
    private readonly GameObject root;
    private readonly LineRenderer safetyRing;
    private readonly Material ringMaterial;
    private readonly AudioSource audioSource;
    private readonly AudioClip cautionClip;
    private readonly AudioClip contactClip;
    private int previousSeverity = -1;

    public SurgicalFeedbackVisualizer(GameObject host)
    {
        root = new GameObject("Surgical Safety Feedback");
        safetyRing = root.AddComponent<LineRenderer>();
        safetyRing.useWorldSpace = true;
        safetyRing.loop = true;
        safetyRing.positionCount = 64;
        safetyRing.startWidth = 0.01f;
        safetyRing.endWidth = 0.01f;

        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null)
        {
            shader = Shader.Find("Sprites/Default");
        }
        if (shader != null)
        {
            ringMaterial = new Material(shader);
            safetyRing.material = ringMaterial;
        }
        safetyRing.enabled = false;

        audioSource = host.AddComponent<AudioSource>();
        audioSource.playOnAwake = false;
        audioSource.spatialBlend = 0f;
        audioSource.volume = 0.18f;
        cautionClip = CreateTone("Caution Tone", 540f, 0.08f);
        contactClip = CreateTone("Contact Tone", 820f, 0.12f);
    }

    public void Update(
        Vector3 center,
        float radius,
        bool visible,
        int severity,
        bool warningSoundEnabled)
    {
        safetyRing.enabled = visible;
        if (visible)
        {
            for (int index = 0; index < safetyRing.positionCount; index++)
            {
                float angle = index / (float)safetyRing.positionCount * Mathf.PI * 2f;
                safetyRing.SetPosition(
                    index,
                    center + new Vector3(Mathf.Cos(angle), 0.008f, Mathf.Sin(angle)) * radius);
            }

            Color color = severity >= 2
                ? new Color(1f, 0.1f, 0.05f, 0.95f)
                : severity == 1
                    ? new Color(1f, 0.72f, 0.08f, 0.9f)
                    : new Color(0.1f, 0.85f, 0.45f, 0.75f);
            safetyRing.startColor = color;
            safetyRing.endColor = color;
        }

        if (warningSoundEnabled && severity > previousSeverity && severity > 0)
        {
            audioSource.PlayOneShot(severity >= 2 ? contactClip : cautionClip);
        }
        previousSeverity = severity;
    }

    public void ResetState()
    {
        previousSeverity = -1;
        safetyRing.enabled = false;
    }

    public void Destroy()
    {
        if (root != null)
        {
            Object.Destroy(root);
        }
        if (ringMaterial != null)
        {
            Object.Destroy(ringMaterial);
        }
        if (cautionClip != null)
        {
            Object.Destroy(cautionClip);
        }
        if (contactClip != null)
        {
            Object.Destroy(contactClip);
        }
    }

    private static AudioClip CreateTone(string clipName, float frequency, float duration)
    {
        const int sampleRate = 44100;
        int sampleCount = Mathf.CeilToInt(sampleRate * duration);
        float[] samples = new float[sampleCount];
        for (int index = 0; index < sampleCount; index++)
        {
            float envelope = 1f - index / (float)sampleCount;
            samples[index] = Mathf.Sin(2f * Mathf.PI * frequency * index / sampleRate)
                * envelope * 0.22f;
        }

        AudioClip clip = AudioClip.Create(clipName, sampleCount, 1, sampleRate, false);
        clip.SetData(samples, 0);
        return clip;
    }
}