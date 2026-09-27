using UnityEngine;

public sealed class SurgicalToolVisualizer
{
    private readonly GameObject root;
    private SurgicalToolType activeTool = SurgicalToolType.None;
    private float transitionStartTime;

    public string ActiveToolName => GetDisplayName(activeTool);
    public SurgicalToolType ActiveToolType => activeTool;

    public SurgicalToolVisualizer()
    {
        root = new GameObject("Active Surgical Tool");
        root.SetActive(false);
    }

    public void SetTool(SurgicalToolType toolType)
    {
        if (activeTool == toolType)
        {
            return;
        }

        for (int index = root.transform.childCount - 1; index >= 0; index--)
        {
            Object.Destroy(root.transform.GetChild(index).gameObject);
        }

        activeTool = toolType;
        root.SetActive(toolType != SurgicalToolType.None);
        transitionStartTime = Time.time;
        root.transform.localScale = Vector3.zero;
        if (toolType == SurgicalToolType.None)
        {
            return;
        }

        switch (toolType)
        {
            case SurgicalToolType.NavigationProbe:
                CreateCylinder("Probe Shaft", new Vector3(0f, 0f, -0.12f), new Vector3(0.018f, 0.12f, 0.018f), new Color(0.65f, 0.85f, 1f));
                CreateSphere("Probe Tip", Vector3.zero, 0.035f, new Color(0.1f, 0.75f, 1f));
                break;
            case SurgicalToolType.MicroScalpel:
                CreateCylinder("Scalpel Handle", new Vector3(0f, 0f, -0.13f), new Vector3(0.025f, 0.1f, 0.025f), new Color(0.15f, 0.55f, 0.7f));
                CreateBox("Scalpel Blade", new Vector3(0f, 0f, -0.015f), new Vector3(0.025f, 0.008f, 0.075f), new Color(0.88f, 0.92f, 0.95f));
                break;
            case SurgicalToolType.IrrigationAspirator:
                CreateCylinder("Aspirator Tube", new Vector3(0f, 0f, -0.14f), new Vector3(0.03f, 0.14f, 0.03f), new Color(0.1f, 0.8f, 0.72f));
                CreateCylinder("Aspirator Nozzle", new Vector3(0f, 0f, -0.015f), new Vector3(0.014f, 0.045f, 0.014f), new Color(0.8f, 0.9f, 0.9f));
                break;
            case SurgicalToolType.MicroForceps:
                CreateCylinder("Forceps Left", new Vector3(-0.025f, 0f, -0.11f), new Vector3(0.012f, 0.12f, 0.012f), new Color(0.82f, 0.86f, 0.9f));
                CreateCylinder("Forceps Right", new Vector3(0.025f, 0f, -0.11f), new Vector3(0.012f, 0.12f, 0.012f), new Color(0.82f, 0.86f, 0.9f));
                break;
            case SurgicalToolType.ClosureInstrument:
                CreateCylinder("Closure Handle", new Vector3(0f, 0f, -0.12f), new Vector3(0.025f, 0.12f, 0.025f), new Color(0.72f, 0.32f, 0.85f));
                CreateBox("Closure Tip", Vector3.zero, new Vector3(0.07f, 0.02f, 0.035f), new Color(0.95f, 0.78f, 1f));
                break;
            case SurgicalToolType.Endoscope:
                CreateCylinder("Endoscope Shaft", new Vector3(0f, 0f, -0.14f), new Vector3(0.02f, 0.14f, 0.02f), new Color(0.12f, 0.2f, 0.25f));
                CreateSphere("Endoscope Lens", Vector3.zero, 0.036f, new Color(0.15f, 0.95f, 0.8f));
                break;
            case SurgicalToolType.SimulatedCautery:
                CreateCylinder("Cautery Handle", new Vector3(0f, 0f, -0.13f), new Vector3(0.024f, 0.12f, 0.024f), new Color(0.22f, 0.24f, 0.28f));
                CreateCylinder("Cautery Tip", new Vector3(0f, 0f, -0.015f), new Vector3(0.01f, 0.05f, 0.01f), new Color(1f, 0.35f, 0.08f));
                break;
        }
    }

    public void UpdatePose(Vector3 tipPosition, Vector3 targetPosition, Vector3 fallbackDirection)
    {
        Vector3 direction = targetPosition - tipPosition;
        if (direction.sqrMagnitude < 0.000001f)
        {
            direction = fallbackDirection;
        }

        root.transform.position = tipPosition;
        root.transform.rotation = Quaternion.LookRotation(direction.normalized, Vector3.up);
        float transition = Mathf.SmoothStep(
            0f,
            1f,
            Mathf.Clamp01((Time.time - transitionStartTime) / 0.35f));
        root.transform.localScale = Vector3.one * transition;
        root.transform.Rotate(0f, (1f - transition) * 35f, 0f, Space.Self);
    }

    public void Destroy()
    {
        if (root != null)
        {
            Object.Destroy(root);
        }
    }

    public static string GetDisplayName(SurgicalToolType toolType)
    {
        switch (toolType)
        {
            case SurgicalToolType.NavigationProbe:
                return "Navigation probe";
            case SurgicalToolType.MicroScalpel:
                return "Micro-scalpel";
            case SurgicalToolType.IrrigationAspirator:
                return "Irrigation / aspiration tool";
            case SurgicalToolType.MicroForceps:
                return "Micro-forceps";
            case SurgicalToolType.ClosureInstrument:
                return "Closure instrument";
            case SurgicalToolType.Endoscope:
                return "Camera / endoscope";
            case SurgicalToolType.SimulatedCautery:
                return "Simulated cautery";
            default:
                return "No tool";
        }
    }

    private void CreateCylinder(string objectName, Vector3 localPosition, Vector3 localScale, Color color)
    {
        GameObject part = CreatePart(objectName, PrimitiveType.Cylinder, localPosition, localScale, color);
        part.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
    }

    private void CreateSphere(string objectName, Vector3 localPosition, float scale, Color color)
    {
        CreatePart(objectName, PrimitiveType.Sphere, localPosition, Vector3.one * scale, color);
    }

    private void CreateBox(string objectName, Vector3 localPosition, Vector3 localScale, Color color)
    {
        CreatePart(objectName, PrimitiveType.Cube, localPosition, localScale, color);
    }

    private GameObject CreatePart(
        string objectName,
        PrimitiveType primitiveType,
        Vector3 localPosition,
        Vector3 localScale,
        Color color)
    {
        GameObject part = GameObject.CreatePrimitive(primitiveType);
        part.name = objectName;
        part.transform.SetParent(root.transform, false);
        part.transform.localPosition = localPosition;
        part.transform.localScale = localScale;
        Collider collider = part.GetComponent<Collider>();
        collider.enabled = false;

        Renderer renderer = part.GetComponent<Renderer>();
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader != null)
        {
            renderer.material = new Material(shader);
        }
        renderer.material.color = color;
        return part;
    }
}

public enum SurgicalToolType
{
    None,
    NavigationProbe,
    MicroScalpel,
    IrrigationAspirator,
    MicroForceps,
    ClosureInstrument,
    Endoscope,
    SimulatedCautery
}