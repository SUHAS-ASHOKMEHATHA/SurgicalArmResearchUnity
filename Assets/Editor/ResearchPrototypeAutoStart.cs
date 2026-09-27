using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

[InitializeOnLoad]
public static class ResearchPrototypeAutoStart
{
    private const string SessionKey = "ResearchPrototypeVlmObserverFinal";
    private const string ScenePath = "Assets/Scenes/RobotControl.unity";

    static ResearchPrototypeAutoStart()
    {
        EditorApplication.delayCall += StartPrototype;
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.EnteredEditMode)
        {
            EditorApplication.delayCall += StartPrototype;
        }
        else if (state == PlayModeStateChange.EnteredPlayMode)
        {
            EditorApplication.delayCall += ShowGameView;
        }
    }

    private static void StartPrototype()
    {
        if (SessionState.GetBool(SessionKey, false))
        {
            return;
        }

        if (EditorApplication.isPlaying)
        {
            EditorApplication.isPlaying = false;
            return;
        }

        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            return;
        }

        if (EditorApplication.isCompiling || EditorApplication.isUpdating)
        {
            EditorApplication.delayCall += StartPrototype;
            return;
        }

        if (SceneManager.GetActiveScene().path != ScenePath)
        {
            EditorSceneManager.OpenScene(ScenePath);
        }

        SessionState.SetBool(SessionKey, true);
        EditorApplication.isPlaying = true;
    }

    private static void ShowGameView()
    {
        EditorApplication.ExecuteMenuItem("Window/General/Game");
    }

}