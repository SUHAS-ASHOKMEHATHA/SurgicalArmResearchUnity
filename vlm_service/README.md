# Read-Only VLM Bridge

This service forwards Unity camera frames to an OpenAI-compatible multimodal
model endpoint. It has no robot command API.

## Dry-run validation

```bash
/usr/bin/python3 server.py --dry-run
```

Dry-run mode validates transport and JSON parsing but does not perform VLM
inference. Dry-run predictions must not be included in research accuracy results.

## Model-backed mode

Configure an OpenAI-compatible endpoint through environment variables:

```bash
export VLM_BASE_URL="http://127.0.0.1:11434/v1"
export VLM_MODEL="your-multimodal-model"
export VLM_API_KEY="optional-provider-key"
/usr/bin/python3 server.py
```

The bridge listens on `http://127.0.0.1:8765` by default.

## Unity configuration

Unity creates `S7netUnityProject/VlmResults/vlm_config.json` on first run.
Dataset capture is enabled by default; inference is disabled. Stop Play mode,
set `inferenceEnabled` to `true`, start the bridge, then enter Play mode again.

Press `F9` during Play mode to toggle the optional VLM monitor. Ground-truth
labels are written locally and are never sent in the inference request.

## Tests

```bash
/usr/bin/python3 -m unittest discover -s vlm_service -p "test_*.py"
```