"""Pure formatting helpers kept separate from ROS and the GUI for testing."""

import json


def parse_qr_result(payload):
    """Return (code, detail) for qr_decision's CSV String payload."""
    text = str(payload).strip()
    if not text:
        return '', ''

    fields = [part.strip() for part in text.split(',')]
    code = fields[0]
    details = []
    if len(fields) >= 2 and fields[1]:
        details.append(fields[1])
    if len(fields) >= 4 and fields[2] and fields[3]:
        details.append(f'位置 ({fields[2]}, {fields[3]})')
    elif len(fields) >= 3:
        details.append('  '.join(part for part in fields[2:] if part))
    return code, '  ·  '.join(details)


def parse_vlm_result(payload):
    """Return (caption, detail) for JSON or plain-text VLM String payloads."""
    text = str(payload).strip()
    if not text:
        return '', ''

    try:
        body = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return text, ''

    if not isinstance(body, dict):
        return str(body), ''

    caption = str(body.get('text', '')).strip()
    details = []
    index = body.get('index')
    if index not in (None, '', -1):
        details.append(f'路点 {index}')
    elapsed = body.get('elapsed_s')
    if isinstance(elapsed, (int, float)):
        details.append(f'{elapsed:.1f} 秒')
    model = str(body.get('model', '')).strip()
    if model:
        details.append(model)
    return caption, '  ·  '.join(details)
