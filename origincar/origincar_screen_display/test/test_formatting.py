import json

from origincar_screen_display.formatting import parse_qr_result, parse_vlm_result


def test_parse_qr_result_with_direction_and_position():
    assert parse_qr_result('123,顺时针,1.25,-0.50') == (
        '123', '顺时针  ·  位置 (1.25, -0.50)')


def test_parse_qr_result_accepts_plain_payload():
    assert parse_qr_result('HELLO') == ('HELLO', '')


def test_parse_vlm_json_payload():
    payload = json.dumps({
        'index': 56,
        'text': '立牌上写着安全第一。',
        'elapsed_s': 2.34,
        'model': 'qwen-test',
    }, ensure_ascii=False)
    assert parse_vlm_result(payload) == (
        '立牌上写着安全第一。', '路点 56  ·  2.3 秒  ·  qwen-test')


def test_parse_vlm_plain_text_payload():
    assert parse_vlm_result('一只红色的猫') == ('一只红色的猫', '')
