from rpycrsf.protocol import (
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
    CRSF_SYNC_BYTE,
    crc8,
    pack_rc_channels,
    unpack_rc_channels,
)


def test_crc8_known_value():
    # CRC-8/DVB-S2 of an empty message is 0.
    assert crc8(b"") == 0


def test_pack_unpack_roundtrip():
    channels = [172, 992, 1811, 992, 172, 1811] + [992] * 10
    frame = pack_rc_channels(channels)

    assert frame[0] == CRSF_SYNC_BYTE
    assert frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED
    assert frame[1] == len(frame) - 2  # length covers type + payload + crc

    payload = frame[3:-1]
    assert unpack_rc_channels(payload) == channels


def test_pack_frame_crc_validates():
    channels = [992] * 16
    frame = pack_rc_channels(channels)
    assert crc8(frame[2:-1]) == frame[-1]


def test_pack_rejects_wrong_channel_count():
    try:
        pack_rc_channels([992] * 15)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for wrong channel count")
