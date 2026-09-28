import csv
import json
import sys
import tempfile
import types
from pathlib import Path

import numpy as np

try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    cv2_stub = types.ModuleType("cv2")
    cv2_stub.FONT_HERSHEY_SIMPLEX = 0
    cv2_stub.LINE_AA = 0

    def rectangle(image, start, end, colour, thickness):
        del thickness
        x1, y1 = start
        x2, y2 = end
        image[y1:y2 + 1, x1] = colour
        image[y1:y2 + 1, x2] = colour
        image[y1, x1:x2 + 1] = colour
        image[y2, x1:x2 + 1] = colour

    def circle(image, centre, radius, colour, thickness):
        del radius, thickness
        image[centre[1], centre[0]] = colour

    def put_text(image, *_args, **_kwargs):
        image[0, 0] = 255

    cv2_stub.rectangle = rectangle
    cv2_stub.circle = circle
    cv2_stub.putText = put_text
    sys.modules["cv2"] = cv2_stub

from lib import session_replay


def timeline_latest_and_nearest():
    events = [{"time": 0.1, "value": 1}, {"time": 0.3, "value": 2}]
    timeline = session_replay.EventTimeline(events, "time")
    assert timeline.latest(0.05) is None
    assert timeline.latest(0.2)["value"] == 1
    assert timeline.nearest(0.29, 0.02)["value"] == 2
    assert timeline.nearest(0.2, 0.05) is None


def loads_session_and_converts_legacy_field_order():
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        (directory / "metadata.json").write_text(
            json.dumps({"schema_version": 1}),
            encoding="utf-8",
        )
        with (directory / "game.csv").open(
            "w",
            encoding="utf-8",
            newline="",
        ) as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=(
                    "elapsed_s",
                    "x",
                    "y",
                    "yaw",
                    "ball_x",
                    "ball_y",
                    "ball_captured",
                    "bot_mode",
                    "steering_state",
                    "direction",
                    "speed",
                    "rotation",
                    "kick",
                    "dribbler",
                ),
            )
            writer.writeheader()
            writer.writerow(
                {
                    "elapsed_s": 1.5,
                    "x": 10,
                    "y": 20,
                    "yaw": 30,
                    "ball_x": "",
                    "ball_y": "",
                    "ball_captured": False,
                    "bot_mode": "DEFENCE",
                    "steering_state": True,
                    "direction": 40,
                    "speed": 500,
                    "rotation": 50,
                    "kick": False,
                    "dribbler": True,
                }
            )
        with (directory / "detections.csv").open(
            "w",
            encoding="utf-8",
            newline="",
        ) as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=(
                    "elapsed_s",
                    "video_time_s",
                    "detected",
                    "bbox_x",
                    "bbox_y",
                    "bbox_w",
                    "bbox_h",
                    "centre_x",
                    "centre_y",
                    "confidence",
                ),
            )
            writer.writeheader()
            writer.writerow(
                {
                    "elapsed_s": 1.5,
                    "video_time_s": 1.0,
                    "detected": False,
                }
            )

        session = session_replay.load_recorded_session(directory)
        tokens = session_replay.game_event_tokens(session.game_events[0])
        assert tokens[:3] == ["10", "20", "30"]
        assert not session.detection_events[0]["detected"]
        assert session.metadata["video_start_elapsed_s"] == 0.5


def ignores_rows_truncated_by_power_loss():
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        (directory / "metadata.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "video_file": "video.ts",
                    "game_file": "game.csv",
                    "detections_file": "detections.csv",
                    "finalized": False,
                }
            ),
            encoding="utf-8",
        )
        # Use the actual field names; the second data row ends mid-write.
        game_fields = (
            "elapsed_s,x,y,yaw,ball_x,ball_y,ball_captured,bot_mode,"
            "steering_state,direction,speed,rotation,kick,dribbler\n"
        )
        (directory / "game.csv").write_text(
            game_fields + "0.1,1,2,3,None,None,False,DEFENCE,False,0,0,0,False,False\n0.2,1",
            encoding="utf-8",
        )
        (directory / "detections.csv").write_text(
            "elapsed_s,video_time_s,detected\n0.1,0.0,False\n0.2",
            encoding="utf-8",
        )

        session = session_replay.load_recorded_session(directory)
        assert len(session.game_events) == 1
        assert len(session.detection_events) == 1


def annotation_does_not_modify_source_frame():
    source = np.zeros((80, 100, 3), dtype=np.uint8)
    annotated = session_replay.annotate_video_frame(
        source,
        {
            "detected": True,
            "bbox_x": 10,
            "bbox_y": 10,
            "bbox_w": 20,
            "bbox_h": 20,
            "centre_x": 20,
            "centre_y": 20,
            "confidence": 0.9,
        },
    )
    assert not np.any(source)
    assert np.any(annotated)


def annotation_draws_all_bots_without_a_ball():
    source = np.zeros((140, 160, 3), dtype=np.uint8)
    annotated = session_replay.annotate_video_frame(
        source,
        {
            "detected": False,
            "bots": [
                {"bbox": [20, 60, 20, 40], "centre": [30, 80], "confidence": 0.8},
                {"bbox": [90, 60, 20, 40], "centre": [100, 80], "confidence": 0.7},
            ],
        },
    )
    assert not np.any(source)
    assert np.all(annotated[80, 30] == 255)
    assert np.all(annotated[80, 100] == 255)
    assert np.any(annotated[100, 20])
    assert np.any(annotated[100, 90])


def test_video_reader_steps_backward_through_variable_frame_times():
    from fractions import Fraction

    import av

    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "frames.mkv"
        with av.open(str(path), mode="w") as output:
            stream = output.add_stream("ffv1", rate=30)
            stream.width = 16
            stream.height = 16
            stream.pix_fmt = "bgr0"
            stream.time_base = Fraction(1, 1000)
            stream.codec_context.time_base = Fraction(1, 1000)
            for index, timestamp in enumerate([2000, 2100, 2110, 2120, 2500]):
                pixels = np.full((16, 16, 3), index * 40, dtype=np.uint8)
                frame = av.VideoFrame.from_ndarray(pixels, format="rgb24")
                frame.pts = timestamp
                frame.time_base = Fraction(1, 1000)
                for packet in stream.encode(frame):
                    output.mux(packet)
            for packet in stream.encode():
                output.mux(packet)

        reader = session_replay.VideoReader(path)
        try:
            assert reader.start_time > 0
            assert len(reader.frame_times) == 5
            # Forward, backward, repeated, and forward again, including EOF.
            for index in [0, 1, 2, 3, 4, 3, 2, 1, 0, 0, 1, 4, 4, 3]:
                pixels = reader.frame_at(reader.frame_times[index])
                assert np.all(pixels == index * 40), index
        finally:
            reader.close()


def test_video_reader_rewinds_h264_transport_stream():
    from fractions import Fraction

    import av

    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "video.ts"
        with av.open(str(path), mode="w", format="mpegts") as output:
            stream = output.add_stream("libx264", rate=30)
            stream.width = 32
            stream.height = 32
            stream.pix_fmt = "yuv420p"
            stream.codec_context.gop_size = 30
            stream.codec_context.max_b_frames = 0
            stream.options = {"sc_threshold": "0"}
            for index in range(120):
                pixels = np.full((32, 32, 3), index * 2, dtype=np.uint8)
                frame = av.VideoFrame.from_ndarray(pixels, format="rgb24")
                frame.pts = index
                frame.time_base = Fraction(1, 30)
                for packet in stream.encode(frame):
                    output.mux(packet)
            for packet in stream.encode():
                output.mux(packet)

        with av.open(str(path)) as source:
            expected = [frame.to_ndarray(format="rgb24") for frame in source.decode(video=0)]
        reader = session_replay.VideoReader(path)
        try:
            assert len(reader.frame_times) == len(expected)
            for index in [*range(80), *range(78, 20, -1), 100, 99, 0, 1]:
                actual = reader.frame_at(reader.frame_times[index])
                np.testing.assert_array_equal(actual, expected[index], err_msg=f"frame {index}")
        finally:
            reader.close()
