"""Record a BeamNG camera at a fixed video frame rate."""

import math
import os
from pathlib import Path
import shutil
import subprocess
import logging

logger = logging.getLogger("BeamNG Video Recorder")

DEFAULT_VIDEO_RESOLUTION = (1280, 720)
DEFAULT_FIELD_OF_VIEW_Y = 70.0


class BeamNGVideoRecorder:
    """Capture and encode frames without advancing the simulation.

    Call :meth:`record_frame` whenever the owner is ready to advance BeamNG.
    The method captures the current simulation state and returns the number of
    simulation steps that the owner should advance before calling it again.
    """

    def __init__(
        self,
        beamng,
        vehicle,
        beamng_steps_per_second,
        video_fps,
        video_path,
        video_resolution=DEFAULT_VIDEO_RESOLUTION,
        ffmpeg_path=None,
        camera_name="ego_video_camera",
    ):
        self.beamng_steps_per_second = _positive_finite_float(
            beamng_steps_per_second,
            "beamng_steps_per_second",
        )
        self.video_fps = _positive_finite_float(video_fps, "video_fps")
        if self.video_fps > self.beamng_steps_per_second:
            raise ValueError(
                "video_fps cannot exceed beamng_steps_per_second; "
                "each video frame needs a distinct simulation step"
            )

        resolution = tuple(video_resolution)
        if len(resolution) != 2 or any(int(value) <= 0 for value in resolution):
            raise ValueError("video_resolution must contain a positive width and height")
        self.video_resolution = tuple(int(value) for value in resolution)

        self.video_path = Path(video_path)
        if not self.video_path.suffix:
            self.video_path = self.video_path.with_suffix(".mp4")
        self.frame_dir = self.video_path.with_name(f"{self.video_path.stem}_frames")
        self.ffmpeg_path = ffmpeg_path

        self.frame_count = 0
        self.capture_count = 0
        self._camera_removed = False
        self._finalized = False
        self._artifact_path = None
        self._camera = _create_camera(
            camera_name=camera_name,
            beamng=beamng,
            vehicle=vehicle,
            video_fps=self.video_fps,
            video_resolution=self.video_resolution,
        )

    def record_frame(self):
        """Capture the current frame and return steps to the next frame.

        This method never calls ``beamng.step``. A capture attempt still moves
        the schedule forward when BeamNG does not return a colour image, so a
        temporarily unavailable frame does not change the video's timing.
        """
        if self._finalized:
            raise RuntimeError("cannot record a frame after finalizing the video")

        self.frame_dir.mkdir(parents=True, exist_ok=True)
        images = self._camera.poll()
        colour = images.get("colour")
        if colour is not None:
            frame = colour.convert("RGB")
            frame.save(self.frame_dir / f"frame_{self.frame_count:06d}.png")
            self.frame_count += 1
        else:
            logger.error("Unable to get frame!")

        current_target_step = self._target_step(self.capture_count)
        self.capture_count += 1
        next_target_step = self._target_step(self.capture_count)
        return next_target_step - current_target_step

    def finalize(self):
        """Remove the camera, encode captured frames, and return the artifact path.

        If neither ffmpeg nor imageio can encode the MP4, the returned artifact
        is the directory containing the PNG frames. Repeated calls are safe.
        """
        if self._finalized:
            return self._artifact_path

        self._remove_camera()
        self._artifact_path = _finalize_video(
            frame_dir=self.frame_dir,
            video_path=self.video_path,
            video_fps=self.video_fps,
            frame_count=self.frame_count,
            ffmpeg_path=self.ffmpeg_path,
        )
        self._finalized = True
        return self._artifact_path

    def _target_step(self, frame_index):
        return max(
            0,
            int(
                round(
                    float(frame_index)
                    * self.beamng_steps_per_second
                    / self.video_fps
                )
            ),
        )

    def _remove_camera(self):
        if self._camera_removed:
            return
        try:
            self._camera.remove()
        except Exception as exc:
            print(f"Warning: could not remove video camera cleanly: {exc}")
        finally:
            self._camera_removed = True


def _positive_finite_float(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return value


def _create_camera(camera_name, beamng, vehicle, video_fps, video_resolution):
    from beamngpy.sensors import Camera

    return Camera(
        camera_name,
        beamng,
        vehicle=vehicle,
        requested_update_time=1.0 / video_fps,
        pos=(0.0, -1.25, 1.28),
        dir=(0.0, -1.0, 0.02),
        up=(0.0, 0.0, 1.0),
        resolution=video_resolution,
        field_of_view_y=DEFAULT_FIELD_OF_VIEW_Y,
        near_far_planes=(0.05, 1000.0),
        is_render_colours=True,
        is_render_annotations=False,
        is_render_instance=False,
        is_render_depth=False,
        is_dir_world_space=False,
    )


def _resolve_ffmpeg(ffmpeg_path=None):
    candidates = [
        ffmpeg_path,
        os.environ.get("FFMPEG_BINARY"),
        os.environ.get("IMAGEIO_FFMPEG_EXE"),
        "ffmpeg",
        "ffmpeg.exe",
    ]
    for candidate in candidates:
        if not candidate:
            continue

        explicit_path = Path(candidate).expanduser()
        if explicit_path.exists():
            return str(explicit_path)

        discovered = shutil.which(str(candidate))
        if discovered:
            return discovered

    return None


def _format_video_fps(video_fps):
    if video_fps.is_integer():
        return str(int(video_fps))
    return f"{video_fps:.6f}".rstrip("0").rstrip(".")


def _encode_video_with_ffmpeg(
    frame_dir,
    video_path,
    video_fps,
    frame_count,
    ffmpeg_path=None,
):
    ffmpeg = _resolve_ffmpeg(ffmpeg_path)
    if ffmpeg is None:
        return False

    video_path.parent.mkdir(parents=True, exist_ok=True)
    base_command = [
        ffmpeg,
        "-y",
        "-framerate",
        _format_video_fps(video_fps),
        "-start_number",
        "0",
        "-i",
        str(frame_dir / "frame_%06d.png"),
        "-frames:v",
        str(frame_count),
    ]
    codec_attempts = [
        ["-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart"],
        [
            "-c:v",
            "mpeg4",
            "-q:v",
            "4",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
        ],
    ]

    failures = []
    for codec_args in codec_attempts:
        result = subprocess.run(
            [*base_command, *codec_args, str(video_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if result.returncode == 0:
            print(f"Encoded video with ffmpeg: {ffmpeg}")
            return True
        failures.append(result.stdout)

    print(
        "Warning: ffmpeg failed to encode video with all configured codecs:\n"
        + "\n".join(failures)
    )
    return False


def _encode_video_with_imageio(frame_dir, video_path, video_fps, frame_count):
    try:
        import imageio.v2 as imageio
    except ImportError:
        return False

    frames = [frame_dir / f"frame_{index:06d}.png" for index in range(frame_count)]
    if not frames:
        return False

    video_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with imageio.get_writer(str(video_path), fps=video_fps) as writer:
            for frame in frames:
                if not frame.exists():
                    return False
                writer.append_data(imageio.imread(frame))
    except Exception as exc:
        print(f"Warning: imageio failed to encode video: {exc}")
        return False
    return True


def _finalize_video(frame_dir, video_path, video_fps, frame_count, ffmpeg_path=None):
    if frame_count <= 0:
        print("Warning: video recording was enabled, but no frames were captured.")
        return None

    encoded = _encode_video_with_ffmpeg(
        frame_dir,
        video_path,
        video_fps,
        frame_count,
        ffmpeg_path=ffmpeg_path,
    ) or _encode_video_with_imageio(frame_dir, video_path, video_fps, frame_count)
    if encoded:
        print(f"Recorded video: {video_path}")
        return str(video_path)

    print(
        "Warning: could not encode MP4 with ffmpeg or imageio. "
        f"Kept video frames in: {frame_dir}"
    )
    return str(frame_dir)
