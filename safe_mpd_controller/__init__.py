"""BeamNG.tech controller backed by the Safe-MPD planner."""

__all__ = ("SafeMPDController",)


def __getattr__(name):
    if name == "SafeMPDController":
        from .safe_mpd import SafeMPDController
        return SafeMPDController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
