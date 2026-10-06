"""Central outcome codes for completed inference with insufficient evidence."""

from enum import Enum


class UnknownReason(str, Enum):
    NO_ROI = "UNKNOWN_NO_ROI"
    NO_DETECTION = "UNKNOWN_NO_DETECTION"
    NO_VALID_EVIDENCE = "UNKNOWN_NO_VALID_EVIDENCE"


UNKNOWN_DISEASE_NAME = "Unknown"

