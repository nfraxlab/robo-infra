"""
Pytest configuration for robo-infra tests.

Handles optional dependency detection and skip markers.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    from _pytest.config import Config

logger = logging.getLogger(__name__)


# =============================================================================
# Optional Dependency Detection
# =============================================================================


def _check_ai_infra_available() -> bool:
    """Check if ai-infra package is installed."""
    try:
        import ai_infra  # noqa: F401

        return True
    except ImportError:
        return False


def _check_svc_infra_available() -> bool:
    """Check if svc-infra package is installed."""
    try:
        import svc_infra  # noqa: F401

        return True
    except ImportError:
        return False


def _check_fastapi_available() -> bool:
    """Check if fastapi package is installed."""
    try:
        import fastapi  # noqa: F401

        return True
    except ImportError:
        return False


# Cache availability at module load
AI_INFRA_AVAILABLE = _check_ai_infra_available()
SVC_INFRA_AVAILABLE = _check_svc_infra_available()
FASTAPI_AVAILABLE = _check_fastapi_available()


# =============================================================================
# Skip Markers
# =============================================================================

requires_ai_infra = pytest.mark.skipif(
    not AI_INFRA_AVAILABLE,
    reason="Requires ai-infra package (pip install robo-infra[ai])",
)

requires_svc_infra = pytest.mark.skipif(
    not SVC_INFRA_AVAILABLE,
    reason="Requires svc-infra package (pip install robo-infra[api])",
)

requires_fastapi = pytest.mark.skipif(
    not FASTAPI_AVAILABLE,
    reason="Requires fastapi package (pip install robo-infra[api])",
)


# =============================================================================
# Pytest Hooks
# =============================================================================


def pytest_configure(config: Config) -> None:
    """Register custom markers."""
    config.addinivalue_line(
        "markers",
        "requires_ai_infra: marks tests as requiring ai-infra package",
    )
    config.addinivalue_line(
        "markers",
        "requires_svc_infra: marks tests as requiring svc-infra package",
    )
    config.addinivalue_line(
        "markers",
        "requires_fastapi: marks tests as requiring fastapi package",
    )


def pytest_collection_modifyitems(config: Config, items: list) -> None:
    """Auto-skip tests based on available dependencies.

    This hook examines test module names and applies appropriate skip markers.
    """
    for item in items:
        # Auto-skip ai-infra tests if package not available
        if "ai_infra" in item.nodeid and not AI_INFRA_AVAILABLE:
            item.add_marker(
                pytest.mark.skip(
                    reason="Requires ai-infra package (pip install robo-infra[ai])"
                )
            )

        # Auto-skip svc-infra tests if package not available
        if "svc_infra" in item.nodeid and not SVC_INFRA_AVAILABLE:
            item.add_marker(
                pytest.mark.skip(
                    reason="Requires svc-infra package (pip install robo-infra[api])"
                )
            )

        # Auto-skip API integration tests if fastapi not available
        if "api_integration" in item.nodeid and not FASTAPI_AVAILABLE:
            item.add_marker(
                pytest.mark.skip(
                    reason="Requires fastapi package (pip install robo-infra[api])"
                )
            )

        # Auto-skip AI integration tests if ai-infra not available
        if "ai_integration" in item.nodeid and not AI_INFRA_AVAILABLE:
            item.add_marker(
                pytest.mark.skip(
                    reason="Requires ai-infra package (pip install robo-infra[ai])"
                )
            )
