"""Mock external systems for integration testing.

This module provides mock implementations of external systems for testing:
- Mock FastAPI TestClient setup
- Mock ai-infra tool execution
- Mock svc-infra observability
- Mock network delays and failures

These mocks allow testing edge cases and error handling.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import pytest


if TYPE_CHECKING:
    from collections.abc import Callable


# =============================================================================
# Mock Network Delay/Failure
# =============================================================================


@dataclass
class NetworkConditions:
    """Simulates network conditions for testing.

    Attributes:
        latency_ms: Simulated network latency in milliseconds.
        failure_rate: Probability of request failure (0.0 to 1.0).
        timeout_ms: Maximum time before timeout.
    """

    latency_ms: float = 0.0
    failure_rate: float = 0.0
    timeout_ms: float = 30000.0
    _request_count: int = field(default=0, repr=False)

    def apply_latency(self) -> None:
        """Apply simulated network latency."""
        if self.latency_ms > 0:
            time.sleep(self.latency_ms / 1000.0)

    async def apply_latency_async(self) -> None:
        """Apply simulated network latency (async)."""
        if self.latency_ms > 0:
            await asyncio.sleep(self.latency_ms / 1000.0)

    def maybe_fail(self) -> None:
        """Randomly fail based on failure_rate."""
        import random

        self._request_count += 1
        if random.random() < self.failure_rate:
            raise ConnectionError(f"Simulated network failure (request #{self._request_count})")

    def should_timeout(self, elapsed_ms: float) -> bool:
        """Check if request should timeout."""
        return elapsed_ms > self.timeout_ms


class MockHTTPResponse:
    """Mock HTTP response for testing."""

    def __init__(
        self,
        status_code: int = 200,
        json_data: dict | list | None = None,
        text: str = "",
        headers: dict | None = None,
    ) -> None:
        self.status_code = status_code
        self._json_data = json_data
        self.text = text
        self.headers = headers or {}

    def json(self) -> Any:
        """Return JSON data."""
        if self._json_data is None:
            raise ValueError("No JSON data")
        return self._json_data


class MockHTTPClient:
    """Mock HTTP client for testing external API calls."""

    def __init__(self, network: NetworkConditions | None = None) -> None:
        self.network = network or NetworkConditions()
        self.requests: list[dict] = []
        self.responses: dict[str, MockHTTPResponse] = {}

    def set_response(self, path: str, response: MockHTTPResponse) -> None:
        """Set a mock response for a path."""
        self.responses[path] = response

    def get(self, url: str, **kwargs: Any) -> MockHTTPResponse:
        """Mock GET request."""
        self.network.apply_latency()
        self.network.maybe_fail()

        self.requests.append({"method": "GET", "url": url, **kwargs})

        return self.responses.get(url, MockHTTPResponse(status_code=404))

    def post(self, url: str, **kwargs: Any) -> MockHTTPResponse:
        """Mock POST request."""
        self.network.apply_latency()
        self.network.maybe_fail()

        self.requests.append({"method": "POST", "url": url, **kwargs})

        return self.responses.get(url, MockHTTPResponse(status_code=404))


# =============================================================================
# Mock AI Tool Execution
# =============================================================================


@dataclass
class MockToolCall:
    """Represents a mock tool call."""

    tool_name: str
    arguments: dict[str, Any]
    result: Any = None
    error: Exception | None = None
    duration_ms: float = 0.0


class MockToolExecutor:
    """Mock executor for AI tool calls.

    Simulates how an LLM would call tools.
    """

    def __init__(self) -> None:
        self.tools: dict[str, Callable] = {}
        self.call_history: list[MockToolCall] = []
        self.network = NetworkConditions()

    def register_tools(self, tools: list[Callable]) -> None:
        """Register tools for execution."""
        for tool in tools:
            self.tools[tool.__name__] = tool

    def execute(self, tool_name: str, arguments: dict[str, Any]) -> MockToolCall:
        """Execute a tool by name with arguments."""
        start = time.perf_counter()

        call = MockToolCall(tool_name=tool_name, arguments=arguments)

        try:
            # Apply network conditions
            self.network.apply_latency()
            self.network.maybe_fail()

            # Find and execute tool
            if tool_name not in self.tools:
                raise ValueError(f"Unknown tool: {tool_name}")

            tool = self.tools[tool_name]
            call.result = tool(**arguments)

        except Exception as e:
            call.error = e

        finally:
            call.duration_ms = (time.perf_counter() - start) * 1000
            self.call_history.append(call)

        return call

    def get_tool_definitions(self) -> list[dict]:
        """Get tool definitions in LLM-compatible format."""
        definitions = []
        for name, tool in self.tools.items():
            definitions.append(
                {
                    "type": "function",
                    "function": {
                        "name": name,
                        "description": tool.__doc__ or "",
                    },
                }
            )
        return definitions


# =============================================================================
# Mock Observability
# =============================================================================


@dataclass
class MockSpan:
    """Mock telemetry span for testing."""

    name: str
    attributes: dict[str, Any] = field(default_factory=dict)
    events: list[dict] = field(default_factory=list)
    status: str = "OK"
    start_time: float = field(default_factory=time.time)
    end_time: float | None = None

    def set_attribute(self, key: str, value: Any) -> None:
        """Set span attribute."""
        self.attributes[key] = value

    def add_event(self, name: str, attributes: dict | None = None) -> None:
        """Add span event."""
        self.events.append({"name": name, "attributes": attributes or {}})

    def end(self) -> None:
        """End the span."""
        self.end_time = time.time()


class MockTracer:
    """Mock tracer for observability testing."""

    def __init__(self, name: str = "test") -> None:
        self.name = name
        self.spans: list[MockSpan] = []
        self._current_span: MockSpan | None = None

    def start_span(self, name: str, **kwargs: Any) -> MockSpan:
        """Start a new span."""
        span = MockSpan(name=name)
        self.spans.append(span)
        self._current_span = span
        return span

    @property
    def current_span(self) -> MockSpan | None:
        """Get current active span."""
        return self._current_span


class MockMetrics:
    """Mock metrics collector for testing."""

    def __init__(self) -> None:
        self.counters: dict[str, int] = {}
        self.gauges: dict[str, float] = {}
        self.histograms: dict[str, list[float]] = {}

    def increment(self, name: str, value: int = 1, tags: dict | None = None) -> None:
        """Increment a counter."""
        key = self._make_key(name, tags)
        self.counters[key] = self.counters.get(key, 0) + value

    def gauge(self, name: str, value: float, tags: dict | None = None) -> None:
        """Set a gauge value."""
        key = self._make_key(name, tags)
        self.gauges[key] = value

    def histogram(self, name: str, value: float, tags: dict | None = None) -> None:
        """Record a histogram value."""
        key = self._make_key(name, tags)
        if key not in self.histograms:
            self.histograms[key] = []
        self.histograms[key].append(value)

    def _make_key(self, name: str, tags: dict | None) -> str:
        """Make a metric key from name and tags."""
        if not tags:
            return name
        tag_str = ",".join(f"{k}={v}" for k, v in sorted(tags.items()))
        return f"{name}[{tag_str}]"


class MockLogger:
    """Mock logger for testing."""

    def __init__(self) -> None:
        self.logs: list[dict] = []

    def log(self, level: str, message: str, **extra: Any) -> None:
        """Log a message."""
        self.logs.append({"level": level, "message": message, **extra})

    def debug(self, message: str, **extra: Any) -> None:
        """Log debug message."""
        self.log("DEBUG", message, **extra)

    def info(self, message: str, **extra: Any) -> None:
        """Log info message."""
        self.log("INFO", message, **extra)

    def warning(self, message: str, **extra: Any) -> None:
        """Log warning message."""
        self.log("WARNING", message, **extra)

    def error(self, message: str, **extra: Any) -> None:
        """Log error message."""
        self.log("ERROR", message, **extra)


class MockObservability:
    """Complete mock observability stack for testing."""

    def __init__(self) -> None:
        self.tracer = MockTracer()
        self.metrics = MockMetrics()
        self.logger = MockLogger()


# =============================================================================
# Test Fixtures
# =============================================================================


@pytest.fixture
def mock_network() -> NetworkConditions:
    """Create default network conditions."""
    return NetworkConditions()


@pytest.fixture
def mock_slow_network() -> NetworkConditions:
    """Create slow network conditions (100ms latency)."""
    return NetworkConditions(latency_ms=100)


@pytest.fixture
def mock_unreliable_network() -> NetworkConditions:
    """Create unreliable network (10% failure rate)."""
    return NetworkConditions(failure_rate=0.1)


@pytest.fixture
def mock_http_client(mock_network: NetworkConditions) -> MockHTTPClient:
    """Create a mock HTTP client."""
    return MockHTTPClient(network=mock_network)


@pytest.fixture
def mock_tool_executor() -> MockToolExecutor:
    """Create a mock tool executor."""
    return MockToolExecutor()


@pytest.fixture
def mock_observability() -> MockObservability:
    """Create a mock observability stack."""
    return MockObservability()


# =============================================================================
# Tests for Mock Systems
# =============================================================================


class TestMockNetwork:
    """Test mock network conditions."""

    def test_apply_latency(self) -> None:
        """Test network latency simulation."""
        network = NetworkConditions(latency_ms=50)

        start = time.perf_counter()
        network.apply_latency()
        elapsed = (time.perf_counter() - start) * 1000

        assert elapsed >= 45  # Allow some tolerance

    def test_failure_simulation(self) -> None:
        """Test network failure simulation."""
        network = NetworkConditions(failure_rate=1.0)  # Always fail

        with pytest.raises(ConnectionError):
            network.maybe_fail()

    def test_no_failure_at_zero_rate(self) -> None:
        """Test no failures at 0% rate."""
        network = NetworkConditions(failure_rate=0.0)

        # Should never fail
        for _ in range(100):
            network.maybe_fail()  # No exception


class TestMockHTTPClient:
    """Test mock HTTP client."""

    def test_set_response(self, mock_http_client: MockHTTPClient) -> None:
        """Test setting mock responses."""
        mock_http_client.set_response(
            "/api/test",
            MockHTTPResponse(status_code=200, json_data={"success": True}),
        )

        response = mock_http_client.get("/api/test")

        assert response.status_code == 200
        assert response.json() == {"success": True}

    def test_request_logging(self, mock_http_client: MockHTTPClient) -> None:
        """Test that requests are logged."""
        mock_http_client.get("/api/endpoint1")
        mock_http_client.post("/api/endpoint2", json={"data": 123})

        assert len(mock_http_client.requests) == 2
        assert mock_http_client.requests[0]["method"] == "GET"
        assert mock_http_client.requests[1]["method"] == "POST"


class TestMockToolExecutor:
    """Test mock tool executor."""

    def test_register_and_execute(self) -> None:
        """Test registering and executing tools."""

        def test_tool(x: int, y: int) -> int:
            """Add two numbers."""
            return x + y

        executor = MockToolExecutor()
        executor.register_tools([test_tool])

        result = executor.execute("test_tool", {"x": 2, "y": 3})

        assert result.result == 5
        assert result.error is None

    def test_execution_history(self) -> None:
        """Test that execution history is recorded."""

        def tool_a() -> str:
            return "a"

        def tool_b() -> str:
            return "b"

        executor = MockToolExecutor()
        executor.register_tools([tool_a, tool_b])

        executor.execute("tool_a", {})
        executor.execute("tool_b", {})

        assert len(executor.call_history) == 2
        assert executor.call_history[0].tool_name == "tool_a"
        assert executor.call_history[1].tool_name == "tool_b"

    def test_unknown_tool_error(self) -> None:
        """Test error for unknown tool."""
        executor = MockToolExecutor()

        result = executor.execute("nonexistent", {})

        assert result.error is not None
        assert "Unknown tool" in str(result.error)


class TestMockObservability:
    """Test mock observability stack."""

    def test_tracer_spans(self, mock_observability: MockObservability) -> None:
        """Test tracer span creation."""
        tracer = mock_observability.tracer

        span = tracer.start_span("test_operation")
        span.set_attribute("key", "value")
        span.end()

        assert len(tracer.spans) == 1
        assert tracer.spans[0].name == "test_operation"
        assert tracer.spans[0].attributes["key"] == "value"

    def test_metrics_counter(self, mock_observability: MockObservability) -> None:
        """Test metrics counter."""
        metrics = mock_observability.metrics

        metrics.increment("requests_total")
        metrics.increment("requests_total")
        metrics.increment("requests_total", value=5)

        assert metrics.counters["requests_total"] == 7

    def test_metrics_histogram(self, mock_observability: MockObservability) -> None:
        """Test metrics histogram."""
        metrics = mock_observability.metrics

        metrics.histogram("latency_ms", 10.5)
        metrics.histogram("latency_ms", 20.3)
        metrics.histogram("latency_ms", 15.1)

        assert len(metrics.histograms["latency_ms"]) == 3

    def test_logger(self, mock_observability: MockObservability) -> None:
        """Test logger."""
        logger = mock_observability.logger

        logger.info("Test message", extra_field="value")

        assert len(logger.logs) == 1
        assert logger.logs[0]["level"] == "INFO"
        assert logger.logs[0]["message"] == "Test message"
        assert logger.logs[0]["extra_field"] == "value"
