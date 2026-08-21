"""Small lifecycle-safe wrapper around the GroPy workbench API."""

from __future__ import annotations

from contextlib import contextmanager
from typing import Any, Iterator


class GroIMPError(RuntimeError):
    """Base error raised by the GroIMP bridge."""


class GroIMPConnectionError(GroIMPError):
    """Raised when the GroIMP API cannot be reached."""


class GroIMPRequestError(GroIMPError):
    """Raised when GroIMP rejects an API request."""


def _response_excerpt(response: Any, limit: int = 1000) -> str:
    text = getattr(response, "text", "") or ""
    return str(text)[:limit]


class GroIMPClient:
    """Open and operate GroIMP workbenches while enforcing cleanup."""

    def __init__(self, api_url: str, *, gro_link_factory: Any | None = None):
        self.api_url = api_url.rstrip("/") + "/"
        if gro_link_factory is None:
            try:
                from GroPy import GroPy
            except ImportError as exc:  # pragma: no cover - dependency is installed in production
                raise GroIMPConnectionError("GroPy is not installed") from exc
            gro_link_factory = GroPy.GroLink
        self._link = gro_link_factory(self.api_url)

    @contextmanager
    def open_project(self, project_path: str) -> Iterator[Any]:
        """Open a project and always close its workbench before returning."""

        workbench = None
        try:
            try:
                request = self._link.openWB(path=project_path).run()
            except Exception as exc:
                raise GroIMPConnectionError(
                    f"Cannot connect to GroIMP at {self.api_url}: {exc}"
                ) from exc

            status = getattr(request.result, "status_code", None)
            if status != 200:
                raise GroIMPRequestError(
                    f"GroIMP could not open {project_path!r} (HTTP {status}): "
                    f"{_response_excerpt(request.result)}"
                )
            workbench = request.read()
            yield workbench
        finally:
            if workbench is not None:
                try:
                    workbench.close().run()
                except Exception:
                    # Never mask the extraction error. A successful extraction can still
                    # report cleanup failure through the server logs.
                    pass

    @staticmethod
    def run_function(workbench: Any, function_name: str) -> dict[str, Any]:
        """Run one public RGG function and validate the HTTP response."""

        try:
            request = workbench.runRGGFunction(function_name).run()
        except Exception as exc:
            raise GroIMPConnectionError(
                f"Failed while running RGG function {function_name!r}: {exc}"
            ) from exc
        status = getattr(request.result, "status_code", None)
        if status != 200:
            raise GroIMPRequestError(
                f"RGG function {function_name!r} failed (HTTP {status}): "
                f"{_response_excerpt(request.result)}"
            )
        payload = request.read()
        return payload if isinstance(payload, dict) else {}


def run_json_call(call: Any, *, operation: str) -> dict[str, Any]:
    """Execute one GroPy JSON call and normalize request errors."""

    try:
        request = call.run()
    except Exception as exc:
        raise GroIMPConnectionError(f"GroIMP {operation} failed: {exc}") from exc
    status = getattr(request.result, "status_code", None)
    if status != 200:
        raise GroIMPRequestError(
            f"GroIMP {operation} failed (HTTP {status}): "
            f"{_response_excerpt(request.result)}"
        )
    payload = request.read()
    if not isinstance(payload, dict):
        raise GroIMPRequestError(f"GroIMP {operation} returned a non-object JSON payload")
    return payload
