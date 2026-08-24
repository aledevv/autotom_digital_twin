"""Small lifecycle-safe wrapper around the GroPy workbench API."""

from __future__ import annotations

from contextlib import contextmanager
from typing import Any, Iterator

import requests


DEFAULT_REQUEST_TIMEOUT_SECONDS = 120.0


class GroIMPError(RuntimeError):
    """Base error raised by the GroIMP bridge."""


class GroIMPConnectionError(GroIMPError):
    """Raised when the GroIMP API cannot be reached."""


class GroIMPRequestError(GroIMPError):
    """Raised when GroIMP rejects an API request."""


def _response_excerpt(response: Any, limit: int = 1000) -> str:
    text = getattr(response, "text", "") or ""
    return str(text)[:limit]


def _execute_call(call: Any, *, timeout: float = DEFAULT_REQUEST_TIMEOUT_SECONDS):
    """Run GroPy calls with a finite timeout while retaining test doubles."""

    if all(hasattr(call, name) for name in ("url", "parameters", "content")):
        call.result = requests.post(
            url=call.url,
            params=call.parameters,
            data=call.content,
            timeout=timeout,
        )
        return call
    return call.run()


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
    def create_project(
        self,
        *,
        template: str = "newRGG",
        name: str | None = None,
    ) -> Iterator[Any]:
        """Create a disposable workbench and always close it on exit."""

        workbench = None
        try:
            try:
                request = _execute_call(
                    self._link.createWB(template=template, name=name)
                )
            except Exception as exc:
                raise GroIMPConnectionError(
                    f"Cannot create GroIMP template {template!r} at {self.api_url}: {exc}"
                ) from exc

            status = getattr(request.result, "status_code", None)
            if status != 200:
                raise GroIMPRequestError(
                    f"GroIMP could not create template {template!r} (HTTP {status}): "
                    f"{_response_excerpt(request.result)}"
                )
            workbench = request.read()
            yield workbench
        finally:
            self._close_workbench(workbench)

    @contextmanager
    def open_project(self, project_path: str) -> Iterator[Any]:
        """Open a project and always close its workbench before returning."""

        workbench = None
        try:
            try:
                request = _execute_call(self._link.openWB(path=project_path))
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
            self._close_workbench(workbench)

    @staticmethod
    def _close_workbench(workbench: Any | None) -> None:
        if workbench is None:
            return
        try:
            _execute_call(workbench.close(), timeout=10.0)
        except Exception:
            # Never mask the extraction error with a secondary cleanup failure.
            pass

    @staticmethod
    def update_source(workbench: Any, name: str, content: str) -> dict[str, Any]:
        """Replace one source file in an open workbench."""

        return run_json_call(
            workbench.updateFile(name, content),
            operation=f"source update {name}",
        )

    @staticmethod
    def compile(workbench: Any) -> dict[str, Any]:
        """Compile an open GroIMP workbench and validate the response."""

        return run_json_call(workbench.compile(), operation="project compilation")

    @staticmethod
    def run_function(workbench: Any, function_name: str) -> dict[str, Any]:
        """Run one public RGG function and validate the HTTP response."""

        try:
            request = _execute_call(workbench.runRGGFunction(function_name))
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

    @staticmethod
    def export_subscene_obj(workbench: Any, node_id: int) -> bytes:
        """Export one interpreted GroIMP subscene as OBJ bytes.

        GroIMP's endpoint returns a binary response even though OBJ itself is
        text.  Keeping the payload in memory avoids source-project writes and
        lets callers decide whether a diagnostic mesh should be persisted.
        """

        try:
            request = _execute_call(workbench.exportSubScene("obj", int(node_id)))
        except Exception as exc:
            raise GroIMPConnectionError(
                f"Failed while exporting OBJ subscene for node {node_id}: {exc}"
            ) from exc
        status = getattr(request.result, "status_code", None)
        if status != 200:
            raise GroIMPRequestError(
                f"GroIMP OBJ subscene export for node {node_id} failed "
                f"(HTTP {status}): {_response_excerpt(request.result)}"
            )
        payload = request.read()
        if not isinstance(payload, (bytes, bytearray)):
            raise GroIMPRequestError(
                f"GroIMP OBJ subscene export for node {node_id} returned "
                "a non-binary payload"
            )
        return bytes(payload)

    @staticmethod
    def export_scene_obj(workbench: Any) -> bytes:
        """Export the full interpreted scene, also forcing renderer refresh."""

        try:
            request = _execute_call(workbench.export3d("obj"))
        except Exception as exc:
            raise GroIMPConnectionError(f"Failed while exporting the OBJ scene: {exc}") from exc
        status = getattr(request.result, "status_code", None)
        if status != 200:
            raise GroIMPRequestError(
                f"GroIMP OBJ scene export failed (HTTP {status}): "
                f"{_response_excerpt(request.result)}"
            )
        payload = request.read()
        if not isinstance(payload, (bytes, bytearray)):
            raise GroIMPRequestError("GroIMP OBJ scene export returned a non-binary payload")
        return bytes(payload)


def run_json_call(call: Any, *, operation: str) -> dict[str, Any]:
    """Execute one GroPy JSON call and normalize request errors."""

    try:
        request = _execute_call(call)
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
