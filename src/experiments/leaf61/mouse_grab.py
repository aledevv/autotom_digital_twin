"""Native PhysX Shift + left-drag, matching the main V2 joint-grab setup."""


def configure(app):
    import asyncio

    import carb.settings
    import omni.kit.app

    settings = carb.settings.get_settings()
    manager = omni.kit.app.get_app().get_extension_manager()
    previous = settings.get("/app/player/playSimulations")
    settings.set("/app/player/playSimulations", False)
    try:

        async def enable_ui():
            # Support UI uses asyncio.get_running_loop() during startup in 6.1.
            manager.set_extension_enabled_immediate("omni.physx.ui", True)
            manager.set_extension_enabled_immediate("omni.physx.supportui", True)

        task = asyncio.ensure_future(enable_ui())
        for _ in range(100):
            app.update()
            if task.done():
                task.result()
                break
        else:
            task.cancel()
            raise RuntimeError("PhysX mouse UI did not initialize")
    finally:
        settings.set(
            "/app/player/playSimulations", previous if previous is not None else True
        )
    values = {
        "mouseInteractionEnabled": True,
        "mouseGrab": True,
        "mouseGrabIgnoreInvisible": False,
        "forceGrab": False,
        "pickingForce": 10.0,
    }
    for name, value in values.items():
        settings.set("/physics/" + name, value)
    return {name: settings.get("/physics/" + name) for name in values}
