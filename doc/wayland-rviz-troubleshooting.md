# X11, XWayland, Wayland, and RViz troubleshooting

This guide targets the case where RViz starts from the Mobipick GUI on a
Wayland desktop, prints Mesa loader errors, and exits with code 139.

## What the three display cases mean

- On native X11, Qt connects through DISPLAY and /tmp/.X11-unix.
- On a Wayland desktop with XWayland, Qt uses the same X11 interface. This is
  the default when both DISPLAY and WAYLAND_DISPLAY exist.
- On native Wayland, Qt connects through the socket named by WAYLAND_DISPLAY
  under XDG_RUNTIME_DIR.

The GUI's default display.mode of auto exposes both valid transports but
prefers X11/XWayland for ROS Noetic RViz, Gazebo, Qt 5, and OGRE. Set a
per-user override to test one backend explicitly:

    display:
      mode: x11       # native X11 or XWayland

or:

    display:
      mode: wayland   # native Wayland

The usual override file is
~/.config/mobipick-labs-docker-gui/gui_settings.yaml.

Containers use a private XDG_RUNTIME_DIR owned by their effective user with
mode 0700. Native Wayland mounts only the selected host socket at a neutral
path and links it into that directory. X11 and XWayland therefore avoid Qt's
missing-runtime warning without mounting the host's complete /run/user tree.

## Code 139 after Mesa loader errors

Output like this indicates that RViz initialized Qt and OGRE before the
OpenGL stack failed:

    libGL error: MESA-LOADER: failed to retrieve device information
    Segmentation fault (core dumped)
    Process finished with code 139

This is normally a GPU/OpenGL passthrough or renderer-selection problem, not
proof that the Wayland socket is missing. Open a container with the GUI's
**Open Terminal** action so it receives the same image, GPU request, display
mounts, user, and workspace configuration. Then collect:

    printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\nXDG_RUNTIME_DIR=%s\nQT_QPA_PLATFORM=%s\n' \
      "$DISPLAY" "$WAYLAND_DISPLAY" "$XDG_RUNTIME_DIR" "$QT_QPA_PLATFORM"
    nvidia-smi
    ls -l /dev/dri /dev/nvidia* 2>&1
    glxinfo -B

Expected results for the default XWayland path are:

- QT_QPA_PLATFORM is xcb.
- glxinfo -B names the intended NVIDIA GPU rather than an unexpected Mesa
  software renderer.
- nvidia-smi succeeds inside the container.
- NVIDIA and DRM device nodes are present.

If nvidia-smi fails, repair the host NVIDIA driver or NVIDIA Container Toolkit
before changing display settings. If nvidia-smi works but glxinfo -B selects
the wrong renderer on a hybrid-GPU laptop, retry after starting the GUI with:

    __NV_PRIME_RENDER_OFFLOAD=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    mobipick-labs-docker-gui --verbose 2

The compose services already pass these variables through when they exist.
Do not set them permanently on machines that do not use NVIDIA PRIME.

## Native Wayland checks

Native Wayland needs the Qt Wayland platform plugin inside the selected image:

    dpkg -s qtwayland5

New host-user images built by the setup wizard install this package. Existing
custom images need to be rebuilt or have the package installed. The public
images may differ, so use display.mode auto or x11 when their native Wayland
plugin is unavailable.

With display.mode set to wayland, verify:

    test -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY"
    echo "$QT_QPA_PLATFORM"

The first command should succeed and the second should print wayland.

## X11 authorization

The GUI mounts the host Xauthority cookie read-only when it can find one.
Otherwise it temporarily authorizes only the selected container user with
xhost and revokes that access when the last GUI container stops. This avoids
the old root-only authorization, which did not match host-user containers such
as a container running as mvinci.

If Qt reports that it cannot connect to the display, check:

    echo "$DISPLAY"
    echo "$XAUTHORITY"
    ls -l /tmp/.X11-unix "$XAUTHORITY" 2>&1

That failure is distinct from a Mesa loader crash: it occurs before RViz and
OGRE initialize.

## Remaining X11-only features

The GUI's screen recorder uses ffmpeg x11grab, while window capture and layout
replay use wmctrl and xprop. These features require native X11 or XWayland even
when RViz itself is forced to native Wayland.
