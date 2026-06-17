# Image and Workspace Compatibility Notes

These notes preserve the local Mobipick image/workspace mapping gathered on
OLIMA-P15U. They are private operational notes, not portable defaults.

## Public images

These images are expected to be pullable by regular users and are suitable for
testing the software with the ROS workspace baked into the Docker image. They
do not contain the host user, so the GUI should run them as root and should not
mount a private host workspace for normal simulation runs.

- `ozkrelo/mobipick_labs:noetic`
- `ozkrelo/x_mobipick_labs:noetic-v1.1`
- `ozkrelo/x_mobipick_labs:noetic-v1.2`

The X image `ozkrelo/x_mobipick_labs:noetic-v1.2` has the useful workspace
under `/root/catkin_ws` inside the image. For this mode, selecting a private
workspace in the GUI should still run against the image default workspace
unless a host-user development image is selected.

## Local development images

These images are not expected to be pullable from Docker Hub. They contain a
user matching the host and are suitable for bind-mounted development
workspaces.

- `ozkrelo/x_mobipick_labs:oscar_user_from_1.2`
  - Suitable for `clean_mobipick_labs_ws`.
  - Built from `ozkrelo/x_mobipick_labs:noetic-v1.1`.
  - Uses `/usr/local/bin/entrypoint_user.sh`.
- `ozkrelo/x_mobipick_labs:rae_ws_from_oscar_user`
  - Intended for `rae_upom_mobipick_ws`.
- `ozkrelo/x_mobipick_labs:gpt_ws_from_oscar_user`
  - Intended for `gpt_ws`.
  - This is the image to highlight when `gpt_ws` is active.

## Images to keep unhighlighted for now

- `10.249.104.61:5000/x_mobipick_labs:gpt_ws_from_oscar_user`
  - Purpose is unclear and it may be a duplicate.
- `ozkrelo/mobipick_gpt_minimal:devel`
  - Minimal real-robot/test-only environment.
  - Does not contain a host user.
  - Not expected to run the full simulation.

## Workspace list on this machine

- Docker image default
- `clean_mobipick_labs_ws`
- `mobipick_labs_ws`
- `rae_upom_mobipick_ws`
- `common_tools_ws`
- `gpt_ws`

