# GRPC Node

This is the main interface between gRPC on the LUCI system and the ROS2 system for the SDK.

## How to build and run this package

### Docker

- Follow the documentation on the SDK repository on [github](https://github.com/lucimobility/luci-ros2-sdk/tree/main/development) for manual setup in Docker.

### Native setup

- Follow the steps on the installation tab on [luci-sdk](https://lucimobility.github.io/luci-sdk-docs/) main webpage.
- You have two ways to install the SDK, one by using the provided .deb packages and the other by just cloning and building the repositories. These methods are explained in detail in the [quick start](https://lucimobility.github.io/luci-sdk-docs/Installation/Quick-Start/) and [manual installation](https://lucimobility.github.io/luci-sdk-docs/Installation/manual-installation) steps respectively.

### File structure to follow

```bash
ros_ws
└── luci-ros2
    └── src (Use colcon build at this level)
        ├── luci-ros2-grpc
        └── luci-ros2-msgs
```

There is also an automatic build script called `build-package.sh` that can be run to build the installable `.deb` file. This is what the GitHub Actions workflow calls. You can use it if you want to just put the .deb file in the ROS directory to run when ROS is sourced. A better way is to just download the .deb package using apt.

## Releasing new version

When a new version of this package is ready to be released, there are a couple of steps to follow. It is important to note that most of the process is automated for convenience and the process should be just a couple of button clicks.

### Steps

1. Update release version
    - This should be its own separate PR and should only update the package.xml `<version> </version>` tag.
    - LUCI follows [semver](https://semver.org/) style versioning, so MAJOR.MINOR.PATCH versions are expected.
    - It is okay to not put out versions until multiple changes have happened to the code.
2. Once the version increment is merged, you simply need to create an official release in GitHub. Make sure you make the release version the same as what is now in `package.xml`. We have chosen to keep the GitHub release and package version in sync.
    - This should trigger an action to auto run called `Create and Sign Package`, which you can monitor in the GitHub Actions panel. This should grab the released code, build it, make an installable .deb file, GPG sign it, and push it to JFrog Artifactory.

If everything went smoothly, congratulations — the new package will be released and publicly distributable.

**Note:** Once a PR is merged into the `main` branch, the docs site in the `next` version will update with it that evening.

## Implementation

More detailed implementation docs can be found in [Implementation](docs/grpc_package.md).
