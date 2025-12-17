---
title: ROS 2 Packages and Workspaces
sidebar_position: 4
---

# Chapter 4: ROS 2 Packages and Workspaces

In ROS 2, the fundamental unit of code organization, distribution, and management is the **package**. Packages encapsulate related software components, configurations, and resources, allowing for modular development and easy integration into larger robotic systems. Understanding how to create, manage, and build these packages, along with the concept of **workspaces**, is essential for any ROS 2 developer. This chapter will guide you through these core concepts.

## Introduction to ROS 2 Packages

A ROS 2 package is a directory containing related code, resources, and metadata that together provide specific functionality. This could be a robot driver, a perception algorithm, a navigation module, or a set of utility functions. By breaking down complex robotic systems into manageable packages, developers can foster code reusability, collaboration, and maintainability.

### Structure of a ROS 2 Package

A typical ROS 2 package has a standardized directory structure and contains several key files:

*   **`package.xml`**: This XML file is the manifest for the package. It describes the package's metadata, including its name, version, author, description, and, most importantly, its dependencies on other ROS 2 packages or system libraries. `colcon`, the ROS 2 build tool, uses this file to understand how to build and link the package.
*   **Build System Files**: These files instruct the build tool on how to compile the package.
    *   For C++ packages, `CMakeLists.txt` is used.
    *   For Python packages, `setup.py` and `setup.cfg` are used, managed by `colcon`'s `ament_python` build type.
*   **Source Code**: Directories like `src/` (for C++ headers and source) or Python package directories contain the actual code.
*   **Resources**: Directories like `resource/` or `share/` can store configuration files, launch files, URDF models, meshes, and other non-code assets.

## ROS 2 Workspaces

A **ROS 2 workspace** is a directory structure that organizes your ROS 2 packages and provides an environment for building and managing them. It's not just a collection of source files; it's an integrated development area. A standard ROS 2 workspace typically includes:

*   **`src/`**: This is where you place the source code for your ROS 2 packages. When you create a new package, its source files will reside here.
*   **`build/`**: This directory is created by the build tool (`colcon`) and contains intermediate build files, such as compiled object files and build system-specific files. You generally do not need to modify files in this directory.
*   **`install/`**: After a successful build, `colcon` installs the compiled code, executables, libraries, and resource files into this directory. This `install` space is what makes your custom packages available to ROS 2.

A visual representation of a typical workspace structure:

```
my_ros2_ws/            # The root of your workspace
├── src/               # Contains your package source directories
│   ├── my_custom_robot_pkg/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt (or setup.py for Python)
│   │   └── src/ (or python_package_name/ for Python)
│   └── another_ros_pkg/
│       ├── package.xml
│       └── CMakeLists.txt
├── build/             # Intermediate build artifacts (managed by colcon)
└── install/           # Installed packages and executables
    ├── my_custom_robot_pkg/
    │   ├── lib/
    │   ├── share/
    │   └── ...
    ├── another_ros_pkg/
    └── ...
```

## Creating and Building Packages

### Creating a New Package

ROS 2 provides a command-line tool, `ros2 pkg create`, to generate the basic structure and essential files for a new package.

Let's create a simple Python ROS 2 package named `robot_interfaces` for custom message types:

```bash
# 1. Navigate to your workspace's source directory
cd ~/my_ros2_ws/src

# 2. Create the new Python package
ros2 pkg create --build-type ament_python robot_interfaces
```

This command will:
*   Create a directory named `robot_interfaces` inside `~/my_ros2_ws/src/`.
*   Generate a default `package.xml` file.
*   Create a `setup.py` and `setup.cfg` file suitable for Python packages.
*   Create a `robot_interfaces` directory inside `robot_interfaces/` to hold Python modules.

For C++ packages, you would use `--build-type ament_cmake`.

### Building Your Workspace

Once you have created or downloaded ROS 2 packages into your `src/` directory, you need to build them. `colcon` is the standard build tool for ROS 2.

1.  **Navigate to the workspace root**:
    ```bash
    cd ~/my_ros2_ws
    ```
2.  **Run the build command**:
    ```bash
    colcon build
    ```
    `colcon` will discover all packages in the `src/` directory, resolve their dependencies (using information from `package.xml`), compile them, and install the outputs into the `install/` directory.

### Sourcing Your Workspace

After building, your new packages and executables are not immediately available in your shell. You need to "source" the workspace's setup file. This modifies your shell's environment variables so that ROS 2 tools (like `ros2 run`, `ros2 pkg list`) can find your custom packages and their executables.

```bash
# Source the setup file for your current shell session
source install/setup.bash
```

Now you can verify your package:
```bash
ros2 pkg list | grep robot_interfaces
# Expected output: robot_interfaces
```

And if you had an executable in that package, you could run it:
```bash
# Example if you had an executable node
# ros2 run robot_interfaces my_node
```

## Practical Use Cases

*   **Developing Custom Hardware Drivers**: Create a package for a new sensor or actuator, providing ROS 2 interfaces (topics, services, actions) for it.
*   **Implementing Algorithms**: Package your custom perception, planning, or control algorithms into reusable ROS 2 packages.
*   **Creating Reusable Libraries**: Develop common functionalities, like robot-specific kinematics or utility functions, into packages that can be shared across multiple projects.
*   **Integrating Third-Party Packages**: When using external ROS 2 packages, you typically place them in your workspace's `src/` directory and build them.

## Summary

Packages and workspaces are fundamental concepts in ROS 2 for structuring, building, and managing robotic software. Packages provide modularity by grouping related code and resources, while workspaces offer an integrated environment for building and managing these packages. By mastering the creation, building, and sourcing of packages within a workspace, you establish a solid foundation for developing complex and maintainable ROS 2 robotic systems.
---