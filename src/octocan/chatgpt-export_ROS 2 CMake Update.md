
Converting a CMakeLists file from ROS 1 to ROS 2 involves several key modifications to adapt to the updated build system, package management, and ROS architecture. Here’s a detailed breakdown of the major changes involved:

### 1. **CMake Minimum Required Version**
- **ROS 1**: Usually, the minimum required version is set to 3.0.2.
- **ROS 2**: It’s commonly set to 3.5 or higher. This is due to the updated dependencies and features in CMake that are leveraged by ROS 2.

### 2. **C++ Standard**
- **ROS 1**: Often targets C++11 as it was the standard when many ROS 1 systems were developed.
- **ROS 2**: Defaults to C++14, or sometimes C++17, in line with the updated language features and performance improvements.

### 3. **Build System**
- **ROS 1**: Uses `catkin` which is specifically designed for ROS 1.
- **ROS 2**: Utilizes `ament_cmake`, an adaptation of CMake that is more modular and supports the ROS 2 ecosystem.

### 4. **Package Discovery**
- **ROS 1**: `find_package(catkin REQUIRED COMPONENTS ...)`
- **ROS 2**: Uses `find_package(ament_cmake REQUIRED)` and `find_package(rclcpp REQUIRED)` for the core ROS 2 functionality. Additional dependencies (like message types and service libraries) are included as needed.

### 5. **Message, Services, and Actions Generation**
- **ROS 1**: Messages, services, and actions are handled with `message_generation` and `message_runtime`.
- **ROS 2**: Uses `rosidl_default_generators` for generating interfaces (messages, services, actions). There’s a significant shift towards using `rosidl` to define and build interface files.

### 6. **Executable and Library Targets**
- **ROS 1**: Libraries and executables are linked directly with `catkin` components and dependencies.
- **ROS 2**: The `ament_target_dependencies` macro is used to specify dependencies for libraries and executables, reflecting ament’s approach to handling the build environment.

### 7. **Compiler Options**
- **ROS 1**: Compiler options may vary, often manually specified.
- **ROS 2**: There’s a more standardized approach using `add_compile_options(-Wall -Wextra -Wpedantic)` to ensure code quality and maintain cross-platform compatibility.

### 8. **Dynamic Reconfigure**
- **ROS 1**: Uses `dynamic_reconfigure` for live parameter updates.
- **ROS 2**: Parameter updates are handled differently, often through the ROS 2 parameter service and not requiring specific build instructions in the CMakeLists file.

### 9. **Installation Targets**
- **ROS 1**: Installation paths can be slightly more complex and less standardized.
- **ROS 2**: Simplifies the installation process using ament's installation macros which conform to a standard layout beneficial for package sharing and management.

### 10. **Linting and Testing**
- **ROS 1**: May use `catkin_add_gtest` or similar tools for testing.
- **ROS 2**: Uses `ament_lint_auto` to automate linting and testing setup, reflecting a more unified approach to ensuring code quality.

### Conclusion
These transformations align with ROS 2’s architecture, which emphasizes modularity, security, and support for modern C++ standards. This upgrade process not only updates the build and dependency management tools but also prepares the ROS package for future scalability and interoperability within the ROS ecosystem.

