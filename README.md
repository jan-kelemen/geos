# geos [![GitHub Build status](https://github.com/jan-kelemen/geos/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/jan-kelemen/geos/actions/workflows/ci.yml)

Geometric shape sorter using Bullet3 for physics simulation and Vulkan API for rendering.

## Building
Necessary build tools are:
* CMake 3.27 or higher
* Conan 2.4 or higher
  * See [installation instructions](https://docs.conan.io/2/installation.html)
* One of supported compilers:
  * Clang-18
  * GCC-13
  * Visual Studio 2022 (MSVC v194)

```
conan install . --profile=conan/clang-18 --profile=conan/dependencies --build=missing --settings build_type=Release
cmake --preset release
cmake --build --preset=release
```

Note: Override packages from Conan Center with updated ones from [jan-kelemen/conan-recipes](https://github.com/jan-kelemen/conan-recipes), this is mostly due to hardening or sanitizer options being incompatible with packages on Conan Center.
```
git clone git@github.com:jan-kelemen/conan-recipes.git
conan export conan-recipes/recipes/freetype/meson --version 2.13.2
conan export conan-recipes/recipes/sdl/all --version 2.30.4
conan export conan-recipes/recipes/spdlog/all --version 1.14.1
conan export conan-recipes/recipes/tinygltf/all --version 2.9.0
conan export conan-recipes/recipes/vulkan-memory-allocator/all --version 3.1.0
conan create conan-recipes/recipes/pulseaudio/meson --version 17.0 # Linux only
```

And then execute the build commands.
