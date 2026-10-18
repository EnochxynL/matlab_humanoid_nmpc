from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy
from conan.tools.scm import Git
from pathlib import Path
import os

class Recipe(ConanFile):
    name = 'hpipm'
    version = 'cci.255ffdf38d3a'
    license = 'BSD-2-Clause'
    package_id_non_embed_mode = "full_mode"
    package_id_embed_mode = "full_mode"
    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False]}
    default_options = {"shared": True}
    exports_sources = "CMakeLists.txt", "cmake/*", "cmake/windows/*"

    def layout(self):
        cmake_layout(self)

    def source(self):
        info = self.conan_data["sources"][str(self.version)]
        # A local mirror is only a transport optimization: checkout always uses
        # the pinned commit, never the working tree (including local edits).
        mirrors = os.environ.get("G1_RECIPE_SOURCE_ROOT")
        if mirrors:
            # Archive tracked files at the commit: works with partial clones and
            # excludes all working-tree edits and untracked files.
            import subprocess
            from conan.tools.files import unzip
            archive = os.path.join(self.source_folder, "upstream.tar")
            with open(archive, "wb") as output:
                subprocess.run(["git", "-C", str(Path(mirrors) / str(self.name)),
                                "archive", "--format=tar", info["commit"]],
                               stdout=output, check=True)
            unzip(self, archive, destination="upstream")
            os.remove(archive)
        else:
            Git(self).clone(info["url"], target="upstream",
                            args=["--filter=blob:none", "--no-checkout"])
            Git(self, folder="upstream").checkout(info["commit"])

    def build_requirements(self):
        self.tool_requires("cmake/3.31.8")
        self.tool_requires("ninja/1.12.1")

    def generate(self):
        CMakeDeps(self).generate()
        tc = CMakeToolchain(self)
        tc.variables["BUILD_True_LIBS"] = bool(self.options.shared)
        tc.variables["CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS"] = True
        tc.variables["CMAKE_CXX_FLAGS_RELEASE"] = "-O1 -DNDEBUG"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure(build_script_folder=self.source_folder)
        cmake.build()

    def package(self):
        CMake(self).install()
        copy(self, "*LICEN[CS]E*", src=os.path.join(self.source_folder, "upstream"),
             dst=os.path.join(self.package_folder, "licenses"), keep_path=False)

    def requirements(self):
        self.requires("blasfeo/cci.ae6e2d1dea01", transitive_headers=True)

    def package_info(self):
        self.cpp_info.libs = ["hpipm"]
        self.cpp_info.set_property("cmake_target_name", "hpipm")
        self.cpp_info.requires = ["blasfeo::blasfeo"]
