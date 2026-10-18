from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy
from conan.tools.scm import Git
import os

class Recipe(ConanFile):
    name = 'blasfeo'
    version = 'cci.ae6e2d1dea01'
    license = 'BSD-2-Clause'
    package_id_non_embed_mode = "full_mode"
    package_id_embed_mode = "full_mode"
    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    exports_sources = "CMakeLists.txt", "cmake/*", "cmake/windows/*"

    def layout(self):
        cmake_layout(self)

    def source(self):
        info = self.conan_data["sources"][str(self.version)]
        Git(self).run("init upstream")
        repository = Git(self, folder="upstream")
        repository.run("remote add origin " + info["url"])
        repository.run("fetch --depth=1 origin " + info["commit"])
        repository.checkout(info["commit"])

    def build_requirements(self):
        self.tool_requires("cmake/3.31.8")
        self.tool_requires("ninja/1.12.1")

    def generate(self):
        CMakeDeps(self).generate()
        tc = CMakeToolchain(self)
        tc.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
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

    def package_info(self):
        self.cpp_info.libs = ["blasfeo"]
        self.cpp_info.set_property("cmake_target_name", "blasfeo")
        if self.settings.os != "Windows":
            self.cpp_info.system_libs = ["m"]
