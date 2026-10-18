# Derived from Conan Center Index 1c90a5cb2e6353488efefa7d16020c19f9fae0c5 (MIT).
import os

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.files import get, rmdir, copy, check_sha256, unzip, replace_in_file

required_conan_version = ">=2"

class PinocchioConan(ConanFile):
    name = "pinocchio"
    package_type = "shared-library"
    license = ("BSD-2-Clause")
    url = "https://github.com/conan-io/conan-center-index"
    homepage = "https://stack-of-tasks.github.io/pinocchio/"
    description = "A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives"
    topics = (
        "robotics", "kinematics", "dynamics", "automatic-differentiation",
        "motion-planning", "ros", "rigid-body-dynamics", "analytical-derivatives",
        )

    settings = "os", "compiler", "build_type", "arch"
    options = {
        "with_collision_support": [True, False]
    }
    default_options = {
        "with_collision_support": False
    }

    def layout(self):
        cmake_layout(self, src_folder="src")

    def requirements(self):
        self.requires("eigen/3.4.0", transitive_headers=True)
        self.requires("urdfdom_headers/1.1.1")
        self.requires("urdfdom/4.0.0", transitive_headers=True)
        self.requires("boost/1.88.0", transitive_headers=True)
        if self.options.with_collision_support:
            self.requires("coal/3.0.3")
          
    def build_requirements(self):
        self.tool_requires("cmake/[>=3.22]")

    def source(self):
        source = self.conan_data["sources"][self.version]
        archive = os.getenv("G1_PINOCCHIO_ARCHIVE")
        if archive:
            check_sha256(self, archive, source["sha256"])
            unzip(self, archive, destination=self.source_folder, strip_root=True)
        else:
            get(self, **source, strip_root=True)

    def generate(self):
        tc = CMakeToolchain(self)
        if self.settings.os == "Windows" and self.settings.compiler == "gcc":
            tc.cache_variables["CMAKE_SHARED_LINKER_FLAGS"] = "-Wl,--export-all-symbols"
        tc.cache_variables["BUILD_PYTHON_INTERFACE"] = False
        tc.cache_variables["BUILD_TESTING"] = False
        tc.cache_variables["BUILD_EXAMPLES"] = False
        tc.cache_variables["BUILD_WITH_COLLISION_SUPPORT"] = self.options.with_collision_support
        tc.cache_variables["CMAKE_DISABLE_FIND_PACKAGE_Doxygen"] = True
        tc.cache_variables["BUILD_WITH_SDF_SUPPORT"] = False
        tc.cache_variables["BUILD_WITH_CASADI_SUPPORT"] = False
        tc.cache_variables["BUILD_WITH_AUTODIFF_SUPPORT"] = False
        tc.cache_variables["BUILD_WITH_CODEGEN_SUPPORT"] = False
        tc.cache_variables["BUILD_WITH_OPENMP_SUPPORT"] = False
        tc.cache_variables["BUILD_WITH_EXTRA_SUPPORT"] = False
        tc.generate()

        deps = CMakeDeps(self)
        # coal is a fork/continuation of hpp-fcl, pinocchio still uses hpp-fcl names,
        # but coal provides compatibility headers if the target links against the coal library
        deps.set_property("coal", "cmake_file_name", "coal")
        deps.set_property("coal", "cmake_target_name", "coal::coal")
        deps.generate()

    def build(self):
        # __FUNCSIG__ is an MSVC extension, not a Windows-wide compiler macro.
        replace_in_file(self, os.path.join(self.source_folder, "include", "pinocchio", "src", "macros.hxx"),
                        "#ifdef _WIN32\n  #define PINOCCHIO_PRETTY_FUNCTION",
                        "#ifdef _MSC_VER\n  #define PINOCCHIO_PRETTY_FUNCTION")
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(self, "LICENSE*", src=self.source_folder, dst=os.path.join(self.package_folder, "licenses"))
        cmake = CMake(self)
        cmake.install()

        rmdir(self, os.path.join(self.package_folder, "lib", "pkgconfig"))
        rmdir(self, os.path.join(self.package_folder, "lib", "cmake"))
        rmdir(self, os.path.join(self.package_folder, "share"))

    def package_info(self):
        self.cpp_info.components["pinocchio_headers"].includedirs = ["include", "include/pinocchio/deprecated"]
        self.cpp_info.components["pinocchio_headers"].defines = [
            "BOOST_MPL_LIMIT_LIST_SIZE=30", "BOOST_MPL_LIMIT_VECTOR_SIZE=30",
            "BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS", "BOOST_FUSION_INVOKE_MAX_ARITY=12"]
        self.cpp_info.components["pinocchio_headers"].requires = ["eigen::eigen", "boost::boost"]
        self.cpp_info.components["pinocchio_headers"].libdirs = []

        self.cpp_info.components["pinocchio_default"].libs = ["pinocchio_default"]
        self.cpp_info.components["pinocchio_default"].requires = ["pinocchio_headers"]
        self.cpp_info.components["pinocchio_default"].set_property("cmake_target_name", "pinocchio::pinocchio_default")
        self.cpp_info.components["pinocchio_default"].defines = ["PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION"]

        self.cpp_info.components["pinocchio_visualizers"].libs = ["pinocchio_visualizers"]
        self.cpp_info.components["pinocchio_visualizers"].requires = ["pinocchio_headers", "pinocchio_default"]
        self.cpp_info.components["pinocchio_visualizers"].set_property("cmake_target_name", "pinocchio::pinocchio_visualizers")

        self.cpp_info.components["pinocchio_parsers"].libs = ["pinocchio_parsers"]
        self.cpp_info.components["pinocchio_parsers"].set_property("cmake_target_name", "pinocchio::pinocchio_parsers")
        self.cpp_info.components["pinocchio_parsers"].requires = ["pinocchio_headers", "pinocchio_default", "boost::headers", "urdfdom::urdfdom", "urdfdom_headers::urdfdom_headers"]
        self.cpp_info.components["pinocchio_parsers"].defines = ["PINOCCHIO_WITH_URDFDOM"]
        urdf_version = str(self.dependencies["urdfdom_headers"].ref.version).split(".")
        for part, value in zip(("MAJOR", "MINOR", "PATCH"), urdf_version):
            self.cpp_info.components["pinocchio_parsers"].defines.append(
                f"PINOCCHIO_URDFDOM_HEADERS_{part}_VERSION={value}")

        self.cpp_info.components["pinocchio"].requires = ["pinocchio_default", "pinocchio_parsers", "pinocchio_visualizers"]
        self.cpp_info.components["pinocchio"].set_property("cmake_target_name", "pinocchio::pinocchio")

        if self.options.with_collision_support:
            self.cpp_info.components["pinocchio_collision"].libs = ["pinocchio_collision"]
            self.cpp_info.components["pinocchio_collision"].requires = ["pinocchio_headers", "pinocchio_default", "coal::coal"]
            self.cpp_info.components["pinocchio_collision"].set_property("cmake_target_name", "pinocchio::pinocchio_collision")
            self.cpp_info.components["pinocchio_parsers"].requires.append("pinocchio_collision")
            self.cpp_info.components["pinocchio"].requires.append("pinocchio_collision")
            for comp_name in ["pinocchio_headers", "pinocchio_default", "pinocchio_visualizers", "pinocchio_parsers", "pinocchio"]:
                self.cpp_info.components[comp_name].requires.append("coal::coal")
                self.cpp_info.components[comp_name].defines.append("PINOCCHIO_WITH_COLLISION")
                self.cpp_info.components[comp_name].defines.append("PINOCCHIO_WITH_HPP_FCL")
