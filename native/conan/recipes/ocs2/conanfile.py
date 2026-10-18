from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy
from conan.tools.scm import Git
from pathlib import Path
import os

class Recipe(ConanFile):
    name = 'ocs2'
    version = 'cci.26386754b8bf'
    license = 'BSD-3-Clause'
    package_id_non_embed_mode = "full_mode"
    package_id_embed_mode = "full_mode"
    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    exports_sources = "CMakeLists.txt", "cmake/*", "cmake/windows/*", "Targets.cmake", "modules.json"

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
        tc.variables["BUILD_False_LIBS"] = bool(self.options.shared)
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
        self.requires("eigen/3.4.0", transitive_headers=True)
        self.requires("boost/1.88.0", transitive_headers=True)
        self.requires("pinocchio/4.1.0", transitive_headers=True)
        self.requires("coal/3.0.3", transitive_headers=True)
        self.requires("urdfdom/4.0.0", transitive_headers=True)
        self.requires("hpipm/cci.255ffdf38d3a", transitive_headers=True)
        self.requires("blasfeo/cci.ae6e2d1dea01", transitive_headers=True)

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "ocs2")
        edges = {'ocs2_thirdparty': [], 'ocs2_core': ['ocs2_thirdparty', 'eigen::eigen', 'boost::boost'], 'ocs2_oc': ['ocs2_core'], 'ocs2_mpc': ['ocs2_core', 'ocs2_oc'], 'ocs2_qp_solver': ['ocs2_core', 'ocs2_oc'], 'ocs2_ddp': ['ocs2_core', 'ocs2_oc', 'ocs2_mpc', 'ocs2_qp_solver'], 'ocs2_robotic_tools': ['ocs2_oc', 'ocs2_core', 'ocs2_thirdparty'], 'ocs2_pinocchio_interface': ['ocs2_core', 'ocs2_oc', 'ocs2_robotic_tools', 'pinocchio::pinocchio', 'urdfdom::urdfdom', 'coal::coal'], 'ocs2_centroidal_model': ['ocs2_core', 'ocs2_robotic_tools', 'ocs2_pinocchio_interface'], 'hpipm_catkin': ['ocs2_core', 'ocs2_oc', 'ocs2_qp_solver', 'hpipm::hpipm', 'blasfeo::blasfeo'], 'ocs2_sqp': ['ocs2_core', 'ocs2_mpc', 'ocs2_oc', 'ocs2_qp_solver', 'hpipm_catkin']}
        for name, dependencies in edges.items():
            component = self.cpp_info.components[name]
            component.set_property("cmake_target_name", name)
            component.libs = [] if name == "ocs2_thirdparty" else [name]
            component.requires = dependencies
            component.defines = ["BOOST_ALL_DYN_LINK", "BOOST_MPL_LIMIT_LIST_SIZE=30",
                "BOOST_MPL_LIMIT_VECTOR_SIZE=30", "BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS",
                "BOOST_FUSION_INVOKE_MAX_ARITY=10", "BOOST_FUSION_INVOKE_PROCEDURE_MAX_ARITY=10",
                "BOOST_FUSION_INVOKE_FUNCTION_OBJECT_MAX_ARITY=10",
                "PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR", "PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR"]
            if self.settings.os == "Windows":
                component.defines += ["NOMINMAX", "_USE_MATH_DEFINES"]
                component.cxxflags = ["-Wa,-mbig-obj"]
        self.cpp_info.components["ocs2_core"].cxxflags.append("-fopenmp")
        self.cpp_info.components["ocs2_core"].system_libs = ["gomp", "mingwthrd"] if self.settings.os == "Windows" else ["gomp", "pthread", "dl"]
