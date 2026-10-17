# Conan 2 配方静态审查（2026-10-17）

本轮仅浏览 Conan Center 配方；未安装 Conan、下载依赖、运行依赖解析或编译。在线 master 和页面缓存会变化，以下不是已经冻结的配方提交。

## 结论

Conan 值得作为第三方库的统一构建/锁定层，但现成 Pinocchio/Coal 配方不能直接用来复制当前 Linux 基线。需要本地配方维护和 MinGW 实编验证；没有看到配方显式禁止 MinGW，不等于已经支持该组合。

## 已审查项目

| 配方 | 静态发现 | 本工程影响 |
|---|---|---|
| Pinocchio | Center 页面提供 3.8.0；配方要求 Eigen >=3.4 <4、Boost >=1.84 <1.90、URDFDOM 4.0.0、urdfdom_headers 1.1.1；碰撞默认关闭 | 目标为 4.1.0；Boost 与 Linux 1.83 不同；需要本地版本/约束/功能审计 |
| Coal | Center 页面为 3.0.2；配方固定 Boost 1.88、Assimp 5.4.3、可选 OctoMap 1.10，最低 C++14 | 目标为 3.0.3；上述依赖分别不同于 Linux 1.83/5.3/1.9.7，不能默认升级 |
| URDFDOM | 4.0 分支依赖 TinyXML2 10.0.0、console_bridge 1.0.2；最低 C++14；支持 shared 选项 | TinyXML2 与已记录 Linux 库相符；其余仍需完整包版本证据 |
| Boost | 配方有 GCC 工具集、编译器路径及多线程配置逻辑 | 需要验证所选模块在 MinGW 下的构建；配方存在不是二进制保证 |
| Abseil | 20250512.1 起要求 C++17；此前 C++14；存在 MSVC 共享库特定限制 | 未找到所审查代码对 MinGW 的一刀切禁止；仍需锁定 Linux 对应版本并实际构建 |

补查后已取得 urdfdom_headers 独立配方：header-library，最低 C++11，include/urdfdom 布局，无二进制库。console_bridge 独立配方全文仍因网页获取失败而未核验，不将可用包页面当作源码审查通过。

## 额外集成工作

- Pinocchio 配方固定 shared-library，默认不开碰撞，并删除上游 CMake 配置、使用 CMakeDeps 重建 target 信息。4.1.0 需要检查组件、导出宏和功能定义；其现有 hpp-fcl 别名映射也需对照 4.1.0 的 Coal 查找方式。
- URDFDOM 配方合并附属 headers，Pinocchio 又单独依赖 urdfdom_headers，需要核对二者版本和 include 优先级。
- 项目当前还使用 urdfdom_LIBRARIES/urdfdom_INCLUDE_DIRS、Boost_LIBRARIES 等变量；Conan 的 CMakeDeps 是否完整提供这些旧接口需要验证，必要时使用明确 imported targets。
- 已核对 Abseil 的 cmake_file_name 为 absl、组件 targets 为 absl::*；包名差异本身不是阻塞。当前项目只查找 absl，未在 cpp 或目标链接中发现直接使用；暂不据此删除它。
- Boost 当前项目定义 BOOST_ALL_DYN_LINK，因此 Conan 的共享/静态选项不能随意用默认值；所有跨 DLL 的 C++ 库需要统一工具链和运行库。
- 自建库应由同一 host profile 构建，显式指定编译器绝对路径、架构、标准、Release 和必要选项。UCRT/MSVCRT、线程模型等还应记录并隔离缓存；不能只写 compiler=gcc 就认为 ABI 完整锁定。
- lockfile 用于锁定依赖/配方修订，还需保存 profile、Conan 版本、工具链证据、构建选项和实际包清单。尚未生成 lockfile，不能提供虚构 recipe revision。

## 建议工作顺序

1. 补 Linux 安装库的完整版本与功能证据，区分 SONAME 和包版本。
2. 选定具体 MinGW 发行版（R2025b 支持的 8.1 或另行验证的新版），保持“不安装”约束直到用户另行授权。
3. 固定 Conan Center Index 提交，准备 Pinocchio 4.1.0 / Coal 3.0.3 本地配方，逐项审计依赖约束；不要用 override 掩盖兼容性要求。
4. 在获准安装后，先做依赖解析、基础库和最小 Pinocchio URDF/动力学程序，验证 Windows 加载来源，再构建本工程。
5. 运行时 AD Windows 平台层仍单独处理；Conan 不补齐 fork/dlopen 等缺口。

建议职责：MSYS2 或明确选定的工具链提供编译工具；Conan 管理第三方 C++ 库；OCS2/BLASFEO/HPIPM 保留固定子模块。不要同时混用 pacman 和 Conan 的同名 C++ 库。

## 来源

- https://conan.io/center/recipes/pinocchio
- https://conan.io/center/recipes/coal
- https://github.com/conan-io/conan-center-index/blob/master/recipes/pinocchio/all/conanfile.py
- https://github.com/conan-io/conan-center-index/blob/master/recipes/coal/all/conanfile.py
- https://github.com/conan-io/conan-center-index/blob/master/recipes/urdfdom/all/conanfile.py
- https://github.com/conan-io/conan-center-index/blob/master/recipes/boost/all/conanfile.py
- https://github.com/conan-io/conan-center-index/blob/master/recipes/abseil/all/conanfile.py

## 收敛审查结果

已追查本机可读证据和可获取的关键配方；不再将“继续查一个包”作为下一轮前提。下面明确分开源码审查结果、证据缺口和实编门槛。

### 基础和传递配方补查

| 配方 | 补查结果 |
|---|---|
| Eigen | header-library，明确导出 Eigen3 / Eigen3::Eigen，匹配当前工程；是否使用 3.4.0 仍需 Linux 版本证据，不能以常见系统版本代填 |
| urdfdom_headers | header-library，最低 C++11；Conan 安装到 include/urdfdom 并传播 include 路径；需检查 URDFDOM 自带的另一份 headers 是否同版 |
| TinyXML2 | 支持 shared；Windows 共享模式传播 TINYXML2_IMPORT；target 为 tinyxml2::tinyxml2，无显式 MinGW 排除 |
| OctoMap | 有 shared/openmp 选项；已审查校验主要针对 MSVC 静态运行库；旧版本有 CMake 4 策略补充逻辑；target 按共享/静态改变，Coal 配方做了映射，需实际消费测试 |
| console_bridge、Assimp | 已确认被现有配方依赖，但多种 GitHub/raw/API 获取均失败，独立配方源码及其补丁尚不可核验；不能宣称审查通过 |

### Linux 基线证据穷尽后的边界

1. build/standalone/CMakeCache.txt 与 build/directory_reconfigure.log 同时确认 Boost 1.83.0。
2. 验收进程 maps 确认 Pinocchio 4.1.0、Coal 3.0.3、Assimp 5.3.0、OctoMap 1.9.7、TinyXML2 10.0.0；URDFDOM .so.4.0、console_bridge .so.1.0 仅提供 SONAME。
3. 缓存提供 Eigen、Abseil、URDFDOM 的安装路径，但未包含足以冻结其完整包版本/补丁的清单。仓库未携带这些安装头文件及完整包数据库，不能在本机恢复未知版本。
4. Linux 配置曾发现 eigenpy / Python / Boost.Python；最新验收进程 maps 未匹配到这些库。说明“配置时发现”不等于“运行时加载”；本项目不调用 Python 接口，但是否关闭相关 Pinocchio 构建功能需显式记录和验证。
5. 仅在 cpp/problem 中找到自定义碰撞球坐标帧并不能证明使用 Coal 碰撞引擎；也不能仅凭未找到 buildGeom 就自动裁掉 Coal。保留基线功能为默认策略。

### 新确认的构建工具问题

BLASFEO 锁定 CMakeLists.txt 使用 cmake_minimum_required(VERSION 2.8.11)。CMake 4 移除了低于 3.5 的策略兼容，默认配置存在失败风险。不能任由 Conan 的 cmake/[>=3.22] 选到任意 4.x；优先验证固定的 3.x 工具，或使用独立的 CMAKE_POLICY_VERSION_MINIMUM 配置适配，不能修改子模块来规避。

根项目最低声明 3.16，但 PinocchioCompatibility.cmake 使用 file(CONFIGURE)，该命令需要较新 CMake；实际工具版本应统一高于项目声明下限，后续实现时同步校正最低版本。Boost 的模块查找在现代 CMake 还有 CMP0167 变化，推荐以 CMakeDeps 的配置包/targets 消费，避免依赖旧 FindBoost 猜测。

### 可实施方案与尚不能保证的事项

**结论：采用 Conan 2 的架构可行；原样采用 Center 的版本/选项组合不可接受；MinGW 8.1 的完整构建可行性尚未得到证明。**

实施应形成以下受版本控制的材料，而不是直接执行默认 conan install：

- 固定 Center Index 提交与本地配方来源；Pinocchio 4.1.0、Coal 3.0.3 本地配方必须审查源码校验值、补丁、组件、导出宏、依赖约束。
- 将已知 Linux 版本作为明确约束；未知的完整包版本保留为空/待证据，禁止自动解算到最新版并声称基线等价。
- 显式配置 shared、碰撞、OctoMap、OpenMP、Python 等选项。不能按 Conan 默认值改变已验收构建功能。
- host/build profiles、编译器绝对路径和发行版、CRT/线程模型、固定 CMake/Ninja/Conan 版本、独立缓存和部署目录。GCC 主版本相同不足以替代上述记录。
- 依赖图解析后生成真实 lockfile；静态审查阶段不能虚构 recipe revisions 或 package IDs。

剩余工作已属于以下具体门槛：

| 门槛 | 判定方式 |
|---|---|
| 补齐未知 Linux 版本/补丁 | 读取原 Linux 安装头文件、CMake ConfigVersion、包数据库和构建选项；Windows 当前副本无法替代 |
| 独立配方获取缺口 | 获取固定 Index 快照后本地审查 console_bridge、Assimp 及所有启用依赖的补丁；当前网络失败不是配方不存在 |
| MinGW/依赖适配 | 用选定工具链构建最小依赖图，不能靠 profile 语法正确判 PASS |
| Conan CMake 消费 | 分别验证 Eigen/Boost/URDFDOM/Pinocchio targets、宏和传递库；检查实际加载 DLL |
| 控制器可用性 | 核心、AD 首次生成、新进程重载、Planner MEX 协议和严格回放逐层通过 |

本轮没有安装授权，所以未越过安装/依赖下载构建阶段；未生成看似可执行但含未知版本的 conanfile/lockfile。审查已给出采用条件和失败边界，剩余不应继续靠浏览器反复推测。

补充来源：
- https://github.com/conan-io/conan-center-index/blob/master/recipes/urdfdom_headers/all/conanfile.py
- https://github.com/conan-io/conan-center-index/blob/master/recipes/eigen/all/conanfile.py
- https://github.com/conan-io/conan-center-index/blob/master/recipes/tinyxml2/all/conanfile.py
- https://github.com/conan-io/conan-center-index/blob/master/recipes/octomap/all/conanfile.py
- https://cmake.org/cmake/help/latest/command/cmake_minimum_required.html
- https://cmake.org/cmake/help/latest/policy/CMP0167.html
