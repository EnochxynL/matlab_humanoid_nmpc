# Windows 初步核查（2026-10-17）

本轮仅核查环境和源码，未安装/更新软件，未配置 MEX，未编译控制器或运行仿真。

## 本机结果

- MATLAB 会话：R2025b，25.2.0.2998904，PCWIN64，mexw64。
- MATLAB 安装：`C:\Program Files\MATLAB\R2025b`。
- C++ MEX 的 Selected 和 Installed 配置均为空。
- 会话当前目录属于其他工程；没有更改 MATLAB 目录或路径。
- VS 2026 Build Tools：MSVC 14.51.36231，cl 19.51.36260，x64；同一开发环境带 CMake 和 Ninja。
- 仅发现 VS 18 和上述 MSVC 工具集，没有发现 VS 2022。
- 开发脚本被默认 PowerShell 执行策略阻止；单独 PowerShell 进程使用 ExecutionPolicy Bypass 可加载，未修改持久策略。
- MSYS2 位于 `C:\msys64`；检查的 UCRT64/MINGW64/CLANG64 C++ 编译器路径均不存在；pacman 已安装列表仅匹配到 gcc-libs，未匹配到编译器、CMake、Ninja 或数值依赖包。
- 首次启动 MSYS2 登录 shell 自动触发初始化提示；沙箱用户 home 创建失败，回退到 /tmp。未执行安装或更新。
- 磁盘容量 CIM 查询被拒绝，剩余空间未确认。

## 仓库与基准

- 分支为 port/windows；OCS2、BLASFEO、HPIPM 的 gitlink 与 dependencies.lock.json 一致。
- HPIPM 已有工作区差异：experimental/andrea/notes/syscop.bib，二进制 37 → 82 字节；未修改或恢复。
- 交接文档列出的 fixture.bin、states.bin、fixture.mat、conversion_values.mat、四份回放基准 CSV 和 acceptance.json 均存在。
- fixture.bin 为 11908968 字节，states.bin 为 1114064 字节；尚未解析文件头或验证内容完整性。
- acceptance.json 记录 Linux 历史验收通过：2401 ticks / 289 MPC updates，四份 CSV 一致；这不是 Windows 验收结果。

## 编译器边界

R2025b 官方表列出 MSVC 2022/2019/2017 和 MinGW 8.1/6.3，未列 VS 2026。
mex 版本历史明确把 VS 2026 支持列为 R2026a 新增。
因此不能把当前 VS 2026 当作 R2025b 已支持的 MEX 工具链，不能通过伪装 VS 版本绕过。
初次核查时工程核心使用 C++20 std::jthread；现已按用户允许改用 std::thread，保留显式退出标志、唤醒及 join，并拒绝重复启动。CMake 暂时仍要求 C++20；老 MinGW 的 MATLAB 支持并不意味着已经验证本核心可编译。

来源：
- https://www.mathworks.com/content/dam/mathworks/mathworks-dot-com/support/sysreq/files/system-requirements-release-2025b-supported-compilers.pdf
- https://www.mathworks.com/help/matlab/ref/mex.html

## 锁定源码中的 AD 边界

- CppAdInterface.cpp 直接实例化 GccCompiler 和 LinuxDynamicLib。
- cg/model/system 只有 system.hpp 和 linux_system.hpp；后者使用 fork/execv，动态库扩展为 .so/.dylib。
- LinuxDynamicLib 受 CPPAD_CG_SYSTEM_LINUX 保护，使用 dlfcn/dlopen。
- cppadcg.hpp 定义 Windows 检测宏不等于具备 Windows 运行实现；本次未找到 LoadLibrary 对应实现。
- 这意味着安装 MSYS2/MinGW 本身不会补齐原生 Windows AD。需要单独设计进程调用、DLL 加载/符号解析、导出和缓存路径适配。
- 适配应在本工程生成副本或独立适配层，保留上游 checkout 和 AD 公式；首次生成及重新加载必须分别验收。

## 已确认的其他适配项

- cpp/CMakeLists.txt 仅声明 LIBRARY 安装目录，缺少 Windows RUNTIME/ARCHIVE 安装规则。
- 共享核心需要明确符号导出；数值共享库也需审计。
- PlannerContext.cpp 的 /proc/self/maps、tests/core_replay.cpp 的 setenv 需要平台处理。
- .gitignore 尚无 mexw64 规则。
- 三个小 MEX 可通过 adapters/matlab 独立构建；其中两个依赖 Eigen，reference_extract 不依赖 Eigen，但当前目录配置仍统一要求 Eigen。

## 待讨论的依赖方案（未选择、未执行）

| 候选 | 优点 | 本工程待核实点 |
|---|---|---|
| MSVC + vcpkg manifest | 可用 baseline/overrides 固定依赖，与 CMake 集成 | 目标 Pinocchio/Coal/URDFDOM 版本和功能是否可用；必要时需 overlay port；不能默认最新版本符合 Linux 基线 |
| MSVC + Conan 2 | 支持 lockfile，可维护自定义 recipe | 所需版本的 recipe 覆盖和维护成本尚未核实 |
| MSYS2 UCRT64 + pacman | 统一 GNU 环境 | R2025b 支持范围、C++20、依赖版本冻结和 AD Windows 实现均未解决；不能视为 Linux 兼容层 |

用户当前倾向 MinGW 路线，允许把 jthread 改写为 thread。优先评估 R2025b 支持的 MinGW 8.1 与当前依赖/源码的兼容性；如需新版 MSYS2 GCC，需单独记录超出 R2025b 官方支持范围的验证结果。当前不安装、不更换版本，也尚未选定包管理器。
包管理器不解决运行时 AD 的平台实现缺口。

来源：
- https://learn.microsoft.com/en-us/vcpkg/consume/lock-package-versions
- https://docs.conan.io/2/tutorial/versioning/lockfiles.html
- https://www.msys2.org/docs/environments/
- https://www.msys2.org/docs/updating/ （滚动更新模式，不能只凭包名保证版本可复现）

后续顺序：确定编译器兼容路线 → 核对 Linux 数值依赖实际版本 → 比较候选包管理器覆盖 → 讨论后选择和安装 → 小 MEX → 独立核心 → AD 首次生成/重载 → Planner MEX → 严格回放。真实物理仿真仍需数值门禁通过及用户确认。

## MinGW 路线补充检查

以下均为静态审查结果，尚未通过 MinGW 实际编译验证：

- 替换 jthread 后，在已扫描的 cpp、adapters 和锁定 OCS2 源码中未发现 span/ranges/concepts 等明确 C++20 用法。C++17 是待验证候选；不能仅凭关键字搜索证明所有模板依赖兼容。暂未改根 CMake 的 C++20 要求。
- cpp 中存在 std::filesystem 和 std::clamp，至少需 C++17。GCC 8 的 filesystem 通常需显式链接 stdc++fs；GCC 9 发布说明才宣布不再需要此额外链接。具体 MinGW 8.1 发行版仍需最小编译/链接探针验证。
- OCS2 ThreadPool.cpp 包含 SetThreadPriority.h，后者无条件使用 pthread_t、pthread_setschedparam、SCHED_FIFO、pthread_self，以及 std::thread::native_handle。必须核对选定 MinGW 的线程模型和 winpthreads 接口，不能仅验证 std::thread 存在。
- BLASFEO 锁定 CMake 提及 MinGW 交叉编译器，且项目已固定 GENERIC；这些线索不足以证明 Windows 共享库导出和运行行为通过。
- HPIPM 的外部 BLASFEO 导入分支将非 MSVC 共享库写为 .so，但本工程先添加 blasfeo target，正常应绕过该分支；不能把它误报为当前必然阻塞。
- 本机携带多个 Linux CMakeCache，不能用于 Windows 配置或重用构建。已有 dependency_check.json 主要验证命名空间和加载库命名，并未提供完整安装依赖版本清单。

AD 最小适配范围（设计审查，未实现）：

1. 为 system.hpp 的目录、文件、绝对路径、扩展名及 callExecutable 提供 Windows 实现；进程调用要正确处理参数引用、标准输入/输出、退出码和含空格路径。
2. GccCompiler::buildDynamic 中 linkerName 只在 Apple/Linux 分支定义，Windows 下不仅链接参数不兼容，还存在编译期未定义变量；需 Windows 构建分支，审计 -rdynamic/-soname 和符号导出。
3. 为 DynamicLib/模型包装提供 LoadLibrary/GetProcAddress/FreeLibrary 等对应实现，保留库关闭前模型通知和 onClose 行为。
4. CppAdInterface 的首次创建、已有库加载均需接入平台选择。仅调整 createDynamicLibrary 的扩展名不会解决加载实现缺口。
5. 使用同一 MinGW 工具链，独立 Windows 缓存目录；验证生成值、Jacobian/Hessian、首次编译及新进程重载，再接入控制器。

GCC filesystem 来源：https://gcc.gnu.org/gcc-9/changes.html

当前结论：MinGW 8.1 + C++17 是值得验证的路线，不再被 jthread 单独阻塞；完整依赖版本、线程实现和 AD 平台层仍需验证。尚无证据要求升级数值库或改变数学。

## MSYS2 包覆盖核查（只读，2026-10-17）

使用本机已有同步数据库执行 pacman -Sl，未执行 -Sy/-S/-Su。索引未刷新，以下是本机索引版本，不是已安装版本或最终选定版本：

| UCRT64 包尾名 | 本机索引版本 |
|---|---|
| gcc | 16.2.0-4 |
| eigen3 | 5.0.1-1 |
| boost | 1.92.0-3 |
| abseil-cpp | 20260526.0-1 |
| assimp | 6.0.5-2 |
| tinyxml2 | 11.0.0-1 |
| cmake | 4.4.3-3 |
| ninja | 1.13.2-1 |

UCRT64 前缀为 mingw-w64-ucrt-x86_64-。在线包页复核 Eigen 5.0.1；Boost 和 GCC 的包修订已分别为 -4 和 -6，进一步说明索引/仓库会漂移。
本机全部仓库索引按 urdf、coal、hpp-fcl、console_bridge/console-bridge、pinocchio、octomap 检索均无匹配。在线直接包页也未确认 urdfdom/coal，因此应作为待自建的候选，不能承诺全部交给 pacman。

最近 Linux 已验收进程加载记录明确包含：Pinocchio 4.1.0、Coal 3.0.3、URDFDOM .so.4.0，均位于 /opt/ros/jazzy；Boost 1.83.0、Assimp 5.3.0、OctoMap 1.9.7、console_bridge .so.1.0、TinyXML2 10.0.0 位于 /usr/lib。共享库 SONAME 不一定包含完整包版本，不能充当完整依赖锁。

Pinocchio v4.1.0 上游 CMake 确认：URDF 功能需要 urdfdom_headers 和 urdfdom；核心查找 Eigen3、Boost filesystem/serialization；Python 接口可关闭，关闭后不需 eigenpy。碰撞、SDF、CasADi 等选项不能擅自变更，需要和 Linux 构建功能对照。此时尚未确定 Coal 是否可从当前控制路径裁去。

管理方案的实际含义：

- 可以采用 MSYS2/pacman 获取工具和部分依赖，但至少还需核查 Pinocchio、URDFDOM、urdfdom_headers、console_bridge、Coal 的源码构建及其依赖。
- 当前 pacman 滚动版本与 Linux 验收版本不同；不能以迁移为由直接升级 Eigen/Boost 等数值依赖。
- 不能承诺 MATLAB MinGW 8.1 与当前 UCRT64 预编译 C++ 包兼容。MSVCRT/UCRT、GCC/libstdc++、线程模型和 ABI 必须一致；仅名称同为 MinGW 不足以保证。
- 若坚持官方 R2025b GCC 8.1，需要统一的对应工具链及依赖重建/合适历史包验证；若采用当前 MSYS2 GCC，则要单独验证超出 R2025b 支持表的 MEX 路线。
- MSYS2 官方仅支持完整升级。复现需保存所选包归档/校验值、完整包清单和自建包配方；仅列 pacman 包名不是版本锁。

参考：
- https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-eigen3
- https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-boost
- https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-gcc
- https://github.com/stack-of-tasks/pinocchio/blob/v4.1.0/CMakeLists.txt
- https://www.msys2.org/docs/environments/
- https://www.msys2.org/docs/updating/
