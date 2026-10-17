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
