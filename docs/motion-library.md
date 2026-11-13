# 导览动作库：Pose / Motion / Sequence

## 当前只需两步

在机器人稳定站立、硬件急停可用、现场有人监护且没有其他 `rt/arm_sdk` 发布进程时，从项目根目录运行：

```bash
./build/teach_with_name
```

按 Enter 记录关键帧，输入 `q`，然后输入清单中的动作名。`neutral` 只需一帧；动态动作可以只录一帧目标手势，也可以录多帧核心轨迹，不用额外录入 neutral。

录制完成后可打开本机时间轴窗口，调整帧间比例并试播：

```bash
python3 tools/motion_timing_ui.py
```

浏览器打开 `http://127.0.0.1:8765`。窗口中的“动作核心总时长”只用于本次试播；“保存比例”只修改 `.dat` 中的相邻关键帧时间比例，不保存试播总时长。按当前比例完成真机试播后直接保存，窗口会一并确认该版本；修改比例后需重新试播。也可以继续使用原单命令试播，例如：

网页分别显示核心请求、核心规划和包含入口过渡、末帧保持的动作段规划。平滑模式会按速度和加速度约束延长过短的请求；原版线性按请求时间执行。改变输入后旧规划数字会清除，重新试播才显示新结果。

“试播方式”可选“平滑”或“原版线性”。原版线性参照 `play.cpp` 的 `lerp_arm`：双臂每 2 ms 更新一次指令，入口和帧间段均作线性插值，不在关键帧处添加保持或五次缓起缓停，也不按 0.4 rad/s、0.8 rad/s² 演示限值扩时。切换到原版线性时，网页把入口设为 300 ms，核心总时长设为“帧间段数 × 300 ms”；可继续调整总时长和比例，设置 500 ms 的核心请求就规划为 500 ms。单帧姿态或动作也可以用原版线性进入目标姿态；帧间比例仍仅对多帧动作开放。

原版线性试播的速度由关节角度差和所填时长直接决定，**不保证元数据中的速度、加速度上限，也不保证关键帧处速度连续**，仅用于有人监督的单次试播。继续保留状态超时、遥控退出、跟踪误差和已核定的关节位置范围检查。末帧保持结束后，按各段原有用时倒放已走过的轨迹，返回接管姿态再释放控制权；取消时先停止继续播放，再从当前轨迹位置倒序收回，不等待走完剩余动作。实际指令发送受约 2 ms 控制周期和系统调度影响，机械运动还受伺服跟踪影响。

两种试播方式都需要约 2 秒接管渐入和约 2 秒释放渐出，这些时间以及自动返回不计入“动作段规划”。正式 Sequence 使用原有平滑规划和限速规则。

通过 Windows 上的 VS Code Remote SSH 使用时，在“端口”面板转发远端 `8765`，再用 Windows 浏览器打开转发后的本机地址。当前远程工作区已配置自动转发 `8765` 并打开浏览器。服务在启动它的 Ubuntu 终端前台运行；按 Ctrl+C 或点击网页中的“关闭网页服务”会关闭服务。试播中须先点“取消”，等待控制权释放后再关闭。重复运行脚本时若已有同一时间轴服务，会显示现有地址，不会再启动一个进程。修改比例后可直接点“真机试播”，本次试播使用页面中尚未保存的比例；确认合适后再保存。

```bash
python3 tools/motion_ctl.py preview greet_open
python3 tools/motion_ctl.py preview greet_open --interpolation linear --duration-ms 500
```

命令会自动完成真机试播并释放控制。播放完成后会询问是否确认动作可用：输入 `y` 自动记录确认人和时间，输入其他内容则保留文件但不确认。当前录制阶段无需手动运行 `list`、`validate`、`edit-meta`、`mark-tested` 或 `approve`；这些信息由录制和播放程序内部处理。播放未完成或被中断时不会出现确认提示。

## 源码基线与兼容性

- `teach_with_name` 原来按 Enter 记录双臂 14 关节姿态，把每一行写成 `0.3秒 + 14角度`，保存于 `motions/<类别>/<名字>.dat`。它本身从未自动补 neutral，但旧录制说明建议手工补首尾 neutral。
- `play` 使用独立的 `arm7_action.dat`，线性插值播放，L2 时倒序返回起始姿态；它不是导览动作库播放器。
- `play_with_motions` 是旧拳击示例，读取平铺 `motions/*.dat`，线性插值并依赖 guard 姿态；不读取导览元数据。
- 旧四段 JSON 标签仍把 a/b/c/d 当作四个独立文件，以目标总时长缩放每个文件的全部行。此接口保持兼容，但首帧仍计入旧接口时长；新的过渡和核心时长语义只由 Sequence 提供。

旧 `.dat` 的两行文件头和 14/15 列动作点格式未变；旧文件可通过 `init-meta` 导入。新版用元数据 `schema_version=3` 表达首尾锚点和时间信息，旧 v2 已批准动作仍可用于四段接口，但 Sequence 要求 v3。

## 三层目录

| 层 | 目录 | 内容 |
| --- | --- | --- |
| Pose Library | `poses/basic/` | 手动录制的 `neutral`，恰好 1 帧 |
| Motion Library | `motions/<类别>/` | `greet_open` 等动态动作，每个至少 2 帧 |
| Sequence Library | `sequences/*.yaml` | 以姿态、手势、过渡、保持时间组合流程 |

22 个合法名称、类型和优先级在 `config/motion_catalog.json`。录制器只接受清单内的名称。`metadata/<名字>.json` 统一保存审批记录和 SHA-256；不存在默认的 neutral 入口或出口。`neutral` 是你自己录制并验收的静态姿态，不从宇树运控自动抓取，也不等同于交还控制权。

## 录制一个新动作

从项目根目录运行 `./build/teach_with_name`，确认机器人稳定站立，硬件急停可用，并关闭其他 `rt/arm_sdk` 发布进程。程序只控制双臂 14 关节并保持腰部，不接管下半身。先把手臂摆到该动作的**真实入口姿态**，按 Enter 记录首帧；移动到关键姿态依次按 Enter；在**真实出口姿态**按 Enter 记录末帧，再输入 `q` 和清单动作名。

静态 `pose` 恰好记录一帧。动态 `motion` 可以只记录一帧目标手势：播放时从当前手臂姿态平滑过渡至该帧，不需要额外录入 neutral；它没有帧间比例，元数据标称核心时长为 0。多帧 `motion` 的首尾帧是核心轨迹锚点；帧间比例初值为等比例，每段在 `.dat` 中暂记 0.3 秒。按 Enter 的实际间隔不影响比例。第一行的 0.3 秒只用于旧播放器兼容，Sequence 不把它算入核心动作时长。旧动作仍按原有时间比例读取。保存时自动写 v3 `draft` 元数据；如果覆盖旧文件，会增加版本并清除验收状态。

录制 `neutral` 时只按一次 Enter，输入 `q` 后命名为 `neutral`；保存到 `poses/basic/neutral.dat`。录制器本身会接管双臂并锁定腰部，只能在机器人稳定站立时运行，录制完成后会释放控制。导览服务运行时才使用下面的 `acquire` / `release` 流程。

元数据包含动作名称/类型、14 关节顺序、关键帧时间戳、标称时长、首尾关节角、循环与中断策略、standing 模式、速度/加速度上限及关节限位配置引用。录制器自动生成，试播确认后自动更新；当前无需手工管理。`loopable` 默认为 false，`interruptible` 默认为 true。

## 验收与关节限位

真机试播完成后，在终端确认动作可用即可。程序自动记录本机用户名与带时区的确认时间，播放失败不会自动确认。

Sequence 会额外要求 `config/arm_joint_limits.json` 中逐关节边界和验收记录为 `verified=true`。仓库模板故意留空且未核定；必须由 G1 EDU+ 实机资料和现场人员核对后填写。未核定时服务端拒绝 Sequence。软件仅对关节位置、速度、加速度和命令跟踪做约束；不会证明避碰、站立稳定、行走兼容或按键接触安全。当前服务端仅用于静止站立，下半身继续交给宇树原有运控。

### 未批准动作的单次试播

试播是独立于正式播放的入口，接受 `draft` / `tested` / `approved` 的 v3 动作，并校验清单、文件摘要、首尾锚点和速度/加速度。如果关节限位配置已核定，试播还会检查逐关节范围；未核定时允许由现场操作员监督试播，但状态会显示 `joint_limits_verified=false`，此时**没有逐关节范围校验**。`--check` 只从动作首帧姿态离线规划，不连接机器人：

```bash
python3 tools/motion_ctl.py preview neutral --check
```

当前仓库的 `config/arm_joint_limits.json` 仍是未核定的空模板。上述命令可以通过，但会提示未做关节范围校验；正式 Sequence 仍要求按 G1 EDU+ 资料核定 14 个关节的 `min_rad`、`max_rad` 后才能运行。不要按录制轨迹的极值反推机械限位。

确认机器人稳定站立、硬件急停可用、现场有人监护且无其他 `rt/arm_sdk` 发布者后，在项目根目录运行**一条命令**：

```bash
python3 tools/motion_ctl.py preview neutral
```

命令会先做离线校验，再自动启动专用服务端、观察至少 500 ms 有效 lowstate、渐入接管、试播、回到本次接管前姿态、释放控制权，然后退出。与原始 `play` 一样，这种有人监护的单次试播不根据腿部和机身瞬时速度自行判断是否停稳；操作员须先确认机器人没有行走。仍会检查 lowstate 时效、遥控退出和关节跟踪。无需另开终端或输入 `PREVIEW`。按 Ctrl+C 会请求取消和释放；硬件急停仍须可用。只想检查数据或演练完整通信流程时分别使用：

```bash
python3 tools/motion_ctl.py preview neutral --check
python3 tools/motion_ctl.py preview neutral --offline
```

每次命令只试播**一个**动作。多帧 motion 默认按标称核心时长播放，也可传入 `--duration-ms` 和 `--weights-json` 临时调整；单帧 pose 或 motion 只做入口过渡和保持，可用 `--transition-ms` 和 `--hold-ms` 调整试播。平滑模式入口默认 1.5 秒，受速度/加速度约束；`--interpolation linear` 的入口默认 300 ms，入口和核心均按请求时间线性执行。两种方式末帧默认保持 0.5 秒，然后自动返回并释放控制。只有真机单命令试播完成并输入 `y`，程序才会记录确认；`--check`、`--offline` 和 `--send` 不会自动确认。命令报错时检查其输出。

## Sequence 与播放

示例：[greet_guest.yaml](../sequences/greet_guest.yaml)。YAML 由 PyYAML 安全解析，依赖见 `requirements.txt`。

```bash
python3 tools/motion_ctl.py sequence --file sequences/greet_guest.yaml --check
./build/motion_library_server
python3 tools/motion_ctl.py acquire
python3 tools/motion_ctl.py status
python3 tools/motion_ctl.py sequence --file sequences/greet_guest.yaml
python3 tools/motion_ctl.py status
python3 tools/motion_ctl.py release
python3 tools/motion_ctl.py status
```

`--check` 不连接机器人，从首个步骤的入口姿态离线规划；仍要求动作已批准且关节限位已核定。正式服务端读取当前实测双臂姿态；若与当前命令姿态偏差不超过 0.03 rad，会先用一段限速轨迹接到实测姿态，再进入步骤首帧，偏差更大则拒绝序列。多帧 motion 的 `transition_ms` 控制进入首帧，`duration_ms` 控制首帧到末帧的核心轨迹总时长；单帧 motion 的 `transition_ms` 控制进入目标姿态，若设置 `duration_ms` 则以它覆盖该过渡目标时长。`hold_ms` 是末帧保持。请求过快时规划器会延长实际时长。每段使用五次多项式，段边界位置、速度和加速度连续。`repeat>1` 仅适用于已标记 `loopable=true` 的动作。

服务端默认以 `ownership=released` 启动，不发布上半身关节指令。机器人停稳后发送 `python3 tools/motion_ctl.py acquire`，确认 `status` 中 `ownership=owned`，再播放 Sequence 或单个动作。行走前发送 `python3 tools/motion_ctl.py release`，它先取消动作并减速、回到本次接管前的双臂和腰部姿态，再将 `arm_sdk` 权重降为零；确认 `status` 中 `ownership=released` 后才启动导航。`neutral` 只在需要静止展示的 Sequence 里调用，不承担行走过渡姿态。重新停稳至少 500 ms 后才能再次接管。若接管末端实测姿态偏差过大，控制器会退回 `released`；若拥有控制权时检测到腿部关节速度超限，会立即将权重设为零并退出服务端，需人工检查后重启。此流程尚未联调机器人原生运控的走动/静止模式切换，不能仅凭关节速度判定所有运控模式；导航端还需遵守上述状态握手。

`interrupt` 减速后暂停并记录当前 Sequence 步骤/关键帧；`resume` 从未完成部分重新规划；`stop` 取消并清除断点。不可普通中断的步骤会拒绝 `interrupt`，但 `stop` 仍可安全取消。若短减速轨迹不满足约束，控制器会走到当前步骤的零速边界后暂停，响应时间可能延长。讲解文本、TTS 与导航断点仍由上层系统管理，机器人硬件急停必须独立可用。
