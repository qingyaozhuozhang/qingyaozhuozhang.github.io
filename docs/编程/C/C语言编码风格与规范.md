---
title: C 代码规范与工程实践｜命名、接口、编译和内存安全
description: 区分团队风格、语言标准与编译器扩展，提供实用的代码审查清单。
tags:
  - C语言
  - 编码规范
  - 代码审查
---

# C 语言编码风格与规范：可执行的检查标准

> 本章专门讨论**怎样写出可靠、可维护的 C 代码**；基础语法与指针原理放在《C 语言核心语法》。原笔记混合了命名规则、编译原理、volatile、内存与 ABI，这里按工程检查场景重排。

## 1. 先区分三种规则

| 规则类别 | 例子 | 强制程度 |
| --- | --- | --- |
| C 标准/未定义行为 | 不能越界访问，不能读未初始化对象 | 必须遵守 |
| 工具链/平台约束 | `__attribute__`、链接脚本、ABI 对齐 | 依平台遵守 |
| 团队风格 | 4 空格、`snake_case`、宏大写 | 团队保持一致 |

例如“所有结构体 typedef 名必须以 `_t` 结尾”是某些团队的习惯，但在 POSIX 环境中 `_t` 后缀可能与保留命名空间冲突，不应称为通用 C 语言强制规则。

## 2. 命名与目录组织

```text
project/
├── include/
│   └── sensor_reader.h
├── src/
│   ├── sensor_reader.c
│   └── main.c
├── tests/
│   └── test_sensor_reader.c
└── CMakeLists.txt
```

| 对象 | 建议 | 示例 |
| --- | --- | --- |
| 函数 | 动词 + 目的，snake_case | `sensor_read_frame` |
| 变量 | 描述含义与单位 | `timeout_ms` |
| 宏 | 大写加下划线 | `MAX_FRAME_SIZE` |
| 文件 | 与模块或职责对应 | `sensor_reader.c` |
| 状态枚举 | 有清晰的名字空间前缀 | `SENSOR_STATE_READY` |

不必要的变量编号 `a1`, `a2` 可优先改为数组；但也不要机械地拒绝常见局部循环变量 `i`, `j`。

## 3. 排版与函数复杂度

```c
int filter_score(int score, int minimum, int maximum) {
    if (minimum > maximum) {
        return -1;
    }
    if (score < minimum) {
        return minimum;
    }
    if (score > maximum) {
        return maximum;
    }
    return score;
}
```

统一缩进、括号、空格及换行；函数太长时按职责拆分。**推荐提前返回处理错误**，降低嵌套层级。缩进最多几层属于团队可选约束，不是所有项目的强制语言规则。

## 4. 接口契约：输入、输出和所有权

函数设计必须明确：

1. 参数允许为 `NULL` 吗？数组大小如何传入？
2. 错误如何表示（返回码、输出参数还是 `errno`）？
3. 谁申请内存、谁负责释放？
4. 函数是否可重入、能否在中断或线程中调用？

```c
#include <stddef.h>

// 把 src 的前 n 个元素求和，结果写到 out。
// 返回 0 表示成功；-1 表示参数非法。
int sum_ints(const int *src, size_t n, long long *out) {
    if (out == NULL || (src == NULL && n != 0)) return -1;
    long long total = 0;
    for (size_t i = 0; i < n; ++i) total += src[i];
    *out = total;
    return 0;
}
```

真实工程还需要检查整数求和是否可能溢出；示例重点在接口边界与所有权约定。

## 5. `const`、`static`、`volatile` 别混淆

| 关键字 | 核心含义 | 常见误解 |
| --- | --- | --- |
| `const` | 通过该左值不能修改对象 | 不代表对象永远不会被其他路径修改 |
| `static` | 作用域不同而有静态存储期或内部链接 | 不等于“线程安全” |
| `volatile` | 强制观察特定对象访问的副作用 | **不保证原子性或线程同步** |

```c
static unsigned int module_counter = 0; // 仅本翻译单元可见

void increment(void) {
    ++module_counter;
}
```

硬件寄存器访问可能使用 `volatile`，线程之间的同步通常需使用 C11 原子操作或平台同步原语，不能仅凭 `volatile` 解决数据竞争。

## 6. 编译、链接与宏

```mermaid
flowchart LR
    A[预处理] --> B[编译]
    B --> C[汇编]
    C --> D[链接]
    D --> E[可执行文件]
```

宏文本替换容易出现重复求值和括号错误；常用逻辑优先用函数或 `static inline`：

```c
static inline int square(int x) { return x * x; }
```

注意：`int` 平方仍可能溢出，函数替代宏只能解决部分问题。

## 7. 结构体对齐、字节序和跨平台

结构体可能包含填充字节，字段宽度和大小端也可能因平台而异。网络包/磁盘格式应使用**明确的协议序列化规则**，而不是直接写 `fwrite(&struct, sizeof struct, 1, fp)` 期待所有平台互通。

GNU `__attribute__`、编译器内置函数、特定 ABI 不属于 ISO C 标准，使用时应明确工具链版本并提供可移植性说明。

## 8. 嵌入式与中断上下文约束

- 尽量缩短中断处理程序，非必要工作放到任务/线程处理。
- 避免在实时关键路径里使用不确定延迟的动态分配、阻塞 I/O。
- 与 ISR 共享状态时理解原子性、内存序与平台临界区实现。
- 某些嵌入式项目禁用递归是出于栈深难以界定的工程约束，**不是 C 语言禁止递归**。

## 9. 工具链与审查检查表

```bash
gcc -std=c11 -Wall -Wextra -Wconversion -Wshadow -g -O0 main.c -o app
# Linux 主机调试可使用 sanitizer；目标 MCU 不一定支持
gcc -std=c11 -fsanitize=address,undefined -g main.c -o app_asan
```

代码审查按顺序检查：边界与返回码 → 内存所有权 → 数据竞争 → 整数溢出与转换 → 接口命名和模块职责 → 测试覆盖。

## 10. 一次完整代码审查练习

设某函数负责把传感器帧保存到缓冲区。要求添加最大长度检查、返回状态、空指针检查、资源清理，并用异常长度（0、最大值、超出最大值）测试。只有“代码格式好看”不能说明接口安全。
