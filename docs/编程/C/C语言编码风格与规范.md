## 一.省流

> **省流表示简洁明了地概括内容**
>
> 1.命名用下划线命名法，宏定义字母全大写，预编译命令的#else,#endif行末加注释说明条件
>
> 2.全局变量加g\_前缀，全局指针加p_前缀
>
> 3.结构体/联合体/枚举等复杂数据类型，除非不公开，统一用typedef定义名称，并加上_t后缀
>
> 4.缩进最多接受6层，超过6层必须重写；预编译如果嵌套命令不缩进
>
> 5.嵌入式程序中不准使用递归
>
> 6,.对于有返回值的函数，必须返回一个默认值，不能什么都不返回

---

## 二.命名禁忌

> 1.单字母：int a,b,c;（但是循环变量可以使用i,j,k，只是不要把i,j,k定义成全局变量）
>
> 2.重复无意义的字母：int aa, bb,cc,dd;
>
> 3.拼音及拼音首字母缩写：char[30] mingZi,sb,nb;(对于一些公认的拼音，可以使用)
>
> 4.不规范的缩写:int cont_freq

---

## 三.命名规则

> 1.统一使用下划线命名法，对于第三方代码保留其命名风格，不要修改
>
> 2.统一性：编写一个模块的代码时，最好使用一个统一的前缀，比如操作NAND Flash的代码，可以加一个nand_的前缀
>
> 3.与作用域关联：变量命名的长度和他的作用域有关，作用域越大，命名尽可能越长。这样能最大程度避免命名冲突
>
> 4.最长的长度：尽量不要超过6个单词
>
> 5.避免出现编号：如number1,number2,可以使用数组代替。除非在逻辑上必须使用编号，无法用数组代替，比如：USART1,USART2

---

## 四.

> 1.函数
>
> > （1）函数命名要和功能或者行为准确对应。一般是动词 + 名词

> 2.变量
>
> > （1）变量命名尽量用形容词 + 名词

> 3.句柄
>
> > （1）句柄加_handle后缀，句柄名称与类型和功能匹配就可以了

> 4.头文件
>
> > （1）只引用需要的头文件，不要包含一堆头文件
> >
> > （2）#include中的文件名对大小写敏感
> >
> > （3）头文件放在Inc目录，源文件放在Src目录
> >
> > （4）对于 C++ 代码，头文件使用 .hpp 后缀
> >
> > （5）如果一个目录中有多个子目录中的头文件需要包含，使用相对路径，不要在IncludePath中添加

> 5.源文件
>
> > （1）对于 C++ 代码，使用.cpp后缀
> >
> > （2）头文件开头和末尾必须加上固定格式的宏定义，即#ifdefine ... #endif（#ifdefine表示如果没有定义则条件为真，可以进入，如果有定义则跳过这串代码）
> >
> > （3）二元、三元操作符前后都要加空格，如：= + - < > * / % ^ <= >= == != ? :
> >
> > （4）一元操作符后不要加空格，如：&（取地址） *（取指针的内容） + - ~ ！，但是如果声明指针变量，*星号放在靠近变量的一侧，星号左边有一个空格
> >
> > （6）自增和自减运算符前后都不加空格，如：num++, --num
> >
> > （7）逗号，分号只在后面添加空格，如：int age_max, age_min;如果分号在行末，空格可加可不加
> >
> > （8）块注释/* xxx */ 与注释内容 xxx 前后各一个空格
> >
> > （9）行注释// xxx 双斜线后有一个空格，且两条斜线与行对齐，不要放在行首

---

## 五.

> 1.所有左花括号不换行，右花括号单独一行，对于else语句右花括号不换行。这样是为了减少代码阅读负担，一个左花括号占一行代码会看起来比较长，右花括号单独一行可以提醒读者这个代码块到底了
>
> 2.空行
>
> > （1）文件头注释后有一个空行
> >
> > （2）函数值前须加一个空行
> >
> > （3）在注释某个函数，结构体、枚举时候不要加空行

> 3.代码行
>
> > （1）一行代码只做一件事情，比如定义一个变量，或者一条语句
> >
> > （2）一行最多80个字符，写代码经常需要分屏，一行代码如果太长就不太方便了
> >
> > （3）if,for,while,do这类语句，必须加花括号，一个语句也必须加
> >
> > （4）及时清理未使用的函数、变量等死代码（未使用的代码）（要注意逻辑上永远不可能执行的代码）

> 4.函数
>
> > （1）一个函数尽量不要超过10个局部变量
> >
> > （2）函数参数不要太多，最多6个
> >
> > （3）有返回值的函数，末尾必须要return默认值，如果没有返回任何值编译会给警告
> >
> > （4）对于无参的函数，加void,不要在括号里什么都不写。例如：int main(void),而非int main()
> >
> > （5）函数的功能要单一，不能处理太多的业务逻辑，对于单一模块的文件服务的对象也要单一，不要让一个文件或者函数成大杂烩

> 5.变量
>
> > （1）所有局部变量哪里用就在哪里声明。使用前必须定义，不准使用未定义的变量
> >
> > （2）如果初值可以确定，写在一行上，不要另起一行进行赋值操作
> >
> > （3）不确定初值的变量给0或1，不确定初值的指针给NULL,不确定的结构体变量给{0}
> >
> > （4）如果一个全局变量只有一个文件使用，**尽量加static修饰符**，这样能避免其他人在其他文件调用本文件的变量，产生预料之外的后果。全局变量如果不加static,整个程序是可以访问的，任何一个地方的extern声明都可以访问到这个变量

> 6.复杂数据类型
>
> > （1）对于结构体或者联合体，除非不公开，只用来声明一个或几个变量，否则统一使用typedef定义名称，并结尾带_t后缀
> >
> > （2）对于枚举，枚举成员要使用大写
> >
> > （3）申请的内存必须及时释放，否则会造成内存泄漏
> >
> > （4）指针嵌套（比如指向指针int*的指针int**）最多三层，数组最多三维数组
> >
> > （5）对于结构体、联合体这种复杂数据类型，函数之间传递时尽量使用指针，以减少内存开销，以及避免可能产生的内存泄漏

> 7.杂项
>
> > （1）**不要对浮点数做相等或者不等判断**，浮点数由于精度问题并不会按照我们的方式执行。示例：0.1 + 0.2 == 0.3的值，结果显示0或false
> >
> > （2）不要过渡注释
> >
> > （3）文件编码统一使用UTF-8+
>
> 8.结构体
>
> > （1）结构体前注明简介，每个成员都说明作用

---









# C语言编译原理

> 1.C语言代码怎么一步一步变成可执行文件
>
> C语言是一个高级语言
>
> 想要被机器识别需要先转化为一个汇编语言 → 然后转换为二进制文件（机器语言） 
>
> C语言文件后缀为.c，汇编语言后缀是.s，机器语言后缀是.o，可执行文件的后缀是.exe
>
> 2.C语言到汇编语言
>
> （1）进行预处理（可以把.c文件呈现在.txt文件）（预处理是不生成任何文件的，只是把内容用另外一种形式表达出来）
>
> （2）变成汇编语言（从.txt文件变成.s文件）
>
> 3.汇编语言到机器语言
>
> 从.s文件变成.obj文件（里面都是二进制的内容）（二进制指令被称为机器码）
>
> 4.编译器本身也是一个程序
>
> 5.语法高亮显示、静态分析、物件导向编程、函数式编程、函数库、链接工具、构建工具和除错工具

---

> （1）C源代码文件（2）预处理阶段（3）编译阶段（4）链接阶段（5）可执行文件
>
> 预处理
>
> > -1 头文件包含
> >
> > -2 宏定义
> >
> > -3 条件编译

> 编译
>
> > -1 词法分析
> >
> > -2 语法分析
> >
> > -3 语义分析
> >
> > -4 优化

> 汇编
>
> > 将编译器生成的中间代码转换为目标机的汇编语言执行

> 链接
>
> > -1 符号解析
> >
> > -2 地址分配
> >
> > -3 重定位

---

> .obj和.exe文件的区别
>
> .obj文件：编译后的中间文件，包含相对地址和未解决的外部符号，不能直接运行
>
> .exe文件：连接后的可执行文件，包含绝对地址和所有必要的资源，可以直接运行
>
> 二者都是二进制文件
> 
>
>
> 函数声明由返回类型和函数签名组成。函数签名就是函数的名称和它的输入参数列表。
>
>
> 

---

> 详细具体步骤
>
> 1.构建C项目
>
> （1）头文件（.h）与源文件（.c）
>
> 头文件通常包含枚举、宏和类型定义，以及函数、全局变量和结构的声明
>
> 2.预处理
>
> 在预处理步骤之后，头文件的内容被复制到源文件中，从而得到一段代码（编译单元）
>
> 函数的声明都从头文件复制到编译单元，注释也会被删除
>
> 3.编译
>
> 编译步骤的输入是由上一步处理所得的编译单元，输出是相应的汇编代码。
>
> 编译器解析编译单元，将其转换为目标体系结构下的汇编代码。目标体系结构是指编译程序并在其上运行的硬件或CPU
>
> 4.汇编
>
> 汇编的目标是基于上一步中编译器生成的汇编代码生成实际的机器指令。每个体系结构都有自己的汇编器。通过汇编得到的含有机器级指令的文件称为目标文件。（这类目标文件是不可执行的）
>
> 汇编是编译单个源文件的最后一步（当我们得到与源文件对应的可重定位目标文件时，就完成了对它的编译）
>
> 5.链接
>
> 组合可重定位的目标文件，来创建可执行的目标文件
>
> 汇编器和链接器可以独立于编译器运行

---

## 编译器的类型

> 编译器是一种将高级编程语言编写的源代码转换为机器语言或中间代码的工具，使计算机能够执行该程序
>
> 1.基于源语言的分类
>
> （1）C编译器（2）C++编译器（3）Java编译器（4）Python编译器
>
> 2.基于目标语言的分类
>
> （1）机器码编译器（2）中间语言编译器
>
> 3.单次和多次通过编译器
>
> （1）单次通过编译器（2）多次通过编译器
>
> 4.跨编译器
>
> 跨编译器在一种平台上运行，但生成另一种平台的代码，常用于嵌入式系统或为不同硬件架构编写软件时非常重要
>
> 5.常见的编译器
>
> （1）GCC（2）Clang（3）MSVC（4）Javac（5）Intel Compiler

---

> 嵌入式编译器和PC编译器的区别
>
> （1）目标平台不同
>
> 嵌入式编译器
>
> > 1.针对特定嵌入式硬件（如ARM、RISC-V、STM32等微控制器/处理器）
> >
> > 2.通常需要交叉编译（在PC上编译，输出目标设备可执行的二进制文件）
> >
> > 3.需适配硬件的寄存器、中断、外设等底层细节
>
> PC编译器
>
> > 1.面向x86 / AMD64架构（如Intel / AMD处理器）
> >
> > 2.直接在本地编译并且运行，无需交叉编译
> >
> > 3.依赖操作系统（如Windows、Linux）提供的标准库和API
>
> (2)资源优化方向不同
>
> 嵌入式编译器
>
> > 1.代码体积优先：因嵌入式设备内存（如Flash/RAM）优先，需优化代码大小（如使用精简库、压缩中间文件）
> >
> > 2.功耗优化：针对电池供电设备，优化指令执行效率以降低能耗
> >
> > 3.实时性：支持硬实时系统（如FreeRTOS），确保任务响应时间确定性
>
> PC编译器
>
> > 1.执行速度优先：利用多核、缓存等硬件特性，优化运行效率
> >
> > 2.对代码体积不敏感（PC内存通常以GB为单位）
> >
> > 3.依赖操作系统调度，实时性要求较低
>
> （3）开发环境差异00000000
>
> 嵌入式编译器
>
> > 1.集成开发环境（IDE）通常包含硬件调试工具（如JTAG/SWD调试接口)
> >
> > 2.需要手动配置硬件初始化代码（如时钟、外设寄存器）
> >
> > 3.依赖特定厂商库（如STM32CubeMx、Arduino库）
>
> PC编译器
>
> > 1.IDE更侧重软件调试（如GDB、VS Code调试器）
> >
> > 2.无需关心硬件底层细节，直接调用系统API
> >
> > 3.支持标准库（如C++STL、Python库）和框架(如Qt、NET)
>
> （4）编译流程复杂度
>
> 嵌入式编译器
>
> > 1.需手动配置链接脚本（定义内存分区、入口地址等）
> >
> > 2.可设计Bootloader、内核、应用程序的多阶段编译
> >
> > 3.需处理硬件相关的中断向量表、异常处理等
>
> PC编译器
>
> > 1.自动处理连接和内存分配，用户无需干预底层细节
> >
> > 2.直接生成可执行文件或动态库，依赖操作系统加载
>
> （5）典型工具举例
>
> 嵌入式编译器
>
> > 1.Keil MDK（ARM微控制器）、GCC、IAR
> >
> > 2.集成硬件调试工具（如Segger J-Link）
>
> PC编译器
>
> > 1.GCC、Clang(Linux/macOS)、MSVC（Windows）
> >
> > 2.支持跨平台开发（如通过MinGW在Windows编译Linux程序）
>
> 最主要的点：嵌入式编译器针对特定硬件（如ARM、STM32）进行定制，需要交叉编译并且要优化代码体积和功耗；PC编译器面向通用x86架构，本地编译且侧重运行效率）

















### 位运算

- 位运算与写入
  - 位置零用 [与] & ，置1用[或] |
  - 从x位到y位一般表示成bit[y:x]，y在x前面是因为低位在后面。而我们说从x到y一般是从低位到高为。而说从x到y个字节可以表示成byte[x:y]
  - 其实借助结构体位域也可以实现位操作，参照：位域。但是位域不具有可移植性。如果你的代码在不同平台与不同编译器下编译，那么位域的大小和地址不一定是固定的
- iso646.h
- 取余与位运算
  - 取余数也叫取模
  - (void)result;       这个是显式忽略变量result的值
  - 对2的倍数的取余运算，编译器会直接优化为按位与运算
- 乘除运算与位运算
  - 乘除运算如果是2的倍数也会简化为位运算

### 变量

- 全局变量和static变量的初始值必须在编译时就确定的，如果变量不是在编译时确定的，那么就会报错。

  - 示例：

  - ```
    int a_demo_function(void) {
        return 1;
    }
    
    int global_variable = a_demo_function();
    
    int main(void) {
        static int local_static_variable = a_demo_function();
    
        return 0;
    }
    ```

- static

  - 全局变量前的static
    - 作用域：x仅在当前文件内可见，其他文件无法通过extern引用
    - 生命周期：程序运行期间一直存在，初始化一次
    - 用途：隐藏全局变量，避免命名冲突
  - 局部变量前的static
    - 作用域：仍限于函数内部，但生命周期延长到程序结束
    - 初始化：仅在第一次调用函数时初始化，后续调用保留上次的值
  - 类成员变量前的static
    - 存储：所有对象共享同一个实例，不占用对象的内存空间
    - 访问：通过类名直接访问：MyClass::x，无需创建对象
  - 类成员函数前的static
    - 不依赖于对象，可以直接通过类名调用：MyClass::func()
    - 不能访问非晶态成员变量或函数（因为没有this指针）
  - 总结：
    - 作用域限制
      - 全局static:仅当前文件可见
      - 局部static:仅限于函数/代码块内
      - 类中static:属于类而非对象，所有实例共享，可通过类名直接访问
    - 生命周期延长
      - 无论修饰全局、局部还是类成员，static变量的生命周期均为程序运行全程（从启动到结束），仅初始化一次
    - 函数修饰
      - static函数仅在当前文件可见，避免与其他文件的同名函数冲突

- 编译和运行

  - 编译时的操作对象是源代码文件，生成可执行文件。其中会涉及到语法检查、类型检查、优化、链接等操作
  - 运行时就是指运行时的动作，比如动态内存分配、动态链接等操作

- 整数

  - uint8表示占用8个二进制位

  - stdint.h

    - 这个头文件会根据系统平台与CPU架构来确定整数的范围

  - inttypes.h

    - 处理int8_t这些类型的数据

  - 整数的隐式类型转换

    - ```
      #include <stdint.h>
      #include <stdio.h>
      #include <inttypes.h>
      
      int main(void) {
          /* 无符号到有符号 */
          uint16_t u16num_1 = 45024;
          int16_t s16num_1 = u16num_1;
          printf("u16num = %" PRIu16 ", s16num = %" PRId16 "\n", u16num_1, s16num_1);
      
          /* 大范围到小范围 */
          int32_t s32num_2 = 40328;
          int16_t s16num_2 = s32num_2;
          printf("s32num = %" PRId32 ", s16num = %" PRId16 "\n", s32num_2, s16num_2);
      
          /* 小范围到无符号大范围 */
          int16_t s16num_3 = -1;
          uint32_t u32num_3 = s16num_3;
          printf("s16num = %" PRId16 ", s32num = %" PRIu32 "\n", s16num_3, u32num_3);
      
          return 0;
      }
      ```

    - printf("u16num = %" PRIu16 ", s16num = %" PRId16 "\n", u16num_1, s16num_1);（中的PRIu16等价于"%hu"，然后相邻字符串字面量会自动拼接，导致PRIu16可以不被双引号包裹）

    - 隐式类型转换是有顺序的，char,short等低类型会被转换成int高类型，优先级如下：char, short --> int --> unsigned --> long --> double <-- float

    - 示例（不同类型的数运算）：
    
      - ```
        #include <stdint.h>
        #include <stdio.h>
        
        int main(void) {
            uint32_t u32_num = 10;
            int32_t s32_num = -1;
        
            if (s32_num > u32_num) {
                printf("%d > %u\n", s32_num, u32_num);
            }
        
            if ((s32_num - u32_num) > 0) {
                printf("signed:     %d - %u = %d\n", s32_num, u32_num,
                       (s32_num - u32_num));
                printf("unsigned:   %d - %u = %u\n", s32_num, u32_num,
                       (s32_num - u32_num));
            }
        
            return 0;
        }
        ```
        
        这是由于 `uint32_t` 与 `int32_t` 运算时，结果会自动变成 `uint32_t` ，也就是说，第一个 `if` 虽然在数学上 `s32_num > u32_num` ，但是计算机在比较的时候第二个数隐式转换成了 `uint32_t` ，按照二进制补码，-1 就是 `0xFFFF FFFF` ，转换成十进制就是 4,294,967,295，显然这么大的数比 10 大的多，因此计算机认为第一个 `if` 条件为真。

### 复杂数据类型

- 枚举用来处理状态、错误时非常有用，而且非常容易扩展。

  - 如果不定义成员初值，默认从0开始递增。如果中间有个成员定义初值，后面的成员会从这个成员的值开始递增。经常和switch-case联系在一起使用

- 结构体赋值与函数间传递

  - 可以以（.成员名称）的方式赋值

  - ```
    student_t student2 = {
        .num = 12300, /* 学号为12300 */
        .age = 22,    /* 年龄为22 */
        /* 嵌套结构体定义初值 */
        .scores.Chinese = 100, /* 语文100分 */
        .scores.math = 90,     /* 数学90分 */
        .scores.English = 98   /* 英语98分 */
    };
    ```

  - 无法一次性输出整个结构体变量，只能输出结构体变量的成员

  - 如果结构体类型作为函数参数，在函数参数压栈时结构体变量会复制一份，这不仅意味着会花费更多的时间与空间，还意味着函数内所有对结构体的修改操作都不会修改调用方的结构体

  - 如果结构体他类型作为返回值，相较于放回指针会花费更长时间

  - ```
    #include <stdio.h>
    #include <string.h>
    
    typedef struct {
        char name[10];
        int age;
        char sex;
    } student_t;
    
    void modify_name(student_t student);
    
    int main(void) {
        student_t student1 = {.age = 20, .sex = 'M'};
        strcpy(student1.name, "Tom");
        printf("Student 1 name: %s\n", student1.name);
        modify_name(student1);
        printf("Student 1 new name: %s\n", student1.name);
        return 0;
    }
    
    void modify_name(student_t student) {
        puts("Enter new name: ");
        scanf("%s", student.name);
    }
    ```

    ```
    运行结果：
    Student 1 name: Tom
    Enter new name: Jerry
    Student 1 new name: Tom
    ```

- 结构体内存对齐
  - 为了加快CPU访问速度，结构体成员在内存中并不是连续分布的。内存对齐遵循以下的规则
    - 第一个成员偏移地址为0
    - 结构体偏移地址，必须是对齐数的整数倍
    - 结构体总大小，必须是最大对齐数的整数倍
      - 偏移地址：从结构体的首地址到成员首地址的偏移量
      - 对齐数：【成员自身大小】和【默认对齐数】的较小值
      - 最大对齐数：结构体内成员对齐数最大的那个
  - offsetof(TYPE,MEMBER)：获取结构体TYPE的成员MEMBER的偏移地址，在stddef.h中
  
- 嵌套结构体内存对齐

  - 嵌套结构体对齐在自己最大对齐数的整数倍

- 指定对齐数

  - #pragma pack(n)指定默认对齐数
  - _\_attribute__关键字设置内存对齐

- 位域

  - 位域是将数据以位的形式存储成员，并可以按位操作成员。

  - 位域的生命格式是：类型 成员名称 ： 位数

  - ```
    struct foo {
        int id : 8;
        int result : 4;
        int temperature : 8;
    }
    ```

    ```
    上面结构体中id占 8 bit，result占 4 bit，temperature占 8 bit
    ```

  - 如果一个成员溢出后并不会影响其他成员。位域只能使用整数，不可以用浮点与指针

  - ```
    struct foo {
        float fp8: 8;
        int *ptr4: 4;
        double fp16: 16;
    };
    会报错
    ```

- 联合体

  - 联合体在通信中的应用

    - 一般在通信中，数据时按字节传输的。如果我们要传输int,float,double这种多字节的数据，可以用联合体，发送时将数据拆分成字节发送，接收时将字节组合成数据

    - ```
      #include <stdio.h>
      #include <stdint.h>
      
      typedef union {
          double data;
          uint8_t byte[8];
      } fp_data_t;
      
      typedef union {
          int32_t data;
          uint8_t byte[4];
      } s32_data_t;
      
      /* In actually program, you can use interrupt to fill the buff. */
      fp_data_t recv_fp_buf;
      s32_data_t recv_s32_buf;
      
      void send_fp_data(fp_data_t *fp_data);
      
      void send_s32_data(s32_data_t *s32_data);
      
      int main(void) {
          fp_data_t send_fp;
          s32_data_t send_s32;
      
          send_fp.data = 3.14f;
          send_s32.data = 3321;
          printf("Send data: %lf, %d\n", send_fp.data, send_s32.data);
      
          send_fp_data(&send_fp);
          send_s32_data(&send_s32);
      
          printf("Received data: %lf, %d\n", recv_fp_buf.data, recv_s32_buf.data);
      
          return 0;
      }
      
      void send_fp_data(fp_data_t *fp_data) {
          /* Just a demonstration, fill the received buf.
           * If successful, the received data is the same as the sent data. */
          for (int i = 0; i < 8; ++i) {
              recv_fp_buf.byte[i] = fp_data->byte[i];
          }
      }
      
      void send_s32_data(s32_data_t *s32_data) {
          /* Just a demonstration, fill the received buf.
           * If successful, the received data is the same as the sent data. */
          for (int i = 0; i < 4; ++i) {
              recv_s32_buf.byte[i] = s32_data->byte[i];
          }
      }
      ```

    - ```
      修改data会直接影响byte数组的内容
      修改byte数组也会直接影响data的值
      ```

  - 联合体模拟函数重载

    - 函数重载就是允许程序中存在同名的函数，同名函数可以有不同的参数类型，参数数量和返回值

### 数组与指针

- 零长数组和变长数组

- ```
  int arr1[4] = {[1] 4, [3] 8};
  int arr2[5] = {[2] = 5, [4] = 9};
  ```

  - ```
    #include <stdio.h>
    
    int *new_array(void) {
        int arr[5] = {1, 2, 3, 4, 5};
        return arr;
    }
    
    int main(void) {
        int *arr = new_array();
        printf("arr = %p\n", arr);
    
        for (int i = 0; i < 5; ++i) {
            printf("arr[%d] = %d\t", i, arr[i]);
        }
    
        putchar('\n');
    
        return 0;
    }
    ```

  - ```
    ubuntu@hi3798mv100:~/C-Learn$ gcc main.c -o main
    main.c: In function ‘new_array’:
    main.c:5:12: warning: function returns address of local variable [-Wreturn-local-addr]
        5 |     return arr;
          |            ^~~
    ubuntu@hi3798mv100:~/C-Learn$ ./main
    arr = (nil)
    Segmentation fault
    ubuntu@hi3798mv100:~/C-Learn$
    ```

  - ```
    C语言不允许我们返回局部变量的数组，如果返回一个局部变量的数组，为了安全，编译器会让这个函数返回空指针
    但是写成这样是可以的static int arr[5] = {1, 2, 3, 4, 5};
    
    #include <stdio.h>
    
    int *new_array(void) {
        static int arr[5] = {1, 2, 3, 4, 5};
        return arr;
    }
    
    int main(void) {
        int *arr;
        arr = new_array();
        printf("arr = %p\n", arr);
    
        for (int i = 0; i < 5; ++i) {
            printf("arr[%d] = %d\t", i, arr[i]);
        }
    
        putchar('\n');
    
        return 0;
    }
    ubuntu@hi3798mv100:~/C-Learn$ gcc main.c -o main && ./main
    arr = 0x4d3008
    arr[0] = 1      arr[1] = 2      arr[2] = 3      arr[3] = 4      arr[4] = 5
    ubuntu@hi3798mv100:~/C-Learn$
    ```

- 数组退化指针

  - ```
    #include <stdio.h>
    
    int main(void) {
        int arr[5] = {1, 2, 3, 4, 5};
    
        int *ptr = arr;
        printf("sizeof(arr) = %zu\n", sizeof(arr));
        printf("sizeof(ptr) = %zu\n", sizeof(ptr));
    
        return 0;
    }
    ```

  - ```
    运行结果：
    sizeof(arr) = 20
    sizeof(ptr) = 8
    ```

  - int [] 类型隐式转换为 int * 类型时允许的，但是反过来是不允许的（数组不能作为普通的指针变量一样赋值）

  - int* ptr1, ptr2;表示ptr1是int*型的指针变量，但是ptr2是int型的变量，它不是指针。所以需要把星号放在变量一侧，也就是int *ptr1, ptr2;

- 取内容运算符 *

  - & 与 * 是一对互逆的运算，&是取地址， * 是取地址里的内容



---

### C语言中的栈和堆跟数据结构中的栈和堆的区别

- C语言中的内存区域

  - 栈

    - 由操作系统自动管理的内存区域，用于存储局部变量、函数调用信息（如返回地址、参数）等

    - ```
      void func() {
          int a = 10;         // 栈上分配的局部变量
          char buffer[100];   // 栈上分配的数组
      }                       // 函数返回时，a和buffer自动释放
      ```

  - 堆

    - 由程序员手动管理的内存区域，用于动态分配内存（如malloc、free）

    - ```
      void func() {
          int* p = (int*)malloc(sizeof(int));  // 堆上分配内存
          if (p != NULL) {
              *p = 10;
              free(p);  // 必须手动释放，否则内存泄漏
          }
      }
      ```

- 数据结构中的抽象类型

  - 栈

    - 一种遵循“后进先出（LIFO）”原则的数据结构，仅允许在栈顶进行插入（push）和删除(pop)操作

    - ```
      // 数据结构中的栈（链表实现）
      struct StackNode {
          int data;
          struct StackNode* next;
      };
      
      void push(struct StackNode** top, int data);  // 入栈
      int pop(struct StackNode** top);              // 出栈
      ```

  - 堆

    - 一种遵循完全二叉树结构且满足堆属性的数据结构（最大堆或最小堆）

    - ```
      // 数据结构中的堆（最小堆）
      struct MinHeap {
          int* array;
          int size;
          int capacity;
      };
      
      void insert(struct MinHeap* heap, int data);  // 插入元素
      int extractMin(struct MinHeap* heap);         // 获取最小值
      ```

- 总结

  

  - | **维度**     | **C 语言中的栈（内存区域）** | **C 语言中的堆（内存区域）** | **数据结构中的栈** | **数据结构中的堆**  |
    | ------------ | ---------------------------- | ---------------------------- | ------------------ | ------------------- |
    | **本质**     | 内存管理方式                 | 内存管理方式                 | 抽象数据类型       | 抽象数据类型        |
    | **管理方式** | 操作系统自动管理             | 程序员手动管理               | 手动实现           | 手动实现            |
    | **访问规则** | LIFO                         | 随机访问                     | LIFO               | 完全二叉树 + 堆属性 |
    | **典型应用** | 局部变量、函数调用           | 动态内存分配                 | 函数调用栈、回溯   | 优先队列、堆排序    |
    | **内存效率** | 高                           | 低（涉及碎片）               | 取决于实现         | 取决于实现          |

---

- 一个C程序内存大概可以分为五个部分：（1）程序段（2）初始化的数据段（3）未初始化的数据段（4）栈（5）堆
- 一个程序在运行之前，需要操作系统将程序文件加载在内存中，然后设置命令行参数和环境变量。运行时，程序会将全局变量初始化（初始化数据段），未初始化的变量将会清零。
- 程序可以管理三类内存：静态、自动和动态。静态内存的地址在程序运行前就固定好的。自动内存是程序运行时自动分配、清理，例如函数里的局部变量，它是分配在栈当中的。动态内存的运行是动态的分配、清理，它与自动内存的区别是需要在代码中管理内存；自动内存不需要我们在代码中关心分配与释放
  - 如果使用malloc分配内存，内存区域的数据时随机的；而使用calloc会帮我们把内存区域清零。malloc的参数是传入要分配的内存大小（以字节为单位），而calloc有两个参数：第一个参数是要分配成员的个数（单位不是字节）；第二个参数是单个成员的大小，也就是，总共分配的内存大小（字节）=成员个数（参数1） * 单个成员大小（参数2）（calloc实际上就是先malloc，然后再memset成0）

- 指向零地址的指针我们叫做空指针，指向不存在的地址我们叫做野指针

###　预处理命令

- 预处理指令以 # 开头，不以分好结尾。如果一行写不下，可以在行尾加一个 \ 作为续行符，代表下一行人仍然是属于这个宏定义的一部分

- 头文件保护

  - 在一个程序中一个头文件不可能只被包含一次，，但每个源文件每包含一次就需要编译一次，当编译完链接的时候就出问题了。所以我们应该控制连接器只链接一个

  - ```
    #ifndef __XXX_H
    #define __XXX_H
    /* ... */
    #endif /* __XXX_H */
    ```

  - 这段代码的意思是如果没有定义 `__XXX_H` ，就定义 `__XXX_H` ，换句话说如果定义了那么下面的东西就不需要了。链接器去链接的时候：第一个 `.o` 没定义 `__XXX_H` ，定义一下，把这里面的东西都保留下来；第二个 `.o` 已经定义了 `__XXX_H` ，直接跳过这部分到 `#endif /* __XXX_H */` 。这样就避免了链接器发现重复定义，也就是保护了头文件。

  - #pragma once也行

- 头文件路径

  - 使用双引号的头文件，查找头文件的顺序为：
    - **在源文件所在目录里查找**
    - 编译器设置的头文件查找路径，编译器默认的头文件查找路径（也即使stdio.h这类编译器自带的头文件路径）
    - 环境变量CPLUS_INCLUDE_PATH 与 C_INCLUDE_PATH指定的头文件路径
  - 使用尖括号的头文件，查找头文件顺序为：
    - 编译器设置的头文件查找路径，编译器默认的头文件查找路径（也即使stdio.h这类编译器自带的头文件路径）
    - 环境变量CPLUS_INCLUDE_PATH 与 C_INCLUDE_PATH指定的头文件路径

- 判断头文件是否存在

  - __has_include

### 函数调用约定

- 主要规定：

  - 参数压栈顺序
  - 调用前谁来压栈，调用后谁来清栈
  - 返回值如何返回

- 常见的调用约定有：

  - stdcall
  - cdecl
  - fastcall
  - thiscall
  - nakedcall

- 如何指定调用约定

  - 在函数名前加__callName

  - ```
    int __stdcall function(int param1, int param2)
    int __cdecl function(int param1, int param2)
    int __thiscall function(int param1, int param2)
    ```

### GNU扩展

- 在标准C中，会定义_\_STDC__与\_\_STDC_VERSION\_\_宏，我们可以通过这两个宏判断C标准

  - 

    | 标准   |              宏              |
    | :----- | :--------------------------: |
    | C89/90 |          `__STDC__`          |
    | C99    | `__STDC_VERSION__ = 199901L` |
    | C11    | `__STDC_VERSION__ = 201112L` |

- _\_attribute__关键字

  - 语法是_\_attribute__((attribute-list))

- 设置变量属性

  - 设置内存对齐

    - _\_attribute__((packed))可以取消对齐，按照实际的成员大小对齐内存地址

    - ```
      #include <stddef.h>
      #include <stdint.h>
      #include <stdio.h>
      
      typedef struct foo {
          uint8_t member1;
          int8_t member2;
          uint32_t member3;
          uint16_t member4;
          uint64_t member5;
      }  __attribute__((packed)) foo_t;
      
      int main(void) {
          printf("sizeof foo_t: %zu\n", sizeof(foo_t));
          printf("offset of member1: %zu\n", offsetof(foo_t, member1));
          printf("offset of member2: %zu\n", offsetof(foo_t, member2));
          printf("offset of member3: %zu\n", offsetof(foo_t, member3));
          printf("offset of member4: %zu\n", offsetof(foo_t, member4));
          printf("offset of member5: %zu\n", offsetof(foo_t, member5));
      }
      ```

    - _\_attribute__((align(n)))是设置内存对齐，也就是说变量或类型大小必须是n的整数倍

    - alignof()宏用来获取对齐数

- 设置函数属性

  - 设置弱定义函数

    - 声明格式为_\_attribute__((weak))。弱定义函数使是指的函数可以被重定义，如果函数没有重定义，使用默认的弱函数
    - 弱函数和重定义函数需要在不同文件中

  - 设置非空参数

    - 声明格式为_\_attribute__((nonnull))。这个属性是用来限制函数传入空指针的

  - 设置不返回函数

    - 声明格式为_\_attribute__((noreturn))。在某些情况下，函数是不可以返回的。比如发送错误以后需要在错误处理函数中卡死，不能继续执行。

    - ```
      __attribute__((noreturn)) void error_handle(void) {
          printf("Error occurred");
          while (1)
              ;
      }
      ```

- 包装头文件

  - 假如你的项目中已经有一个头文件 `A.h` 了，但是 `A.h` 不能满足需求，我想修改 `A.h` 的内容。我没有权限直接修改或删除 `A.h` 。最主要的问题是还必须得用 `A.h` 这个文件名，而且还得用旧的 `A.h` 里的内容。那这怎么办呢？先不管其他的创建一个 `A.h` 再说。创建后怎么处理旧的 `A.h` ？有下面几个问题：
    - 在新的头文件中直接包含旧的 `A.h` ，即 `#include "A.h"` 。但这有个问题，头文件一般都会做编译保护，引入旧的 `A.h` 可能不会被编译；如果删除了编译保护，两个文件将会无限 `#include` 循环导致编译错误
    - 在新的头文件中用绝对路径包含旧的 `A.h` ，这不会产生上面的问题，但是如果文件被移动，或者其他系统没有这个文件也不能编译
  - 在 ISO C 中无法解决这个问题，但是可以用 GNU 的扩展指令， `#include_next "filename"` 。意思是去包含目录里找下一个叫 `filename` 的文件。它跟 `#include` 很像，但是不同的一点是不会有上面所说的编译保护问题。

- 零长数组

  - 一般情况下零长数组搭配结构体组成变长结构体来使用

  - ```
    typedef struct {
        int len;
        char str[0];
    } string_t;
    ```

  - 可以减少占用的空间

  - 零长数组成员必须放在结构体的最后一个成员（因为最后一个成员相当于是数组的首地址，它后面紧挨着的就是数组，如果不放到最后，会造成数组的内容写到结构体其他成员的位置）

- 变长数组

  - 全局数组不可以用变量来定义长度，哪怕使用const修饰的变量也不可以

- 数组初始化

  - ```
    #include <stdio.h>
    
    int array[] = {[0 ... 2] = 1, [4 ... 6] = 2};
    
    int main(void) {
        printf("length of array: %d\n", sizeof(array) / sizeof(array[0]));
        for (int i = 0; i < sizeof(array) / sizeof(array[0]); i++) {
            printf("array[%d] = %d\t", i, array[i]);
        }
        printf("\n");
        return 0;
    }
    ```

  - 三个点前后必须有空格，否则会导致编译错误